// Package robotiq implements the gripper from robotiq.
// commands found at
// https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_CB-Series_PDF_20190329.pdf
package robotiq

import (
	"context"
	"fmt"
	"net"
	"strconv"
	"strings"
	"time"

	"github.com/pkg/errors"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

// Per-call URCap socket deadline so a half-dead connection can't block forever.
const sendTimeout = 5 * time.Second

// Model for viam supported robotiq 2f-grippers.
var Model = resource.NewModel("viam", "robotiq", "2f-grippers")

// Config is used for converting config attributes.
type Config struct {
	Host string `json:"host"`
}

// Validate ensures all parts of the config are valid.
func (cfg *Config) Validate(path string) ([]string, error) {
	if cfg.Host == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	return nil, nil
}

func init() {
	resource.RegisterComponent(gripper.API, Model, resource.Registration[gripper.Gripper, *Config]{
		Constructor: func(
			ctx context.Context, _ resource.Dependencies, conf resource.Config, logger logging.Logger,
		) (gripper.Gripper, error) {
			newConf, err := resource.NativeConfig[*Config](conf)
			if err != nil {
				return nil, err
			}
			return newGripper(ctx, conf, newConf.Host, logger)
		},
	})
}

// robotiqGripper TODO.
type robotiqGripper struct {
	resource.Named
	resource.AlwaysRebuild

	host string

	openLimit  string
	closeLimit string
	logger     logging.Logger
	opMgr      *operation.SingleOperationManager
	geometries []spatialmath.Geometry
}

// newGripper instantiates a new Gripper of robotiqGripper type.
func newGripper(ctx context.Context, conf resource.Config, host string, logger logging.Logger) (gripper.Gripper, error) {
	g := &robotiqGripper{
		Named:      conf.ResourceName().AsNamed(),
		host:       host,
		openLimit:  "0",
		closeLimit: "255",
		logger:     logger,
		opMgr:      operation.NewSingleOperationManager(),
		geometries: []spatialmath.Geometry{},
	}

	// Don't fail construction if no gripper is coupled yet: a tool changer may
	// attach one later and call activate. Avoids construction-retry churn.
	if err := g.activate(ctx); err != nil {
		logger.CWarnf(ctx, "robotiq: initial activation failed "+
			"(no gripper coupled yet?); using default limits open=%s close=%s: %v",
			g.openLimit, g.closeLimit, err)
	}

	if conf.Frame != nil && conf.Frame.Geometry != nil {
		geometry, err := conf.Frame.Geometry.ParseConfig()
		if err != nil {
			return nil, err
		}
		g.geometries = []spatialmath.Geometry{geometry}
	}

	return g, nil
}

// MultiSet TODO.
func (g *robotiqGripper) MultiSet(ctx context.Context, cmds [][]string) error {
	for _, i := range cmds {
		err := g.Set(i[0], i[1])
		if err != nil {
			return err
		}

		// TODO(erh): the next 5 lines are infuriatng, help!
		var waitTime time.Duration
		if i[0] == "ACT" {
			waitTime = 1600 * time.Millisecond
		} else {
			waitTime = 500 * time.Millisecond
		}
		if !utils.SelectContextOrWait(ctx, waitTime) {
			return ctx.Err()
		}
	}

	return nil
}

// Send runs one Robotiq command over a fresh TCP connection. Persistent sockets
// to the PolyScope X URCap stall on reads after idle periods or hot-swaps.
func (g *robotiqGripper) Send(msg string) (string, error) {
	conn, err := net.Dial("tcp", g.host+":63352")
	if err != nil {
		return "", err
	}
	defer utils.UncheckedErrorFunc(conn.Close)
	utils.UncheckedError(conn.SetDeadline(time.Now().Add(sendTimeout)))
	if _, err := conn.Write([]byte(msg)); err != nil {
		return "", err
	}
	return g.read(conn)
}

// Set TODO.
func (g *robotiqGripper) Set(what, to string) error {
	res, err := g.Send(fmt.Sprintf("SET %s %s\r\n", what, to))
	if err != nil {
		return err
	}
	if res != "ack" {
		return errors.Errorf("didn't get ack back, got [%s]", res)
	}
	return nil
}

// Get TODO.
func (g *robotiqGripper) Get(what string) (string, error) {
	return g.Send(fmt.Sprintf("GET %s\r\n", what))
}

func (g *robotiqGripper) read(conn net.Conn) (string, error) {
	buf := make([]byte, 128)
	x, err := conn.Read(buf)
	if err != nil {
		return "", err
	}
	if x > 100 {
		return "", errors.Errorf("read too much: %d", x)
	}
	if x == 0 {
		return "", nil
	}
	return strings.TrimSpace(string(buf[0:x])), nil
}

// activate activates the gripper. Used at startup and after a tool changer swap,
// which drops the gripper to the reset state (STA 0). We clear rACT to 0 before
// setting it to 1 because activation only runs on a 0->1 transition, and after a
// swap the URCap can still hold rACT=1 (so a bare "ACT 1" is a no-op). Activation
// runs the gripper's own open/close self-test, which leaves it open and
// establishes the normalized 0..255 travel range, so no separate calibration is
// needed.
func (g *robotiqGripper) activate(ctx context.Context) error {
	if err := g.Set("ACT", "0"); err != nil {
		return err
	}
	if !utils.SelectContextOrWait(ctx, 500*time.Millisecond) {
		return ctx.Err()
	}

	init := [][]string{
		{"ACT", "1"},
		{"GTO", "1"},
		{"FOR", "200"},
		{"SPE", "255"},
	}
	if err := g.MultiSet(ctx, init); err != nil {
		return err
	}
	return g.waitForActivation(ctx)
}

const activationTimeout = 10 * time.Second

// waitForActivation polls until the gripper reports STA 3 (active), timing out
// after activationTimeout. STA values: 0=reset, 1/2=activating, 3=active.
func (g *robotiqGripper) waitForActivation(ctx context.Context) error {
	ctx, cancel := context.WithTimeout(ctx, activationTimeout)
	defer cancel()
	for {
		sta, err := g.Get("STA")
		if err != nil {
			return err
		}
		if sta == "STA 3" {
			return nil
		}
		if !utils.SelectContextOrWait(ctx, 100*time.Millisecond) {
			return errors.Wrapf(ctx.Err(), "gripper did not finish activating (last STA=%q)", sta)
		}
	}
}

// SetPos returns true iff reached desired position.
func (g *robotiqGripper) SetPos(ctx context.Context, pos string) (bool, error) {
	// Never send a non-numeric target: the URCap silently ignores "SET POS ?"
	// and the read then blocks until the deadline. Fail fast instead.
	if _, err := strconv.Atoi(pos); err != nil {
		return false, errors.Errorf("invalid target position %q; run "+
			"DoCommand{\"activate\":true} after a tool swap", pos)
	}

	err := g.Set("POS", pos)
	if err != nil {
		return false, err
	}

	prev := ""
	prevCount := 0

	for {
		x, err := g.Get("POS")
		if err != nil {
			return false, err
		}
		if x == "POS "+pos {
			return true, nil
		}

		if prev == x {
			if prevCount >= 5 {
				return false, nil
			}
			prevCount++
		} else {
			prevCount = 0
		}
		prev = x

		if !utils.SelectContextOrWait(ctx, 100*time.Millisecond) {
			return false, ctx.Err()
		}
	}
}

// Open TODO.
func (g *robotiqGripper) Open(ctx context.Context, extra map[string]interface{}) error {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	_, err := g.SetPos(ctx, g.openLimit)
	return err
}

// Close TODO.
func (g *robotiqGripper) Close(ctx context.Context) error {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	_, err := g.SetPos(ctx, g.closeLimit)
	return err
}

// Grab returns true iff grabbed something.
func (g *robotiqGripper) Grab(ctx context.Context, extra map[string]interface{}) (bool, error) {
	ctx, done := g.opMgr.New(ctx)
	defer done()

	res, err := g.SetPos(ctx, g.closeLimit)
	if err != nil {
		return false, err
	}
	if res {
		// we closed, so didn't grab anything
		return false, nil
	}

	// we didn't close, let's see if we actually got something
	val, err := g.Get("OBJ")
	if err != nil {
		return false, err
	}
	return val == "OBJ 2", nil
}

// Stop is unimplemented for robotiqGripper.
func (g *robotiqGripper) Stop(ctx context.Context, extra map[string]interface{}) error {
	// RSDK-388: Implement Stop
	err := g.Set("GTO", "0")
	if err != nil {
		return err
	}
	return nil
}

// IsMoving returns whether the gripper is moving.
func (g *robotiqGripper) IsMoving(ctx context.Context) (bool, error) {
	return g.opMgr.OpRunning(), nil
}

// ModelFrame is unimplemented for robotiqGripper.
func (g *robotiqGripper) ModelFrame() referenceframe.Model {
	return nil
}

// Geometries returns the geometries associated with robotiqGripper.
func (g *robotiqGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return g.geometries, nil
}

// DoCommand exposes raw position control and a manual activate action.
// Raw Robotiq units: 0 = fully open, 255 = fully closed.
//
//	{"get": true}       -> {"pos": <int>}          current position
//	{"set": <number>}   -> {"position": <int>}     move to raw position
//	{"activate": true}  -> {"activated": true}     re-run gripper activation
func (g *robotiqGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["activate"] == true {
		if err := g.activate(ctx); err != nil {
			return nil, err
		}
		return map[string]interface{}{"activated": true}, nil
	}
	if cmd["get"] == true {
		raw, err := g.Get("POS")
		if err != nil {
			return nil, err
		}
		pos, err := strconv.Atoi(strings.TrimPrefix(raw, "POS "))
		if err != nil {
			return nil, errors.Wrapf(err, "bad POS response %q", raw)
		}
		return map[string]interface{}{"pos": pos}, nil
	}
	if posF, ok := cmd["set"].(float64); ok {
		ctx, done := g.opMgr.New(ctx)
		defer done()

		pos := int(posF)
		if _, err := g.SetPos(ctx, strconv.Itoa(pos)); err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": pos}, nil
	}
	return map[string]interface{}{}, nil
}
