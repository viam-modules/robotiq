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
	"sync"
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

	// connMu serializes URCap access across the dial-per-send connections.
	connMu sync.Mutex
	host   string

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

	init := [][]string{
		{"ACT", "1"},   // robot activate
		{"GTO", "1"},   // gripper activate
		{"FOR", "200"}, // force (0-255)
		{"SPE", "255"}, // speed (0-255)
	}
	if err := g.MultiSet(ctx, init); err != nil {
		return nil, err
	}

	if err := g.Calibrate(ctx); err != nil { // TODO(erh): should this live elsewhere?
		return nil, err
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

// Send dials a fresh TCP connection, writes one Robotiq command, reads the
// response, and closes the connection. Persistent sockets to the PolyScope X
// URCap stall on reads after idle periods or hot-swaps; fresh sockets always work.
func (g *robotiqGripper) Send(msg string) (string, error) {
	g.connMu.Lock()
	defer g.connMu.Unlock()
	conn, err := net.Dial("tcp", g.host+":63352")
	if err != nil {
		return "", err
	}
	defer conn.Close()
	_ = conn.SetDeadline(time.Now().Add(sendTimeout))
	if _, err := conn.Write([]byte(msg)); err != nil {
		return "", err
	}
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

// reactivate re-runs ACT/GTO/FOR/SPE. Use after the URCap was reset (e.g. a
// tool changer swapping the attached gripper) and needs a fresh handshake.
func (g *robotiqGripper) reactivate(ctx context.Context) error {
	init := [][]string{
		{"ACT", "1"},
		{"GTO", "1"},
		{"FOR", "200"},
		{"SPE", "255"},
	}
	return g.MultiSet(ctx, init)
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

// SetPos returns true iff reached desired position.
func (g *robotiqGripper) SetPos(ctx context.Context, pos string) (bool, error) {
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

// Calibrate TODO.
func (g *robotiqGripper) Calibrate(ctx context.Context) error {
	err := g.Open(ctx, map[string]interface{}{})
	if err != nil {
		return err
	}

	x, err := g.Get("POS")
	if err != nil {
		return err
	}
	g.openLimit = x[4:]

	err = g.Close(ctx)
	if err != nil {
		return err
	}

	x, err = g.Get("POS")
	if err != nil {
		return err
	}
	g.closeLimit = x[4:]

	g.logger.CDebugf(ctx, "limits %s %s", g.openLimit, g.closeLimit)
	return nil
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

// DoCommand exposes raw position control and a manual reactivate action.
// Raw Robotiq units: 0 = fully open, 255 = fully closed (bounded by calibrated openLimit/closeLimit).
//   {"get": true}         -> {"pos": <int>}            current position
//   {"set": <number>}     -> {"position": <int>}       move to raw position
//   {"reactivate": true}  -> {"reactivated": true}     force socket re-dial and re-init
func (g *robotiqGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["reactivate"] == true {
		if err := g.reactivate(ctx); err != nil {
			return nil, err
		}
		return map[string]interface{}{"reactivated": true}, nil
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
