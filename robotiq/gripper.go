// Package robotiq implements the gripper from robotiq.
// commands found at
// https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_Instruction_Manual_CB-Series_PDF_20190329.pdf
package robotiq

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"net"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

// Model for viam supported robotiq 2f-grippers.
var Model = resource.NewModel("viam", "robotiq", "2f-grippers")

// Config is used for converting config attributes.
type Config struct {
	Host string `json:"host"`
}

// Validate ensures all parts of the config are valid.
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Host == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "host")
	}
	return nil, nil, nil
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

	conn   net.Conn
	connMu sync.Mutex // serializes request/response pairs on conn

	openLimit  string
	closeLimit string
	logger     logging.Logger
	opMgr      *operation.SingleOperationManager
	geometries []spatialmath.Geometry
	model      referenceframe.Model
}

// 2F-85 collision envelope, measured from the ROS-Industrial robotiq_2f_85 URDF + collision meshes
// (matches the datasheet: 85 mm stroke, ~148 mm wide open, ~163 mm mounting face to closed
// fingertip). Units mm; X is the jaw travel axis, Z points out of the gripper.
//
// The gripper frame origin is taken to be the closed fingertip, i.e. the frame is configured with
// the TCP offset from the arm flange (coupling + 163 mm, e.g. z ~ 174 on a UR coupling). The
// mounting face therefore sits at Z = -mountToTip. The coupling itself is not modeled.
//
// The fingertips are separate boxes, sized for the fully open pose, so the space between the pads
// stays free and a grasped object does not register as a collision.
const mountToTip = 163.0

type boxSpec struct {
	label  string
	center r3.Vector // relative to the mounting face
	dims   r3.Vector
}

var geometry2F85 = []boxSpec{
	{"body", r3.Vector{Z: 45}, r3.Vector{X: 85, Y: 75, Z: 90}},                 // chassis, z 0..90
	{"linkage", r3.Vector{Z: 75}, r3.Vector{X: 150, Y: 40, Z: 60}},             // knuckles, z 45..105, +-74 open
	{"finger-left", r3.Vector{X: -50, Z: 125}, r3.Vector{X: 15, Y: 40, Z: 50}}, // pad + inner finger, |x| 43..57
	{"finger-right", r3.Vector{X: 50, Z: 125}, r3.Vector{X: 15, Y: 40, Z: 50}},
}

// defaultGeometries returns the 2F-85 collision boxes in the gripper frame.
func defaultGeometries() ([]spatialmath.Geometry, error) {
	geoms := make([]spatialmath.Geometry, 0, len(geometry2F85))
	for _, b := range geometry2F85 {
		c := b.center
		c.Z -= mountToTip
		box, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(c), b.dims, b.label)
		if err != nil {
			return nil, err
		}
		geoms = append(geoms, box)
	}
	return geoms, nil
}

// geometryModel wraps geometries in a 0-DoF model. RDK drops grippers whose Kinematics errors
// from the frame system, so without this the gripper is invisible to motion planning.
func geometryModel(name string, geoms []spatialmath.Geometry) (referenceframe.Model, error) {
	cfg := &referenceframe.ModelConfigJSON{Name: name}
	parent := referenceframe.World
	for _, geom := range geoms {
		frame, err := referenceframe.NewStaticFrameWithGeometry(geom.Label(), spatialmath.NewZeroPose(), geom)
		if err != nil {
			return nil, err
		}
		link, err := referenceframe.NewLinkConfig(frame)
		if err != nil {
			return nil, err
		}
		link.Parent = parent
		parent = geom.Label()
		cfg.Links = append(cfg.Links, *link)
	}
	raw, err := json.Marshal(cfg)
	if err != nil {
		return nil, err
	}
	cfg.OriginalFile = &referenceframe.ModelFile{Bytes: raw, Extension: "json"}
	return cfg.ParseConfig(name)
}

// newGripper instantiates a new Gripper of robotiqGripper type.
func newGripper(ctx context.Context, conf resource.Config, host string, logger logging.Logger) (gripper.Gripper, error) {
	conn, err := net.Dial("tcp", host+":63352")
	if err != nil {
		return nil, err
	}
	g := &robotiqGripper{
		Named:      conf.ResourceName().AsNamed(),
		conn:       conn,
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
	err = g.MultiSet(ctx, init)
	if err != nil {
		return nil, err
	}

	err = g.Calibrate(ctx) // TODO(erh): should this live elsewhere?
	if err != nil {
		return nil, err
	}

	if conf.Frame != nil && conf.Frame.Geometry != nil {
		geometry, err := conf.Frame.Geometry.ParseConfig()
		if err != nil {
			return nil, err
		}
		g.geometries = []spatialmath.Geometry{geometry}
	} else {
		g.geometries, err = defaultGeometries()
		if err != nil {
			return nil, err
		}
	}
	g.model, err = geometryModel(conf.Name, g.geometries)
	if err != nil {
		return nil, err
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

// Send TODO.
func (g *robotiqGripper) Send(msg string) (string, error) {
	g.connMu.Lock()
	defer g.connMu.Unlock()

	_, err := g.conn.Write([]byte(msg))
	if err != nil {
		return "", err
	}

	res, err := g.read()
	if err != nil {
		return "", err
	}

	return res, err
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

func (g *robotiqGripper) read() (string, error) {
	buf := make([]byte, 128)
	x, err := g.conn.Read(buf)
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

// ModelFrame returns the gripper's static geometry model.
func (g *robotiqGripper) ModelFrame() referenceframe.Model {
	return g.model
}

// Kinematics returns a 0-DoF model carrying the gripper's collision geometry.
func (g *robotiqGripper) Kinematics(ctx context.Context) (referenceframe.Model, error) {
	return g.model, nil
}

// CurrentInputs returns the current inputs for the gripper (always empty for this gripper).
func (g *robotiqGripper) CurrentInputs(ctx context.Context) ([]referenceframe.Input, error) {
	return []referenceframe.Input{}, nil
}

// GoToInputs moves the gripper to the given inputs (unimplemented).
func (g *robotiqGripper) GoToInputs(ctx context.Context, inputSteps ...[]referenceframe.Input) error {
	return errors.New("GoToInputs not supported")
}

// IsHoldingSomething returns whether the gripper is currently holding onto an object.
func (g *robotiqGripper) IsHoldingSomething(ctx context.Context, extra map[string]interface{}) (gripper.HoldingStatus, error) {
	val, err := g.Get("OBJ")
	if err != nil {
		return gripper.HoldingStatus{}, err
	}
	return gripper.HoldingStatus{IsHoldingSomething: val == "OBJ 2"}, nil
}

// Geometries returns the geometries associated with robotiqGripper.
func (g *robotiqGripper) Geometries(ctx context.Context, extra map[string]interface{}) ([]spatialmath.Geometry, error) {
	return g.geometries, nil
}

// DoCommand exposes raw position control.
// Raw Robotiq units: 0 = fully open, 255 = fully closed (bounded by calibrated openLimit/closeLimit).
//
//	{"get": true}                     -> {"pos": <int>}        current position
//	{"set": <number>}                 -> {"position": <int>}   move to raw position, block until it stops
//	{"set": <number>, "wait": false}  -> {"position": <int>}   send the target and return immediately
//
// "wait": false is for streaming targets (teleop): the gripper retargets mid-motion, so each
// new command preempts the last without waiting for the jaw to arrive.
func (g *robotiqGripper) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
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
		pos := int(math.Round(math.Max(0, math.Min(255, posF))))

		// Starting a new operation cancels any in-flight Open/Close/blocking set poll loop.
		ctx, done := g.opMgr.New(ctx)
		defer done()

		if wait, ok := cmd["wait"].(bool); ok && !wait {
			if err := g.Set("POS", strconv.Itoa(pos)); err != nil {
				return nil, err
			}
			return map[string]interface{}{"position": pos}, nil
		}
		if _, err := g.SetPos(ctx, strconv.Itoa(pos)); err != nil {
			return nil, err
		}
		return map[string]interface{}{"position": pos}, nil
	}
	return nil, errors.Errorf("unknown command %v; expected {\"get\": true} or {\"set\": <0-255>}", cmd)
}
