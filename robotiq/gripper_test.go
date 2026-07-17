package robotiq

import (
	"context"
	"net"
	"strings"
	"sync"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/operation"
	"go.viam.com/test"
)

func TestSetPosRejectsNonNumeric(t *testing.T) {
	g := &robotiqGripper{}
	_, err := g.SetPos(context.Background(), "?")
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "invalid target position")
}

// fakeURCap is a minimal in-process stand-in for the Robotiq URCap on port
// 63352. It responds "ack" to SET commands and answers GET STA with a state
// value it updates in response to SET ACT writes: SET ACT 0 -> STA 0, SET ACT
// 1 -> STA 3 (instant activation).
type fakeURCap struct {
	ln   net.Listener
	mu   sync.Mutex
	seen []string
	sta  string
}

func newFakeURCap(t *testing.T) *fakeURCap {
	t.Helper()
	ln, err := net.Listen("tcp", "127.0.0.1:63352")
	test.That(t, err, test.ShouldBeNil)
	f := &fakeURCap{ln: ln, sta: "0"}
	go f.serve()
	return f
}

func (f *fakeURCap) close() { _ = f.ln.Close() }

func (f *fakeURCap) commands() []string {
	f.mu.Lock()
	defer f.mu.Unlock()
	out := make([]string, len(f.seen))
	copy(out, f.seen)
	return out
}

func (f *fakeURCap) serve() {
	for {
		conn, err := f.ln.Accept()
		if err != nil {
			return
		}
		go f.handle(conn)
	}
}

func (f *fakeURCap) handle(conn net.Conn) {
	defer conn.Close()
	buf := make([]byte, 128)
	n, err := conn.Read(buf)
	if err != nil {
		return
	}
	cmd := strings.TrimSpace(string(buf[:n]))

	f.mu.Lock()
	f.seen = append(f.seen, cmd)
	switch {
	case cmd == "SET ACT 0":
		f.sta = "0"
	case cmd == "SET ACT 1":
		f.sta = "3"
	}
	var resp string
	switch {
	case cmd == "GET STA":
		resp = "STA " + f.sta
	default:
		resp = "ack"
	}
	f.mu.Unlock()
	_, _ = conn.Write([]byte(resp))
}

// TestActivateDrivesRACTEdgeAndWaitsForSTA3 verifies the core fix: activate
// forces a SET ACT 0 -> SET ACT 1 transition (bare SET ACT 1 was a no-op after
// a swap because rACT was still 1), then waits for STA 3.
//
// Slow (~3.6s) because MultiSet has hard-coded post-write waits.
func TestActivateDrivesRACTEdgeAndWaitsForSTA3(t *testing.T) {
	urcap := newFakeURCap(t)
	defer urcap.close()

	g := &robotiqGripper{
		host:   "127.0.0.1",
		logger: logging.NewTestLogger(t),
		opMgr:  operation.NewSingleOperationManager(),
	}

	err := g.activate(context.Background())
	test.That(t, err, test.ShouldBeNil)

	seen := urcap.commands()
	idx0 := indexOf(seen, "SET ACT 0")
	idx1 := indexOf(seen, "SET ACT 1")
	test.That(t, idx0, test.ShouldBeGreaterThanOrEqualTo, 0)
	test.That(t, idx1, test.ShouldBeGreaterThan, idx0)
	test.That(t, indexOf(seen, "GET STA"), test.ShouldBeGreaterThan, idx1)
}

func indexOf(ss []string, target string) int {
	for i, s := range ss {
		if s == target {
			return i
		}
	}
	return -1
}
