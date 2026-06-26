# [`robotiq` module](https://github.com/viam-modules/robotiq)

This [robotiq module](https://app.viam.com/module/viam/robotiq) implements a robotiq [2f-grippers gripper](https://robotiq.com/products/adaptive-grippers#Two-Finger-Gripper), using the [`rdk:component:gripper` API](https://docs.viam.com/appendix/apis/components/gripper/).

> [!NOTE]
> Before configuring your gripper, you must [create a machine](https://docs.viam.com/cloud/machines/#add-a-new-machine).

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in the [Viam app](https://app.viam.com/).
[Add gripper / robotiq:2f-grippers to your machine](https://docs.viam.com/configure/#components).

## Configure your 2f-grippers gripper

Copy and paste the following attributes into your JSON configuration:
```json
{
  "host": "<your-gripper-ip-address>"
}
```

### Attributes

The following attributes are available for `viam:robotiq:2f-grippers` grippers:

| Attribute | Type | Required? | Description |
| --------- | ---- | --------- | ----------  |
| `host` | string | **Required** | The host address of your gripper machine. |

## Example configuration

```json
  {
      "name": "<your-robotiq-2f-grippers-name>",
      "model": "viam:robotiq:2f-grippers",
      "type": "gripper",
      "namespace": "rdk",
      "attributes": {
        "host": "0.0.0.0"
      },
      "depends_on": []
  }
```

### DoCommand

Raw position control. Robotiq units: `0` = fully open, `255` = fully closed (bounded by the calibrated open/close limits detected at startup).

```go
// Get the current position
resp, err := gripperComponent.DoCommand(context.Background(), map[string]interface{}{"get": true})
// resp["pos"] contains the position

// Move to a specific position
resp, err := gripperComponent.DoCommand(context.Background(), map[string]interface{}{"set": 128.0})
// resp["position"] contains the commanded position
```

#### Reactivate

Forces the module to close its URCap socket, re-dial, and re-run the activation sequence. Useful as a manual escape hatch when the gripper is in a known-bad state, or after a deliberate physical reconfiguration (e.g. a tool changer just swapped the attached gripper) when you want to ensure a clean handshake before issuing commands. Auto-recovery on transport failures is already built into every command path, so explicit `reactivate` is rarely necessary.

```go
resp, err := gripperComponent.DoCommand(context.Background(), map[string]interface{}{"reactivate": true})
// resp["reactivated"] is true on success
```

### Socket behavior

The module dials a fresh TCP connection to the URCap for every command, sends one request, reads the response, and closes the connection. PolyScope X's URCap leaves persistent sockets in states where reads stall indefinitely after idle periods or state changes (e.g. tool-changer hot swaps), so the dial-per-send pattern avoids that whole class of failures. A 5-second read/write deadline bounds each call. Concurrent callers are serialized so commands don't interleave on the URCap.

If the URCap state itself was reset (for example after a physical disconnect such as a tool changer swapping the attached gripper), commands will be accepted but the gripper won't physically move until it's re-activated. Call the `reactivate` DoCommand to re-run the activation sequence (`ACT`/`GTO`/`FOR`/`SPE`) over the URCap socket.

### Next Steps
- To test your gripper, expand the **TEST** section of its configuration pane or go to the [**CONTROL** tab](https://docs.viam.com/fleet/control/).
- To write code against your gripper, use one of the [available SDKs](https://docs.viam.com/sdks/).
- To view examples using a gripper component, explore [these tutorials](https://docs.viam.com/tutorials/).
