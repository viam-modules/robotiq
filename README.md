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

### Collision geometry

The gripper reports a static (0-DoF) kinematic model, so it appears in the frame system, the 3D visualizer and motion planning. By default it carries four boxes approximating a 2F-85 in its fully open pose:

| Box | Size X × Y × Z (mm) | Notes |
| --- | ------------------- | ----- |
| `body` | 85 × 75 × 90 | Chassis, from the mounting face |
| `linkage` | 150 × 40 × 60 | Knuckles, full open width |
| `finger-left`, `finger-right` | 15 × 40 × 50 each | Fingertips at x = ±50 mm; the gap between the pads stays free for grasped objects |

X is the jaw travel axis and Z points out of the gripper. The frame origin is the closed fingertip: the mounting face sits 163 mm behind it, so set the component's frame `translation` to the TCP offset from the arm flange (coupling thickness + 163 mm, about `z: 174` on a UR coupling). The coupling is not modeled.

To use your own shape instead (for example on a 2F-140), set `frame.geometry` on the component; it replaces the default boxes.

### Next Steps
- To test your gripper, expand the **TEST** section of its configuration pane or go to the [**CONTROL** tab](https://docs.viam.com/fleet/control/).
- To write code against your gripper, use one of the [available SDKs](https://docs.viam.com/sdks/).
- To view examples using a gripper component, explore [these tutorials](https://docs.viam.com/tutorials/).
