# [`robotiq` module](https://github.com/viam-modules/robotiq)

This [robotiq module](https://app.viam.com/module/viam/robotiq) implements a robotiq [2f-grippers gripper](https://robotiq.com/products/adaptive-grippers#Two-Finger-Gripper), using the [`rdk:component:gripper` API](https://docs.viam.com/appendix/apis/components/gripper/).

> [!NOTE]
> Before configuring your gripper, you must [create a machine](https://docs.viam.com/cloud/machines/#add-a-new-machine).

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in the [Viam app](https://app.viam.com/).
[Add gripper / robotiq:2f-grippers to your machine](https://docs.viam.com/configure/#components).

## Configure your 2f-grippers gripper

On the new component panel, copy and paste the following attribute template into your gripper's attributes field:

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

### `viam:robotiq:2f-grippers`
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

### Next Steps
- To test your gripper, expand the **TEST** section of its configuration pane or go to the [**CONTROL** tab](https://docs.viam.com/fleet/control/).
- To write code against your gripper, use one of the [available SDKs](https://docs.viam.com/sdks/).
- To view examples using a gripper component, explore [these tutorials](https://docs.viam.com/tutorials/).
