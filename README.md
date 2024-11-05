# [`robotiq` module](https://github.com/viam-modules/robotiq)

This [robotiq module](https://app.viam.com/module/viam/robotiq) implements a robotiq [2f-grippers gripper](<LINK TO HARDWARE>), used for <DESCRIPTION> using the [`rdk:component:gripper` API](https://docs.viam.com/appendix/apis/components/gripper/).

> [!NOTE]
> Before configuring your gripper, you must [create a machine](https://docs.viam.com/cloud/machines/#add-a-new-machine).

## Configure your 2f-grippers gripper

Navigate to the [**CONFIGURE** tab](https://docs.viam.com/configure/) of your [machine](https://docs.viam.com/fleet/machines/) in the [Viam app](https://app.viam.com/).
[Add gripper / robotiq:2f-grippers to your machine](https://docs.viam.com/configure/#components).

On the new component panel, copy and paste the following attribute template into your gripper's attributes field:

```json
{
  <ATTRIBUTES>
}
```

### Attributes

The following attributes are available for `viam:robotiq:2f-grippers` grippers:

<EXAMPLE !!>
| Attribute | Type | Required? | Description |
| --------- | ---- | --------- | ----------  |
| `i2c_bus` | string | **Required** | The index of the I<sup>2</sup>C bus on the board that the gripper is wired to. |
| `i2c_address` | string | Optional | Default: `0x77`. The [I<sup>2</sup>C device address](https://learn.adafruit.com/i2c-addresses/overview) of the gripper. |

## Example configuration

### `viam:robotiq:2f-grippers`
```json
  {
      "name": "<your-robotiq-2f-grippers-gripper-name>",
      "model": "viam:robotiq:2f-grippers",
      "type": "gripper",
      "namespace": "rdk",
      "attributes": {
      },
      "depends_on": []
  }
```

### Next Steps
- To test your gripper, expand the **TEST** section of its configuration pane or go to the [**CONTROL** tab](https://docs.viam.com/fleet/control/).
- To write code against your gripper, use one of the [available SDKs](https://docs.viam.com/sdks/).
- To view examples using a gripper component, explore [these tutorials](https://docs.viam.com/tutorials/).