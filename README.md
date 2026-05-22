# teleop2servo

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)
[![prek](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/j178/prek/master/docs/assets/badge-v0.json)](https://github.com/j178/prek)

ROS 2 teleoperation node for controlling a robot using MoveIt Servo with various input devices.

---

## What it does
* Sends JointJog and TwistStamped commands from input device
* Supports joint and cartesian (base frame and tool frame) control
* Clean shutdown with Ctrl+C

## Run
> [!IMPORTANT]
> Make sure MoveIt Servo is running and the terminal window has focus.
>
> For more details, see: https://github.com/AGH-CEAI/aegis_ros/tree/humble-devel/aegis_moveit_config

> [!NOTE]
> Install dependencies:
> `rosdep install --from-paths src --ignore-src -r -y`

### For gamepad input
> [!NOTE]
> Make sure the ROS 2 joy package is installed.

```bash
ros2 launch teleop2servo teleop.launch.py
```
You can configure `GamepadTeleopNode` parameters in `config/gamepad_config.yaml`.

---
## Development notes

This project uses various tools for aiding the quality of the source code. Currently most of them are executed by the `pre-commit`. As a faster alternative it is suggested to use `prek`. Please make sure to enable its hooks:

```bash
# In case of pre-commit
pre-commit install
# In case of prek
prek install
```

---
## License
This repository is licensed under the Apache 2.0, see LICENSE for details.
