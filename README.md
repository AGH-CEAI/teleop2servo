# teleop2servo
ROS 2 teleoperation node for controlling a robot using MoveIt Servo with various input devices.

## What it does
* Sends JointJog and TwistStamped commands from keyboard input
* Supports joint and ~~Cartesian (base) control~~ (underconstruction)
* Clean shutdown with Ctrl+C

## Run
### For gamepad input
> **Note:**
> Make sure the ROS 2 joy package is installed, e.g.:
> `sudo apt install ros-$ROS_DISTRO-joy`

```bash
ros2 launch teleop2servo gamepad_teleop.launch.py
```
> ⚠️ **IMPORTANT:**
> Make sure MoveIt Servo is running and the terminal window has focus.

## License
Apache 2.0
