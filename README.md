# teleop2servo
ROS 2 teleoperation node for controlling a robot using MoveIt Servo with various input devices.

## What it does
* Sends JointJog and TwistStamped commands from keyboard input
* Supports joint and ~~Cartesian (base) control~~ (underconstruction)
* Clean shutdown with Ctrl+C

## Run
### For keyboard input:

```bash
ros2 run teleop2servo keyboard_teleop_node 
```
With special params:
```bash
ros2 run teleop2servo keyboard_teleop_node --ros-args --params-file "path to /teleop2servo/config/teleop2servo.yaml"
```

> ⚠️ **IMPORTANT:**
> Make sure MoveIt Servo is running and the terminal window has focus.

## License
Apache 2.0
