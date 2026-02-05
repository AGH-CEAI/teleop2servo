# teleop2servo
ROS 2 keyboard teleoperation node for controlling a robot using MoveIt Servo.

## What it does
* Sends JointJog and TwistStamped commands from keyboard input
* Supports joint and ~~Cartesian (base) control~~ (underconstruction)
* Clean shutdown with Ctrl+C

## Run
```bash
ros2 run teleop2servo teleop_keyboard_node --ros-args --params-file "path to /teleop2servo/config/teleop_keyboard.yaml"
```
Make sure MoveIt Servo is running and the terminal window has focus.

## License
Apache 2.0
