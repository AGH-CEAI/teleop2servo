# teleop2servo
ROS 2 teleoperation node for controlling a robot using MoveIt Servo with various input devices.

## What it does
* Sends JointJog and TwistStamped commands from keyboard input
* Supports joint and cartesian (base frame and tool frame) control
* Clean shutdown with Ctrl+C

## Run
> ⚠️ **IMPORTANT:**
> Make sure MoveIt Servo is running and the terminal window has focus.
>
> For more details, see: https://github.com/AGH-CEAI/aegis_ros/tree/humble-devel/aegis_moveit_config
### For keyboard input:

```bash
ros2 run teleop2servo keyboard_teleop_node
```
With special params:
```bash
ros2 run teleop2servo keyboard_teleop_node --ros-args --params-file "path to /teleop2servo/config/teleop2servo.yaml"
```

## License
Apache 2.0
