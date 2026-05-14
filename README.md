# teleop2servo
ROS 2 teleoperation node for controlling a robot using MoveIt Servo with various input devices.

## What it does
* Sends JointJog and TwistStamped commands from keyboard input
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
ros2 launch teleop2servo gamepad_teleop.launch.py
```
You can configure `GamepadTeleopNode` parameters in `config/gamepad_config.yaml`.

## License
Apache 2.0
