# teleop2servo
ROS 2 teleoperation node for controlling a robot using MoveIt Servo with various input devices.

## What it does
* Sends JointJog and TwistStamped commands from keyboard input
* Supports joint and cartesian (base frame and tool frame) control
* Clean shutdown with Ctrl+C

## Installation
Clone the package into your workspace, install its dependencies (declared in `package.xml`) with `rosdep`, then build it:

```bash
cd ~/ceai_ws/src
git clone https://github.com/AGH-CEAI/teleop2servo.git
cd ~/ceai_ws
sudo apt update
rosdep update
rosdep install --from-paths src/teleop2servo --ignore-src -y
colcon build --symlink-install --packages-select teleop2servo
```

## Run
> ⚠️ **IMPORTANT:**
> Make sure MoveIt Servo is running and the terminal window has focus.
>
> For more details, see: https://github.com/AGH-CEAI/aegis_ros/tree/humble-devel/aegis_moveit_config
### For keyboard input:

```bash
ros2 launch teleop2servo teleop.launch.py teleop_device:=keyboard
```
or directly:
```bash
ros2 run teleop2servo teleop_keyboard_node --ros-args --params-file "path to /teleop2servo/config/teleop_keyboard.yaml"
```

### For gamepad input:

```bash
ros2 launch teleop2servo teleop.launch.py teleop_device:=gamepad
```

## License
Apache 2.0
