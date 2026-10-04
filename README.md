# teleop2servo

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)
[![prek](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/j178/prek/master/docs/assets/badge-v0.json)](https://github.com/j178/prek)

<p align="center">
    <img src="docs/banner.svg" alt="teleop2servo" width="640"/>
    <img src="docs/devices.svg" alt="Gamepad and keyboard" width="640"/>
</p>


ROS 2 teleoperation node for controlling a robot using MoveIt Servo with various input devices.

`teleop2servo` turns user input into `control_msgs/JointJog` (joint control) and `geometry_msgs/TwistStamped` (Cartesian control) commands for MoveIt Servo. It can optionally switch the `ros2_control` controllers and start Servo for you when you begin teleoperation, and restore them when you finish.

## Features

- **Two input devices:** a terminal keyboard (no extra drivers) and a gamepad (via the [`joy`](https://index.ros.org/p/joy/) package). Ready to extend and add different input devices.
- **Three control modes:** `JOINT` (single joints), `BASE` (Cartesian, base frame) and `TOOL` (Cartesian, end-effector frame).
- **Speed modes:** `STEP` (one fixed step per press) and continuous `CONT` at 5 / 10 / 25 / 50 / 75 / 100 % of the configured maximum.
- **Safe by default:** the device starts **blocked** and has to be unblocked explicitly; blocking it again stops the motion immediately.
- **Optional Servo activation:** on unblock, switch controllers and call `start_servo`; on block or Ctrl+C, stop Servo and restore the original controllers.
- **Terminal UI:** live status (blocked/ready, mode, speed) and the controls for the current mode.

## Requirements

- ROS 2 **Humble**
- A running **MoveIt Servo** node configured for your robot
- For the gamepad: the [`joy`](https://index.ros.org/p/joy/) package and a connected controller
- For automatic Servo activation: a robot driven by `ros2_control` (`controller_manager`)

## Installation

```bash
cd ~/ceai_ws/src
git clone https://github.com/AGH-CEAI/teleop2servo.git
cd ~/ceai_ws
rosdep install --from-paths src/teleop2servo --ignore-src -y
colcon build --symlink-install --packages-select teleop2servo
source install/setup.bash
```

## Quick start

1. Start your robot and MoveIt Servo.
2. Launch teleop with the device of your choice:

   ```bash
   # keyboard (default) – keep the terminal focused while driving
   ros2 launch teleop2servo teleop.launch.py teleop_device:=keyboard

   # gamepad – starts joy_node and the teleop node in one container
   ros2 launch teleop2servo teleop.launch.py teleop_device:=gamepad
   ```

3. Unblock the device (see [Controls](#controls)) and drive.
4. Press **Ctrl+C** to exit.

To run a node directly with your own parameters:

```bash
ros2 run teleop2servo teleop_keyboard_node --ros-args --params-file ~/ceai_ws/src/teleop2servo/teleop2servo/config/teleop_keyboard.yaml
ros2 run teleop2servo teleop_gamepad_node_exec --ros-args --params-file ~/ceai_ws/src/teleop2servo/teleop2servo/config/teleop_gamepad.yaml  # needs joy_node
```

## Controls

The current controls are always printed in the terminal; the tables below are a reference.

## MoveIt Servo activation

With `servo_activation.enabled: true` the node manages Servo for you:

| Event                   | Action                                                                                          |
| ----------------------- | ----------------------------------------------------------------------------------------------- |
| Device unblocked        | switch controllers (`deactivate_controllers` → `activate_controllers`), then call `start_servo` |
| Device blocked / Ctrl+C | call `stop_servo`, then switch the controllers back                                             |

If a service is unavailable or fails, the device stays blocked and the error is shown in the terminal. Set `enabled: false` if Servo and the controllers are managed elsewhere. To skip a single step, set its service name to `""`.

Example for a UR robot (Servo publishing to `/forward_position_controller/commands`):

```yaml
servo_activation:
  enabled: true
  switch_controller_service: "/controller_manager/switch_controller"
  activate_controllers: ["forward_position_controller"]
  deactivate_controllers: ["scaled_joint_trajectory_controller"]
  start_servo_service: "/servo_node/start_servo"
  stop_servo_service: "/servo_node/stop_servo"
```

Use `ros2 control list_controllers` to see which controllers your robot provides and which one Servo's `command_out_topic` points to.

## ROS interface

| Type                      | Name (default)                                      | Message / service                              |
| ------------------------- | --------------------------------------------------- | ---------------------------------------------- |
| Publisher                 | `/servo_node/delta_joint_cmds`                      | `control_msgs/msg/JointJog`                    |
| Publisher                 | `/servo_node/delta_twist_cmds`                      | `geometry_msgs/msg/TwistStamped`               |
| Subscriber (gamepad)      | `/joy`                                              | `sensor_msgs/msg/Joy`                          |
| Service client (optional) | `/controller_manager/switch_controller`             | `controller_manager_msgs/srv/SwitchController` |
| Service client (optional) | `/servo_node/start_servo`, `/servo_node/stop_servo` | `std_srvs/srv/Trigger`                         |

## Parameters

Default configurations: [`config/teleop_keyboard.yaml`](teleop2servo/config/teleop_keyboard.yaml) and [`config/teleop_gamepad.yaml`](teleop2servo/config/teleop_gamepad.yaml).

### Common

| Parameter                               | Default                    | Description                                             |
| --------------------------------------- | -------------------------- | ------------------------------------------------------- |
| `servo_publish_hz`                      | `250.0`                    | Command publishing rate                                 |
| `servo_ticks_per_policy_step`           | `10`                       | Number of publish ticks a single `STEP` lasts           |
| `twist_topic` / `joint_topic`           | `/servo_node/delta_*_cmds` | Servo input topics                                      |
| `queue_size`                            | `10`                       | Publisher queue size                                    |
| `base_frame_id` / `ee_frame_id`         | `base_link` / `tool0`      | Frames for `BASE` and `TOOL` modes                      |
| `joint_names`                           | UR joint names             | Joints commanded in `JOINT` mode                        |
| `joint_vel_step` / `joint_vel_cont_max` | `0.1` / `0.8`              | Joint command in `STEP` / at `CONT 100%`                |
| `twist_lin_step` / `twist_lin_cont_max` | `0.1` / `0.5`              | Linear command in `STEP` / at `CONT 100%`               |
| `twist_ang_step` / `twist_ang_cont_max` | `0.1` / `0.8`              | Angular command in `STEP` / at `CONT 100%`              |
| `servo_activation.*`                    | `enabled: true`            | See [MoveIt Servo activation](#moveit-servo-activation) |

Velocity values are normalized commands, scaled by Servo's `scale.*` parameters.

### Keyboard

| Parameter               | Default | Description                                         |
| ----------------------- | ------- | --------------------------------------------------- |
| `reading_keyboard_hz`   | `250.0` | Keyboard polling rate                               |
| `key_initial_timeout_s` | `0.55`  | Must be greater than the system key-repeat delay    |
| `key_repeat_timeout_s`  | `0.1`   | Must be greater than the system key-repeat interval |

### Gamepad

| Parameter   | Default | Description                 |
| ----------- | ------- | --------------------------- |
| `joy_topic` | `/joy`  | Input topic from `joy_node` |

## Keyboard notes

- The keyboard node reads the controlling terminal (`/dev/tty`), so the terminal must have focus.
- Terminals report no key-release events. A key counts as released when no auto-repeat arrives within the timeouts above, so motion in `CONT` stops about 0.5 s after a short tap.
- Only one key is handled at a time.
- Caps Lock must be off. While it is on, input is paused and the robot stops; it resumes as soon as Caps Lock is turned off. The state is read from `/sys/class/leds`; if it can't be read (e.g. over SSH), the node asks for confirmation at startup.
- If motion stutters while a key is held, check your key-repeat settings (on GNOME: `gsettings get org.gnome.desktop.peripherals.keyboard delay` and `repeat-interval`) and adjust the timeouts.

## Development notes

This project uses various tools for aiding the quality of the source code. Currently most of them are executed by the `pre-commit`. As a faster alternative it is suggested to use `prek`. Please make sure to enable its hooks:

```bash
# In case of pre-commit
pre-commit install
# In case of prek
prek install
```

## License

This repository is licensed under the Apache 2.0, see LICENSE for details.
