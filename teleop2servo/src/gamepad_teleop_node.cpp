#include <string>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/gamepad_teleop_node.hpp"
#include "teleop2servo/utils.hpp"

namespace teleop2servo{

GamepadTeleopNode::GamepadTeleopNode(const rclcpp::NodeOptions& options)
    : Node("gamepad_teleop", options)
{
    RCLCPP_INFO(this->get_logger(), "##### Joy to servo node run successfully ######");
    print_gamepad_layout();
}

GamepadTeleopNode::~GamepadTeleopNode() = default;

void GamepadTeleopNode::print_gamepad_layout()
{
//   RCLCPP_INFO(get_logger(), COLOR_BOLD "\n\n================ TELEOP GAMEPAD =================" COLOR_RESET);
//   RCLCPP_INFO(get_logger(),
//     "Mode: " COLOR_CYAN "%s" COLOR_RESET
//     " | Speed: " COLOR_YELLOW "%s" COLOR_RESET,
//     to_string(control_mode_).c_str(),
//     to_string(speed_mode_).c_str()
//   );

//   RCLCPP_INFO(get_logger(), R"(
//                          [ BACK ]     [ START ]

//               [ LB ]                               [ RB ]
//               [ LT ]                               [ RT ]

//                     .-------------------------------.
//                   .'                                 '.
//                  /      LEFT STICK       RIGHT STICK
//                 /        (LX / LY)        (RX / RY)
//                |                                           |
//                |      D-PAD                       Y         |
//                |    [↑] [↓]                   X       B     |
//                |    [←] [→]                       A         |
//                 \                                         /
//                  '.                                     .'
//                    '-----------------------------------'
// )");

//   RCLCPP_INFO(get_logger(), "---------------------------");

//   if (control_mode_ == ControlMode::JOINT)
//   {
//     RCLCPP_INFO(get_logger(), "JOINT MODE:");
//     RCLCPP_INFO(get_logger(), "  LEFT STICK vertical     -> J1 + / -");
//     RCLCPP_INFO(get_logger(), "  LEFT STICK horizontal   -> J2 + / -");
//     RCLCPP_INFO(get_logger(), "  RIGHT STICK vertical    -> J3 + / -");
//     RCLCPP_INFO(get_logger(), "  RIGHT STICK horizontal  -> J4 + / -");
//     RCLCPP_INFO(get_logger(), "  LT / RT                 -> J5 - / +");
//     RCLCPP_INFO(get_logger(), "  D-PAD left / right      -> J6 - / +");
//   }
//   else
//   {
//     RCLCPP_INFO(get_logger(), "CARTESIAN MODE:");
//     RCLCPP_INFO(get_logger(), "  LEFT STICK vertical     -> linear X");
//     RCLCPP_INFO(get_logger(), "  LEFT STICK horizontal   -> linear Y");
//     RCLCPP_INFO(get_logger(), "  LT / RT                 -> linear Z");
//     RCLCPP_INFO(get_logger(), "  RIGHT STICK vertical    -> angular Y");
//     RCLCPP_INFO(get_logger(), "  RIGHT STICK horizontal  -> angular Z");
//     RCLCPP_INFO(get_logger(), "  D-PAD left / right      -> angular X");
//   }

//   RCLCPP_INFO(get_logger(), "---------------------------");
//   RCLCPP_INFO(get_logger(), "START     : switch control mode [JOINT / BASE / TOOL]");
//   RCLCPP_INFO(get_logger(), "BACK      : switch speed [STEP / CONT 5%% / CONT 10%% / CONT 25%%]");
//   RCLCPP_INFO(get_logger(), "LB / L1   : deadman enable");
//   RCLCPP_INFO(get_logger(), "A         : stop motion");
//   RCLCPP_INFO(get_logger(), COLOR_RED "Ctrl+C to exit." COLOR_RESET);

  RCLCPP_INFO(get_logger(), COLOR_BOLD "\n\n================ TELEOP GAMEPAD =================" COLOR_RESET);
  RCLCPP_INFO(get_logger(),
    "Mode: " COLOR_CYAN "MODE" COLOR_RESET
    " | Speed: " COLOR_YELLOW "SPEED" COLOR_RESET
  );

  RCLCPP_INFO(get_logger(), R"(

                           [ BACK ]     [ START ]
               [ LB ]                                 [ RB ]
               [ LT ]                                 [ RT ]
                    .---------------------------------.
                  .'                                   '.
                 /      LEFT STICK       RIGHT STICK     \
                /        (LX / LY)        (RX / RY)       \
               |                                           |
               |      D-PAD                   Y            |
               |    [↑] [↓]               X       B        |
               |    [←] [→]                   A            |
                \                                         /
                 '.                                     .'
                   '-----------------------------------'
)");

  RCLCPP_INFO(get_logger(), "---------------------------");

  if (1==1)
  {
    RCLCPP_INFO(get_logger(), "JOINT MODE:");
    RCLCPP_INFO(get_logger(), "  LEFT STICK vertical     -> J1 + / -");
    RCLCPP_INFO(get_logger(), "  LEFT STICK horizontal   -> J2 + / -");
    RCLCPP_INFO(get_logger(), "  RIGHT STICK vertical    -> J3 + / -");
    RCLCPP_INFO(get_logger(), "  RIGHT STICK horizontal  -> J4 + / -");
    RCLCPP_INFO(get_logger(), "  LT / RT                 -> J5 - / +");
    RCLCPP_INFO(get_logger(), "  D-PAD left / right      -> J6 - / +");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "CARTESIAN MODE:");
    RCLCPP_INFO(get_logger(), "  LEFT STICK vertical     -> linear X");
    RCLCPP_INFO(get_logger(), "  LEFT STICK horizontal   -> linear Y");
    RCLCPP_INFO(get_logger(), "  LT / RT                 -> linear Z");
    RCLCPP_INFO(get_logger(), "  RIGHT STICK vertical    -> angular Y");
    RCLCPP_INFO(get_logger(), "  RIGHT STICK horizontal  -> angular Z");
    RCLCPP_INFO(get_logger(), "  D-PAD left / right      -> angular X");
  }

  RCLCPP_INFO(get_logger(), "---------------------------");
  RCLCPP_INFO(get_logger(), "START     : switch control mode [JOINT / BASE / TOOL]");
  RCLCPP_INFO(get_logger(), "BACK      : switch speed [STEP / CONT 5%% / CONT 10%% / CONT 25%%]");
  RCLCPP_INFO(get_logger(), "LB / L1   : deadman enable");
  RCLCPP_INFO(get_logger(), "A         : stop motion");
  RCLCPP_INFO(get_logger(), COLOR_RED "Ctrl+C to exit." COLOR_RESET);
}

} // namespace teleop2servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(teleop2servo::GamepadTeleopNode)
