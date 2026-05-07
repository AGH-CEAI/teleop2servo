#include <string>

#include <rclcpp/rclcpp.hpp>

#include "teleop2servo/gamepad_teleop_node.hpp"
#include "teleop2servo/utils.hpp"

namespace teleop2servo{

GamepadTeleopNode::GamepadTeleopNode(const rclcpp::NodeOptions& options)
    : Node("gamepad_teleop", options)
{
    // RCLCPP_INFO(this->get_logger(), "##### Joy to servo node run successfully ######");
    load_parameters();
}

GamepadTeleopNode::~GamepadTeleopNode() = default;

void GamepadTeleopNode::load_parameters()
{
  this->get_parameter_or("publish_hz", publish_hz_, 250);
  this->get_parameter_or("step_publish_ticks", step_publish_ticks_, 25);

  this->get_parameter_or("twist_topic", twist_topic_, std::string("/servo_node/delta_twist_cmds"));
  this->get_parameter_or("joint_topic", joint_topic_, std::string("/servo_node/delta_joint_cmds"));
  this->get_parameter_or("queue_size", queue_size_, 10);

  this->get_parameter_or("base_frame_id", base_frame_id_, std::string("base_link"));
  this->get_parameter_or("ee_frame_id", ee_frame_id_, std::string("tool0"));

  this->get_parameter_or("joint_names", joint_names_, std::vector<std::string>{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  });

  this->get_parameter_or("joint_vel_step", joint_vel_step_, 0.3);
  this->get_parameter_or("joint_vel_cont_max", joint_vel_cont_max_, 1.0);

  this->get_parameter_or("twist_lin_step", twist_lin_step_, 0.3);
  this->get_parameter_or("twist_lin_cont_max", twist_lin_cont_max_, 1.0);

  this->get_parameter_or("twist_rot_step", twist_rot_step_, 0.3);
  this->get_parameter_or("twist_rot_cont_max", twist_rot_cont_max_, 1.0);
}

void GamepadTeleopNode::print_gamepad_layout()
{
    RCLCPP_INFO(get_logger(), COLOR_BOLD "\n\n================ TELEOP GAMEPAD =================" COLOR_RESET);
    RCLCPP_INFO(get_logger(), R"(

                                [ BACK ]     [ START ]
                    [ LB ]                               [ RB ]
                    [ LT ]                               [ RT ]
                        .---------------------------------.
                    .'                                   '.
                    /      LEFT STICK       RIGHT STICK     \
                    /        (LX / LY)        (RX / RY)       \
                    |                                          |
                    |      D-PAD                   )"
                    COLOR_YELLOW "Y" COLOR_RESET R"(           |
                    |    [↑] [↓]               )"
                    COLOR_CYAN "X" COLOR_RESET R"(       )"
                    COLOR_RED "B" COLOR_RESET R"(       |
                    |    [←] [→]                   )"
                    COLOR_GREEN "A" COLOR_RESET R"(           |
                    \                                         /
                    '.                                     .'
                    '-----------------------------------'
    )");

    RCLCPP_INFO(get_logger(), "---------------------------");
    RCLCPP_INFO(get_logger(),
        "Mode: " COLOR_CYAN "MODE" COLOR_RESET
        " | Speed: " COLOR_YELLOW "SPEED" COLOR_RESET
    );
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
