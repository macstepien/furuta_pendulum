#include <memory>
#include <filesystem>

#include <torch/script.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace furuta_pendulum_rl
{

// TODO: move it to common furuta_pendulum_utils
double limit_minus_pi_pi(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle > M_PI) {
    angle = 2 * M_PI - angle;
  } else if (angle < -M_PI) {
    angle = 2 * M_PI + angle;
  }
  return angle;
}

class RlControllerNode : public rclcpp::Node
{
public:
  RlControllerNode(const rclcpp::NodeOptions & options)
  : Node("furuta_pendulum_rl_controller_node", options)
  {
    this->declare_parameter("action_scale", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("dtheta0_alpha", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("dtheta1_alpha", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("action_alpha", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("agent_name", rclcpp::PARAMETER_STRING);
    try {
      action_scale_ = this->get_parameter("action_scale").as_double();
      dtheta0_alpha_ = this->get_parameter("dtheta0_alpha").as_double();
      dtheta1_alpha_ = this->get_parameter("dtheta1_alpha").as_double();
      action_alpha_ = this->get_parameter("action_alpha").as_double();
      agent_name_ = this->get_parameter("agent_name").as_string();
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
      throw e;
    }

    std::string model_path =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_rl")) /
      "trained_agents" / agent_name_;

    try {
      module_ = torch::jit::load(model_path);
    } catch (const c10::Error & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error loading the model: " << e.what());
      throw e;
    }

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&RlControllerNode::StateCb, this, std::placeholders::_1));
    torque_cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_control", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;
  torch::jit::script::Module module_;

  double action_scale_ = 1.0;

  double dtheta0_alpha_ = 1.0;
  double dtheta1_alpha_ = 1.0;

  double action_alpha_ = 1.0;

  double dtheta0_filtered_ = 0.0;
  double dtheta1_filtered_ = 0.0;

  double action_filtered_ = 0.0;

  std::string agent_name_;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    dtheta0_filtered_ =
      dtheta0_alpha_ * msg->velocity[0] + (1.0 - dtheta0_alpha_) * dtheta0_filtered_;
    dtheta1_filtered_ =
      dtheta1_alpha_ * msg->velocity[1] + (1.0 - dtheta1_alpha_) * dtheta1_filtered_;

    std::vector<double> observations = {
      std::sin(msg->position[0]),        std::cos(msg->position[0]),
      std::sin(msg->position[1] - M_PI), std::cos(msg->position[1] - M_PI),
      dtheta0_filtered_ / 22.0,          dtheta1_filtered_ / 50.0};

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::tensor(observations));

    at::Tensor output = module_.forward(inputs).toTensor();
    double action = output.item<double>();
    action_filtered_ = action_alpha_ * action + (1.0 - action_alpha_) * action_filtered_;

    std_msgs::msg::Float64MultiArray torque_cmd_msg;
    torque_cmd_msg.data.push_back(action_filtered_ * action_scale_);
    torque_cmd_pub_->publish(torque_cmd_msg);
  }
};
}  // namespace furuta_pendulum_rl

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_rl::RlControllerNode)
