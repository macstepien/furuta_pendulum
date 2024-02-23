#include <memory>
#include <filesystem>

#include <torch/script.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace furuta_pendulum_rl
{

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
    std::string model_path =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_rl")) /
      "trained_agents" / "furuta_pendulum_full.pt";

    try {
      module_ = torch::jit::load(model_path);
    } catch (const c10::Error & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error loading the model: " << e.what());
      throw e;
    }

    this->declare_parameter("torque_multiplier", rclcpp::PARAMETER_DOUBLE);
    try {
      torque_multiplier_ = this->get_parameter("torque_multiplier").as_double();
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
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

  double torque_multiplier_ = 1.0;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::vector<double> data = {sin(msg->position[0]),        cos(msg->position[0]),
                                sin(msg->position[1] - M_PI), cos(msg->position[1] - M_PI),
                                msg->velocity[0] / 22.0,      msg->velocity[1] / 50.0};

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::tensor(data));

    at::Tensor output = module_.forward(inputs).toTensor();
    double u = output.item<double>();

    std_msgs::msg::Float64MultiArray torque_cmd_msg;
    torque_cmd_msg.data.push_back(u * torque_multiplier_);
    torque_cmd_pub_->publish(torque_cmd_msg);
  }
};
}  // namespace furuta_pendulum_rl

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_rl::RlControllerNode)
