#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace furuta_pendulum
{
class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(const rclcpp::NodeOptions & options)
  : Node("furuta_pendulum_controller_node", options)
  {
    std::vector<double> K_params;
    this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE_ARRAY);
    try {
      K_params = this->get_parameter("K").as_double_array();
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
      throw e;
    }

    K = Eigen::Vector4d(K_params.data());

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&ControllerNode::StateCb, this, std::placeholders::_1));
    torque_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("torque", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_cmd_pub_;

  Eigen::Vector4d K;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    Eigen::Vector4d state_vector;
    state_vector(0) = msg->position[0];
    state_vector(1) = msg->position[1] - M_PI;
    state_vector(2) = msg->velocity[0];
    state_vector(3) = msg->velocity[1];

    double u = -K.dot(state_vector);

    std_msgs::msg::Float64 torque_cmd_msg;
    torque_cmd_msg.data = u;
    torque_cmd_pub_->publish(torque_cmd_msg);
  }
};
}  // namespace furuta_pendulum

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum::ControllerNode)
