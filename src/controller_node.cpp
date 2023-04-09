#include <furuta_pendulum/controller_node.hpp>

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace furuta_pendulum
{

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
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

void ControllerNode::StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
{
  double u = 0.0;
  if (fabs(msg->position[1] - M_PI) < 0.7) {
    u = LqrControl(msg);
  } else {
    u = SwingupControl(msg);
  }

  std_msgs::msg::Float64 torque_cmd_msg;
  torque_cmd_msg.data = u;
  torque_cmd_pub_->publish(torque_cmd_msg);
}

double ControllerNode::LqrControl(sensor_msgs::msg::JointState::SharedPtr current_state)
{
  Eigen::Vector4d state_vector;
  state_vector(0) = current_state->position[0];
  state_vector(1) = current_state->position[1] - M_PI;
  state_vector(2) = current_state->velocity[0];
  state_vector(3) = current_state->velocity[1];

  return -K.dot(state_vector);
}

double ControllerNode::SwingupControl(sensor_msgs::msg::JointState::SharedPtr current_state)
{
  // based on http://bulletin.pan.pl/(52-3)153.pdf
  double dtheta2 = current_state->velocity[1];
  double theta2 = current_state->position[1];
  double E = 0.5 * m2 * pow(l2, 2.0) * pow(dtheta2, 2.0) + m2 * g * l2 * cos(theta2);
  double E0 = m2 * g * l2;

  double x = (E - E0) * dtheta2 * cos(theta2);
  if (x > 0) {
    return -u_max;
  } else {
    return u_max;
  }
}

}  // namespace furuta_pendulum

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum::ControllerNode)
