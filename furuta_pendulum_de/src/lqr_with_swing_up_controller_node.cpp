#include <furuta_pendulum_de/lqr_with_swing_up_controller_node.hpp>

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace furuta_pendulum_de
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

LqrWithSwingUpControllerNode::LqrWithSwingUpControllerNode(const rclcpp::NodeOptions & options)
: Node("furuta_pendulum_controller_node", options)
{
  this->declare_parameter("K", rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter("m2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("m1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("l2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("l1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("L2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("L1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("J1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("J2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("u_max", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("alpha", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("torque_multiplier", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("lqr_transition_angle", rclcpp::PARAMETER_DOUBLE);

  try {
    K_ = Eigen::Vector4d(this->get_parameter("K").as_double_array().data());
    m2_ = this->get_parameter("m2").as_double();
    m1_ = this->get_parameter("m1").as_double();
    l2_ = this->get_parameter("l2").as_double();
    l1_ = this->get_parameter("l1").as_double();
    L2_ = this->get_parameter("L2").as_double();
    L1_ = this->get_parameter("L1").as_double();
    J2_ = this->get_parameter("J2").as_double();
    J1_ = this->get_parameter("J1").as_double();
    alpha_ = this->get_parameter("alpha").as_double();
    torque_multiplier_ = this->get_parameter("torque_multiplier").as_double();

    u_max_ = this->get_parameter("u_max").as_double();
    lqr_transition_angle_ = this->get_parameter("lqr_transition_angle").as_double();
  } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
    throw e;
  }

  state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&LqrWithSwingUpControllerNode::StateCb, this, std::placeholders::_1));
  torque_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_control", 10);
}

void LqrWithSwingUpControllerNode::StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
{
  dtheta2_filtered_ = alpha_ * msg->velocity[1] + (1.0 - alpha_) * dtheta2_filtered_;
  dtheta1_filtered_ = alpha_ * msg->velocity[0] + (1.0 - alpha_) * dtheta1_filtered_;

  double u = 0.0;
  if (fabs(limit_minus_pi_pi(msg->position[1] - M_PI)) < lqr_transition_angle_) {
    u = torque_multiplier_ * LqrControl(msg);
  } else {
    u = SwingUpControl(msg);
  }

  std_msgs::msg::Float64MultiArray torque_cmd_msg;
  torque_cmd_msg.data.push_back(u);
  // torque_cmd_msg.data.push_back(0.0);
  torque_cmd_pub_->publish(torque_cmd_msg);
}

double LqrWithSwingUpControllerNode::LqrControl(
  sensor_msgs::msg::JointState::SharedPtr current_state)
{
  Eigen::Vector4d state_vector;
  state_vector(0) = current_state->position[0];
  state_vector(1) = limit_minus_pi_pi(current_state->position[1] - M_PI);
  state_vector(2) = dtheta1_filtered_;
  state_vector(3) = dtheta2_filtered_;

  return -K_.dot(state_vector);
}

double LqrWithSwingUpControllerNode::SwingUpControl(
  sensor_msgs::msg::JointState::SharedPtr current_state)
{
  // based on http://bulletin.pan.pl/(52-3)153.pdf
  const double dtheta2 = current_state->velocity[1];
  const double theta2 = current_state->position[1];

  const double dtheta1 = current_state->velocity[0];
  const double theta1 = current_state->position[0];

  // const double E = 0.5 * m2_ * pow(l2_, 2) * pow(dtheta2, 2) + m2_ * g_ * l2_ * cos(theta2);

  // double Ek1 = 0.5 * pow(dtheta1, 2) * (m1_ * pow(l1_, 2) + J1_);
  // double Ep2 = m2_ * g_ * l2_ * (1 - cos(theta2));
  // double Ek2 = 0.5 * pow(dtheta1, 2) *
  //                (m2_ * pow(L2_, 2) + (m2_ * pow(l2_, 2) + J2_) * pow(sin(theta2), 2) +
  //                 J2_ * pow(cos(theta2), 2)) +
  //              0.5 * pow(dtheta2, 2) * (J2_ + m2_ * pow(l2_, 2)) +
  //              m2_ * L1_ * l2_ * cos(theta2) * dtheta1 * dtheta2;

  // // double E = Ek1 + Ep2 + Ek2;
  // double E = Ep2 + Ek2;

  // const double E0 = m2_ * g_ * l2_;

  // const double x = (E0 - E) * dtheta2 * (1 - cos(theta2));
  // const double x = -dtheta2 * (cos(theta2));
  // if (x > 0.0) {
  //   return -u_max_ * fabs(E - E0);
  // } else {
  //   return u_max_ * fabs(E - E0);
  // }
  // if (x > 0.0) {
  //   return -u_max_;
  // } else {
  //   return u_max_;
  // }
  // if (fabs(dtheta2) > 0.02) {
  if (dtheta2_filtered_ * cos(theta2) > 0) {
    return -u_max_;
  } else {
    return u_max_;
  }
  // if (swing_up_started_) {
  //   double K = u_max_;
  //   return K * pow(cos(theta2), 4) * dtheta2_filtered_ *
  //          (9.81 * (1 - cos(theta2)) - 0.0075 * pow(dtheta2_filtered_, 2));
  // } else {
  //   swing_up_started_ = true;
  //   return 0.1;
  // }
}

}  // namespace furuta_pendulum_de

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_de::LqrWithSwingUpControllerNode)
