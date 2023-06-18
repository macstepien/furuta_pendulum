#include <furuta_pendulum_de/de_simulation_node.hpp>

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace furuta_pendulum_de
{

DeSimulationNode::DeSimulationNode(const rclcpp::NodeOptions & options)
: Node("furuta_pendulum_simulation_node", options)
{
  this->declare_parameter("initial_conditions.theta1", 0.0);
  this->declare_parameter("initial_conditions.theta2", 0.0);
  this->declare_parameter("initial_conditions.dtheta1", 0.0);
  this->declare_parameter("initial_conditions.dtheta2", 0.0);
  this->declare_parameter("initial_conditions.tau1", 0.0);
  this->declare_parameter("initial_conditions.tau2", 0.0);
  theta1_ = this->get_parameter("initial_conditions.theta1").as_double();
  theta2_ = this->get_parameter("initial_conditions.theta2").as_double();
  dtheta1_ = this->get_parameter("initial_conditions.dtheta1").as_double();
  dtheta2_ = this->get_parameter("initial_conditions.dtheta2").as_double();
  tau1_ = this->get_parameter("initial_conditions.tau1").as_double();
  tau2_ = this->get_parameter("initial_conditions.tau2").as_double();

  this->declare_parameter("simulation_dt", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("max_torque", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("max_velocity", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("m1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("m2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("l1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("l2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("L1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("L2", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("b1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("b2", rclcpp::PARAMETER_DOUBLE);

  this->declare_parameter("J1", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("J2", rclcpp::PARAMETER_DOUBLE);
  try {
    dt_ = this->get_parameter("simulation_dt").as_double();
    max_torque_ = this->get_parameter("max_torque").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    m1_ = this->get_parameter("m1").as_double();
    m2_ = this->get_parameter("m2").as_double();
    l1_ = this->get_parameter("l1").as_double();
    l2_ = this->get_parameter("l2").as_double();
    L1_ = this->get_parameter("L1").as_double();
    L2_ = this->get_parameter("L2").as_double();
    b1_ = this->get_parameter("b1").as_double();
    b2_ = this->get_parameter("b2").as_double();

    double J1 = this->get_parameter("J1").as_double();
    double J2 = this->get_parameter("J2").as_double();
    J2_hat_ = J2 + m2_ * l2_ * l2_;
    J0_hat_ = J1 + m1_ * l1_ * l1_ + m2_ * L1_ * L1_;

  } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
    throw e;
  }

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  simulation_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_), std::bind(&DeSimulationNode::Simulate, this));

  effort_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "effort_control", 10, std::bind(&DeSimulationNode::SetEffortCb, this, std::placeholders::_1));

  rviz_disturbance_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 10,
    std::bind(&DeSimulationNode::RvizDisturbanceCb, this, std::placeholders::_1));
}

Eigen::Vector4d DeSimulationNode::F(Eigen::Vector4d y)
{
  // based on https://www.hindawi.com/journals/jcse/2011/528341/

  // theta3 = dtheta1
  // theta4 = dtheta2
  // dtheta3 = ddtheta1
  // dtheta4 = ddtheta2

  // double theta1 = y(0);
  double theta2 = y(1);
  double theta3 = y(2);
  double theta4 = y(3);

  Eigen::Matrix<double, 5, 1> vec11;
  vec11(0) = -J2_hat_ * b1_;
  vec11(1) = m2_ * L1_ * l2_ * cos(theta2) * b2_;
  vec11(2) = -pow(J2_hat_, 2) * sin(2.0 * theta2);
  vec11(3) = -0.5 * J2_hat_ * m2_ * L1_ * l2_ * cos(theta2) * sin(2.0 * theta2);
  vec11(4) = J2_hat_ * m2_ * L1_ * l2_ * sin(theta2);

  Eigen::Matrix<double, 5, 1> vec21;
  vec21(0) = m2_ * L1_ * l2_ * cos(theta2) * b1_;
  vec21(1) = -b2_ * (J0_hat_ + J2_hat_ * pow(sin(theta2), 2));
  vec21(2) = m2_ * L1_ * l2_ * J2_hat_ * cos(theta2) * sin(2.0 * theta2);
  vec21(3) = -0.5 * sin(2.0 * theta2) * (J0_hat_ * J2_hat_ + pow(J2_hat_ * sin(theta2), 2));
  vec21(4) = -0.5 * pow(m2_ * L1_ * l2_, 2) * sin(2.0 * theta2);

  Eigen::Matrix<double, 5, 1> thetas_vec;
  thetas_vec(0) = theta3;
  thetas_vec(1) = theta4;
  thetas_vec(2) = theta3 * theta4;
  thetas_vec(3) = pow(theta3, 2);
  thetas_vec(4) = pow(theta4, 2);

  Eigen::Matrix<double, 3, 1> vec12;
  vec12(0) = J2_hat_;
  vec12(1) = -m2_ * L1_ * l2_ * cos(theta2);
  vec12(2) = 0.5 * pow(m2_ * l2_, 2) * L1_ * sin(2.0 * theta2);

  Eigen::Matrix<double, 3, 1> vec22;
  vec22(0) = -m2_ * L1_ * l2_ * cos(theta2);
  vec22(1) = J0_hat_ + J2_hat_ * pow(sin(theta2), 2);
  vec22(2) = -m2_ * l2_ * sin(theta2) * (J0_hat_ + J2_hat_ * pow(sin(theta2), 2));

  Eigen::Matrix<double, 3, 1> taus_g_vec;
  taus_g_vec(0) = tau1_;
  taus_g_vec(1) = tau2_;
  taus_g_vec(2) = g_;

  double denominator =
    J0_hat_ * J2_hat_ + pow(J2_hat_ * sin(theta2), 2) - pow(m2_ * L1_ * l2_ * cos(theta2), 2);

  auto numerator1 = (vec11.transpose() * thetas_vec + vec12.transpose() * taus_g_vec);
  auto numerator2 = (vec21.transpose() * thetas_vec + vec22.transpose() * taus_g_vec);

  Eigen::Vector4d dy;
  dy(0) = theta3;
  dy(1) = theta4;
  dy(2) = numerator1(0) / denominator;
  dy(3) = numerator2(0) / denominator;

  return dy;
}

Eigen::Vector4d DeSimulationNode::integrate_rk4(Eigen::Vector4d y_n)
{
  Eigen::Vector4d k1 = F(y_n);
  Eigen::Vector4d k2 = F(y_n + (dt_ / 2.0) * k1);
  Eigen::Vector4d k3 = F(y_n + (dt_ / 2.0) * k2);
  Eigen::Vector4d k4 = F(y_n + dt_ * k3);

  Eigen::Vector4d y_n1 = y_n + (k1 + 2.0 * k2 + 2.0 * k3 + k4) * dt_ / 6.0;
  return y_n1;
}

void DeSimulationNode::Simulate()
{
  Eigen::Vector4d y_n;
  y_n(0) = theta1_;
  y_n(1) = theta2_;
  y_n(2) = dtheta1_;
  y_n(3) = dtheta2_;

  Eigen::Vector4d y_n1 = integrate_rk4(y_n);
  theta1_ = y_n1(0);
  theta2_ = y_n1(1);
  dtheta1_ = y_n1(2);
  dtheta2_ = y_n1(3);

  current_time_ += dt_;

  PublishJointStates();

  // treat disturbance as impulse and set it back to 0.
  tau2_ = 0.0;
}

void DeSimulationNode::PublishJointStates()
{
  sensor_msgs::msg::JointState joint_state_msg;

  joint_state_msg.header.stamp.sec = current_time_;
  joint_state_msg.header.stamp.nanosec = (current_time_ - (int)current_time_) * 1000000000.0;

  joint_state_msg.name.push_back("joint1");
  joint_state_msg.position.push_back(theta1_);
  joint_state_msg.velocity.push_back(dtheta1_);

  joint_state_msg.name.push_back("joint2");
  joint_state_msg.position.push_back(theta2_);
  joint_state_msg.velocity.push_back(dtheta2_);

  joint_state_pub_->publish(joint_state_msg);
}

}  // namespace furuta_pendulum_de

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_de::DeSimulationNode)
