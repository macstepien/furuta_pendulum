#include <algorithm>
#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace furuta_pendulum
{
class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode(const rclcpp::NodeOptions & options)
  : Node("furuta_pendulum_simulation_node", options)
  {
    // from https://www.hindawi.com/journals/jcse/2011/528341/
    m1 = 0.3;
    m2 = 0.075;

    l1 = 0.15;
    l2 = 0.148;

    L1 = 0.278;
    L2 = 0.3;

    double J1 = 0.0248;
    double J2 = 0.00386;

    J2_hat = J2 + m2 * l2 * l2;
    J0_hat = J1 + m1 * l1 * l1 + m2 * L1 * L1;

    b1 = 0.0001;
    b2 = 0.00028;

    L = 0.005;
    R = 7.8;
    Km = 0.09;

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    simulation_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt), std::bind(&SimulationNode::Simulate, this));

    torque_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "torque", 10, std::bind(&SimulationNode::SetTorqueCb, this, std::placeholders::_1));
    disturbance_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "disturbance", 10, std::bind(&SimulationNode::SetDisturbanceCb, this, std::placeholders::_1));
  }

private:
  double dt = 0.001;
  static constexpr double g = 9.80665;

  // Mechanical System
  double J0_hat, J2_hat;

  double m1, m2;
  double l1, l2;
  double L1, L2;
  double b1, b2;

  double theta1 = 0.0, theta2 = M_PI;
  double dtheta1 = 0.0, dtheta2 = 0.0;
  double ddtheta1 = 0.0, ddtheta2 = 0.0;

  double tau1 = 0.0, tau2 = 0.0;

  double max_torque_ = 1.47;
  double max_velocity_ = 10.0;

  // Motor
  double L, R, Km;

  //  ROS Stuff
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr disturbance_sub_;

  void SetTorqueCb(std_msgs::msg::Float64::SharedPtr msg) { SetTorque(msg->data); }
  void SetDisturbanceCb(std_msgs::msg::Float64::SharedPtr msg) { SetDisturbance(msg->data); }

  void Simulate()
  {
    Eigen::Matrix2d matrix_1;
    matrix_1(0, 0) = J0_hat + J2_hat * pow(sin(theta2), 2);
    matrix_1(0, 1) = m2 * L1 * l2 * cos(theta2);
    matrix_1(1, 0) = matrix_1(0, 1);
    matrix_1(1, 1) = J2_hat;

    Eigen::Matrix2d matrix_2;
    matrix_2(0, 0) = b1 + 0.5 * dtheta2 * J2_hat * sin(2.0 * theta2);
    matrix_2(0, 1) =
      0.5 * dtheta2 * J2_hat * sin(2.0 * theta2) - m2 * L1 * l2 * sin(theta2) * dtheta2;
    matrix_2(1, 0) = -0.5 * dtheta1 * J2_hat * sin(2.0 * theta2);
    matrix_2(1, 1) = b2;

    Eigen::Vector2d vector_1;
    vector_1(0) = dtheta1;
    vector_1(1) = dtheta2;

    Eigen::Vector2d vector_2;
    vector_2(0) = 0.0;
    vector_2(1) = g * m2 * l2 * sin(theta2);

    Eigen::Vector2d vector_3;
    vector_3(0) = tau1;
    vector_3(1) = tau2;

    Eigen::Vector2d vector_4 = matrix_1.inverse() * (matrix_2 * vector_1 - vector_2 + vector_3);

    ddtheta1 = vector_4(0);
    ddtheta2 = vector_4(1);

    dtheta1 = std::clamp(dtheta1 + ddtheta1 * dt, -max_velocity_, max_velocity_);
    dtheta2 = std::clamp(dtheta2 + ddtheta2 * dt, -max_velocity_, max_velocity_);

    theta1 += dtheta1 * dt;
    theta2 += dtheta2 * dt;

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();

    joint_state_msg.name.push_back("joint1");
    joint_state_msg.position.push_back(theta1);
    joint_state_msg.velocity.push_back(dtheta1);

    joint_state_msg.name.push_back("joint2");
    joint_state_msg.position.push_back(theta2);
    joint_state_msg.velocity.push_back(dtheta2);

    joint_state_pub_->publish(joint_state_msg);
    
    // treat disturbance as impulse and set it back to 0.
    tau2 = 0.0;
  }

  void SetTorque(double torque) { tau1 = std::clamp(torque, -max_torque_, max_torque_); }
  void SetDisturbance(double disturbance)
  {
    tau2 = std::clamp(disturbance, -max_torque_, max_torque_);
  }
};
}  // namespace furuta_pendulum

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum::SimulationNode)
