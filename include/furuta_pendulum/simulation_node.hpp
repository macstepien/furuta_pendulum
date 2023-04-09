#ifndef FURUTA_PENDULUM_SIMULATION_NODE_HPP_
#define FURUTA_PENDULUM_SIMULATION_NODE_HPP_

#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace furuta_pendulum
{
class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode(const rclcpp::NodeOptions & options);

private:
  double dt_ = 0.001;
  static constexpr double g_ = 9.80665;

  // Mechanical System parameters
  double J0_hat_, J2_hat_;

  double m1_, m2_;
  double l1_, l2_;
  double L1_, L2_;
  double b1, b2;

  double theta1_ = 0.0, theta2_ = 0.0;
  double dtheta1_ = 0.0, dtheta2_ = 0.0;
  double ddtheta1_ = 0.0, ddtheta2_ = 0.0;

  double tau1_ = 0.0, tau2_ = 0.0;

  double max_torque_ = 1.47;
  double max_velocity_ = 10.0;

  // Motor parameters
  double L_, R_, Km_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr disturbance_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_disturbance_sub_;

  void Simulate();

  void PublishJointStates();

  void SetTorqueCb(std_msgs::msg::Float64::SharedPtr msg) { SetTorque(msg->data); }
  void SetDisturbanceCb(std_msgs::msg::Float64::SharedPtr msg) { SetDisturbance(msg->data); }
  void RvizDisturbanceCb(geometry_msgs::msg::PointStamped::SharedPtr) { SetDisturbance(10.0); }

  void SetTorque(double torque) { tau1_ = std::clamp(torque, -max_torque_, max_torque_); }
  void SetDisturbance(double disturbance)
  {
    tau2_ = std::clamp(disturbance, -max_torque_, max_torque_);
  }
};
}  // namespace furuta_pendulum

#endif