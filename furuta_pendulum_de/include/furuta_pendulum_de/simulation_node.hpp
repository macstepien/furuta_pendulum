#ifndef FURUTA_PENDULUM_DE_SIMULATION_NODE_HPP_
#define FURUTA_PENDULUM_DE_SIMULATION_NODE_HPP_

#include <algorithm>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace furuta_pendulum_de
{
class SimulationNode : public rclcpp::Node
{
public:
  SimulationNode(const rclcpp::NodeOptions & options);

private:
  static constexpr long double g_ = 9.80665;
  long double dt_;

  // Mechanical System parameters
  long double J0_hat_, J2_hat_;

  long double m1_, m2_;
  long double l1_, l2_;
  long double L1_, L2_;
  long double b1_, b2_;

  long double theta1_, theta2_;
  long double dtheta1_, dtheta2_;
  long double ddtheta1_ = 0.0, ddtheta2_ = 0.0;

  long double tau1_, tau2_;

  long double max_torque_;
  long double max_velocity_;

  // Motor parameters
  long double L_, R_, Km_;

  long double current_time_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rviz_disturbance_sub_;

  void Simulate();
  void Simulate2();

  void PublishJointStates();

  void SetEffortCb(std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() > 1) {
      SetTorque(msg->data[0]);
    }
    // Ignore disturbance from effort command
    SetDisturbance(msg->data[1]);
  }
  void RvizDisturbanceCb(geometry_msgs::msg::PointStamped::SharedPtr) { SetDisturbance(-10.0); }

  // void SetTorque(double torque) { tau1_ = std::clamp(torque, -max_torque_, max_torque_); }
  void SetTorque(long double torque) { tau1_ = torque; }
  void SetDisturbance(long double disturbance)
  {
    tau2_ = disturbance;
    // tau2_ = std::clamp(disturbance, -max_torque_, max_torque_);
  }
  Eigen::Vector4d F(Eigen::Vector4d y);
  Eigen::Vector4d integrate_rk4(Eigen::Vector4d y_n);
};

}  // namespace furuta_pendulum_de

#endif