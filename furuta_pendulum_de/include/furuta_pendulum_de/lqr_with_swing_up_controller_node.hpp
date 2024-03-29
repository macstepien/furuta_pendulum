#ifndef FURUTA_PENDULUM_DE_LQR_WITH_SWING_UP_CONTROLLER_NODE_HPP_
#define FURUTA_PENDULUM_DE_LQR_WITH_SWING_UP_CONTROLLER_NODE_HPP_

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace furuta_pendulum_de
{
class LqrWithSwingUpControllerNode : public rclcpp::Node
{
public:
  LqrWithSwingUpControllerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;

  Eigen::Vector4d K_;

  double m1_, m2_;
  double l1_, l2_;
  double J1_, J2_;
  double L1_, L2_;

  double u_max_;

  double lqr_transition_angle_;

  double last_control_ = 0.0;
  int last_change_count_ = 0;
  bool swing_up_started_ = false;

  double torque_multiplier_;
  double alpha_;
  double alpha_swingup_;
  double dtheta2_filtered_ = 0.0;
  double dtheta2_filtered_swingup_ = 0.0;
  double dtheta1_filtered_ = 0.0;

  static constexpr double g_ = 9.80665;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg);
  double LqrControl(sensor_msgs::msg::JointState::SharedPtr current_state);
  double SwingUpControl(sensor_msgs::msg::JointState::SharedPtr current_state);
};
}  // namespace furuta_pendulum_de

#endif