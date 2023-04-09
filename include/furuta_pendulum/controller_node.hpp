#ifndef FURUTA_PENDULUM_CONTROLLER_NODE_HPP_
#define FURUTA_PENDULUM_CONTROLLER_NODE_HPP_

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace furuta_pendulum
{
class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_cmd_pub_;

  Eigen::Vector4d K;

  double m2 = 0.075;
  double l2 = 0.148;
  double g = 9.80665;
  double u_max = 10.0;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg);
  double LqrControl(sensor_msgs::msg::JointState::SharedPtr current_state);
  double SwingupControl(sensor_msgs::msg::JointState::SharedPtr current_state);
};
}  // namespace furuta_pendulum

#endif