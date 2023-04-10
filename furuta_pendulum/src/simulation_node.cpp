#include <furuta_pendulum/simulation_node.hpp>

#include <chrono>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace furuta_pendulum
{

SimulationNode::SimulationNode(const rclcpp::NodeOptions & options)
: Node("furuta_pendulum_simulation_node", options)
{
  this->declare_parameter("initial_conditions.theta1", 0.0);
  this->declare_parameter("initial_conditions.theta2", 0.0);
  this->declare_parameter("initial_conditions.dtheta1", 0.0);
  this->declare_parameter("initial_conditions.dtheta2", 0.0);
  this->declare_parameter("initial_conditions.ddtheta1", 0.0);
  this->declare_parameter("initial_conditions.ddtheta2", 0.0);
  this->declare_parameter("initial_conditions.tau1", 0.0);
  this->declare_parameter("initial_conditions.tau2", 0.0);
  theta1_ = this->get_parameter("initial_conditions.theta1").as_double();
  theta2_ = this->get_parameter("initial_conditions.theta2").as_double();
  dtheta1_ = this->get_parameter("initial_conditions.dtheta1").as_double();
  dtheta2_ = this->get_parameter("initial_conditions.dtheta2").as_double();
  ddtheta1_ = this->get_parameter("initial_conditions.ddtheta1").as_double();
  ddtheta2_ = this->get_parameter("initial_conditions.ddtheta2").as_double();
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
  this->declare_parameter("L", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("R", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("Km", rclcpp::PARAMETER_DOUBLE);

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
    L_ = this->get_parameter("L").as_double();
    R_ = this->get_parameter("R").as_double();
    Km_ = this->get_parameter("Km").as_double();

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
    std::chrono::duration<double>(dt_), std::bind(&SimulationNode::Simulate, this));

  effort_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "effort_control", 10, std::bind(&SimulationNode::SetEffortCb, this, std::placeholders::_1));

  rviz_disturbance_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 10,
    std::bind(&SimulationNode::RvizDisturbanceCb, this, std::placeholders::_1));
}

void SimulationNode::Simulate()
{
  // based on https://www.hindawi.com/journals/jcse/2011/528341/

  Eigen::Matrix2d inertia_matrix;
  inertia_matrix(0, 0) = J0_hat_ + J2_hat_ * pow(sin(theta2_), 2);
  inertia_matrix(0, 1) = m2_ * L1_ * l2_ * cos(theta2_);
  inertia_matrix(1, 0) = inertia_matrix(0, 1);
  inertia_matrix(1, 1) = J2_hat_;

  Eigen::Matrix2d viscous_damping_centripetal_coriolis;
  viscous_damping_centripetal_coriolis(0, 0) = b1_ + 0.5 * dtheta2_ * J2_hat_ * sin(2.0 * theta2_);
  viscous_damping_centripetal_coriolis(0, 1) =
    0.5 * dtheta2_ * J2_hat_ * sin(2.0 * theta2_) - m2_ * L1_ * l2_ * sin(theta2_) * dtheta2_;
  viscous_damping_centripetal_coriolis(1, 0) = -0.5 * dtheta1_ * J2_hat_ * sin(2.0 * theta2_);
  viscous_damping_centripetal_coriolis(1, 1) = b2_;

  Eigen::Vector2d dtheta;
  dtheta(0) = dtheta1_;
  dtheta(1) = dtheta2_;

  Eigen::Vector2d gravity_torque;
  gravity_torque(0) = 0.0;
  gravity_torque(1) = g_ * m2_ * l2_ * sin(theta2_);

  Eigen::Vector2d input_torque;
  input_torque(0) = tau1_;
  input_torque(1) = tau2_;

  Eigen::Vector2d ddtheta =
    inertia_matrix.inverse() *
    (viscous_damping_centripetal_coriolis * dtheta - gravity_torque + input_torque);

  ddtheta1_ = ddtheta(0);
  ddtheta2_ = ddtheta(1);

  dtheta1_ = std::clamp(dtheta1_ + ddtheta1_ * dt_, -max_velocity_, max_velocity_);
  dtheta2_ = std::clamp(dtheta2_ + ddtheta2_ * dt_, -max_velocity_, max_velocity_);

  theta1_ += dtheta1_ * dt_;
  theta2_ += dtheta2_ * dt_;

  PublishJointStates();

  // treat disturbance as impulse and set it back to 0.
  tau2_ = 0.0;
}

void SimulationNode::PublishJointStates()
{
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = this->get_clock()->now();

  joint_state_msg.name.push_back("joint1");
  joint_state_msg.position.push_back(theta1_);
  joint_state_msg.velocity.push_back(dtheta1_);

  joint_state_msg.name.push_back("joint2");
  joint_state_msg.position.push_back(theta2_);
  joint_state_msg.velocity.push_back(dtheta2_);

  joint_state_pub_->publish(joint_state_msg);
}

}  // namespace furuta_pendulum

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum::SimulationNode)
