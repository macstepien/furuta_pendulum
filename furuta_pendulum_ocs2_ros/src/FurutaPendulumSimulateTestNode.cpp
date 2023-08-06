#include <furuta_pendulum_ocs2/dynamics/FurutaPendulumSystemDynamics.h>
#include <furuta_pendulum_ocs2/definitions.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_core/integration/Integrator.h>

namespace furuta_pendulum_ocs2_ros
{
class OCS2SimulationNode : public rclcpp::Node
{
public:
  OCS2SimulationNode(const rclcpp::NodeOptions & options)
  : Node("furuta_pendulum_simulation_node", options), current_time_(0.0)
  {
    state_ = ocs2::ad_vector_t::Zero(ocs2::furuta_pendulum::STATE_DIM);

    this->declare_parameter("initial_conditions.theta1", 0.0);
    this->declare_parameter("initial_conditions.theta2", 0.0);
    this->declare_parameter("initial_conditions.dtheta1", 0.0);
    this->declare_parameter("initial_conditions.dtheta2", 0.0);
    state_(0) = this->get_parameter("initial_conditions.theta1").as_double();
    state_(1) = this->get_parameter("initial_conditions.theta2").as_double();
    state_(2) = this->get_parameter("initial_conditions.dtheta1").as_double();
    state_(3) = this->get_parameter("initial_conditions.dtheta2").as_double();

    this->declare_parameter("simulation_dt", rclcpp::PARAMETER_DOUBLE);

    try {
      dt_ = this->get_parameter("simulation_dt").as_double();
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
      throw e;
    }

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    simulation_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&OCS2SimulationNode::Simulate, this));

    std::string libraryFolder =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "auto_generated";
    dynamics_ =
      std::make_unique<ocs2::furuta_pendulum::FurutaPendulumSystemDynamics>(libraryFolder, false);
    integrator_ = ocs2::newIntegrator(ocs2::IntegratorType::ODE45_OCS2);
  }

private:
  std::unique_ptr<ocs2::furuta_pendulum::FurutaPendulumSystemDynamics> dynamics_;
  ocs2::ad_vector_t state_;
  ocs2::ad_scalar_t current_time_;
  // ocs2::scalar_t current_time_;

  double dt_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  std::unique_ptr<ocs2::IntegratorBase> integrator_;

  // void Simulate()
  // {
  //   ocs2::vector_array_t xTraj;
  //   ocs2::Observer observer(&xTraj);
  //   integrator_->integrateConst(
  //     *dynamics_, observer, state_, current_time_, current_time_ + dt_, dt_);

  //   current_time_ += dt_;
  //   PublishJointStates();
  // }

  // TEST approach
  // void Simulate()
  // {
  //   ocs2::ad_vector_t input(ocs2::furuta_pendulum::INPUT_DIM);
  //   ocs2::ad_vector_t parameters;

  //   ocs2::ad_vector_t state_derivative_ =
  //     dynamics_->systemFlowMap(current_time_, state_, input, parameters);
  //   state_(0) += state_derivative_(0) * dt_;
  //   state_(1) += state_derivative_(1) * dt_;
  //   state_(2) += state_derivative_(2) * dt_;
  //   state_(3) += state_derivative_(3) * dt_;

  //   current_time_ += dt_;
  //   PublishJointStates();
  // }

  void Simulate()
  {
    state_ = IntegrateRK4(state_);
    current_time_ += dt_;

    PublishJointStates();
  }

  ocs2::ad_vector_t IntegrateRK4(ocs2::ad_vector_t y_n)
  {
    ocs2::ad_vector_t input(ocs2::furuta_pendulum::INPUT_DIM);
    ocs2::ad_vector_t parameters;

    ocs2::ad_vector_t k1 = dynamics_->systemFlowMap(current_time_, y_n, input, parameters);
    ocs2::ad_vector_t k2 = dynamics_->systemFlowMap(
      current_time_, y_n + ocs2::ad_scalar_t(dt_ / 2.0) * k1, input, parameters);
    ocs2::ad_vector_t k3 = dynamics_->systemFlowMap(
      current_time_, y_n + ocs2::ad_scalar_t(dt_ / 2.0) * k2, input, parameters);
    ocs2::ad_vector_t k4 =
      dynamics_->systemFlowMap(current_time_, y_n + ocs2::ad_scalar_t(dt_) * k3, input, parameters);

    ocs2::ad_vector_t y_n1 =
      y_n + (k1 + ocs2::ad_scalar_t(2.0) * k2 + ocs2::ad_scalar_t(2.0) * k3 + k4) *
              ocs2::ad_scalar_t(dt_ / 6.0);
    return y_n1;
  }

  void PublishJointStates()
  {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();

    joint_state_msg.name.push_back("joint0");
    joint_state_msg.position.push_back(CppAD::Value(state_(0)).getValue());
    joint_state_msg.velocity.push_back(CppAD::Value(state_(2)).getValue());

    joint_state_msg.name.push_back("joint1");
    joint_state_msg.position.push_back(CppAD::Value(state_(1)).getValue());
    joint_state_msg.velocity.push_back(CppAD::Value(state_(3)).getValue());

    joint_state_pub_->publish(joint_state_msg);
  }
};

}  // namespace furuta_pendulum_ocs2_ros

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_ocs2_ros::OCS2SimulationNode)
