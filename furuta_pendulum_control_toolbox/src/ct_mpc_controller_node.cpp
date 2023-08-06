// Based on NLOC_MPC example from control_toolbox library
// https://github.com/ethz-adrl/control-toolbox/blob/v3.0.2/ct_models/examples/mpc/InvertedPendulum/NLOC_MPC.cpp

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

#include <ct/rbd/rbd.h>
#include <ct/core/types/Time.h>
#include <ct/core/types/StateVector.h>
#include <ct/core/integration/Integrator.h>
#include <ct/core/control/continuous_time/Controller.h>

#include <furuta_pendulum/FurutaPendulum.h>
#include <furuta_pendulum/FurutaPendulumSystem.h>
#include <furuta_pendulum/FurutaPendulumNLOC.h>
#include <furuta_pendulum/FurutaPendulumNLOC-impl.h>
#include <furuta_pendulum/FurutaPendulumMPC.h>

namespace furuta_pendulum_control_toolbox
{

using PendulumState = ct::rbd::FixBaseRobotState<ct::rbd::FurutaPendulum::Kinematics::NJOINTS>;
using FPSystem = ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>;
using FurutaPendulumNLOCSystem = ct::rbd::FurutaPendulumNLOC<FPSystem>;

class CtMPCControllerNode : public rclcpp::Node
{
public:
  CtMPCControllerNode(const rclcpp::NodeOptions & options) : Node("controller_node", options)
  {
    this->declare_parameter("alpha", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("torque_multiplier", rclcpp::PARAMETER_DOUBLE);

    alpha_ = this->get_parameter("alpha").as_double();
    torque_multiplier_ = this->get_parameter("torque_multiplier").as_double();

    RCLCPP_INFO(this->get_logger(), "Starting calculations");

    const bool verbose = true;
    std::string config_file = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                "furuta_pendulum_control_toolbox")) /
                              "config" / "mpc_config.info";

    mpc_ = CreateMPCController(config_file, verbose);

    RCLCPP_INFO(this->get_logger(), "Finished calculations, starting controller");

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&CtMPCControllerNode::StateCb, this, std::placeholders::_1));
    torque_cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_control", 10);

    rclcpp::Time current_time = this->get_clock()->now();
    start_sim_time_ = current_time.nanoseconds() / 1000000000.0;
    // RCLCPP_INFO_STREAM(this->get_logger(), current_time.sec << " " << current_time.nanosec);
    RCLCPP_INFO_STREAM(this->get_logger(), start_sim_time_);

    RCLCPP_INFO(this->get_logger(), "Preparing first iteration");
    mpc_->prepareIteration(0.0);
    RCLCPP_INFO(this->get_logger(), "Finished preparing first iteration");
  }

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

  double dtheta2_filtered_ = 0.0;
  double dtheta1_filtered_ = 0.0;
  double alpha_ = 0.0;
  double torque_multiplier_ = 0.0;
  bool initial_position_ = false;
  double initial_joint0_ = 0.0;
  double initial_joint1_ = 0.0;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!initial_position_) {
      initial_position_ = true;
      initial_joint0_ = msg->position[0];
      initial_joint1_ = msg->position[1];
    }

    rclcpp::Time current_time = this->get_clock()->now();
    double current_sim_time = current_time.nanoseconds() / 1000000000.0 - start_sim_time_;

    dtheta1_filtered_ = alpha_ * msg->velocity[0] + (1.0 - alpha_) * dtheta1_filtered_;
    dtheta2_filtered_ = alpha_ * msg->velocity[1] + (1.0 - alpha_) * dtheta2_filtered_;

    ct::core::StateVector<FPSystem::STATE_DIM> x;
    // x(1) = limit_minus_pi_pi(msg->position[1] - M_PI);
    // x(0) = msg->position[0] - initial_joint0_;
    // x(1) = msg->position[1] - initial_joint1_;
    x(0) = msg->position[0];
    x(1) = msg->position[1];
    x(2) = dtheta1_filtered_;
    x(3) = dtheta2_filtered_;

    double feedback_timestamp =
      msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0 - start_sim_time_;

    std::shared_ptr<ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
      new_controller(
        new ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);

    bool success = mpc_->finishIteration(x, feedback_timestamp, *new_controller, controller_ts_);
    if (!success) {
      throw std::runtime_error("Failed to finish MPC iteration.");
    }
    controller_ = new_controller;

    ct::core::ControlVector<FPSystem::CONTROL_DIM> control_action;
    controller_->computeControl(x, current_sim_time, control_action);
    double u = control_action(0);

    std_msgs::msg::Float64MultiArray torque_cmd_msg;
    torque_cmd_msg.data.push_back(u * torque_multiplier_);
    torque_cmd_msg.data.push_back(0.0);
    torque_cmd_pub_->publish(torque_cmd_msg);

    mpc_->prepareIteration(current_sim_time);
  }

private:
  double start_sim_time_;
  std::unique_ptr<
    ct::optcon::MPC<ct::optcon::NLOptConSolver<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>>
    mpc_;
  ct::core::Time controller_ts_;
  std::shared_ptr<
    ct::core::Controller<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM, FPSystem::SCALAR>>
    controller_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;
};

}  // namespace furuta_pendulum_control_toolbox

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_control_toolbox::CtMPCControllerNode)
