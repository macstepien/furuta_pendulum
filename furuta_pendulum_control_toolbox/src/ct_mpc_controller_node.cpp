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
    RCLCPP_INFO(this->get_logger(), "Starting calculations");

    const bool verbose = true;
    std::string config_file = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                "furuta_pendulum_control_toolbox")) /
                              "config" / "nloc_config.info";

    mpc_ = CreateMPCController(config_file, verbose);

    RCLCPP_INFO(this->get_logger(), "Finished calculations, starting controller");

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&CtMPCControllerNode::StateCb, this, std::placeholders::_1));
    torque_cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_control", 10);

    builtin_interfaces::msg::Time current_time = this->get_clock()->now();
    start_sim_time_ = current_time.sec + 1000000000.0 * current_time.nanosec;

    mpc_->prepareIteration(0.0);
  }

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    ct::core::StateVector<FPSystem::STATE_DIM> x;
    x(0) = msg->position[0];
    x(1) = msg->position[1];
    x(2) = msg->velocity[0];
    x(3) = msg->velocity[1];

    double current_sim_time =
      msg->header.stamp.sec + 1000000000.0 * msg->header.stamp.nanosec - start_sim_time_;

    std::shared_ptr<ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
      new_controller(
        new ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);

    bool success = mpc_->finishIteration(x, current_sim_time, *new_controller, controller_ts_);
    if (!success) {
      throw std::runtime_error("Failed to finish MPC iteration.");
    }
    controller_ = new_controller;

    ct::core::ControlVector<FPSystem::CONTROL_DIM> control_action;
    controller_->computeControl(x, current_sim_time, control_action);
    double u = control_action(0);

    std_msgs::msg::Float64MultiArray torque_cmd_msg;
    torque_cmd_msg.data.push_back(u);
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
