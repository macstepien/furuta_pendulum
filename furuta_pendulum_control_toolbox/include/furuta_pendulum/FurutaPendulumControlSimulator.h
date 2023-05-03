#pragma once

#include <memory>

#include <ct/core/types/Time.h>
#include <ct/core/types/StateVector.h>
#include <ct/core/integration/Integrator.h>
#include <ct/core/control/continuous_time/Controller.h>

namespace ct
{
namespace core
{

template <class CONTROLLED_SYSTEM>
class FurutaPendulumControlSimulator : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const size_t STATE_DIM = CONTROLLED_SYSTEM::STATE_DIM;
  static const size_t CONTROL_DIM = CONTROLLED_SYSTEM::CONTROL_DIM;

  using SCALAR = typename CONTROLLED_SYSTEM::SCALAR;

  FurutaPendulumControlSimulator(
    Time sim_dt, const StateVector<STATE_DIM> & x0,
    std::shared_ptr<CONTROLLED_SYSTEM> controlled_system,
    ct::optcon::MPC<ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>> & mpc)
  : Node("ct_furuta_pendulum_simulation_mpc_node"),
    sim_dt_(sim_dt),
    x0_(x0),
    system_(controlled_system),
    mpc_(mpc)
  {
    system_->getController(controller_);
    controller_.reset(new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);

    x_ = x0_;
    // const IntegrationType & intType = IntegrationType::EULERCT;
    integrator_ = std::make_unique<Integrator<STATE_DIM>>(system_, ct::core::RK4);

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    simulation_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(sim_dt_),
      std::bind(&FurutaPendulumControlSimulator::simulateSystem, this));
  }

  virtual void simulateSystem()
  {
    mpc_.prepareIteration(current_sim_time_);

    integrator_->integrate_n_steps(x_, 0, 1, sim_dt_);

    current_sim_time_ += sim_dt_;

    std::shared_ptr<ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>> new_controller(
      new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);

    bool success = mpc_.finishIteration(x_, current_sim_time_, *new_controller, controller_ts_);
    if (!success) throw std::runtime_error("Failed to finish MPC iteration.");
    controller_ = new_controller;
    system_->setController(controller_);

    PublishJointStates();
  }

  void PublishJointStates()
  {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();

    joint_state_msg.name.push_back("joint1");
    joint_state_msg.position.push_back(x_(0));
    joint_state_msg.velocity.push_back(x_(2));

    joint_state_msg.name.push_back("joint2");
    joint_state_msg.position.push_back(x_(1));
    joint_state_msg.velocity.push_back(x_(3));

    joint_state_pub_->publish(joint_state_msg);
  }

private:
  double current_sim_time_ = 0.0;

  Time sim_dt_;

  StateVector<STATE_DIM> x0_;
  StateVector<STATE_DIM> x_;

  std::shared_ptr<CONTROLLED_SYSTEM> system_;
  std::shared_ptr<Controller<STATE_DIM, CONTROL_DIM, SCALAR>> controller_;

  std::unique_ptr<Integrator<STATE_DIM>> integrator_;
  ct::optcon::MPC<ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>> & mpc_;
  ct::core::Time controller_ts_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
};

}  // namespace core
}  // namespace ct
