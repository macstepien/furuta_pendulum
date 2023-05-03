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

namespace furuta_pendulum_control_toolbox
{

using PendulumState = ct::rbd::FixBaseRobotState<ct::rbd::FurutaPendulum::Kinematics::NJOINTS>;
using FPSystem = ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>;
using FurutaPendulumNLOCSystem = ct::rbd::FurutaPendulumNLOC<FPSystem>;

class CtControllerNode : public rclcpp::Node
{
public:
  CtControllerNode(const rclcpp::NodeOptions & options) : Node("controller_node", options)
  {
    const bool verbose = true;
    RCLCPP_INFO(this->get_logger(), "Starting calculations");
    try {
      std::string config_file = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "furuta_pendulum_control_toolbox")) /
                                "config" / "nloc_config.info";

      std::shared_ptr<FPSystem> fp_system(new FPSystem());

      // Control signal bounds
      Eigen::VectorXd u_lb(FPSystem::CONTROL_DIM);
      Eigen::VectorXd u_ub(FPSystem::CONTROL_DIM);
      u_lb.setConstant(-4.0);
      u_ub = -u_lb;

      // constraint terms
      std::shared_ptr<
        ct::optcon::ControlInputConstraint<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
        control_input_bound(
          new ct::optcon::ControlInputConstraint<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>(
            u_lb, u_ub));
      control_input_bound->setName("control_input_bound");

      // input box constraint constraint container
      std::shared_ptr<
        ct::optcon::ConstraintContainerAnalytical<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
        input_box_constraints(new ct::optcon::ConstraintContainerAnalytical<
                              FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>());

      // add and initialize constraint terms
      input_box_constraints->addIntermediateConstraint(control_input_bound, verbose);
      input_box_constraints->initialize();

      // State constraints
      Eigen::VectorXi sp_state(FPSystem::STATE_DIM);
      sp_state << 0, 1, 0, 1;
      Eigen::VectorXd x_lb(2);
      Eigen::VectorXd x_ub(2);
      x_lb.setConstant(-10.0);
      x_ub = -x_lb;
      // constraint terms
      std::shared_ptr<ct::optcon::StateConstraint<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
        state_bound(new ct::optcon::StateConstraint<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>(
          x_lb, x_ub, sp_state));
      state_bound->setName("state_bound");

      // input box constraint constraint container
      std::shared_ptr<
        ct::optcon::ConstraintContainerAnalytical<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
        state_box_constraints(new ct::optcon::ConstraintContainerAnalytical<
                              FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>());

      // add and initialize constraint terms
      state_box_constraints->addIntermediateConstraint(state_bound, verbose);
      state_box_constraints->initialize();

      // NLOC settings
      ct::optcon::NLOptConSettings nloc_settings;
      nloc_settings.load(config_file, verbose, "ilqr");

      std::shared_ptr<ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
        term_quad_interm(new ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);
      term_quad_interm->loadConfigFile(config_file, "term0", verbose);

      std::shared_ptr<ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
        term_quad_final(new ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);
      term_quad_final->loadConfigFile(config_file, "term1", verbose);

      std::shared_ptr<
        ct::optcon::CostFunctionAnalytical<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
        new_cost(
          new ct::optcon::CostFunctionAnalytical<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);
      size_t int_term_id = new_cost->addIntermediateTerm(term_quad_interm);
      /*size_t final_term_id = */ new_cost->addFinalTerm(term_quad_final);

      ct::core::Time time_horizon;
      FurutaPendulumNLOCSystem::FeedbackArray::value_type fbD;
      PendulumState x0;
      PendulumState xf;

      ct::core::loadScalar(config_file, "time_horizon", time_horizon);
      ct::core::loadMatrix(config_file, "K_init", fbD);
      PendulumState::state_vector_t xftemp, x0temp;
      ct::core::loadMatrix(config_file, "x_0", x0temp);
      ct::core::loadMatrix(config_file, "term1.weights.x_des", xftemp);
      x0.fromStateVector(x0temp);
      xf.fromStateVector(xftemp);

      std::shared_ptr<ct::core::LinearSystem<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM, double>>
        linear_system = nullptr;

      ct::optcon::ContinuousOptConProblem<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>
        opt_con_problem(time_horizon, x0.toStateVector(), fp_system, new_cost, linear_system);
      opt_con_problem.setInputBoxConstraints(input_box_constraints);
      opt_con_problem.setStateBoxConstraints(state_box_constraints);

      FurutaPendulumNLOCSystem nloc_solver(
        new_cost, nloc_settings, fp_system, verbose, linear_system);

      int K = nloc_solver.getSettings().computeK(time_horizon);

      FurutaPendulumNLOCSystem::StateVectorArray state_ref_traj(K + 1, x0.toStateVector());
      FurutaPendulumNLOCSystem::FeedbackArray fb_trajectory(K, -fbD);
      FurutaPendulumNLOCSystem::ControlVectorArray ff_trajectory(
        K, FurutaPendulumNLOCSystem::ControlVector::Zero());

      int init_type = 0;
      ct::core::loadScalar(config_file, "init_type", init_type);

      switch (init_type) {
        case 0:  // steady state
        {
          ct::core::ControlVector<FPSystem::CONTROL_DIM> uff_ref;
          nloc_solver.initializeSteadyPose(x0, time_horizon, K, uff_ref, -fbD);

          std::vector<std::shared_ptr<
            ct::optcon::CostFunctionQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>> &
            inst1 = nloc_solver.getSolver()->getCostFunctionInstances();

          for (size_t i = 0; i < inst1.size(); i++) {
            inst1[i]->getIntermediateTermById(int_term_id)->updateReferenceControl(uff_ref);
          }
          break;
        }
        case 1:  // linear interpolation
        {
          nloc_solver.initializeDirectInterpolation(x0, xf, time_horizon, K, -fbD);
          break;
        }
        default: {
          throw std::runtime_error("illegal init type");
          break;
        }
      }

      // std::cout << "waiting 1 second for begin" << std::endl;
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      nloc_solver.solve();
      ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>
        initial_solution = nloc_solver.getSolution();
      FurutaPendulumNLOCSystem::StateVectorArray x_nloc = initial_solution.x_ref();

      ct::optcon::NLOptConSettings ilqr_settings_mpc(nloc_solver.getSettings());
      ilqr_settings_mpc.max_iterations = 1;
      ilqr_settings_mpc.printSummary = false;

      ct::optcon::mpc_settings mpc_settings;
      mpc_settings.stateForwardIntegration_ = false;
      mpc_settings.postTruncation_ = false;
      mpc_settings.measureDelay_ = false;
      mpc_settings.delayMeasurementMultiplier_ = 1.0;
      mpc_settings.mpc_mode = ct::optcon::MPC_MODE::CONSTANT_RECEDING_HORIZON;
      mpc_settings.coldStart_ = false;
      mpc_settings.minimumTimeHorizonMpc_ = 3.0;

      ilqr_mpc_ = std::make_unique<
        ct::optcon::MPC<ct::optcon::NLOptConSolver<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>>(
        opt_con_problem, ilqr_settings_mpc, mpc_settings);
      ilqr_mpc_->setInitialGuess(initial_solution);

    } catch (std::runtime_error & e) {
      std::cout << "Exception caught: " << e.what() << std::endl;
    }
    RCLCPP_INFO(this->get_logger(), "Finished calculations, starting controller");

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&CtControllerNode::StateCb, this, std::placeholders::_1));
    torque_cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_control", 10);

    builtin_interfaces::msg::Time current_time = this->get_clock()->now();
    start_sim_time_ = current_time.sec + 1000000000.0 * current_time.nanosec;

    ilqr_mpc_->prepareIteration(0.0);
  }

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    ct::core::StateVector<FPSystem::STATE_DIM> x;
    x(0) = msg->position[0];
    x(1) = msg->position[1];
    // x(0) = remainder(msg->position[0], 2.0 * M_PI);
    // x(1) = remainder(msg->position[1] - M_PI, 2.0 * M_PI);
    x(2) = msg->velocity[0];
    x(3) = msg->velocity[1];

    double current_sim_time =
      msg->header.stamp.sec + 1000000000.0 * msg->header.stamp.nanosec - start_sim_time_;

    std::shared_ptr<ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
      new_controller(
        new ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);

    bool success = ilqr_mpc_->finishIteration(x, current_sim_time, *new_controller, controller_ts_);
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

    ilqr_mpc_->prepareIteration(current_sim_time);
  }

private:
  double start_sim_time_;
  std::unique_ptr<
    ct::optcon::MPC<ct::optcon::NLOptConSolver<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>>
    ilqr_mpc_;
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
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_control_toolbox::CtControllerNode)
