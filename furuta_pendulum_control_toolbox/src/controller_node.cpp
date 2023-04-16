// Based on NLOC_MPC example from control_toolbox library
// https://github.com/ethz-adrl/control-toolbox/blob/v3.0.2/ct_models/examples/mpc/InvertedPendulum/NLOC_MPC.cpp

#include <chrono>
#include <memory>
#include <algorithm>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <ct/rbd/rbd.h>
#include <ct/core/types/Time.h>
#include <ct/core/types/StateVector.h>
#include <ct/core/integration/Integrator.h>
#include <ct/core/control/continuous_time/Controller.h>

#include <furuta_pendulum/FurutaPendulum.h>
#include <furuta_pendulum/FurutaPendulumSystem.h>
#include <furuta_pendulum/FurutaPendulumNLOC.h>
#include <furuta_pendulum/FurutaPendulumNLOC-impl.h>

using namespace ct::rbd;

const size_t njoints = ct::rbd::FurutaPendulum::Kinematics::NJOINTS;

using RobotState_t = ct::rbd::FixBaseRobotState<njoints>;
static const size_t state_dim = RobotState_t::NSTATE;
static const size_t control_dim = 1;

using IPDynamics = ct::rbd::FurutaPendulum::tpl::Dynamics<double>;
using IPSystem = ct::rbd::FurutaPendulumSystem<IPDynamics>;
using LinearSystem = ct::core::LinearSystem<state_dim, control_dim, double>;

using FurutaPendulumNLOCSystem = FurutaPendulumNLOC<IPSystem>;

namespace furuta_pendulum_control_toolbox
{
class CtControllerNode : public rclcpp::Node
{
public:
  CtControllerNode(const rclcpp::NodeOptions & options) : Node("controller_node", options)
  {
    const bool verbose = true;
    RCLCPP_INFO(this->get_logger(), "Starting calculations");
    try {
      std::string workingDirectory =
        "/home/maciej/ros2_ws/src/furuta_pendulum/furuta_pendulum_control_toolbox/config";

      std::string configFile = workingDirectory + "/solver_furuta_pendulum.info";
      std::string costFunctionFile = workingDirectory + "/cost_furuta_pendulum.info";

      std::shared_ptr<IPSystem> ipSystem(new IPSystem());

      // NLOC settings
      ct::optcon::NLOptConSettings nloc_settings;
      nloc_settings.load(configFile, verbose, "ilqr");

      std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
        termQuadInterm(new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
      termQuadInterm->loadConfigFile(costFunctionFile, "term0", verbose);

      std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
        termQuadFinal(new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
      termQuadFinal->loadConfigFile(costFunctionFile, "term1", verbose);

      std::shared_ptr<
        ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
        newCost(new ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
      size_t intTermID = newCost->addIntermediateTerm(termQuadInterm);
      /*size_t finalTermID = */ newCost->addFinalTerm(termQuadFinal);

      ct::core::Time timeHorizon;
      FurutaPendulumNLOCSystem::FeedbackArray::value_type fbD;
      RobotState_t x0;
      RobotState_t xf;

      ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
      ct::core::loadMatrix(costFunctionFile, "K_init", fbD);
      RobotState_t::state_vector_t xftemp, x0temp;
      ct::core::loadMatrix(costFunctionFile, "x_0", x0temp);
      ct::core::loadMatrix(costFunctionFile, "term1.weights.x_des", xftemp);
      x0.fromStateVector(x0temp);
      xf.fromStateVector(xftemp);

      std::shared_ptr<LinearSystem> linSystem = nullptr;

      ct::optcon::ContinuousOptConProblem<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM> optConProblem(
        timeHorizon, x0.toStateVector(), ipSystem, newCost, linSystem);

      FurutaPendulumNLOCSystem nloc_solver(newCost, nloc_settings, ipSystem, verbose, linSystem);

      int K = nloc_solver.getSettings().computeK(timeHorizon);

      FurutaPendulumNLOCSystem::StateVectorArray stateRefTraj(K + 1, x0.toStateVector());
      FurutaPendulumNLOCSystem::FeedbackArray fbTrajectory(K, -fbD);
      FurutaPendulumNLOCSystem::ControlVectorArray ffTrajectory(
        K, FurutaPendulumNLOCSystem::ControlVector::Zero());

      int initType = 0;
      ct::core::loadScalar(configFile, "initType", initType);

      switch (initType) {
        case 0:  // steady state
        {
          ct::core::ControlVector<IPSystem::CONTROL_DIM> uff_ref;
          nloc_solver.initializeSteadyPose(x0, timeHorizon, K, uff_ref, -fbD);

          std::vector<std::shared_ptr<
            ct::optcon::CostFunctionQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>> &
            inst1 = nloc_solver.getSolver()->getCostFunctionInstances();

          for (size_t i = 0; i < inst1.size(); i++) {
            inst1[i]->getIntermediateTermById(intTermID)->updateReferenceControl(uff_ref);
          }
          break;
        }
        // case 1:  // linear interpolation
        // {
        //   nloc_solver.initializeDirectInterpolation(x0, xf, timeHorizon, K, -fbD);
        //   break;
        // }
        default: {
          throw std::runtime_error("illegal init type");
          break;
        }
      }

      // std::cout << "waiting 1 second for begin" << std::endl;
      // std::this_thread::sleep_for(std::chrono::seconds(1));

      nloc_solver.solve();
      ct::core::StateFeedbackController<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>
        initialSolution = nloc_solver.getSolution();
      FurutaPendulumNLOCSystem::StateVectorArray x_nloc = initialSolution.x_ref();

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
        ct::optcon::MPC<ct::optcon::NLOptConSolver<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>>(
        optConProblem, ilqr_settings_mpc, mpc_settings);
      ilqr_mpc_->setInitialGuess(initialSolution);

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
    ct::core::StateVector<IPSystem::STATE_DIM> x;
    x(0) = msg->position[0];
    x(1) = msg->position[1];
    // x(0) = remainder(msg->position[0], 2.0 * M_PI);
    // x(1) = remainder(msg->position[1] - M_PI, 2.0 * M_PI);
    x(2) = msg->velocity[0];
    x(3) = msg->velocity[1];

    double current_sim_time =
      msg->header.stamp.sec + 1000000000.0 * msg->header.stamp.nanosec - start_sim_time_;

    std::shared_ptr<ct::core::StateFeedbackController<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
      new_controller(
        new ct::core::StateFeedbackController<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);

    bool success = ilqr_mpc_->finishIteration(x, current_sim_time, *new_controller, controller_ts_);
    if (!success) {
      throw std::runtime_error("Failed to finish MPC iteration.");
    }
    controller_ = new_controller;

    ct::core::ControlVector<IPSystem::CONTROL_DIM> control_action;
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
    ct::optcon::MPC<ct::optcon::NLOptConSolver<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>>
    ilqr_mpc_;
  ct::core::Time controller_ts_;
  std::shared_ptr<
    ct::core::Controller<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM, IPSystem::SCALAR>>
    controller_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;
};

}  // namespace furuta_pendulum_control_toolbox

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_control_toolbox::CtControllerNode)
