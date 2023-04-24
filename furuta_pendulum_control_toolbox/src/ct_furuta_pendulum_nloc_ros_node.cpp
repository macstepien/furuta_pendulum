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

#include <furuta_pendulum/FurutaPendulum.h>
#include <furuta_pendulum/FurutaPendulumSystem.h>
#include <furuta_pendulum/FurutaPendulumNLOC.h>
#include <furuta_pendulum/FurutaPendulumNLOC-impl.h>
#include <furuta_pendulum/FurutaPendulumControlSimulator.h>

using namespace ct::rbd;

const size_t njoints = ct::rbd::FurutaPendulum::Kinematics::NJOINTS;

using RobotState_t = ct::rbd::FixBaseRobotState<njoints>;
static const size_t state_dim = RobotState_t::NSTATE;
static const size_t control_dim = 1;

using IPDynamics = ct::rbd::FurutaPendulum::tpl::Dynamics<double>;
using IPSystem = ct::rbd::FurutaPendulumSystem<IPDynamics>;
using LinearSystem = ct::core::LinearSystem<state_dim, control_dim, double>;

using FurutaPendulumNLOCSystem = FurutaPendulumNLOC<IPSystem>;

int main(int argc, char * argv[])
{
  const bool verbose = true;
  try {
    std::string workingDirectory =
      "/home/maciej/ros2_ws/src/furuta_pendulum/furuta_pendulum_control_toolbox/config";

    std::string configFile = workingDirectory + "/nloc_config.info";

    std::shared_ptr<IPSystem> ipSystem(new IPSystem());

    // NLOC settings
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(configFile, verbose, "ilqr");

    std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
      termQuadInterm(new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
    termQuadInterm->loadConfigFile(configFile, "term0", verbose);

    std::shared_ptr<ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
      termQuadFinal(new ct::optcon::TermQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
    termQuadFinal->loadConfigFile(configFile, "term1", verbose);

    std::shared_ptr<ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
      newCost(new ct::optcon::CostFunctionAnalytical<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>);
    size_t intTermID = newCost->addIntermediateTerm(termQuadInterm);
    /*size_t finalTermID = */ newCost->addFinalTerm(termQuadFinal);

    ct::core::Time timeHorizon;
    FurutaPendulumNLOCSystem::FeedbackArray::value_type fbD;
    RobotState_t x0;
    RobotState_t xf;

    ct::core::loadScalar(configFile, "timeHorizon", timeHorizon);
    ct::core::loadMatrix(configFile, "K_init", fbD);
    RobotState_t::state_vector_t xftemp, x0temp;
    ct::core::loadMatrix(configFile, "x_0", x0temp);
    ct::core::loadMatrix(configFile, "term1.weights.x_des", xftemp);
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
          ct::optcon::CostFunctionQuadratic<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>> & inst1 =
          nloc_solver.getSolver()->getCostFunctionInstances();

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

    std::cout << "waiting 1 second for begin" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    nloc_solver.solve();
    ct::core::StateFeedbackController<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM> initialSolution =
      nloc_solver.getSolution();
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

    ct::optcon::MPC<ct::optcon::NLOptConSolver<IPSystem::STATE_DIM, IPSystem::CONTROL_DIM>>
      ilqr_mpc(optConProblem, ilqr_settings_mpc, mpc_settings);
    ilqr_mpc.setInitialGuess(initialSolution);

    ct::core::Time sim_dt;
    ct::core::loadScalar(configFile, "sim_dt", sim_dt);

    ct::core::Time control_dt;
    ct::core::loadScalar(configFile, "control_dt", control_dt);

    double simulation_time;
    ct::core::loadScalar(configFile, "simulation_time", simulation_time);

    std::cout << "simulating 3 seconds" << std::endl;

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ct::core::FurutaPendulumControlSimulator<IPSystem>>(
      sim_dt, control_dt, x0.toStateVector(), ipSystem, ilqr_mpc));

    rclcpp::shutdown();

    ilqr_mpc.printMpcSummary();

  } catch (std::runtime_error & e) {
    std::cout << "Exception caught: " << e.what() << std::endl;
  }
}
