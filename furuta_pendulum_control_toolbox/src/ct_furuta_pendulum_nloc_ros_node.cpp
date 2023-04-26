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

#include <furuta_pendulum/FurutaPendulum.h>
#include <furuta_pendulum/FurutaPendulumSystem.h>
#include <furuta_pendulum/FurutaPendulumNLOC.h>
#include <furuta_pendulum/FurutaPendulumNLOC-impl.h>
#include <furuta_pendulum/FurutaPendulumControlSimulator.h>

using PendulumState = ct::rbd::FixBaseRobotState<ct::rbd::FurutaPendulum::Kinematics::NJOINTS>;
using FPSystem = ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>;
using FurutaPendulumNLOCSystem = ct::rbd::FurutaPendulumNLOC<FPSystem>;

int main(int argc, char * argv[])
{
  const bool verbose = true;
  try {
    std::string config_file = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                "furuta_pendulum_control_toolbox")) /
                              "config" / "nloc_config.info";

    std::shared_ptr<FPSystem> fp_system(new FPSystem());

    // NLOC settings
    ct::optcon::NLOptConSettings nloc_settings;
    nloc_settings.load(config_file, verbose, "ilqr");

    std::shared_ptr<ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
      term_quad_interm(new ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);
    term_quad_interm->loadConfigFile(config_file, "term0", verbose);

    std::shared_ptr<ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
      term_quad_final(new ct::optcon::TermQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);
    term_quad_final->loadConfigFile(config_file, "term1", verbose);

    std::shared_ptr<ct::optcon::CostFunctionAnalytical<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
      new_cost(new ct::optcon::CostFunctionAnalytical<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>);
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

    ct::optcon::ContinuousOptConProblem<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM> opt_con_problem(
      time_horizon, x0.toStateVector(), fp_system, new_cost, linear_system);

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
          ct::optcon::CostFunctionQuadratic<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>> & inst1 =
          nloc_solver.getSolver()->getCostFunctionInstances();

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

    std::cout << "waiting 1 second for begin" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    nloc_solver.solve();
    ct::core::StateFeedbackController<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM> initial_solution =
      nloc_solver.getSolution();
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

    ct::optcon::MPC<ct::optcon::NLOptConSolver<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
      ilqr_mpc(opt_con_problem, ilqr_settings_mpc, mpc_settings);
    ilqr_mpc.setInitialGuess(initial_solution);

    ct::core::Time sim_dt;
    ct::core::loadScalar(config_file, "sim_dt", sim_dt);

    ct::core::Time control_dt;
    ct::core::loadScalar(config_file, "control_dt", control_dt);

    double simulation_time;
    ct::core::loadScalar(config_file, "simulation_time", simulation_time);

    std::cout << "simulating 3 seconds" << std::endl;

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ct::core::FurutaPendulumControlSimulator<FPSystem>>(
      sim_dt, control_dt, x0.toStateVector(), fp_system, ilqr_mpc));

    rclcpp::shutdown();

    ilqr_mpc.printMpcSummary();

  } catch (std::runtime_error & e) {
    std::cout << "Exception caught: " << e.what() << std::endl;
  }
}
