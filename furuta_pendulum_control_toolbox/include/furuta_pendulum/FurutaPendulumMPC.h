#pragma once

#include <memory>

#include <ct/rbd/rbd.h>
#include <ct/optcon/optcon.h>

#include <furuta_pendulum/FurutaPendulum.h>
#include <furuta_pendulum/FurutaPendulumSystem.h>
#include <furuta_pendulum/FurutaPendulumNLOC.h>
#include <furuta_pendulum/FurutaPendulumNLOC-impl.h>

namespace furuta_pendulum_control_toolbox
{

std::unique_ptr<ct::optcon::MPC<ct::optcon::NLOptConSolver<
  ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>::STATE_DIM,
  ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>::CONTROL_DIM>>>
CreateMPCController(std::string config_file, bool verbose = true)
{
  using PendulumState = ct::rbd::FixBaseRobotState<ct::rbd::FurutaPendulum::Kinematics::NJOINTS>;
  using FPSystem = ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>;
  using FurutaPendulumNLOCSystem = ct::rbd::FurutaPendulumNLOC<FPSystem>;

  try {
    std::shared_ptr<FPSystem> fp_system(new FPSystem());

    // Control signal bounds
    Eigen::VectorXd u_lb(FPSystem::CONTROL_DIM);
    Eigen::VectorXd u_ub(FPSystem::CONTROL_DIM);

    double control_signal_bound = 0.0;
    ct::core::loadScalar(config_file, "control_signal_bound", control_signal_bound);

    u_lb.setConstant(-control_signal_bound);
    u_ub = -u_lb;

    // constraint terms
    std::shared_ptr<ct::optcon::ControlInputConstraint<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>
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

    double velocity_bound = 0.0;
    ct::core::loadScalar(config_file, "velocity_bound", velocity_bound);

    x_lb.setConstant(-velocity_bound);
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
    mpc_settings.mpc_mode = ct::optcon::MPC_MODE::FIXED_FINAL_TIME_WITH_MIN_TIME_HORIZON;
    mpc_settings.coldStart_ = false;

    ct::core::Time minimum_time_horizon;
    ct::core::loadScalar(config_file, "minimum_time_horizon", minimum_time_horizon);
    mpc_settings.minimumTimeHorizonMpc_ = minimum_time_horizon;

    std::unique_ptr<
      ct::optcon::MPC<ct::optcon::NLOptConSolver<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>>
      ilqr_mpc = std::make_unique<
        ct::optcon::MPC<ct::optcon::NLOptConSolver<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>>(
        opt_con_problem, ilqr_settings_mpc, mpc_settings);
    ilqr_mpc->setInitialGuess(initial_solution);
    return ilqr_mpc;

  } catch (std::runtime_error & e) {
    std::cout << "Exception caught: " << e.what() << std::endl;
    throw e;
  }
}
}  // namespace furuta_pendulum_control_toolbox