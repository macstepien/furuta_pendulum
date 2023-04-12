// Based on NLOC_MPC example from control_toolbox library
// https://github.com/ethz-adrl/control-toolbox/blob/v3.0.2/ct_models/examples/mpc/InvertedPendulum/NLOC_MPC.cpp

// basically only for intellisense
#define CPPADCG

#include <ct/optcon/optcon.h>
#include <ct/rbd/rbd.h>

#include <ct/core/systems/continuous_time/linear/ADCodegenLinearizer.h>

#include <furuta_pendulum/FurutaPendulum.h>

int main()
{
  std::string workingDirectory =
    "/home/maciej/ros2_ws/src/furuta_pendulum/furuta_pendulum_control_toolbox/config";

  const size_t njoints = ct::rbd::FurutaPendulum::Kinematics::NJOINTS;
  const size_t actuator_state_dim = 2;

  static const size_t state_dim = ct::rbd::FixBaseRobotState<njoints, actuator_state_dim>::NSTATE;
  static const size_t control_dim = njoints;

  const double k_spring = 160;
  const double gear_ratio = 50;
  std::shared_ptr<ct::rbd::SEADynamicsFirstOrder<njoints>> actuatorDynamics(
    new ct::rbd::SEADynamicsFirstOrder<njoints>(k_spring, gear_ratio));

  std::shared_ptr<ct::rbd::FixBaseFDSystem<
    ct::rbd::FurutaPendulum::tpl::Dynamics<double>, actuator_state_dim, false>>
    ipSystem(new ct::rbd::FixBaseFDSystem<
             ct::rbd::FurutaPendulum::tpl::Dynamics<double>, actuator_state_dim, false>(
      actuatorDynamics));

  // create an Auto-Differentiation Linearizer with code generation on the quadrotor model
  ct::core::ADCodegenLinearizer<state_dim, control_dim> adLinearizer(ipSystem);
  // compile the linearized model just-in-time
  adLinearizer.compileJIT();
  // define the linearization point around steady state
  ct::core::StateVector<state_dim> x;
  x.setZero();
  ct::core::ControlVector<control_dim> u;
  u.setZero();
  double t = 0.0;
  // compute the linearization around the nominal state using the Auto-Diff Linearizer
  auto A = adLinearizer.getDerivativeState(x, u, t);
  auto B = adLinearizer.getDerivativeControl(x, u, t);
  // load the weighting matrices
  ct::optcon::TermQuadratic<state_dim, control_dim> quadraticCost;
  quadraticCost.loadConfigFile(workingDirectory + "/lqr_cost.info", "termLQR");
  auto Q = quadraticCost.stateSecondDerivative(x, u, t);    // x, u and t can be arbitrary here
  auto R = quadraticCost.controlSecondDerivative(x, u, t);  // x, u and t can be arbitrary here
  // design the LQR controller
  ct::optcon::LQR<state_dim, control_dim> lqrSolver;
  ct::core::FeedbackMatrix<state_dim, control_dim> K;
  std::cout << "A: " << std::endl << A << std::endl << std::endl;
  std::cout << "B: " << std::endl << B << std::endl << std::endl;
  std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
  std::cout << "R: " << std::endl << R << std::endl << std::endl;
  lqrSolver.compute(Q, R, A, B, K);
  std::cout << "LQR gain matrix:" << std::endl << K << std::endl;
}
