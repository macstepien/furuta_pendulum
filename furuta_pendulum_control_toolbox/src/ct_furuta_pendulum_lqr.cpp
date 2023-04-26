// Based on NLOC_MPC example from control_toolbox library
// https://github.com/ethz-adrl/control-toolbox/blob/v3.0.2/ct_models/examples/mpc/InvertedPendulum/NLOC_MPC.cpp

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ct/optcon/optcon.h>
#include <ct/rbd/rbd.h>

#include <ct/core/systems/continuous_time/linear/ADCodegenLinearizer.h>

#include <furuta_pendulum/FurutaPendulum.h>

int main()
{
  std::string config_file = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                              "furuta_pendulum_control_toolbox")) /
                            "config" / "lqr_config.info";

  // obtain the state dimension
  const size_t STATE_DIM =
    ct::rbd::FixBaseFDSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>::STATE_DIM;

  const size_t CONTROL_DIM = 2;

  // create an instance of the system
  std::shared_ptr<ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, ct::core::ADCGScalar>>
    dynamics(
      new ct::rbd::FixBaseFDSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<ct::core::ADCGScalar>>);

  // create an Auto-Differentiation Linearizer with code generation on the quadrotor model
  ct::core::ADCodegenLinearizer<STATE_DIM, CONTROL_DIM> adLinearizer(dynamics);
  // compile the linearized model just-in-time
  adLinearizer.compileJIT();
  // define the linearization point around steady state
  ct::core::StateVector<STATE_DIM> x;
  x.setZero();
  x(1) = M_PI;
  ct::core::ControlVector<CONTROL_DIM> u;
  u.setZero();
  double t = 0.0;
  // compute the linearization around the nominal state using the Auto-Diff Linearizer
  auto A = adLinearizer.getDerivativeState(x, u, t);
  auto B = adLinearizer.getDerivativeControl(x, u, t);

  std::cout << B << std::endl;
  std::cout << B(0) << std::endl;
  std::cout << B(0, 0) << std::endl;
  B(2, 1) = 0.0;
  B(3, 1) = 0.0;

  Eigen::MatrixXd real_B(4, 1);
  real_B(0, 0) = B(0, 0);
  real_B(1, 0) = B(1, 0);
  real_B(2, 0) = B(2, 0);
  real_B(3, 0) = B(3, 0);

  const size_t REAL_CONTROL_DIM = 1;
  ct::core::ControlVector<REAL_CONTROL_DIM> real_u;
  real_u.setZero();

  // load the weighting matrices
  ct::optcon::TermQuadratic<STATE_DIM, REAL_CONTROL_DIM> quadraticCost;
  quadraticCost.loadConfigFile(config_file, "termLQR");
  auto Q = quadraticCost.stateSecondDerivative(x, real_u, t);    // x, u and t can be arbitrary here
  auto R = quadraticCost.controlSecondDerivative(x, real_u, t);  // x, u and t can be arbitrary here
  // design the LQR controller
  ct::optcon::LQR<STATE_DIM, REAL_CONTROL_DIM> lqrSolver;
  ct::core::FeedbackMatrix<STATE_DIM, REAL_CONTROL_DIM> K;
  std::cout << "A: " << std::endl << A << std::endl << std::endl;
  std::cout << "B: " << std::endl << real_B << std::endl << std::endl;
  std::cout << "Q: " << std::endl << Q << std::endl << std::endl;
  std::cout << "R: " << std::endl << R << std::endl << std::endl;
  lqrSolver.compute(Q, R, A, real_B, K);
  std::cout << "LQR gain matrix:" << std::endl << K << std::endl;
}
