// Based on NLOC_MPC example from control_toolbox library
// https://github.com/ethz-adrl/control-toolbox/blob/v3.0.2/ct_models/examples/mpc/InvertedPendulum/NLOC_MPC.cpp

#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ct/rbd/rbd.h>

#include <furuta_pendulum/FurutaPendulum.h>
#include <furuta_pendulum/FurutaPendulumSystem.h>
#include <furuta_pendulum/FurutaPendulumNLOC.h>
#include <furuta_pendulum/FurutaPendulumNLOC-impl.h>
#include <furuta_pendulum/FurutaPendulumMPC.h>

using PendulumState = ct::rbd::FixBaseRobotState<ct::rbd::FurutaPendulum::Kinematics::NJOINTS>;
using FPSystem = ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>;
using FurutaPendulumNLOCSystem = ct::rbd::FurutaPendulumNLOC<FPSystem>;
class MPCSimulator : public ct::core::ControlSimulator<FPSystem>
{
public:
  MPCSimulator(
    ct::core::Time sim_dt, ct::core::Time control_dt, const PendulumState & x0,
    std::shared_ptr<FPSystem> ip_system,
    ct::optcon::MPC<ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>> & mpc)
  : ct::core::ControlSimulator<FPSystem>(sim_dt, control_dt, x0.toStateVector(), ip_system),
    mpc_(mpc)
  {
    controller_.reset(new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);
  }

  void finishSystemIteration(ct::core::Time) override
  {
    control_mtx_.lock();
    system_->setController(controller_);
    control_mtx_.unlock();
  }

  void prepareControllerIteration(ct::core::Time sim_time) override
  {
    mpc_.prepareIteration(sim_time);
  }
  void finishControllerIteration(ct::core::Time sim_time) override
  {
    state_mtx_.lock();
    ct::core::StateVector<STATE_DIM> x_temp = x_;
    state_mtx_.unlock();

    std::shared_ptr<ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>> new_controller(
      new ct::core::StateFeedbackController<STATE_DIM, CONTROL_DIM>);

    bool success = mpc_.finishIteration(x_temp, sim_time, *new_controller, controller_ts_);

    if (!success) throw std::runtime_error("Failed to finish MPC iteration.");

    control_mtx_.lock();
    controller_ = new_controller;
    control_mtx_.unlock();
  }

private:
  ct::optcon::MPC<ct::optcon::NLOptConSolver<STATE_DIM, CONTROL_DIM>> & mpc_;
  ct::core::Time controller_ts_;
};

int main()
{
  const bool verbose = true;
  std::string config_file = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                              "furuta_pendulum_control_toolbox")) /
                            "config" / "nloc_config.info";
  try {
    std::unique_ptr<
      ct::optcon::MPC<ct::optcon::NLOptConSolver<FPSystem::STATE_DIM, FPSystem::CONTROL_DIM>>>
      ilqr_mpc = furuta_pendulum_control_toolbox::CreateMPCController(config_file, verbose);

    ct::core::Time sim_dt;
    ct::core::loadScalar(config_file, "sim_dt", sim_dt);

    ct::core::Time control_dt;
    ct::core::loadScalar(config_file, "control_dt", control_dt);

    double simulation_time;
    ct::core::loadScalar(config_file, "simulation_time", simulation_time);

    PendulumState x0;
    PendulumState::state_vector_t x0temp;
    ct::core::loadMatrix(config_file, "x_0", x0temp);
    x0.fromStateVector(x0temp);

    std::shared_ptr<FPSystem> fp_system(new FPSystem());

    MPCSimulator mpc_sim(sim_dt, control_dt, x0, fp_system, *ilqr_mpc);
    std::cout << "simulating 3 seconds" << std::endl;
    mpc_sim.simulate(simulation_time);
    mpc_sim.finish();

    ilqr_mpc->printMpcSummary();

  } catch (std::runtime_error & e) {
    std::cout << "Exception caught: " << e.what() << std::endl;
  }
}
