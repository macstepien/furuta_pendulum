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
#include <furuta_pendulum/FurutaPendulumMPC.h>

using PendulumState = ct::rbd::FixBaseRobotState<ct::rbd::FurutaPendulum::Kinematics::NJOINTS>;
using FPSystem = ct::rbd::FurutaPendulumSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>;
using FurutaPendulumNLOCSystem = ct::rbd::FurutaPendulumNLOC<FPSystem>;

int main(int argc, char * argv[])
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

    double simulation_time;
    ct::core::loadScalar(config_file, "simulation_time", simulation_time);

    PendulumState x0;
    PendulumState::state_vector_t x0temp;
    ct::core::loadMatrix(config_file, "x_0", x0temp);
    x0.fromStateVector(x0temp);

    std::shared_ptr<FPSystem> fp_system(new FPSystem());

    std::cout << "simulating 3 seconds" << std::endl;

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<ct::core::FurutaPendulumControlSimulator<FPSystem>>(
      sim_dt, x0.toStateVector(), fp_system, *ilqr_mpc));

    rclcpp::shutdown();

    ilqr_mpc->printMpcSummary();

  } catch (std::runtime_error & e) {
    std::cout << "Exception caught: " << e.what() << std::endl;
  }
}
