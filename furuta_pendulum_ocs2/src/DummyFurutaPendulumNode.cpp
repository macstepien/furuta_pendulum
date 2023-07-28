// Based on ocs2 ballbot example

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <ocs2_mpc/SystemObservation.h>

#include <furuta_pendulum_ocs2/FurutaPendulumInterface.h>
#include <furuta_pendulum_ocs2/definitions.h>

namespace furuta_pendulum_ocs2
{
class FurutaPendulumOCS2 : public rclcpp::Node
{
public:
  FurutaPendulumOCS2(const rclcpp::NodeOptions & options)
  : Node("furuta_pendulum_ocs2_node", options)
  {
    dt_ = 0.001;
    simulation_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&FurutaPendulumOCS2::Simulate, this));

    const std::string robotName = "furuta_pendulum";

    std::string taskFile =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "config" / "mpc" / "task.info";
    std::string libraryFolder =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "auto_generated";

    ocs2::furuta_pendulum::FurutaPendulumInterface furutaPendulumInterface(taskFile, libraryFolder);

    RCLCPP_INFO_STREAM(this->get_logger(), "created FP");

    // // MRT
    // ocs2::MRT_ROS_Interface mrt(robotName);
    // mrt.initRollout(&furutaPendulumInterface.getRollout());
    // mrt.launchNodes(nodeHandle);

    // // Visualization
    // auto ballbotDummyVisualization =
    //   std::make_shared<ocs2::furuta_pendulum::FurutaPendulumDummyVisualization>(nodeHandle);

    // // Dummy ballbot
    // ocs2::MRT_ROS_Dummy_Loop dummyFurutaPendulum(
    //   mrt, furutaPendulumInterface.mpcSettings().mrtDesiredFrequency_,
    //   furutaPendulumInterface.mpcSettings().mpcDesiredFrequency_);
    // dummyFurutaPendulum.subscribeObservers({ballbotDummyVisualization});

    // initial state
    ocs2::SystemObservation initObservation;
    initObservation.state = furutaPendulumInterface.getInitialState();
    initObservation.input.setZero(ocs2::furuta_pendulum::INPUT_DIM);
    initObservation.time = 0.0;

    // initial command
    const ocs2::TargetTrajectories initTargetTrajectories(
      {initObservation.time}, {initObservation.state}, {initObservation.input});
  }

private:
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  double dt_;

  void Simulate()
  {
    // Run dummy (loops while ros is ok)
    // dummyFurutaPendulum.run(initObservation, initTargetTrajectories);
  }
};

}  // namespace furuta_pendulum_ocs2

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_ocs2::FurutaPendulumOCS2)