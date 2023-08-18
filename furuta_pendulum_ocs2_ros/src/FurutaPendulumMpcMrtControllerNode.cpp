#include <memory>

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <furuta_pendulum_ocs2/FurutaPendulumInterface.h>

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <furuta_pendulum_ocs2_ros/FurutaPendulumDummyVisualization.h>

namespace furuta_pendulum_ocs2_ros
{

class OCS2MPCControllerNode : public rclcpp::Node
{
public:
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  OCS2MPCControllerNode() : Node("controller_node")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Starting");
    // InitializeController();

    // cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options;
    options.callback_group = cb_group_;

    // state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    //   "joint_states", 1, std::bind(&OCS2MPCControllerNode::StateCb, this, std::placeholders::_1),
    //   options);

    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&OCS2MPCControllerNode::StateCb, this, std::placeholders::_1));

    torque_cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_control", 10);
  }

  void InitializeController(ocs2::SystemObservation initial_observation)
  {
    // Robot interface
    std::string task_file =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "config" / "mpc" / "task.info";
    std::string library_folder =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "auto_generated";
    furuta_pendulum_interface_ =
      std::make_unique<ocs2::furuta_pendulum::FurutaPendulumInterface>(task_file, library_folder);

    /*
   * Set up the MPC and the MPC_MRT_interface.
   * For this example we add a command interface and a visualization which both communicate over ros
   */

    // MPC
    mpc_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
      furuta_pendulum_interface_->mpcSettings(), furuta_pendulum_interface_->ddpSettings(),
      furuta_pendulum_interface_->getRollout(),
      furuta_pendulum_interface_->getOptimalControlProblem(),
      furuta_pendulum_interface_->getInitializer());

    // Create the MPC MRT Interface
    mpc_mrt_interface_ = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_);
    mpc_mrt_interface_->initRollout(&furuta_pendulum_interface_->getRollout());

    // Initial command
    const ocs2::TargetTrajectories init_target_trajectory(
      {initial_observation.time}, {furuta_pendulum_interface_->getInitialTarget()},
      {ocs2::vector_t::Zero(ocs2::furuta_pendulum::INPUT_DIM)});

    // Set the first observation and command and wait for optimization to finish
    RCLCPP_INFO(this->get_logger(), "Waiting for the initial policy ...");
    mpc_mrt_interface_->setCurrentObservation(initial_observation);
    mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(init_target_trajectory);
    double time_step = 1e9 / furuta_pendulum_interface_->mpcSettings().mrtDesiredFrequency_;

    while (!mpc_mrt_interface_->initialPolicyReceived() && rclcpp::ok()) {
      mpc_mrt_interface_->advanceMpc();
      rclcpp::sleep_for(std::chrono::nanoseconds(int(time_step)));
    }
    RCLCPP_INFO(this->get_logger(), "Initial policy has been received.");

    // mrt_timer_ = this->create_wall_timer(
    //   std::chrono::duration<double>(
    //     1. / furuta_pendulum_interface_->mpcSettings().mrtDesiredFrequency_),
    //   std::bind(&OCS2MPCControllerNode::MRTCalc, this));

    /*
   * Launch the computation of the MPC in a separate thread.
   * This thread will be triggered at a given frequency and execute an optimization based on the latest available observation.
   */
    mpc_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(
        1. / furuta_pendulum_interface_->mpcSettings().mpcDesiredFrequency_),
      std::bind(&OCS2MPCControllerNode::MPCCalc, this), cb_group_);

    rclcpp::Time current_time = this->get_clock()->now();
    start_time_ = current_time.nanoseconds() / 1000000000.0;
  }

  bool first_mpc_call_ = true;

  void MPCCalc()
  {
    RCLCPP_INFO(this->get_logger(), "Advancing MPC");
    if (first_mpc_call_) {
      first_mpc_call_ = false;
      return;
    }
    try {
      mpc_mrt_interface_->advanceMpc();
    } catch (std::runtime_error & err) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Advance MPC exception caught: " << err.what());
      mpc_error_ = true;
    }
  }
  bool mpc_error_ = false;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Running state cb");

    if (!controller_initialized_) {
      start_sim_time_ = msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0;
    }

    // State estimation
    ocs2::SystemObservation new_observation;
    new_observation.state = ocs2::vector_t(4);
    new_observation.state << msg->position[0], msg->position[1], msg->velocity[0], msg->velocity[1];

    new_observation.time = msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0;
    // - start_sim_time_;

    new_observation.input = ocs2::vector_t(1);
    new_observation.input(0) = last_input_;

    if (!controller_initialized_) {
      InitializeController(new_observation);
      controller_initialized_ = true;
      return;
    }

    // Update the current state of the system
    mpc_mrt_interface_->setCurrentObservation(new_observation);

    // Load the latest MPC policy
    bool policy_updated = mpc_mrt_interface_->updatePolicy();
    // if (policy_updated) {
    //   RCLCPP_INFO_STREAM(LOGGER, "<<< New MPC policy received at " << currentObservation.time);
    // }

    // Evaluate the current policy
    ocs2::vector_t optimized_state;
    ocs2::vector_t optimized_input;
    size_t planned_mode;

    rclcpp::Time current_time = this->get_clock()->now();
    ocs2::scalar_t current_time_scalar = current_time.nanoseconds() / 1000000000.0 - start_time_;
    // mpc_mrt_interface_->evaluatePolicy(
    //   new_observation.time, new_observation.state, optimized_state, optimized_input, planned_mode);
    mpc_mrt_interface_->evaluatePolicy(
      current_time_scalar, new_observation.state, optimized_state, optimized_input, planned_mode);

    // Send the commands to the actuators
    std_msgs::msg::Float64MultiArray torque_cmd_msg;
    if (mpc_error_) {
      RCLCPP_INFO_STREAM(this->get_logger(), "MPC error, sending 0 command");
      torque_cmd_msg.data.push_back(0.0);
    } else {
      torque_cmd_msg.data.push_back(optimized_input(0));
    }
    torque_cmd_pub_->publish(torque_cmd_msg);
    last_input_ = optimized_input(0);

    // RCLCPP_INFO_STREAM(this->get_logger(), "Finished running state cb");
  }

private:
  double start_sim_time_ = 0.0;
  double start_time_ = 0.0;
  double last_input_ = 0.0;
  bool controller_initialized_ = false;

  std::unique_ptr<ocs2::furuta_pendulum::FurutaPendulumInterface> furuta_pendulum_interface_;
  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mpc_mrt_interface_;

  // rclcpp::TimerBase::SharedPtr mrt_timer_;
  rclcpp::TimerBase::SharedPtr mpc_timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;
};

}  // namespace furuta_pendulum_ocs2_ros

// Register the component with class_loader
// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_ocs2_ros::OCS2MPCControllerNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::cerr << "Starting";
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  rclcpp::Node::SharedPtr node =
    std::make_shared<furuta_pendulum_ocs2_ros::OCS2MPCControllerNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
