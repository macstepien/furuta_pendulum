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

static auto LOGGER = rclcpp::get_logger("furuta_pendulum_mpc_mrt");

/**
 * This function implements the evaluation of the MPC policy
 * @param currentObservation : current system observation {time, state, input} to compute the input for. (input can be left empty)
 * @param mpcMrtInterface : interface used for communication with the MPC optimization (running in a different thread)
 * @return system input u(t)
 */
ocs2::vector_t mpcTrackingController(
  const ocs2::SystemObservation & currentObservation, ocs2::MPC_MRT_Interface & mpcMrtInterface);

namespace furuta_pendulum_ocs2_ros
{

class OCS2MPCControllerNode : public rclcpp::Node
{
public:
  OCS2MPCControllerNode(const rclcpp::NodeOptions & options) : Node("controller_node", options)
  {
    const std::string robotName = "furuta_pendulum";

    // Robot interface
    std::string taskFile =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "config" / "mpc" / "task.info";
    std::string libraryFolder =
      std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_ocs2")) /
      "auto_generated";
    furutaPendulumInterface =
      std::make_unique<ocs2::furuta_pendulum::FurutaPendulumInterface>(taskFile, libraryFolder);

    /*
   * Set up the MPC and the MPC_MRT_interface.
   * For this example we add a command interface and a visualization which both communicate over ros
   */

    // MPC
    mpc_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
      furutaPendulumInterface->mpcSettings(), furutaPendulumInterface->ddpSettings(),
      furutaPendulumInterface->getRollout(), furutaPendulumInterface->getOptimalControlProblem(),
      furutaPendulumInterface->getInitializer());

    // Create the MPC MRT Interface
    mpcMrtInterface = std::make_unique<ocs2::MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface->initRollout(&furutaPendulumInterface->getRollout());

    /*
   * Initialize the simulation and controller
   */

    // Initial state
    ocs2::SystemObservation initObservation;
    initObservation.state = furutaPendulumInterface->getInitialState();
    initObservation.input.setZero(ocs2::furuta_pendulum::INPUT_DIM);
    initObservation.time = 0.0;

    // Initial command
    const ocs2::scalar_t initTime = 0.0;
    ocs2::vector_t goalState = furutaPendulumInterface->getInitialTarget();
    const ocs2::TargetTrajectories initTargetTrajectories(
      {initTime}, {goalState}, {ocs2::vector_t::Zero(ocs2::furuta_pendulum::INPUT_DIM)});

    // Set the first observation and command and wait for optimization to finish

    RCLCPP_INFO(this->get_logger(), "Waiting for the initial policy ...");
    mpcMrtInterface->setCurrentObservation(initObservation);
    mpcMrtInterface->getReferenceManager().setTargetTrajectories(initTargetTrajectories);
    double timeStep = 1e9 / furutaPendulumInterface->mpcSettings().mrtDesiredFrequency_;

    while (!mpcMrtInterface->initialPolicyReceived() && rclcpp::ok()) {
      mpcMrtInterface->advanceMpc();
      rclcpp::sleep_for(std::chrono::nanoseconds(int(timeStep)));
    }
    RCLCPP_INFO(this->get_logger(), "Initial policy has been received.");
    currentObservation = initObservation;
    /*
   * Launch the computation of the MPC in a separate thread.
   * This thread will be triggered at a given frequency and execute an optimization based on the latest available observation.
   */

    // mrt_timer_ = this->create_wall_timer(
    //   std::chrono::duration<double>(
    //     1. / furutaPendulumInterface->mpcSettings().mrtDesiredFrequency_),
    //   std::bind(&OCS2MPCControllerNode::MRTCalc, this));
    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&OCS2MPCControllerNode::StateCb, this, std::placeholders::_1));
    torque_cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("effort_control", 10);

    mpc_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(
        1. / furutaPendulumInterface->mpcSettings().mpcDesiredFrequency_),
      std::bind(&OCS2MPCControllerNode::MPCCalc, this));

    rclcpp::Time current_time = this->get_clock()->now();
    start_sim_time_ = current_time.nanoseconds() / 1000000000.0;

    observationPublisher =
      this->create_publisher<ocs2_msgs::msg::MPCObservation>(robotName + "_mpc_observation", 1);
  }
  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
  rclcpp::TimerBase::SharedPtr mpc_timer_;

  void MPCCalc()
  {
    RCLCPP_INFO(this->get_logger(), "Advancing MPC");
    mpcMrtInterface->advanceMpc();
  }

  rclcpp::Publisher<ocs2_msgs::msg::MPCObservation>::SharedPtr observationPublisher;

  std::unique_ptr<ocs2::MPC_MRT_Interface> mpcMrtInterface;
  std::unique_ptr<ocs2::furuta_pendulum::FurutaPendulumInterface> furutaPendulumInterface;
  std::unique_ptr<ocs2::furuta_pendulum::FurutaPendulumDummyVisualization>
    furutaPendulumDummyVisualization;

  double dtheta2_filtered_ = 0.0;
  double dtheta1_filtered_ = 0.0;
  double alpha_ = 0.0;
  double torque_multiplier_ = 0.0;
  bool initial_position_ = false;
  double initial_joint0_ = 0.0;
  double initial_joint1_ = 0.0;
  double last_input_ = 0.0;

  void StateCb(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // RCLCPP_INFO_STREAM(
    // this->get_logger(), "### Current time " << currentObservation.time);

    /*
    * State estimation would go here to fill "currentObservation".
    * In this example we receive the measurement directly after forward integration at the end of the loop.
    */
    ocs2::SystemObservation newObservation;
    // newObservation.state = furutaPendulumInterface.getInitialState();
    newObservation.state = ocs2::vector_t(4);
    newObservation.state << msg->position[0], msg->position[1], msg->velocity[0], msg->velocity[1];

    newObservation.time =
      msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0 - start_sim_time_;

    newObservation.input = ocs2::vector_t(1);
    newObservation.input << newObservation.input;

    RCLCPP_INFO_STREAM(this->get_logger(), newObservation);

    // Evaluate the control input

    // Update the current state of the system
    mpcMrtInterface->setCurrentObservation(newObservation);

    // Load the latest MPC policy
    bool policyUpdated = mpcMrtInterface->updatePolicy();
    // if (policyUpdated) {
    //   RCLCPP_INFO_STREAM(LOGGER, "<<< New MPC policy received at " << currentObservation.time);
    // }

    // Evaluate the current policy
    ocs2::vector_t optimizedState;  // Evaluation of the optimized state trajectory.
    ocs2::vector_t optimizedInput;  // Evaluation of the optimized input trajectory.
    size_t plannedMode;  // The mode that is active at the time the policy is evaluated at.

    mpcMrtInterface->evaluatePolicy(
      newObservation.time, newObservation.state, optimizedState, optimizedInput, plannedMode);

    /*
    * Sending the commands to the actuators would go here.
    * In this example, we instead do a forward simulation + visualization.
    * Simulation is done with the rollout functionality of the mpcMrtInterface, but this can be replaced by any other simulation.
    */

    std_msgs::msg::Float64MultiArray torque_cmd_msg;
    torque_cmd_msg.data.push_back(optimizedInput(0));
    torque_cmd_pub_->publish(torque_cmd_msg);
    last_input_ = optimizedInput(0);
  }

  ocs2::SystemObservation currentObservation;
  rclcpp::TimerBase::SharedPtr mrt_timer_;
  // void MRTCalc()
  // {
  //   // if (!furutaPendulumDummyVisualization) {
  //   //   // Visualization
  //   //   furutaPendulumDummyVisualization =
  //   //     std::make_unique<ocs2::furuta_pendulum::FurutaPendulumDummyVisualization>(
  //   //       this);
  //   // }

  //   // RCLCPP_INFO_STREAM(
  //   // nodeHandle->get_logger(), "### Current time " << currentObservation.time);

  //   /*
  //          * State estimation would go here to fill "currentObservation".
  //          * In this example we receive the measurement directly after forward integration at the end of the loop.
  //          */

  //   RCLCPP_INFO_STREAM(this->get_logger(), currentObservation);

  //   // Evaluate the control input
  //   const auto systemInput = mpcTrackingController(currentObservation, *mpcMrtInterface);

  //   /*
  //          * Sending the commands to the actuators would go here.
  //          * In this example, we instead do a forward simulation + visualization.
  //          * Simulation is done with the rollout functionality of the mpcMrtInterface, but this can be replaced by any other simulation.
  //          */

  //   // Forward simulation
  //   const auto dt = 1.0 / furutaPendulumInterface->mpcSettings().mrtDesiredFrequency_;
  //   ocs2::SystemObservation nextObservation;
  //   nextObservation.time = currentObservation.time + dt;
  //   mpcMrtInterface->rolloutPolicy(
  //     currentObservation.time, currentObservation.state, dt, nextObservation.state,
  //     nextObservation.input, nextObservation.mode);

  //   // "state estimation"
  //   currentObservation = nextObservation;

  //   // Visualization
  //   // furutaPendulumDummyVisualization->update(
  //   //   currentObservation, mpcMrtInterface->getPolicy(), mpcMrtInterface->getCommand());

  //   // Publish the observation. Only needed for the command interface
  //   observationPublisher->publish(
  //     ocs2::ros_msg_conversions::createObservationMsg(currentObservation));
  // }

private:
  double start_sim_time_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_cmd_pub_;
};

}  // namespace furuta_pendulum_ocs2_ros

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_ocs2_ros::OCS2MPCControllerNode)
