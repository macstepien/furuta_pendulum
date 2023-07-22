#include <ct/rbd/rbd.h>
#include <ct/core/plot/plot.h>

#include <furuta_pendulum/FurutaPendulum.h>

#include <chrono>
#include <memory>
#include <algorithm>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace furuta_pendulum_control_toolbox
{
class CtSimulationNode : public rclcpp::Node
{
public:
  CtSimulationNode(const rclcpp::NodeOptions & options)
  : Node("furuta_pendulum_simulation_node", options)
  {
    state_.setZero();

    this->declare_parameter("initial_conditions.theta1", 0.0);
    this->declare_parameter("initial_conditions.theta2", 0.0);
    this->declare_parameter("initial_conditions.dtheta1", 0.0);
    this->declare_parameter("initial_conditions.dtheta2", 0.0);
    state_(0) = this->get_parameter("initial_conditions.theta1").as_double();
    state_(1) = this->get_parameter("initial_conditions.theta2").as_double();
    state_(2) = this->get_parameter("initial_conditions.dtheta1").as_double();
    state_(3) = this->get_parameter("initial_conditions.dtheta2").as_double();

    this->declare_parameter("simulation_dt", rclcpp::PARAMETER_DOUBLE);

    try {
      dt_ = this->get_parameter("simulation_dt").as_double();
    } catch (const rclcpp::exceptions::ParameterUninitializedException & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Required parameter not defined: " << e.what());
      throw e;
    }

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    simulation_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&CtSimulationNode::Simulate, this));

    // obtain the state dimension

    // create an instance of the system
    dynamics_ =
      std::make_shared<ct::rbd::FixBaseFDSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>>();

    integrator_ = std::make_unique<ct::core::Integrator<STATE_DIM>>(dynamics_, ct::core::RK4);
  }

private:
  static const size_t STATE_DIM =
    ct::rbd::FixBaseFDSystem<ct::rbd::FurutaPendulum::tpl::Dynamics<double>>::STATE_DIM;

  std::unique_ptr<ct::core::Integrator<STATE_DIM>> integrator_;
  std::shared_ptr<ct::core::System<STATE_DIM>> dynamics_;
  ct::core::StateVector<STATE_DIM> state_;

  double dt_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;

  void Simulate()
  {
    integrator_->integrate_n_steps(state_, 0, 1, 0.002);
    PublishJointStates();
  }

  void PublishJointStates()
  {
    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();

    joint_state_msg.name.push_back("joint1");
    joint_state_msg.position.push_back(state_(0));
    joint_state_msg.velocity.push_back(state_(2));

    joint_state_msg.name.push_back("joint2");
    joint_state_msg.position.push_back(state_(1));
    joint_state_msg.velocity.push_back(state_(3));

    joint_state_pub_->publish(joint_state_msg);
  }
};

}  // namespace furuta_pendulum_control_toolbox

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(furuta_pendulum_control_toolbox::CtSimulationNode)
