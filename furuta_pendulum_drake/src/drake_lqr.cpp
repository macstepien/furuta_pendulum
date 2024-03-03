#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <drake/systems/framework/event.h>
#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/controllers/linear_quadratic_regulator.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/framework_common.h>
#include <drake/systems/primitives/affine_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/linear_system.h>

#include <yaml-cpp/yaml.h>

namespace furuta_pendulum_drake
{

template <typename T>
class FurutaPendulumSwingUpController : public drake::systems::LeafSystem<T>
{
public:
  explicit FurutaPendulumSwingUpController(const drake::systems::BasicVector<T> & params)
  {
    this->DeclareVectorInputPort(
      drake::systems::kUseDefaultName, drake::systems::BasicVector<T>(4));
    this->DeclareVectorInputPort(
      drake::systems::kUseDefaultName, drake::systems::BasicVector<T>(1));
    this->DeclareVectorOutputPort(
      drake::systems::kUseDefaultName, drake::systems::BasicVector<T>(1),
      &FurutaPendulumSwingUpController::CalcTau);
    this->DeclareNumericParameter(params);
  }

private:
  void CalcTau(
    const drake::systems::Context<T> & context, drake::systems::BasicVector<T> * output) const
  {
    const auto * state = this->template EvalVectorInput<drake::systems::BasicVector>(context, 0);
    const auto * lqr_output =
      this->template EvalVectorInput<drake::systems::BasicVector>(context, 1);
    const drake::systems::BasicVector<T> & params =
      this->template GetNumericParameter<drake::systems::BasicVector>(context, 0);

    const T theta2 = state[0][1] - M_PI;
    const T u_max = params[3];

    if (fabs(theta2) < 0.7) {
      output[0][0] = lqr_output[0][0];
      output[0][1] = 0.0;
    } else {
      const T g = params[0];
      const T m2 = params[1];
      const T l2 = params[2];

      const T dtheta2 = state[0][3];
      const T E = 0.5 * m2 * std::pow(l2, 2) * std::pow(dtheta2, 2) + m2 * g * l2 * cos(theta2);
      const T E0 = m2 * g * l2;

      double x = (E - E0) * dtheta2 * cos(theta2);

      if (x > 0.0) {
        output[0][0] = -u_max;
      } else {
        output[0][0] = u_max;
      }

      output[0][1] = 0.0;
    }
  }
};

}  // namespace furuta_pendulum_drake

int main()
{
  std::string furuta_pendulum_urdf_path =
    std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_drake")) /
    "urdf" / "furuta_pendulum.urdf";

  std::string params_path =
    std::filesystem::path(ament_index_cpp::get_package_share_directory("furuta_pendulum_drake")) /
    "config" / "params.yaml";
  YAML::Node config = YAML::LoadFile(params_path);

  const double kTargetRealtimeRate = 1.0;
  const double kSimulationTime = 50.0;
  const double kMaxTimeStep = 0.002;

  const double g = 9.80665;
  const double m2 = 0.075;
  const double l2 = 0.148;
  const double u_max = 1.0;
  drake::systems::BasicVector<double> params(Eigen::Vector4d({g, m2, l2, u_max}));

  drake::systems::DiagramBuilder<double> builder;

  auto [furuta_pendulum, scene_graph] =
    drake::multibody::AddMultibodyPlantSceneGraph(&builder, kMaxTimeStep);
  drake::multibody::Parser(&furuta_pendulum, &scene_graph)
    .AddModelFromFile(furuta_pendulum_urdf_path);

  // auto & joint0 = furuta_pendulum.GetMutableJointByName<drake::multibody::RevoluteJoint>("joint0");
  // auto & joint1 = furuta_pendulum.GetMutableJointByName<drake::multibody::RevoluteJoint>("joint1");

  // double max_position = 10.0;
  // joint0.set_position_limits(Vector1d(-max_position), Vector1d(max_position));
  // joint1.set_position_limits(Vector1d(-max_position), Vector1d(max_position));

  // double max_velocity = 1.0;
  // joint0.set_velocity_limits(Vector1d(-max_velocity), Vector1d(max_velocity));
  // joint1.set_velocity_limits(Vector1d(-max_velocity), Vector1d(max_velocity));

  // double max_acceleration = 1.0;
  // joint0.set_acceleration_limits(Vector1d(-max_acceleration), Vector1d(max_acceleration));
  // joint1.set_acceleration_limits(Vector1d(-max_acceleration), Vector1d(max_acceleration));

  // Now the model is complete.
  furuta_pendulum.Finalize();

  // ---------------

  // Create LQR Controller.
  auto furuta_pendulum_context = furuta_pendulum.CreateDefaultContext();
  const int furuta_pendulum_actuation_port = 3;
  // Set nominal torque to zero.
  furuta_pendulum_context->FixInputPort(
    furuta_pendulum_actuation_port,
    drake::Value<drake::systems::BasicVector<double>>(Eigen::VectorXd::Zero(1)));
  // furuta_pendulum.get_actuation_input_port().FixValue(furuta_pendulum_context.get(), 0.0);

  // Set nominal state to the upright fixed point.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
  x0[0] = 0.0;
  x0[1] = M_PI;
  furuta_pendulum_context->SetDiscreteState(x0);

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
  std::cout << config["Q0"] << " " << config["Q1"] << " " << config["Q2"] << " " << config["Q3"]
            << std::endl;
  Q(0, 0) = config["Q0"].as<double>();
  Q(1, 1) = config["Q1"].as<double>();
  Q(2, 2) = config["Q2"].as<double>();
  Q(3, 3) = config["Q3"].as<double>();
  // Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
  drake::Vector1d R = drake::Vector1d::Constant(config["R0"].as<double>());
  Eigen::MatrixXd N;

  auto lqr = builder.AddSystem(drake::systems::controllers::LinearQuadraticRegulator(
    furuta_pendulum, *furuta_pendulum_context, Q, R, N, furuta_pendulum_actuation_port));

  // std::cout << "D: " << lqr->D() << std::endl;
  // Print out in form used later
  std::cout << "K: [" << -lqr->D()(0) << ", " << -lqr->D()(1) << ", " << -lqr->D()(2) << ", "
            << -lqr->D()(3) << "]" << std::endl;
  std::cout << "y0: " << lqr->y0() << std::endl;

  // -------------

  // Swing up
  auto controller =
    builder.AddSystem<furuta_pendulum_drake::FurutaPendulumSwingUpController>(params);
  controller->set_name("controller");

  // ----------------

  builder.Connect(furuta_pendulum.get_state_output_port(), lqr->get_input_port());
  builder.Connect(furuta_pendulum.get_state_output_port(), controller->get_input_port(0));

  builder.Connect(lqr->get_output_port(), controller->get_input_port(1));
  builder.Connect(controller->get_output_port(), furuta_pendulum.get_actuation_input_port());

  // builder.Connect(furuta_pendulum.get_state_output_port(), lqr->get_input_port());
  // builder.Connect(lqr->get_output_port(), furuta_pendulum.get_actuation_input_port());

  drake::geometry::MeshcatVisualizerd::AddToBuilder(
    &builder, scene_graph, std::make_shared<drake::geometry::Meshcat>());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
    diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  drake::systems::Context<double> & furuta_pendulum_context_2 =
    diagram->GetMutableSubsystemContext(furuta_pendulum, diagram_context.get());

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
  positions[0] = 0.0;
  positions[1] = 0.0;
  furuta_pendulum.SetPositions(&furuta_pendulum_context_2, positions);
  furuta_pendulum.SetVelocities(&furuta_pendulum_context_2, positions);

  std::this_thread::sleep_for(std::chrono::duration<double>(5.0));
  std::cout << "Starting simulation" << std::endl;
  drake::systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(kTargetRealtimeRate);
  simulator.Initialize();
  simulator.AdvanceTo(kSimulationTime);

  return 0;
}