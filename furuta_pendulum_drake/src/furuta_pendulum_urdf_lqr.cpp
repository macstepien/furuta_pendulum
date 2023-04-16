#include <iostream>
#include <chrono>
#include <thread>

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

namespace drake
{
namespace examples
{
namespace multibody
{
namespace furuta_pendulum
{
namespace
{

template <typename T>
class FurutaPendulumSwingUpController : public systems::LeafSystem<T>
{
public:
  explicit FurutaPendulumSwingUpController(const systems::BasicVector<T> & params)
  {
    this->DeclareVectorInputPort(systems::kUseDefaultName, systems::BasicVector<T>(4));
    this->DeclareVectorInputPort(systems::kUseDefaultName, systems::BasicVector<T>(1));
    this->DeclareVectorOutputPort(
      systems::kUseDefaultName, systems::BasicVector<T>(1),
      &FurutaPendulumSwingUpController::CalcTau);
    this->DeclareNumericParameter(params);
  }

private:
  void CalcTau(const systems::Context<T> & context, systems::BasicVector<T> * output) const
  {
    const auto * state = this->template EvalVectorInput<systems::BasicVector>(context, 0);
    const auto * lqr_output = this->template EvalVectorInput<systems::BasicVector>(context, 1);
    const systems::BasicVector<T> & params =
      this->template GetNumericParameter<systems::BasicVector>(context, 0);

    // const T theta2 = fmod(state[0][1], 2.0 * M_PI) + 2.0 * M_PI - M_PI;
    const T theta2 = state[0][1] - M_PI;
    const T u_max = params[3];

    if (fabs(theta2) < 0.7) {
      // std::cout << "lqr" << std::endl;
      // output[0][0] = std::clamp(lqr_output[0][0], -u_max, u_max);
      output[0][0] = lqr_output[0][0];
      // output[0][1] = lqr_output[0][1];
      // output[0][0] = 0.0;
      output[0][1] = 0.0;
    } else {
      // std::cout << "swingup" << std::endl;
      // std::cout << "swingup: " << fabs(theta2 - M_PI) << std::endl;

      using std::pow;

      const T g = params[0];
      const T m2 = params[1];
      const T l2 = params[2];

      const T dtheta2 = state[0][3];
      const T E = 0.5 * m2 * pow(l2, 2) * pow(dtheta2, 2) + m2 * g * l2 * cos(theta2);
      const T E0 = m2 * g * l2;

      double x = (E - E0) * dtheta2 * cos(theta2);
      if (E < E0) {
        if (x > 0.0) {
          output[0][0] = -u_max;
        } else {
          output[0][0] = u_max;
        }
      } else {
        output[0][0] = 0.05;
      }

      output[0][1] = 0.0;
    }
  }
};

static const char * const kFurutaPendulumUrdfPath =
  "/home/maciej/ros2_ws/src/furuta_pendulum/furuta_pendulum_drake/urdf/furuta_pendulum.urdf";

void DoMain()
{
  const double kTargetRealtimeRate = 1.0;
  const double kSimulationTime = 50.0;
  const double kMaxTimeStep = 0.001;

  systems::BasicVector<double> params(Eigen::Vector4d({9.80665, 0.075, 0.148, 1.0}));

  systems::DiagramBuilder<double> builder;

  auto [furuta_pendulum, scene_graph] =
    drake::multibody::AddMultibodyPlantSceneGraph(&builder, kMaxTimeStep);
  drake::multibody::Parser(&furuta_pendulum, &scene_graph)
    .AddModelFromFile(kFurutaPendulumUrdfPath);

  // auto & joint1 = furuta_pendulum.GetMutableJointByName<drake::multibody::RevoluteJoint>("joint1");
  // auto & joint2 = furuta_pendulum.GetMutableJointByName<drake::multibody::RevoluteJoint>("joint2");

  // double max_position = 10.0;
  // joint1.set_position_limits(Vector1d(-max_position), Vector1d(max_position));
  // joint2.set_position_limits(Vector1d(-max_position), Vector1d(max_position));

  // double max_velocity = 1.0;
  // joint1.set_velocity_limits(Vector1d(-max_velocity), Vector1d(max_velocity));
  // joint2.set_velocity_limits(Vector1d(-max_velocity), Vector1d(max_velocity));

  // double max_acceleration = 1.0;
  // joint1.set_acceleration_limits(Vector1d(-max_acceleration), Vector1d(max_acceleration));
  // joint2.set_acceleration_limits(Vector1d(-max_acceleration), Vector1d(max_acceleration));

  // Now the model is complete.
  furuta_pendulum.Finalize();

  // ---------------

  // Create LQR Controller.
  auto furuta_pendulum_context = furuta_pendulum.CreateDefaultContext();
  const int furuta_pendulum_actuation_port = 3;
  // Set nominal torque to zero.
  furuta_pendulum_context->FixInputPort(
    furuta_pendulum_actuation_port, Value<systems::BasicVector<double>>(Eigen::VectorXd::Zero(1)));
  // furuta_pendulum.get_actuation_input_port().FixValue(furuta_pendulum_context.get(), 0.0);

  // Set nominal state to the upright fixed point.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
  x0[0] = 0.0;
  x0[1] = M_PI;
  furuta_pendulum_context->SetDiscreteState(x0);

  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
  Q(0, 0) = 0.0;
  Q(1, 1) = 10.0;
  Q(2, 2) = 1.0;
  Q(3, 3) = 1.0;
  // Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);
  Vector1d R = Vector1d::Constant(1.0);
  Eigen::MatrixXd N;

  auto lqr = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(
    furuta_pendulum, *furuta_pendulum_context, Q, R, N, furuta_pendulum_actuation_port));

  std::cout << "D: " << lqr->D() << std::endl;
  std::cout << "y0: " << lqr->y0() << std::endl;

  // -------------

  // Swing up
  auto controller = builder.AddSystem<FurutaPendulumSwingUpController>(params);
  controller->set_name("controller");

  // ----------------

  builder.Connect(furuta_pendulum.get_state_output_port(), lqr->get_input_port());
  builder.Connect(furuta_pendulum.get_state_output_port(), controller->get_input_port(0));

  builder.Connect(lqr->get_output_port(), controller->get_input_port(1));
  builder.Connect(controller->get_output_port(), furuta_pendulum.get_actuation_input_port());

  // builder.Connect(furuta_pendulum.get_state_output_port(), lqr->get_input_port());
  // builder.Connect(lqr->get_output_port(), furuta_pendulum.get_actuation_input_port());

  geometry::MeshcatVisualizerd::AddToBuilder(
    &builder, scene_graph, std::make_shared<geometry::Meshcat>());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double> & furuta_pendulum_context_2 =
    diagram->GetMutableSubsystemContext(furuta_pendulum, diagram_context.get());

  Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
  positions[0] = 0.0;
  positions[1] = 0.0;
  furuta_pendulum.SetPositions(&furuta_pendulum_context_2, positions);
  furuta_pendulum.SetVelocities(&furuta_pendulum_context_2, positions);

  std::this_thread::sleep_for(std::chrono::duration<double>(5.0));
  std::cout << "Starting simulation" << std::endl;
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(kTargetRealtimeRate);
  simulator.Initialize();
  simulator.AdvanceTo(kSimulationTime);
}

}  // namespace
}  // namespace furuta_pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main()
{
  drake::examples::multibody::furuta_pendulum::DoMain();
  return 0;
}