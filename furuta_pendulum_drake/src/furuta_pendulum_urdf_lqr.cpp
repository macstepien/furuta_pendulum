#include <drake/systems/framework/event.h>
#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
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

static const char * const kFurutaPendulumUrdfPath =
  "/home/maciej/ros2_ws/src/furuta_pendulum/furuta_pendulum_drake/urdf/furuta_pendulum.urdf";

void DoMain()
{
  const double kTargetRealtimeRate = 1.0;
  const double kSimulationTime = 10.0;
  const double kMaxTimeStep = 0.001;

  systems::DiagramBuilder<double> builder;

  auto [furuta_pendulum, scene_graph] =
    drake::multibody::AddMultibodyPlantSceneGraph(&builder, kMaxTimeStep);
  drake::multibody::Parser(&furuta_pendulum, &scene_graph)
    .AddModelFromFile(kFurutaPendulumUrdfPath);

  // Now the model is complete.
  furuta_pendulum.Finalize();

  geometry::MeshcatVisualizerd::AddToBuilder(
    &builder, scene_graph, std::make_shared<geometry::Meshcat>());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double> & furuta_pendulum_context =
    diagram->GetMutableSubsystemContext(furuta_pendulum, diagram_context.get());

  // There is no input actuation in this example for the passive dynamics.
  furuta_pendulum.get_actuation_input_port().FixValue(
    &furuta_pendulum_context, Value<systems::BasicVector<double>>(Eigen::VectorXd::Zero(2)));

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_publish_every_time_step(false);
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