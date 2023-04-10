// Based on https://drake.guzhaoyuan.com/drake-controllers/lqr-on-cart-pole

///
/// This file use a fully actuated cart pole model to track a specific state
///

#include <drake/systems/framework/event.h>
#include <drake/common/drake_assert.h>
#include <drake/common/find_resource.h>
#include <drake/geometry/drake_visualizer.h>
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
namespace cart_pole
{
namespace
{

// Fixed path to double pendulum SDF model.
static const char * const kCartPoleSdfPath = "drake/examples/multibody/cart_pole/cart_pole.sdf";

//
// Main function for demo.
//
void DoMain()
{
  const double kTargetRealtimeRate = 1.0;
  const double kSimulationTime = 10.0;
  const double kMaxTimeStep = 0.001;

  systems::DiagramBuilder<double> builder;

  geometry::SceneGraph<double> & scene_graph = *builder.AddSystem<geometry::SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Load and parse double pendulum SDF from file into a tree.
  drake::multibody::MultibodyPlant<double> * cp =
    builder.AddSystem<drake::multibody::MultibodyPlant<double>>(kMaxTimeStep);
  cp->set_name("cart_pole");
  cp->RegisterAsSourceForSceneGraph(&scene_graph);

  drake::multibody::Parser parser(cp);
  const std::string sdf_path = FindResourceOrThrow(kCartPoleSdfPath);
  drake::multibody::ModelInstanceIndex plant_model_instance_index =
    parser.AddModelFromFile(sdf_path);
  (void)plant_model_instance_index;

  // Now the plant is complete.
  cp->Finalize();

  // Create LQR Controller.
  auto cp_context = cp->CreateDefaultContext();
  const int CartPole_actuation_port = 3;
  // Set nominal torque to zero.
  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(1);
  cp_context->FixInputPort(CartPole_actuation_port, Value<systems::BasicVector<double>>(u0));

  // Set nominal state to the upright fixed point.
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(4);
  x0[0] = 1;
  x0[1] = M_PI;
  cp_context->SetDiscreteState(x0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(4, 4);
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1, 1);
  Eigen::MatrixXd N;
  auto lqr = builder.AddSystem(systems::controllers::LinearQuadraticRegulator(
    *cp, *cp_context, Q, R, N, CartPole_actuation_port));

  builder.Connect(cp->get_state_output_port(), lqr->get_input_port());
  builder.Connect(lqr->get_output_port(), cp->get_actuation_input_port());

  // Connect plant with scene_graph to get collision information.
  DRAKE_DEMAND(!!cp->get_source_id());
  builder.Connect(
    cp->get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(cp->get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(), cp->get_geometry_query_input_port());

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> diagram_context = diagram->CreateDefaultContext();

  // Create plant_context to set velocity.
  systems::Context<double> & plant_context =
    diagram->GetMutableSubsystemContext(*cp, diagram_context.get());
  // Set init position.
  Eigen::VectorXd positions = Eigen::VectorXd::Zero(2);
  positions[0] = 0.0;
  positions[1] = 0.0;
  cp->SetPositions(&plant_context, positions);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(kTargetRealtimeRate);
  simulator.Initialize();
  simulator.AdvanceTo(kSimulationTime);
}

}  // namespace
}  // namespace cart_pole
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main()
{
  drake::examples::multibody::cart_pole::DoMain();
  return 0;
}