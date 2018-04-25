// Implements a passive simulation of the Drake-compatible description of the
// PR2 robot. There is no controller, but the contact parameters and integrator
// parameters are set to support reliable gripping of objects if a controller
// is added.

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/semi_explicit_euler_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace cubli {

int DoMain() {
  // Declare the diagram builder and lcm.
  systems::DiagramBuilder<double> diagram_builder;
  drake::lcm::DrakeLcm lcm;

  // Construct the tree for the PR2.
  auto tree_ = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
          FindResourceOrThrow("drake/examples/cubli/Cubli.urdf"),
          multibody::joints::
          kFixed /* our PR2 model moves with actuators, not a floating base */,
          nullptr /* weld to frame */, tree_.get());

  DRAKE_ASSERT(tree_->get_num_positions() + tree_->get_num_velocities() == 14);
  int num_states = 14;

  // Set the plant for the PR2.
  auto plant_ = diagram_builder.AddSystem<systems::RigidBodyPlant<double>>(
          std::move(tree_));
  plant_->set_name("plant_");

  // Send the PR2's actuators zeros in abscence of a controller.
  auto constant_zero_source =
      diagram_builder.AddSystem<systems::ConstantVectorSource<double>>(
          VectorX<double>::Zero(plant_->actuator_command_input_port().size()));
  diagram_builder.Connect(constant_zero_source->get_output_port(),
                          plant_->actuator_command_input_port());

  // Add a visualizer.
  systems::DrakeVisualizer& visualizer_publisher =
          *diagram_builder.template AddSystem<systems::DrakeVisualizer>(
                  plant_->get_rigid_body_tree(), &lcm);
  visualizer_publisher.set_name("visualizer_publisher");
  diagram_builder.Connect(plant_->state_output_port(),
                          visualizer_publisher.get_input_port(0));

  // Set contact parameters that support gripping.
  const double kYoungsModulus = 1e9;  // Pa
  const double kDissipation = 1.0;  // s/m
  const double kStaticFriction = 1.0;
  const double kDynamicFriction = 5e-1;
  systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(kYoungsModulus)
      .set_dissipation(kDissipation)
      .set_friction(kStaticFriction, kDynamicFriction);
  plant_->set_default_compliant_material(default_material);

  const double kStictionSlipTolerance = 1e-3;  // m/s
  const double kContactRadius = 2e-4;  // m
  systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = kContactRadius;
  model_parameters.v_stiction_tolerance = kStictionSlipTolerance;
  plant_->set_contact_model_parameters(model_parameters);

  // Create the simulator.
  std::unique_ptr<systems::Diagram<double>> diagram = diagram_builder.Build();
  systems::Simulator<double> simulator(*diagram);

  // Reset the integrator with parameters that support stable gripping, given
  // the contact parameters.
  systems::Context<double>& context = simulator.get_mutable_context();
  const double max_step_size = 1e-4;
  simulator.reset_integrator<systems::SemiExplicitEulerIntegrator<double>>(
          *diagram, max_step_size, &context);

  // Set the initial joint positions to be something more interesting. Note that
  // the joint position order is the same as the order you get when you read the
  // URDF from top to bottom.
  Eigen::VectorXd initial_state(num_states);
  initial_state << 0, 0, 2.0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  plant_->set_state_vector(&simulator.get_mutable_context(), initial_state);

  // Start the simulation.
  lcm.StartReceiveThread();
  simulator.Initialize();
  simulator.StepTo(FLAGS_simulation_sec);

  return 0;
}

}  // namespace cubli
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::cubli::DoMain();
}
