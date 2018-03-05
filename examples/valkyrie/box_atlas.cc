
#include "drake/examples/valkyrie/box_atlas.h"

namespace drake {
namespace examples {
namespace valkyrie {

using systems::controllers::qp_inverse_dynamics::RobotKinematicState;
using systems::controllers::qp_inverse_dynamics::ParamSet;

// this is the constructor
Box::Box(const RigidBodyTree<double>& tree, lcm::LCM& lcm) :
  translator_(tree),
  lcm_(lcm),
  state_(&tree),
  alias_groups_(&tree) {
    q_ = VectorX<double>(tree.get_num_positions());
    v_ = VectorX<double>(tree.get_num_velocities());

    std::string alias_groups_config = FindResourceOrThrow(
        "drake/examples/valkyrie/test/"
        "atlas.alias_groups");
    alias_groups_.LoadFromFile(alias_groups_config);
}

Box::~Box(){}

Isometry3<double> Box::ComputeBodyPose(RobotKinematicState<double>& state, const RigidBody<double>& body) {
  return state.get_robot().CalcBodyPoseInWorldFrame(state.get_cache(), body);
}

Vector6<double> Box::ComputeBodyVelocity(RobotKinematicState<double>& state, const RigidBody<double>& body) {
  return state.get_robot().CalcBodySpatialVelocityInWorldFrame(state.get_cache(), body);
}

// updates the center of mass, end effector positions, kinematics cache
void Box::handle_message(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg)
{
  translator_.DecodeMessageKinematics(*msg, q_, v_);
  state_.UpdateKinematics(q_,v_);
  com_ = state_.get_com();
  com_velocity_ = state_.get_com_velocity();
  // updates the center of mass and kinematics
  // then goes on to creating the LCM message
  publish_message();
}

// publish message with center of mass data
void Box::publish_message()
{
  bot_core::robot_state_t msg;
  bot_core::position_3d_t lcm_position;
  bot_core::vector_3d_t position_vector;
  bot_core::quaternion_t quaternion_vector;
  bot_core::twist_t twist;
  bot_core::vector_3d_t linear_velocity;
  bot_core::vector_3d_t angular_velocity;

  msg.num_joints = 0;

  // set the position vector
  position_vector.x = com_[0];
  position_vector.y = com_[1];
  position_vector.z = com_[2];

  // set the quaternion orientation
  quaternion_vector.w = 1.0;
  quaternion_vector.x = 0.0;
  quaternion_vector.y = 0.0;
  quaternion_vector.z = 0.0;

  lcm_position.translation = position_vector;
  lcm_position.rotation = quaternion_vector;

  msg.pose = lcm_position;


  linear_velocity.x = com_velocity_[0];
  linear_velocity.y = com_velocity_[1];
  linear_velocity.z = com_velocity_[2];
  angular_velocity.x = 0.0;
  angular_velocity.y = 0.0;
  angular_velocity.z = 0.0;

  twist.linear_velocity = linear_velocity;
  twist.angular_velocity = angular_velocity;
  msg.twist = twist;

  // -------------------------------------------------
  // populate the end effector information with position and velocities
  // -----------------------------------------------

  // for x,y,z position and x,y,z velocity
  msg.num_joints = num_bodies_*num_prefixes_;
  msg.joint_name.resize(msg.num_joints);
  msg.joint_position.resize(msg.num_joints);
  msg.joint_velocity.resize(msg.num_joints);
  msg.joint_effort.resize(msg.num_joints);

  // for using x,y,z position and velocity (no angles)
  for (int i = 0; i < num_bodies_; i++) {
    const RigidBody<double>& body = *alias_groups_.get_body(bodies_[i]);
    // extract just the positions from the matrix in x,y,z order
    Vector3<double> position = ComputeBodyPose(state_, body).translation();
    // std::cout << "POSITION:\n" << position << std::endl;
    // 3 angular, 3 linear components (extract only the linear components)
    Vector3<double> velocity = ComputeBodyVelocity(state_, body).bottomRows(3);
    // std::cout << "VELOCITY:\n" << velocity << std::endl;
    for (int j = 0; j < num_prefixes_; j++) {
      msg.joint_name[i*num_prefixes_ + j] = prefixes_[j] + bodies_[i];
      msg.joint_position[i*num_prefixes_ + j] = position[j];
      msg.joint_velocity[i*num_prefixes_ + j] = velocity[j];
      msg.joint_effort[i*num_prefixes_ + j] = 0.0;
    }
  }

  lcm_.publish("BOX_ATLAS_STATE", &msg);
  // std::cout << "Published message." << std::endl;

}

int box_main() {

  auto tree = std::make_unique<RigidBodyTree<double>>();

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    FindResourceOrThrow(
        "drake/examples/valkyrie/urdf/atlas_urdf/"
        "atlas_minimal_contact.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());

  lcm::LCM lcm;
  if(!lcm.good()) return 1;

  Box box(*tree, lcm);

  lcm.subscribe("EST_ROBOT_STATE", &Box::handle_message, &box);

  while(0 == lcm.handle());
  return 0;

}

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake

int main() {
  return drake::examples::valkyrie::box_main();
}
