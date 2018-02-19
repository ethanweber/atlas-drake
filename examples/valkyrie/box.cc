
#include "drake/examples/valkyrie/box.h"

namespace drake {
namespace examples {
namespace valkyrie {

using systems::controllers::plan_eval::GenericPlan;
using systems::controllers::qp_inverse_dynamics::QpInput;
using systems::controllers::qp_inverse_dynamics::QpOutput;
using systems::controllers::qp_inverse_dynamics::QpInverseDynamics;
using systems::controllers::qp_inverse_dynamics::RobotKinematicState;
using systems::controllers::qp_inverse_dynamics::ParamSet;
using systems::controllers::VectorSetpoint;
using systems::controllers::CartesianSetpoint;

Box::Box(const RigidBodyTree<double>& tree, lcm::LCM& lcm) :
  translator_(tree),
  lcm_(lcm),
  state_(&tree) {
    q_ = VectorX<double>(tree.get_num_positions());
    v_ = VectorX<double>(tree.get_num_velocities());

    const int actuator_size = tree.get_num_actuators();
    k_q_p_ = VectorX<double>::Zero(actuator_size);
    k_q_i_ = VectorX<double>::Zero(actuator_size);
    k_qd_p_ = VectorX<double>::Zero(actuator_size);
    k_f_p_ = VectorX<double>::Zero(actuator_size);
    ff_qd_ = VectorX<double>::Zero(actuator_size);
    ff_qd_d_ = VectorX<double>::Zero(actuator_size);

    // directly feed torque through without any other feedbacks
    ff_f_d_ = VectorX<double>::Constant(actuator_size, 1.);
    ff_const_ = VectorX<double>::Zero(actuator_size);
}

Box::~Box(){}

// updates the center of mass, end effector positions, kinematics cache
void Box::handle_message(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg)
{
  translator_.DecodeMessageKinematics(*msg, q_, v_);
  state_.UpdateKinematics(q_,v_);
  com_ = state_.get_com();

  left_foot_ = translator_.get_robot().CalcFramePoseInWorldFrame(state_.get_cache(),
    *translator_.get_robot().findFrame("l_foot_sole"));
  right_foot_ = translator_.get_robot().CalcFramePoseInWorldFrame(state_.get_cache(),
    *translator_.get_robot().findFrame("r_foot_sole"));

  std::vector<Eigen::Isometry3d> desired_position;
  std::vector<Vector6<double>> desired_velocity;

  compute_body_info();

  publish_message();
}

Isometry3<double> Box::ComputeBodyPose(RobotKinematicState<double>& state, const RigidBody<double>& body) {
  return state.get_robot().CalcBodyPoseInWorldFrame(state.get_cache(), body);
}

Vector6<double> Box::ComputeBodyVelocity(RobotKinematicState<double>& state, const RigidBody<double>& body) {
  return state.get_robot().CalcBodySpatialVelocityInWorldFrame(state.get_cache(), body);
}

void Box::compute_body_info() {

  const RigidBodyTree<double>& robot = translator_.get_robot();

  std::string alias_groups_config = FindResourceOrThrow(
      "drake/examples/valkyrie/test/"
      "valkyrie.alias_groups");
  std::string controller_config = FindResourceOrThrow(
      "drake/examples/valkyrie/test/"
      "valkyrie.id_controller_config");

  // KinematicsProperty
  RigidBodyTreeAliasGroups<double> alias_groups(&robot);
  alias_groups.LoadFromFile(alias_groups_config);

  // Controller config
  ParamSet paramset;
  paramset.LoadFromFile(controller_config, alias_groups);

  QpInput input = paramset.MakeQpInput({"feet"},            /* contacts */
                                       {"right_palm", "pelvis", "torso"}, /* tracked bodies*/
                                       alias_groups);

  const RigidBody<double>& pelvis = *alias_groups.get_body("pelvis");
  const RigidBody<double>& torso = *alias_groups.get_body("torso");
  const RigidBody<double>& right_palm = *alias_groups.get_body("right_palm");

  Vector3<double> Kp_com, Kd_com;
  VectorX<double> Kp_q, Kd_q;
  Vector6<double> Kp_centroidal, Kd_centroidal,
                  Kp_pelvis, Kd_pelvis,
                  Kp_right_palm, Kd_right_palm,
                  Kp_torso, Kd_torso;

  paramset.LookupDesiredBodyMotionGains(pelvis, &Kp_pelvis, &Kd_pelvis);
  paramset.LookupDesiredBodyMotionGains(torso, &Kp_torso, &Kd_torso);
  paramset.LookupDesiredBodyMotionGains(right_palm, &Kp_right_palm, &Kd_right_palm);
  paramset.LookupDesiredCentroidalMomentumDotGains(&Kp_centroidal,
                                                   &Kd_centroidal);
  Kp_com = Kp_centroidal.tail<3>();
  Kd_com = Kd_centroidal.tail<3>();


  // -----------------pelvis-----------------------
  Isometry3<double> desired_pelvis_pose;
  desired_pelvis_pose.matrix() <<
  0.999904,   0.0138047, -0.00143103, -0.00795848,
 -0.0137794,    0.99977,   0.0164094,  0.00130671,
 0.00165722,  -0.0163881,    0.999864,     1.01544,
          0,           0,           0,           1;

   //  0.999904,   0.0138047, -0.00143103, -0.00795848,
   // -0.0137794,    0.99977,   0.0164094,  0.00130671,
   // 0.00165722,  -0.0163881,    0.999864,     1.01544,
   //          0,           0,           0,           1;

  Vector6<double> desired_pelvis_vel;
  desired_pelvis_vel <<
  0.0,0.0,0.0,
  0.0,0.0,0.0;

  // auto desired_pelvis_pose = ComputeBodyPose(state_, pelvis);
  // std::cout << desired_pelvis_pose.matrix() << std::endl;
  // auto desired_pelvis_vel = ComputeBodyVelocity(state_, pelvis);
  // std::cout << desired_pelvis_vel.matrix() << std::endl;

  CartesianSetpoint<double> pelvis_PDff(
      desired_pelvis_pose, desired_pelvis_vel,
      Vector6<double>::Zero(), Kp_pelvis, Kd_pelvis);
  input.mutable_desired_body_motions().at(pelvis.get_name()).mutable_values() =
      pelvis_PDff.ComputeTargetAcceleration(
        ComputeBodyPose(state_, pelvis),
        ComputeBodyVelocity(state_, pelvis));
  // -----------------pelvis-----------------------

  // // -----------------torso-----------------------
  // Isometry3<double> desired_torso_pose;
  // desired_torso_pose.matrix() <<
  // 0.999904,   0.0138047, -0.00143103, -0.00795848,
  // -0.0137794,    0.99977,   0.0164094,  0.00130671,
  // 0.00165722,  -0.0163881,    0.999864,     1.01544,
  //         0,           0,           0,           1;
  //
  // Vector6<double> desired_torso_vel;
  // desired_torso_vel <<
  // 0.0,0.0,0.0,
  // 0.0,0.0,0.0;
  //
  // // auto desired_torso_pose = ComputeBodyPose(state_, torso);
  // // std::cout << desired_torso_pose.matrix() << std::endl;
  // // auto desired_torso_vel = ComputeBodyVelocity(state_, torso);
  // // std::cout << desired_torso_vel.matrix() << std::endl;
  //
  // CartesianSetpoint<double> torso_PDff(
  //     desired_torso_pose, desired_torso_vel,
  //     Vector6<double>::Zero(), Kp_torso, Kd_torso);
  // input.mutable_desired_body_motions().at(torso.get_name()).mutable_values() =
  //     torso_PDff.ComputeTargetAcceleration(
  //       ComputeBodyPose(state_, torso),
  //       ComputeBodyVelocity(state_, torso));
  // // -----------------torso-----------------------

  // -----------------right_palm-----------------------
  Isometry3<double> desired_right_palm_pose;
  desired_right_palm_pose.matrix() <<
  -0.0956289,  -0.479053,   0.872561,  0.0501593,
    0.9483,   0.222654,   0.226171,  -0.422336,
 -0.302627,   0.849078,   0.432994,   0.774994,
         0,          0,          0,          1;

  //  -0.0956289,  -0.479053,   0.872561,  0.0501593,
  //    0.9483,   0.222654,   0.226171,  -0.422336,
  // -0.302627,   0.849078,   0.432994,   1.774994, // randomly changed the last value here from 0.774994
  //         0,          0,          0,          1;

  Vector6<double> desired_right_palm_vel;
  desired_right_palm_vel <<
  0.0,0.0,0.0,
  0.0,0.0,0.0;

  // auto desired_right_palm_pose = ComputeBodyPose(state_, right_palm);
  // std::cout << desired_right_palm_pose.matrix() << std::endl;
  // auto desired_right_palm_vel = ComputeBodyVelocity(state_, right_palm);
  // std::cout << desired_right_palm_vel.matrix() << std::endl;

  CartesianSetpoint<double> right_palm_PDff(
      desired_right_palm_pose, desired_right_palm_vel,
      Vector6<double>::Zero(), Kp_right_palm, Kd_right_palm);
  input.mutable_desired_body_motions().at(right_palm.get_name()).mutable_values() =
      right_palm_PDff.ComputeTargetAcceleration(
        ComputeBodyPose(state_, right_palm),
        ComputeBodyVelocity(state_, right_palm));
  // -----------------right_palm-----------------------


  Vector3<double> desired_com = Vector3<double>(0.0,0.0,0.96);
  input.mutable_desired_centroidal_momentum_dot().mutable_values().tail<3>() =
     (Kp_com.array() * (desired_com - com_).array() -
      Kd_com.array() * state_.get_com_velocity().array())
         .matrix() *
     robot.getMass();

  // solve the qp (quadratic program)
  QpInverseDynamics con;
  QpOutput output(systems::controllers::qp_inverse_dynamics::GetDofNames(robot));
  int status = con.Control(state_, input, &output);
  if (status) {
    std::stringstream err;
    err << input << output;
    drake::log()->info(err.str());
    return;
  }

  // converts the DoF torques into the expected robot actuator order
  systems::BasicVector<double> torques(robot.get_num_actuators());
  torques.get_mutable_value() = robot.B.transpose() * output.dof_torques();

  bot_core::atlas_command_t robot_command{};

  robot_command.utime = time(NULL);
  robot_command.num_joints = robot.get_num_actuators();
  robot_command.joint_names.resize(robot_command.num_joints);
  robot_command.position.resize(robot_command.num_joints);
  robot_command.velocity.resize(robot_command.num_joints);
  robot_command.effort.resize(robot_command.num_joints);

  for (int i = 0; i < robot_command.num_joints; ++i) {
    robot_command.joint_names[i] = robot.actuators[i].name_;
    robot_command.position[i] = 0;
    robot_command.velocity[i] = 0;
    robot_command.effort[i] = torques[i];
  }

  eigenVectorToStdVector(k_q_p_, robot_command.k_q_p);
  eigenVectorToStdVector(k_q_i_, robot_command.k_q_i);
  eigenVectorToStdVector(k_qd_p_, robot_command.k_qd_p);
  eigenVectorToStdVector(k_f_p_, robot_command.k_f_p);
  eigenVectorToStdVector(ff_qd_, robot_command.ff_qd);
  eigenVectorToStdVector(ff_qd_d_, robot_command.ff_qd_d);
  eigenVectorToStdVector(ff_f_d_, robot_command.ff_f_d);
  eigenVectorToStdVector(ff_const_, robot_command.ff_const);

  robot_command.k_effort.resize(robot_command.num_joints, 0);
  robot_command.desired_controller_period_ms = 0;

  lcm_.publish("ROBOT_COMMAND", &robot_command);

}

// void Box::compute_body_info() {
//
//   const RigidBodyTree<double>& robot = translator_.get_robot();
//
//   std::string alias_groups_config = FindResourceOrThrow(
//       "drake/examples/valkyrie/test/"
//       "valkyrie.alias_groups");
//   std::string controller_config = FindResourceOrThrow(
//       "drake/examples/valkyrie/test/"
//       "valkyrie.id_controller_config");
//
//   // KinematicsProperty
//   RigidBodyTreeAliasGroups<double> alias_groups(&robot);
//   alias_groups.LoadFromFile(alias_groups_config);
//
//   // Controller config
//   ParamSet paramset;
//   paramset.LoadFromFile(controller_config, alias_groups);
//
//   QpInput input = paramset.MakeQpInput({"feet"},            /* contacts */
//                                        {"pelvis", "torso"}, /* tracked bodies*/
//                                        alias_groups);
//   QpOutput output(systems::controllers::qp_inverse_dynamics::GetDofNames(translator_.get_robot()));
//
//   QpInverseDynamics con;
//
//   // Set up a tracking problem.
//   // Gains
//   Vector3<double> Kp_com, Kd_com;
//   VectorX<double> Kp_q, Kd_q;
//   Vector6<double> Kp_centroidal, Kd_centroidal;
//
//   // paramset.LookupDesiredBodyMotionGains(pelvis, &Kp_pelvis, &Kd_pelvis);
//   // paramset.LookupDesiredBodyMotionGains(torso, &Kp_torso, &Kd_torso);
//   paramset.LookupDesiredDofMotionGains(&Kp_q, &Kd_q);
//   paramset.LookupDesiredCentroidalMomentumDotGains(&Kp_centroidal,
//                                                    &Kd_centroidal);
//   Kp_com = Kp_centroidal.tail<3>();
//   Kd_com = Kd_centroidal.tail<3>();
//
//   // Setpoints
//   Vector3<double> desired_com = Vector3<double>(0.0,0.0,.96);
//   VectorSetpoint<double> joint_PDff(q_, VectorX<double>::Zero(q_.size()),
//                                     VectorX<double>::Zero(q_.size()), Kp_q,
//                                     Kd_q);
//
//   // const KinematicsCache<double>& cache = state_.get_cache();
//   std::vector<Eigen::Isometry3d> positions;
//   std::vector<Vector6<double>> velocities;
//   for (int i = 0; i < robot.get_num_bodies(); i++) {
//       const RigidBody<double>& body = robot.get_body(i);
//       positions.push_back(ComputeBodyPose(state_, body));
//       velocities.push_back(ComputeBodyVelocity(state_, body));
//   }
//
//   // compute desired linear accelerations
//   std::vector<Vector3<double>> linear_accelerations;
//   for (std::vector<int>::size_type i = 0; i < positions.size(); i++) {
//
//     // start with no change
//     // use index to check which body it is
//
//
//     Vector3<double> desired_pos = positions[i].translation();
//     Vector3<double> desired_vel = velocities[i].topRows(3);
//
//     // std::string body_name = robot.getBodyOrFrameName(i);
//     const std::string& body_name = robot.get_body(i).get_name();
//
//     // std::cout << body_name << std::endl;
//
//     if (body_name == "pelvis") {
//       // std::cout << body_name << std::endl;
//       // change in position
//       desired_pos = Vector3<double>(0,0,3.0);
//       // desired_pos[0] += Vector3<double>(0,0,0);
//       // change in velocity
//       desired_vel = Vector3<double>(0,0,.1);
//       // desired_vel[0] += Vector3<double>(0,0,0);
//     }
//
//     // the linear component
//     Vector3<double> acc =
//     kp_*(desired_pos - positions[i].translation()) +
//     kd_*(desired_vel - velocities[i].bottomRows(3));
//
//     linear_accelerations.push_back(acc);
//
//     Vector6<double> current_acc(6);
//     current_acc[0] = 0.0;
//     current_acc[1] = 0.0;
//     current_acc[2] = 0.0;
//     current_acc[3] = acc[0];
//     current_acc[4] = acc[1];
//     current_acc[5] = acc[2];
//
//     if (body_name == "pelvis") {
//         std::cout << current_acc[3] << std::endl;
//         // input.mutable_desired_body_motions().at(body_name).mutable_values() =
//         //   current_acc;
//     }
//
//   }
//
//   // input.mutable_desired_dof_motions().mutable_values() =
//   //     joint_PDff.ComputeTargetAcceleration(cache.getQ(),
//   //                                          VectorX<double>::Zero(q_.size());
//
//   input.mutable_desired_centroidal_momentum_dot().mutable_values().tail<3>() =
//      (Kp_com.array() * (desired_com - com_).array() -
//       Kd_com.array() * state_.get_com_velocity().array())
//          .matrix() *
//      robot.getMass();
//
//   int status = con.Control(state_, input, &output);
//
//   if (status) {
//     std::stringstream err;
//     err << input << output;
//     drake::log()->info(err.str());
//     return;
//   }
//
//   bot_core::atlas_command_t atlas_command{};
//
//   atlas_command.utime = time(NULL);
//   atlas_command.num_joints = robot.get_num_actuators();
//   atlas_command.joint_names.resize(atlas_command.num_joints);
//   atlas_command.position.resize(atlas_command.num_joints);
//   atlas_command.velocity.resize(atlas_command.num_joints);
//   atlas_command.effort.resize(atlas_command.num_joints);
//
//   // converts the DoF torques into the robot.actuator order as expected
//   systems::BasicVector<double> act_torques(atlas_command.num_joints);
//   act_torques.get_mutable_value() = robot.B.transpose() * output.dof_torques();
//
//   for (int i = 0; i < atlas_command.num_joints; i++) {
//     atlas_command.joint_names[i] = robot.actuators[i].name_;
//     atlas_command.position[i] = 0;
//     atlas_command.velocity[i] = 0;
//     atlas_command.effort[i] = act_torques[i];
//   }
//
//   eigenVectorToStdVector(k_q_p_, atlas_command.k_q_p);
//   eigenVectorToStdVector(k_q_i_, atlas_command.k_q_i);
//   eigenVectorToStdVector(k_qd_p_, atlas_command.k_qd_p);
//   eigenVectorToStdVector(k_f_p_, atlas_command.k_f_p);
//   eigenVectorToStdVector(ff_qd_, atlas_command.ff_qd);
//   eigenVectorToStdVector(ff_qd_d_, atlas_command.ff_qd_d);
//   eigenVectorToStdVector(ff_f_d_, atlas_command.ff_f_d);
//   eigenVectorToStdVector(ff_const_, atlas_command.ff_const);
//
//   atlas_command.k_effort.resize(atlas_command.num_joints, 0);
//   atlas_command.desired_controller_period_ms = 0;
//
//   lcm_.publish("ROBOT_COMMAND", &atlas_command);
//
//   // GENERATE PLAN with ParamSet
//
//   // BodyAcceleration()
//
//   // http://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1controllers_1_1qp__inverse__dynamics_1_1_body_acceleration.html
//
//   // std::cout << linear_accelerations[0] << std::endl;
//
//
//
//
//
//   // std::cout << velocities[2] << std::endl;
//   // std::cout << "---------------" << std::endl;
//   // // std::cout << velocities[2].block<3,1>(0,2) << std::endl;
//   // std::cout << velocities[2].topRows(3) << std::endl;
//   // // velocities[2].bottomRows(3)
//   // std::cout << "---------------" << std::endl;
//
//
//   // std::cout << positions[1].translation() << std::endl;
//
// }

// void Box::get_acceleration(VectorX<double> desired_pos,
//   VectorX<double> desired_vel,
//   VectorX<double> x,
//   VectorX<double> v) {
//     return kp_*(desired_pos - x) - kd_*(desired_vel - v);
// }

// publish message with center of mass data
void Box::publish_message(){
  bot_core::robot_state_t msg;
  bot_core::position_3d_t lcm_position;
  bot_core::vector_3d_t position_vector;
  bot_core::quaternion_t quaternion_vector;

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

  // -------------------------------------------------
  // add information for end effectors
  // -----------------------------------------------
  // msg.num_joints = 1;
  // msg.joint_name.resize(msg.num_joints);
  // msg.joint_position.resize(msg.num_joints);
  // msg.joint_velocity.resize(msg.num_joints);
  // msg.joint_effort.resize(msg.num_joints);
  //
  // msg.joint_name[0] = "foot";
  // msg.joint_position[0] = .3;
  // msg.joint_velocity[0] = .5;
  // msg.joint_effort[0] = .5;
  // --------------------------------------------------

  // std::cout << left_foot_.matrix() << std::endl;
  // std::cout << "---------------" << std::endl;

  lcm_.publish("BOX_VAL_STATE", &msg);

}

int box_main() {

  auto tree = std::make_unique<RigidBodyTree<double>>();

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    FindResourceOrThrow(
        "drake/examples/valkyrie/urdf/urdf/"
        "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"),
      multibody::joints::kRollPitchYaw, tree.get());

  // std::string alias_groups_config = FindResourceOrThrow(
  //     "drake/examples/valkyrie/test/"
  //     "valkyrie.alias_groups");
  // std::string controller_config = FindResourceOrThrow(
  //     "drake/examples/valkyrie/test/"
  //     "valkyrie.id_controller_config");

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
