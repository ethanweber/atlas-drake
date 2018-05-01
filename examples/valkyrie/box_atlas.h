#pragma once

// creating box val state lcm types
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/position_3d_t.hpp"
#include "lcmtypes/bot_core/vector_3d_t.hpp"

#include "lcm/lcm-cpp.hpp"
#include <iostream>

#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/rigid_body_tree.h"

#include <utility>

#include "drake/common/eigen_types.h"

// for the urdf parser
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/util/drakeUtil.h"

#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"

#include "drake/multibody/kinematics_cache.h"

#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"

#include "bot_core/atlas_command_t.hpp"

#include "drake/multibody/rigid_body_tree_alias_groups.h"

#include "drake/examples/valkyrie/valkyrie_constants.h"


namespace drake {
namespace examples {
namespace valkyrie {

using systems::controllers::qp_inverse_dynamics::RobotKinematicState;
using systems::controllers::qp_inverse_dynamics::ParamSet;

class Box {
  public:
    Box(const RigidBodyTree<double>& tree, lcm::LCM& lcm);
    ~Box();
    void handle_message(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg);
    void publish_message();
    Isometry3<double> ComputeBodyPose(RobotKinematicState<double>& state, const RigidBody<double>& body);
    Vector6<double> ComputeBodyVelocity(RobotKinematicState<double>& state, const RigidBody<double>& body);
    Isometry3<double> ComputeFramePose(RobotKinematicState<double>& state, const RigidBodyFrame<double>& frame);
    Vector6<double> ComputeFrameVelocity(RobotKinematicState<double>& state, const RigidBodyFrame<double>& frame);

  private:

    // used for computing kinematics
    const manipulation::RobotStateLcmMessageTranslator translator_;

    // used to keep track of the current state
    VectorX<double> q_;
    VectorX<double> v_;

    // center of mass is always tracked and computed
    Eigen::Vector3d com_;
    Eigen::Vector3d com_velocity_;

    // THESE ARE THE LINES THAT NEED TO BE EDITED TO ADD CHANGE THE TRACKED BODIES
    // ---------------
    // number of bodies and the body groups that should be published
    int num_bodies_ = 0;
    std::string bodies_[0] = {};

    // number of frames and the nameso f those that should be published
    int num_frames_ = 4;
    std::string frames_[4] = {"l_hand_mount", "r_hand_mount", "l_foot_sole", "r_foot_sole"};

    // values that are passed from the bodies over LCM
    int num_prefixes_ = 3;
    std::string prefixes_[3] = {"x_", "y_", "z_"};
    // ---------------

    lcm::LCM lcm_;
    RobotKinematicState<double> state_;
    RigidBodyTreeAliasGroups<double> alias_groups_;
};

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake
