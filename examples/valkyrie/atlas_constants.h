#pragma once

#include "drake/common/eigen_types.h"

namespace drake {
namespace examples {
namespace valkyrie {

// All the following assumes using:
// urdf/urdf/valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf
// TODO(siyuan.feng): Add functinos to auto generate these.
constexpr int kRPYAtlasDof = 36;

VectorX<double> RPYAtlasFixedPointState();

// The nominal torque is generated with the QP controller at the setpoint,
// with acceleration constrained to zero.
VectorX<double> RPYAtlasFixedPointTorque();

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake
