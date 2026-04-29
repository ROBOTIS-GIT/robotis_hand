// Copyright 2026 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Howon Kim

#pragma once

#include <array>
#include <optional>

namespace robotis_hand_playground
{

/**
 * @brief Planar inverse kinematics solver for finger y-z motion.
 */
class FingerPlanarIk {
public:
  static constexpr int fingers_num = 4;
  static constexpr int dof = 3;

  /**
   * @brief Per-finger planar kinematic model.
   */
  struct FingerModel
  {
    std::array<double, dof> joint_min;
    std::array<double, dof> joint_max;
    std::array<double, dof> link_lengths;
  };

  /**
   * @brief Fingertip pose in the planar y-z coordinate frame.
   */
  struct Pose2D
  {
    double y;
    double z;
    double theta;
  };

  explicit FingerPlanarIk(const std::array<FingerModel, fingers_num> & models);

  /**
   * @brief Compute forward kinematics from joint angles.
   */
  Pose2D fk(int finger_idx, const std::array<double, dof> & q) const;

  /**
   * @brief Clamp joint angles to the configured joint limits.
   */
  void clamp_to_limits(int finger_idx, std::array<double, dof> & q) const;

  /**
   * @brief Solve IK for a target shifted in the fingertip local frame.
   */
  std::optional<std::array<double, dof>> solve_shift_local(
    int finger_idx,
    const std::array<double, dof> & current_q,
    double local_dy,
    double local_dz) const;

private:
  /**
   * @brief Solve planar IK for the target fingertip pose.
   */
  std::optional<std::array<double, dof>> solve_ik(
    int finger_idx, const Pose2D & target, const std::array<double, dof> & current_q) const;

  // Per-finger kinematic models.
  std::array<FingerModel, fingers_num> models_;
};

}  // namespace robotis_hand_playground
