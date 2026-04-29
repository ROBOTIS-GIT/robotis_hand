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

#include "robotis_hand_playground/finger_ik_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace robotis_hand_playground
{

namespace
{
double clamp_value(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}
}  // namespace

FingerPlanarIk::FingerPlanarIk(const std::array<FingerModel, fingers_num> & models)
: models_(models)
{}

FingerPlanarIk::Pose2D FingerPlanarIk::fk(int finger_idx, const std::array<double, dof> & q) const
{
  Pose2D pose{0.0, 0.0, 0.0};
  double angle = 0.0;

  // Accumulate each link pose in the planar y-z frame.
  for (int i = 0; i < dof; ++i) {
    angle += q[i];
    pose.y += models_[finger_idx].link_lengths[i] * std::sin(angle);
    pose.z += models_[finger_idx].link_lengths[i] * std::cos(angle);
  }

  pose.theta = angle;
  return pose;
}

void FingerPlanarIk::clamp_to_limits(int finger_idx, std::array<double, dof> & q) const
{
  for (int i = 0; i < dof; ++i) {
    q[i] = clamp_value(q[i], models_[finger_idx].joint_min[i], models_[finger_idx].joint_max[i]);
  }
}

std::optional<std::array<double, FingerPlanarIk::dof>> FingerPlanarIk::solve_shift_local(
  int finger_idx, const std::array<double, dof> & current_q, double local_dy, double local_dz) const
{
  const Pose2D current_pose = fk(finger_idx, current_q);

  // Convert fingertip local frame displacement to the base y-z frame.
  const double c = std::cos(current_pose.theta);
  const double s = std::sin(current_pose.theta);

  Pose2D target = current_pose;

  target.y += c * local_dy + s * local_dz;
  target.z += -s * local_dy + c * local_dz;

  return solve_ik(finger_idx, target, current_q);
}

std::optional<std::array<double, FingerPlanarIk::dof>> FingerPlanarIk::solve_ik(
  int finger_idx, const Pose2D & target, const std::array<double, dof> & current_q) const
{
  const double l1 = models_[finger_idx].link_lengths[0];
  const double l2 = models_[finger_idx].link_lengths[1];
  const double l3 = models_[finger_idx].link_lengths[2];

  const double q1c = current_q[0];
  const double q2c = current_q[1];
  const double q3c = current_q[2];

  // Keep q3 coupled with q1 and q2 changes.
  // q3 = q3c + (q1 - q1c) + (q2 - q2c)
  const double c = q3c - q1c - q2c;

  auto build_q = [&](double q1, double q2) {
      std::array<double, dof> q{};
      q[0] = q1;
      q[1] = q2;
      q[2] = q3c + (q1 - q1c) + (q2 - q2c);
      return q;
    };

  auto residual = [&](double q1, double q2, double & ey, double & ez) {
      const double a1 = q1;
      const double a2 = q1 + q2;
      const double a3 = 2.0 * (q1 + q2) + c; // q1 + q2 + q3

      const double y = l1 * std::sin(a1) + l2 * std::sin(a2) + l3 * std::sin(a3);

      const double z = l1 * std::cos(a1) + l2 * std::cos(a2) + l3 * std::cos(a3);

      ey = target.y - y;
      ez = target.z - z;
    };

  // Try multiple seeds to avoid poor local solutions.
  const std::vector<std::array<double, 2>> seeds = {{q1c, q2c},
    {q1c + 0.05, q2c},
    {q1c - 0.05, q2c},
    {q1c, q2c + 0.05},
    {q1c, q2c - 0.05},
    {q1c + 0.05, q2c + 0.05},
    {q1c - 0.05, q2c - 0.05}};

  std::optional<std::array<double, dof>> best_q;
  double best_cost = std::numeric_limits<double>::infinity();

  for (const auto & seed : seeds) {
    double q1 = seed[0];
    double q2 = seed[1];

    for (int iter = 0; iter < 50; ++iter) {
      double ey = 0.0, ez = 0.0;
      residual(q1, q2, ey, ez);

      const double a1 = q1;
      const double a2 = q1 + q2;
      const double a3 = 2.0 * (q1 + q2) + c;

      // Exact Jacobian for the constrained planar model.
      const double j11 = l1 * std::cos(a1) + l2 * std::cos(a2) + 2.0 * l3 * std::cos(a3);
      const double j12 = l2 * std::cos(a2) + 2.0 * l3 * std::cos(a3);

      const double j21 = -l1 * std::sin(a1) - l2 * std::sin(a2) - 2.0 * l3 * std::sin(a3);
      const double j22 = -l2 * std::sin(a2) - 2.0 * l3 * std::sin(a3);

      const double det = j11 * j22 - j12 * j21;
      if (std::abs(det) < 1e-10) {
        break;
      }

      // Newton update: J * dq = error.
      const double dq1 = (j22 * ey - j12 * ez) / det;
      const double dq2 = (-j21 * ey + j11 * ez) / det;

      q1 += dq1;
      q2 += dq2;

      // Keep the solution inside joint limits at each iteration.
      auto q_tmp = build_q(q1, q2);
      clamp_to_limits(finger_idx, q_tmp);
      q1 = q_tmp[0];
      q2 = q_tmp[1];
    }

    auto q_candidate = build_q(q1, q2);
    clamp_to_limits(finger_idx, q_candidate);

    // Evaluate candidate after joint limit clamping.
    const auto p_chk = fk(finger_idx, q_candidate);
    double ey = target.y - p_chk.y;
    double ez = target.z - p_chk.z;
    const double pos_err = std::sqrt(ey * ey + ez * ez);
    const double joint_change = std::abs(q_candidate[0] - q1c) + std::abs(q_candidate[1] - q2c) +
      std::abs(q_candidate[2] - q3c);

    // Prefer low residual and smaller joint motion.
    const double cost = pos_err + 0.01 * joint_change;

    if (cost < best_cost) {
      best_cost = cost;
      best_q = q_candidate;
    }
  }

  if (!best_q) {
    return std::nullopt;
  }

  // Reject the solution if the final position error is too large.
  const auto p_final = fk(finger_idx, *best_q);
  const double final_err = std::sqrt((target.y - p_final.y) * (target.y - p_final.y) +
      (target.z - p_final.z) * (target.z - p_final.z));

  if (final_err > 5e-4) {
    return std::nullopt;
  }

  return best_q;
}

}  // namespace robotis_hand_playground
