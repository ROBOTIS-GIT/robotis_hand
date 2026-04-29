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

#include "robotis_hand_playground/tactile_grasp_controller.hpp"

#include <cmath>

#include "robotis_hand_playground/correction_planner.hpp"

namespace robotis_hand_playground
{
TactileGraspController::TactileGraspController(const std::string & node_name)
: Node(node_name)
{
  configure_hand("right");
}
TactileGraspController::~TactileGraspController() = default;

void TactileGraspController::configure_hand(const std::string & hand_side)
{
  // Initialize HX5-D20 hand model.
  fingers_ = robotis_hand_playground::init_fingers(hand_side);
  hand_joint_names_ = robotis_hand_playground::init_joint_names(hand_side);
  init_positions_ = robotis_hand_playground::init_positions(hand_side);
  // Initialize planar IK models.
  std::array<FingerPlanarIk::FingerModel, 4> ik_models{};
  for (int i = 0; i < 4; ++i) {
    const int finger_idx = i + 1;  // index=1, middle=2, ring=3, little=4

    ik_models[i].joint_min = {fingers_[finger_idx].joint_min[1],
      fingers_[finger_idx].joint_min[2],
      fingers_[finger_idx].joint_min[3]};

    ik_models[i].joint_max = {fingers_[finger_idx].joint_max[1],
      fingers_[finger_idx].joint_max[2],
      fingers_[finger_idx].joint_max[3]};

    // Link length
    ik_models[i].link_lengths = {0.0235, 0.0355, 0.0355};
  }
  finger_planar_ik_ = std::make_unique<FingerPlanarIk>(ik_models);
  correction_planner_ = std::make_unique<CorrectionPlanner>(*this);

  // Initialize target joints with open positions.
  for (auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      finger.current_joint_targets[j] = get_open_pos(finger.joint_names[j]);
    }
  }
}

void TactileGraspController::handle_idle()
{
  // IDLE
}

bool TactileGraspController::unused_finger(int finger_idx) const
{
  return std::find(param.un_use_finger.begin(), param.un_use_finger.end(), finger_idx) !=
         param.un_use_finger.end();
}

void TactileGraspController::close_unused_finger()
{
  for (int i = 1; i < fingers_num; ++i) {
    if (!unused_finger(i)) {
      continue;
    }
    auto & finger = fingers_[i];

    // Move unused fingers to the closed posture.
    for (int j = 1; j <= 3; ++j) {
      finger.current_joint_targets[j] += param.close_step * 3;
      finger.current_joint_targets[j] =
        clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
    }
  }
}

double TactileGraspController::finger_contact_threshold(int finger_idx) const
{
  if (finger_idx == 0) {
    return param.contact_threshold * param.thumb_contact_ratio;
  }
  return param.contact_threshold;
}

void TactileGraspController::handle_close()
{
  for (int i = 0; i < fingers_num; ++i) {
    auto & finger = fingers_[i];

    if (unused_finger(i)) {
      finger.contact_detected = true;
      desired_force_[i] = 0.0;
      continue;
    }

    if (!finger.contact_detected) {
      // Thumb closes using joint3 and joint4.
      if (i == 0) {
        const double direction = thumb_joint_sign(param.hand_side);
        for (int j = 2; j <= 3; ++j) {
          finger.current_joint_targets[j] += direction * param.close_step;
          finger.current_joint_targets[j] =
            clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      } else {
        // Other fingers close using joint2, joint3, and joint4.
        for (int j = 1; j <= 3; ++j) {
          finger.current_joint_targets[j] += param.close_step;
          finger.current_joint_targets[j] =
            clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      }

      // Detect initial tactile contact.
      if (finger.filtered_force >= finger_contact_threshold(i)) {
        finger.contact_detected = true;
        RCLCPP_INFO(this->get_logger(),
          "[%s] contact detected, force=%.2f",
          finger.name.c_str(),
          finger.filtered_force);
      }
    }
  }
  publish_traj();

  // Switch to state:HOLD after all active fingers make contact.
  if (all_contacted()) {
    set_desired_force();
    state_ = State::HOLD;
    RCLCPP_INFO(this->get_logger(), "State -> HOLD");
  }
}

void TactileGraspController::set_desired_force()
{
  for (int i = 0; i < fingers_num; ++i) {
    desired_force_[i] = std::max(fingers_[i].filtered_force, finger_contact_threshold(i));
    RCLCPP_INFO(this->get_logger(),
      "[%s] desired_force set to %.2f",
      fingers_[i].name.c_str(),
      desired_force_[i]);
  }
}

void TactileGraspController::regulate_grasp_force(int finger_idx)
{
  auto & finger = fingers_[finger_idx];
  const double error = desired_force_[finger_idx] - finger.filtered_force;

  if (error > -deadband && error < deadband) {
    return;
  }

  // Convert force error to joint delta.
  double dq = force_kp_ * error;
  dq = clamp(dq, -param.feedback_max_delta, param.feedback_max_delta);
  dq *= finger_step_scale(finger_idx);

  if (dq > 0.0) {
    grasp_step_hold(finger_idx, dq);
  } else if (dq < 0.0) {
    release_step_hold(finger_idx, -dq);
  }
}

void TactileGraspController::grasp_step_hold(int finger_idx, double step)
{
  auto & finger = fingers_[finger_idx];

  if (finger_idx == 0) {
    // Thumb force regulation uses joint3 and joint4.
    const std::array<int, 2> joints = {2, 3};
    const std::array<double, 2> ratios = {6.0, 4.0};
    const double sum = ratios[0] + ratios[1];
    const double direction = thumb_joint_sign(param.hand_side);

    for (int k = 0; k < 2; ++k) {
      const int j = joints[k];
      finger.current_joint_targets[j] += direction * step * (ratios[k] / sum);
      finger.current_joint_targets[j] =
        clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
    }
    return;
  }
  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, +step);
}

void TactileGraspController::release_step_hold(int finger_idx, double step)
{
  auto & finger = fingers_[finger_idx];

  if (finger_idx == 0) {
    // Thumb force regulation uses joint3 and joint4.
    const std::array<int, 2> joints = {2, 3};
    const std::array<double, 2> ratios = {6.0, 4.0};
    const double sum = ratios[0] + ratios[1];
    const double direction = thumb_joint_sign(param.hand_side);

    for (int k = 0; k < 2; ++k) {
      const int j = joints[k];
      finger.current_joint_targets[j] -= direction * step * (ratios[k] / sum);
      finger.current_joint_targets[j] =
        clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
    }
    return;
  }

  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, -step);
}

void TactileGraspController::handle_hold()
{
  for (int i = 0; i < fingers_num; ++i) {
    if (correction_plans_[i].active) {
      correction_planner_->apply_correction(i);
      continue;
    }

    // During the second stage, re-grasp first if the force is too low.
    if (hold_correction_stage_ == HoldCorrectionStage::Y_SECOND) {
      if (fingers_[i].filtered_force < param.regrasp_force * finger_contact_threshold(i)) {
        grasp_step_hold(i, regrasp_step_);
        continue;
      }
    }
    // Maintain desired force and start a new CoP correction.
    regulate_grasp_force(i);

    if (!y_ik_failed_[i]) {
      correction_planner_->start_correction(i);
    }
  }
  publish_traj();

  // Move from X correction stage to Y correction stage when X correction is blocked.
  if (hold_correction_stage_ == HoldCorrectionStage::X_FIRST) {
    if (correction_planner_->correction_blocked(HoldCorrectionStage::X_FIRST)) {
      hold_correction_stage_ = HoldCorrectionStage::Y_SECOND;

      for (auto & plan : correction_plans_) {
        plan = CorrectionPlan{};
      }
      RCLCPP_INFO(this->get_logger(), "X correction done -> switch to Y correction");
    }
  } else {
    // Finish HOLD when Y correction is blocked and force is satisfied.
    if (correction_planner_->correction_blocked(HoldCorrectionStage::Y_SECOND)) {
      if (all_finger_contacted()) {
        RCLCPP_INFO(this->get_logger(), "Y correction done + force satisfied -> IDLE");
        state_ = State::IDLE;
      } else {
        RCLCPP_INFO(this->get_logger(), "Y correction done but force not satisfied -> keep HOLD");
      }
    }
  }
}

void TactileGraspController::apply_ratio_step(
  int finger_idx,
  const std::array<int, 3> & local_joint_ids,
  const std::array<double, 3> & ratios,
  double signed_step)
{
  auto & finger = fingers_[finger_idx];
  const double ratio_sum = ratios[0] + ratios[1] + ratios[2];
  const double direction = finger_idx == 0 ? thumb_joint_sign(param.hand_side) : 1.0;

  for (int i = 0; i < 3; ++i) {
    const int j = local_joint_ids[i];
    const double delta = direction * signed_step * (ratios[i] / ratio_sum);

    finger.current_joint_targets[j] += delta;
    finger.current_joint_targets[j] =
      clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
  }
}

double TactileGraspController::max_filtered_force() const
{
  double max_force = 0.0;

  // Exclude thumb from force scaling reference.
  for (int i = 1; i < fingers_num; ++i) {
    max_force = std::max(max_force, fingers_[i].filtered_force);
  }
  return max_force;
}

double TactileGraspController::finger_step_scale(int finger_idx) const
{
  if (finger_idx == 0) {
    return min_step_scale_;
  }
  const double f_max = max_filtered_force();

  if (f_max < 1e-6) {
    return 1.0;
  }
  const double ratio = fingers_[finger_idx].filtered_force / f_max;
  return clamp(ratio, min_step_scale_, max_step_scale_);
}

void TactileGraspController::shift_joint(int finger_idx, double delta)
{
  auto & finger = fingers_[finger_idx];
  const int joint_idx = 0;

  finger.current_joint_targets[joint_idx] += delta;
  finger.current_joint_targets[joint_idx] = clamp(finger.current_joint_targets[joint_idx],
    finger.joint_min[joint_idx],
    finger.joint_max[joint_idx]);
}

void TactileGraspController::shift_thumb(int finger_idx, double delta1, double delta2)
{
  auto & finger = fingers_[finger_idx];
  const double direction = thumb_joint_sign(param.hand_side);
  finger.current_joint_targets[0] += direction * delta1 * 0.1;
  finger.current_joint_targets[1] += direction * delta2 * 0.1;

  finger.current_joint_targets[0] =
    clamp(finger.current_joint_targets[0], finger.joint_min[0], finger.joint_max[0]);
  finger.current_joint_targets[1] =
    clamp(finger.current_joint_targets[1], finger.joint_min[1], finger.joint_max[1]);
}

void TactileGraspController::grasp_joint(int finger_idx, double step)
{
  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, +step);
}

void TactileGraspController::release_joint(int finger_idx, double step)
{
  apply_ratio_step(finger_idx, {1, 2, 3}, {5.0, 3.0, 2.0}, -step);
}

void TactileGraspController::reset_grasp()
{
  // Reset contact states.
  for (auto & finger : fingers_) {
    finger.contact_detected = false;
  }
  // Reset IK failure and correction plans.
  for (auto & failed : y_ik_failed_) {
    failed = false;
  }
  for (auto & plan : correction_plans_) {
    plan = CorrectionPlan{};
  }
  hold_correction_stage_ = HoldCorrectionStage::X_FIRST;

  sync_targets();
}

void TactileGraspController::reset_to_init()
{
  reset_grasp();
  state_ = State::IDLE;

  for (auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      finger.current_joint_targets[j] = get_open_pos(finger.joint_names[j]);
    }
  }
}

void TactileGraspController::sync_targets()
{
  if (!joint_received_) {
    return;
  }
  // Start target values from the current joint states.
  for (auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      finger.current_joint_targets[j] = get_joint_pos(finger.joint_names[j]);
    }
  }
}

bool TactileGraspController::all_contacted() const
{
  for (const auto & finger : fingers_) {
    if (!finger.contact_detected) {
      return false;
    }
  }
  return true;
}

bool TactileGraspController::all_finger_contacted() const
{
  for (int i = 0; i < fingers_num; ++i) {
    if (unused_finger(i)) {
      continue;
    }
    if (fingers_[i].filtered_force < param.regrasp_force * finger_contact_threshold(i)) {
      return false;
    }
  }
  return true;
}

std::array<double, 3> TactileGraspController::get_planar_q(int finger_idx) const
{
  const auto & finger = fingers_[finger_idx];
  return {get_joint_pos(finger.joint_names[1]),
    get_joint_pos(finger.joint_names[2]),
    get_joint_pos(finger.joint_names[3])};
}

void TactileGraspController::set_planar_q(int finger_idx, const std::array<double, 3> & q)
{
  auto & finger = fingers_[finger_idx];
  finger.current_joint_targets[1] = q[0];
  finger.current_joint_targets[2] = q[1];
  finger.current_joint_targets[3] = q[2];
}

bool TactileGraspController::correction_ik(int finger_idx, bool forward_z)
{
  // Thumb does not use the planar IK correction.
  if (finger_idx == 0) {
    return false;
  }

  if (!joint_received_ || !finger_planar_ik_) {
    return false;
  }
  // index=1 -> 0, ..., little=4 -> 3
  const int solver_idx = finger_idx - 1;
  const auto current_q = get_planar_q(finger_idx);

  // Shift fingertip along the local z-axis.
  const double local_dy = 0.0;
  // 1mm * scale
  const double local_dz = forward_z ? 0.001 : -0.001;
  const auto maybe_q =
    finger_planar_ik_->solve_shift_local(solver_idx, current_q, local_dy, local_dz);

  if (!maybe_q.has_value()) {
    y_ik_failed_[finger_idx] = true;
    RCLCPP_WARN(this->get_logger(), "[%s] planar IK failed", fingers_[finger_idx].name.c_str());
    return false;
  }

  bool changed = false;
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(current_q[i] - maybe_q.value()[i]) > 1e-6) {
      changed = true;
      break;
    }
  }

  if (changed) {
    set_planar_q(finger_idx, maybe_q.value());
  }
  return changed;
}

bool TactileGraspController::get_target(const std::string & joint_name, double & target) const
{
  for (const auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      if (finger.joint_names[j] == joint_name) {
        target = finger.current_joint_targets[j];
        return true;
      }
    }
  }
  return false;
}

double TactileGraspController::get_joint_pos(const std::string & joint_name) const
{
  auto it = curr_joint_.find(joint_name);
  if (it != curr_joint_.end()) {
    return it->second;
  }
  return 0.0;
}

double TactileGraspController::get_open_pos(const std::string & joint_name) const
{
  for (size_t i = 0; i < hand_joint_names_.size(); ++i) {
    if (hand_joint_names_[i] == joint_name) {
      return init_positions_[i];
    }
  }
  return 0.0;
}

double TactileGraspController::clamp(double v, double min_v, double max_v) const
{
  return std::max(min_v, std::min(v, max_v));
}

}  // namespace robotis_hand_playground
