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

#include "robotis_hand_playground/correction_planner.hpp"

#include "robotis_hand_playground/tactile_grasp_controller.hpp"

namespace robotis_hand_playground
{

CorrectionPlanner::CorrectionPlanner(TactileGraspController & controller)
: controller_(controller)
{}

void CorrectionPlanner::start_correction(int finger_idx)
{
  const auto maybe_decision = controller_.pick_correction(controller_.fingers_[finger_idx].cop);
  if (!maybe_decision.has_value()) {
    return;
  }

  // Run only the correction type allowed by the current hold correction stage.
  if (controller_.hold_correction_stage_ == HoldCorrectionStage::X_FIRST &&
    !is_x_correction(maybe_decision->type))
  {
    return;
  }
  if (controller_.hold_correction_stage_ == HoldCorrectionStage::Y_SECOND &&
    !is_y_correction(maybe_decision->type))
  {
    return;
  }

  // Skip correction if the required joint is already at the limit.
  if (!is_at_joint_limit(finger_idx, maybe_decision->type)) {
    return;
  }

  // Initialize correction plan.
  auto & plan = controller_.correction_plans_[finger_idx];
  plan.type = maybe_decision->type;
  plan.cost = maybe_decision->cost;
  plan.phase = 0;
  plan.active = true;

  switch (plan.type) {
    case CorrectionType::X_LEFT:
    case CorrectionType::X_RIGHT:
    case CorrectionType::Y_TOP:
    case CorrectionType::Y_BOT:
      plan.ticks_remaining = controller_.phase_step_;
      break;
    default:
      plan.active = false;
      break;
  }

  RCLCPP_INFO(controller_.get_logger(),
    "[%s] correction start: %s (cost=%.3f, cop_x_ratio=%.3f, "
    "cop_y_ratio=%.3f)",
    controller_.fingers_[finger_idx].name.c_str(),
    correction_str(plan.type).c_str(),
    plan.cost,
    controller_.fingers_[finger_idx].cop.cop_x_ratio,
    controller_.fingers_[finger_idx].cop.cop_y_ratio);
}

void CorrectionPlanner::apply_correction(int finger_idx)
{
  auto & plan = controller_.correction_plans_[finger_idx];
  const bool done = run_correction(finger_idx, plan);

  if (done) {
    plan = CorrectionPlan{};
    RCLCPP_INFO(controller_.get_logger(),
      "[%s] correction done",
      controller_.fingers_[finger_idx].name.c_str());
  }
}

bool CorrectionPlanner::run_correction(int finger_idx, CorrectionPlan & plan)
{
  if (!plan.active) {
    return true;
  }

  switch (plan.type) {
    case CorrectionType::X_LEFT:
      return run_x_correction(finger_idx, plan, 1);

    case CorrectionType::X_RIGHT:
      return run_x_correction(finger_idx, plan, -1);

    case CorrectionType::Y_TOP:
      return run_y_correction(finger_idx, plan, true);

    case CorrectionType::Y_BOT:
      return run_y_correction(finger_idx, plan, false);

    default:
      return true;
  }
}

bool CorrectionPlanner::run_x_correction(
  int finger_idx, CorrectionPlan & plan, int direction)
{
  const double scale = controller_.finger_step_scale(finger_idx);
  // Phase 0: release joints before lateral correction.
  if (plan.phase == 0) {
    controller_.release_joint(finger_idx, controller_.x_corr_step_ * scale);
    plan.ticks_remaining--;
    if (plan.ticks_remaining <= 0) {
      plan.phase = 1;
      plan.ticks_remaining = controller_.phase_step_;
    }
    return false;
  }
  // Phase 1: shift lateral joint direction.
  if (plan.phase == 1) {
    if (finger_idx == 0) {
      controller_.shift_thumb(
        finger_idx, direction * controller_.shift_step_, direction * controller_.shift_step_);
    } else {
      controller_.shift_joint(finger_idx, -direction * controller_.shift_step_);
    }
    plan.ticks_remaining--;
    if (plan.ticks_remaining <= 0) {
      plan.phase = 2;
      plan.ticks_remaining = controller_.phase_step_;
    }
    return false;
  }
  // Phase 2: re-grasp after lateral shift.
  if (plan.phase == 2) {
    controller_.grasp_joint(finger_idx, controller_.x_corr_step_ * scale);
    plan.ticks_remaining--;
    return plan.ticks_remaining <= 0;
  }
  return true;
}

bool CorrectionPlanner::run_y_correction(
  int finger_idx, CorrectionPlan & plan, bool forward_y)
{
  const bool moved = controller_.correction_ik(finger_idx, forward_y);
  plan.ticks_remaining--;
  return !moved || plan.ticks_remaining <= 0;
}

bool CorrectionPlanner::is_at_joint_limit(int finger_idx, CorrectionType type) const
{
  const auto & finger = controller_.fingers_[finger_idx];
  const double eps = 1e-6;
  const bool is_thumb = (finger_idx == 0);

  auto max_limit = [&](int joint_idx) {
      return finger.current_joint_targets[joint_idx] < finger.joint_max[joint_idx] - eps;
    };

  auto min_limit = [&](int joint_idx) {
      return finger.current_joint_targets[joint_idx] > finger.joint_min[joint_idx] + eps;
    };

  switch (type) {
    case CorrectionType::X_LEFT:
      if (is_thumb) {
        return max_limit(0) || max_limit(1);
      }
      return min_limit(0);

    case CorrectionType::X_RIGHT:
      if (is_thumb) {
        return min_limit(0) || min_limit(1);
      }
      return max_limit(0);

    case CorrectionType::Y_TOP:
    case CorrectionType::Y_BOT:
      return (min_limit(1) && max_limit(1)) || (min_limit(2) && max_limit(2)) ||
             (min_limit(3) && max_limit(3));

    default:
      return false;
  }
}

bool CorrectionPlanner::is_x_correction(CorrectionType type) const
{
  return type == CorrectionType::X_LEFT || type == CorrectionType::X_RIGHT;
}

bool CorrectionPlanner::is_y_correction(CorrectionType type) const
{
  return type == CorrectionType::Y_TOP || type == CorrectionType::Y_BOT;
}

bool CorrectionPlanner::correction_blocked(HoldCorrectionStage stage) const
{
  for (int i = 0; i < fingers_num; ++i) {
    const auto maybe_decision = controller_.pick_correction(controller_.fingers_[i].cop);
    if (!maybe_decision.has_value()) {
      continue;
    }
    if (stage == HoldCorrectionStage::X_FIRST && !is_x_correction(maybe_decision->type)) {
      continue;
    }
    if (stage == HoldCorrectionStage::Y_SECOND && !is_y_correction(maybe_decision->type)) {
      continue;
    }
    if (is_at_joint_limit(i, maybe_decision->type)) {
      return false;
    }
  }
  return true;
}

std::string CorrectionPlanner::correction_str(CorrectionType t) const
{
  switch (t) {
    case CorrectionType::NONE:
      return "NONE";
    case CorrectionType::X_LEFT:
      return "X_LEFT";
    case CorrectionType::X_RIGHT:
      return "X_RIGHT";
    case CorrectionType::Y_TOP:
      return "Y_TOP";
    case CorrectionType::Y_BOT:
      return "Y_BOT";
    default:
      return "UNKNOWN";
  }
}

}  // namespace robotis_hand_playground
