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
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "robotis_hand_playground/finger_ik_solver.hpp"
#include "robotis_hand_playground/hx5d20_init.hpp"
#include "robotis_hand_playground/hx5d20_struct.hpp"
#include "robotis_hand_playground/param.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotis_hand_playground
{
typedef std::array<double, 4> JointValueArray;

class CorrectionPlanner;

/**
 * @brief Base tactile grasp controller for CoP optimization and force regulation.
 */
class TactileGraspController : public rclcpp::Node {
public:
  enum class State
  {
    IDLE,
    CLOSE,
    HOLD
  };

  /**
   * @brief Construct tactile grasp controller.
   */
  explicit TactileGraspController(const std::string & node_name = "tactile_grasp_controller");
  virtual ~TactileGraspController();

  /**
   * @brief Initialize finger information.
   */
  void init_fingers();

  /**
   * @brief Initialize joint target values.
   */
  void init_joints();

  /**
   * @brief Configure hand-specific joint names and initial positions.
   */
  void configure_hand(const std::string & hand_side);

  /**
   * @brief IDLE state.
   */
  void handle_idle();

  /**
   * @brief Close fingers until tactile contact is detected.
   */
  void handle_close();

  /**
   * @brief Optimization grasping using tactile CoP feedback.
   */
  void handle_hold();

  /**
   * @brief Set desired force for each finger after contact detection.
   */
  void set_desired_force();

  /**
   * @brief Regulate grasping force for one finger using tactile feedback.
   */
  void regulate_grasp_force(int finger_idx);

  /**
   * @brief Close one finger by a small step during HOLD state.
   */
  void grasp_step_hold(int finger_idx, double step);

  /**
   * @brief Release one finger by a small step during HOLD state.
   */
  void release_step_hold(int finger_idx, double step);

  /**
   * @brief Apply weighted joint step to selected local joints.
   */
  void apply_ratio_step(
    int finger_idx,
    const std::array<int, 3> & local_joint_ids,
    const std::array<double, 3> & ratios,
    double signed_step);

  /**
   * @brief Shift each finger joint1 for lateral finger correction.
   */
  void shift_joint(int finger_idx, double delta);

  /**
   * @brief Shift thumb lateral correction.
   */
  void shift_thumb(int finger_idx, double delta1, double delta2);

  /**
   * @brief Close joints 2, 3, and 4 with weighted ratios.
   */
  void grasp_joint(int finger_idx, double step);

  /**
   * @brief Release joints 2, 3, and 4 with weighted ratios.
   */
  void release_joint(int finger_idx, double step);

  /**
   * @brief Reset contact states, correction plans, and target joints.
   */
  void reset_grasp();

  /**
   * @brief Return controller state and joint targets to the initial open posture.
   */
  void reset_to_init();

  /**
   * @brief Synchronize target joints with current joint states.
   */
  void sync_targets();

  /**
   * @brief Check whether all fingers have detected contact.
   */
  bool all_contacted() const;

  /**
   * @brief Check whether all active fingers satisfy the target contact force.
   */
  bool all_finger_contacted() const;

  /**
   * @brief Get contact threshold for each finger.
   */
  double finger_contact_threshold(int finger_idx) const;

  /**
   * @brief Calculate force-dependent step scale for each finger.
   */
  double finger_step_scale(int finger_idx) const;

  /**
   * @brief Get maximum filtered force among active fingers.
   */
  double max_filtered_force() const;

  /**
   * @brief Apply planar IK correction for fingertip motion.
   */
  bool correction_ik(int finger_idx, bool forward_z);

  /**
   * @brief Get planar IK joint values for joints 2, 3, and 4.
   */
  std::array<double, 3> get_planar_q(int finger_idx) const;

  /**
   * @brief Set planar IK joint values for joints 2, 3, and 4.
   */
  void set_planar_q(int finger_idx, const std::array<double, 3> & q);

  /**
   * @brief Check whether the finger is configured as unused.
   */
  bool unused_finger(int finger_idx) const;

  /**
   * @brief Move unused fingers to the closed posture.
   */
  void close_unused_finger();

  /**
   * @brief Get target joint value by joint name.
   */
  bool get_target(const std::string & joint_name, double & target) const;

  /**
   * @brief Get current joint position by joint name.
   */
  double get_joint_pos(const std::string & joint_name) const;

  /**
   * @brief Get initial open position by joint name.
   */
  double get_open_pos(const std::string & joint_name) const;

  /**
   * @brief Clamp value between minimum and maximum.
   */
  double clamp(double v, double min_v, double max_v) const;

protected:
  /**
   * @brief Publish current joint targets as a trajectory command.
   */
  virtual void publish_traj() = 0;

  /**
   * @brief Select correction direction from tactile CoP information.
   */
  virtual std::optional<CorrectionDecision> pick_correction(const CopInfo & info) const = 0;

protected:
  // Parameters
  friend class CorrectionPlanner;
  robotis_hand_playground::Params param;

  // IK and correction planner
  std::unique_ptr<FingerPlanarIk> finger_planar_ik_;
  std::unique_ptr<CorrectionPlanner> correction_planner_;

  // Hand joint states
  FingerArray fingers_;
  CorrectionPlanArray correction_plans_;
  std::array<double, fingers_num> desired_force_;

  std::vector<std::string> hand_joint_names_;
  std::vector<double> init_positions_;
  std::map<std::string, double> curr_joint_;

  // Controller state
  State state_{State::IDLE};
  HoldCorrectionStage hold_correction_stage_{HoldCorrectionStage::X_FIRST};
  bool joint_received_ = false;

  // Force control
  double force_kp_ = 0.002;  // Force feedback gain
  double deadband = 5.0;     // Deadband for small force errors

  // CoP correction
  int phase_step_ = 4;
  double x_corr_step_ = 0.03;
  double shift_step_ = 0.02;
  double regrasp_step_ = 0.02;

  // Finger step scaling
  double min_step_scale_ = 0.3;
  double max_step_scale_ = 1.0;

  std::array<bool, fingers_num> y_ik_failed_{false, false, false, false, false};
};

}  // namespace robotis_hand_playground
