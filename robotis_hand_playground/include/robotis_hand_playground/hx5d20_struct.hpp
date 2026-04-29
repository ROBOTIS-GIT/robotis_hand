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
#include <string>
#include <vector>

namespace robotis_hand_playground
{

// HX5-D20 hand constants
constexpr int fingers_num = 5;
constexpr int tactiles_num = 9;
constexpr int joints_per_finger = 4;
typedef std::array<double, tactiles_num> PressureArray;

/**
 * @brief Tactile correction direction selected from CoP error.
 */
enum class CorrectionType
{
  NONE,
  Y_TOP,
  Y_BOT,
  X_LEFT,
  X_RIGHT
};

/**
 * @brief Correction stage used in HOLD state.
 */
enum class HoldCorrectionStage
{
  X_FIRST,
  Y_SECOND
};

/**
 * @brief CoP-based correction decision.
 */
struct CorrectionDecision
{
  CorrectionType type{CorrectionType::NONE};
  double cost = 0.0;
};

/**
 * @brief Multi-step correction plan for one finger.
 */
struct CorrectionPlan
{
  bool active = false;
  CorrectionType type{CorrectionType::NONE};
  int phase = 0;
  int ticks_remaining = 0;
  double cost = 0.0;
};
typedef std::array<CorrectionPlan, fingers_num> CorrectionPlanArray;

/**
 * @brief Tactile center-of-pressure information.
 */
struct CopInfo
{
  std::array<double, tactiles_num> pressure{};
  double total_force = 0.0;

  double cop_y = 0.0;
  double cop_x = 0.0;

  double cop_y_ratio = 0.0;
  double cop_x_ratio = 0.0;

  double x_left_cost = 0.0;
  double x_right_cost = 0.0;
  double y_top_cost = 0.0;
  double y_bot_cost = 0.0;
};

/**
 * @brief Per-finger joint and tactile state.
 */
struct FingerData
{
  std::string name;

  std::array<std::string, joints_per_finger> joint_names{};
  std::array<double, joints_per_finger> joint_min{};
  std::array<double, joints_per_finger> joint_max{};
  std::array<double, joints_per_finger> current_joint_targets{};

  bool contact_detected{false};

  std::array<double, tactiles_num> baseline_sum_tactiles{};
  std::array<double, tactiles_num> baseline_tactiles{};
  std::array<double, tactiles_num> ema_tactiles{};
  int baseline_samples = 0;

  double filtered_force = 0.0;
  CopInfo cop{};
};
typedef std::array<FingerData, fingers_num> FingerArray;

/**
 * @brief Parsed tactile sensor data for one finger.
 */
struct Hx5d20SensorData
{
  std::string name;
  std::array<std::string, tactiles_num> labels{};
  std::array<double, tactiles_num> values{};
};
typedef std::array<Hx5d20SensorData, fingers_num> SensorArray;

}  // namespace robotis_hand_playground
