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

#ifndef ROBOTIS_HAND_PLAYGROUND__HX5D20_STRUCT_HPP_
#define ROBOTIS_HAND_PLAYGROUND__HX5D20_STRUCT_HPP_

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

#endif  // ROBOTIS_HAND_PLAYGROUND__HX5D20_STRUCT_HPP_
