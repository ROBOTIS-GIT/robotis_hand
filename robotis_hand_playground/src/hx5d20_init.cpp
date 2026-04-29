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

#include "robotis_hand_playground/hx5d20_init.hpp"

namespace robotis_hand_playground
{

std::string normalize_hand_side(const std::string & hand_side)
{
  if (hand_side == "left" || hand_side == "l") {
    return "left";
  }
  return "right";
}

std::string hand_suffix(const std::string & hand_side)
{
  return normalize_hand_side(hand_side) == "left" ? "l" : "r";
}

std::string hand_namespace(const std::string & hand_side)
{
  return "/" + normalize_hand_side(hand_side) + "_hand";
}

std::string hand_controller_topic(const std::string & hand_side)
{
  return "/leader/joint_trajectory_command_broadcaster_" + normalize_hand_side(hand_side) +
         "_hand/joint_trajectory";
}

std::string tactile_sensor_prefix(const std::string & hand_side)
{
  return "finger_" + hand_suffix(hand_side) + "_sensor";
}

double thumb_joint_sign(const std::string & hand_side)
{
  return normalize_hand_side(hand_side) == "left" ? -1.0 : 1.0;
}

FingerArray init_fingers(const std::string & hand_side)
{
  FingerArray fingers{};
  const auto suffix = hand_suffix(hand_side);
  const bool is_left = normalize_hand_side(hand_side) == "left";

  // Thumb
  fingers[0].name = "thumb";
  fingers[0].joint_names = {
    "finger_" + suffix + "_joint1",
    "finger_" + suffix + "_joint2",
    "finger_" + suffix + "_joint3",
    "finger_" + suffix + "_joint4"};
  if (is_left) {
    fingers[0].joint_min = {-1.5, -0.5, -1.5, -1.5};
    fingers[0].joint_max = {1.5, 3.5, 1.5, 1.5};
  } else {
    fingers[0].joint_min = {-1.5, -3.5, -1.5, -1.5};
    fingers[0].joint_max = {1.5, 0.5, 1.5, 1.5};
  }

  // Index finger
  fingers[1].name = "index";
  fingers[1].joint_names = {
    "finger_" + suffix + "_joint5",
    "finger_" + suffix + "_joint6",
    "finger_" + suffix + "_joint7",
    "finger_" + suffix + "_joint8"};
  fingers[1].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers[1].joint_max = {0.6, 1.5, 1.5, 1.5};

  // Middle finger
  fingers[2].name = "middle";
  fingers[2].joint_names = {
    "finger_" + suffix + "_joint9",
    "finger_" + suffix + "_joint10",
    "finger_" + suffix + "_joint11",
    "finger_" + suffix + "_joint12"};
  fingers[2].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers[2].joint_max = {0.6, 1.5, 1.5, 1.5};

  // Ring finger
  fingers[3].name = "ring";
  fingers[3].joint_names = {
    "finger_" + suffix + "_joint13",
    "finger_" + suffix + "_joint14",
    "finger_" + suffix + "_joint15",
    "finger_" + suffix + "_joint16"};
  fingers[3].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers[3].joint_max = {0.6, 1.5, 1.5, 1.5};

  // Little finger
  fingers[4].name = "little";
  fingers[4].joint_names = {
    "finger_" + suffix + "_joint17",
    "finger_" + suffix + "_joint18",
    "finger_" + suffix + "_joint19",
    "finger_" + suffix + "_joint20"};
  fingers[4].joint_min = {-0.6, -1.5, -1.5, -1.5};
  fingers[4].joint_max = {0.6, 1.5, 1.5, 1.5};  // 범위 넓혀 놓음 = urdf 랑 joint 5, 17 매칭 필요

  for (auto & finger : fingers) {
    finger.current_joint_targets = {0.0, 0.0, 0.0, 0.0};
  }

  return fingers;
}

// clang-format off
std::vector<std::string> init_joint_names(const std::string & hand_side)
{
  const auto suffix = hand_suffix(hand_side);
  return {
    "finger_" + suffix + "_joint1", "finger_" + suffix + "_joint2",
    "finger_" + suffix + "_joint3", "finger_" + suffix + "_joint4",
    "finger_" + suffix + "_joint5", "finger_" + suffix + "_joint6",
    "finger_" + suffix + "_joint7", "finger_" + suffix + "_joint8",
    "finger_" + suffix + "_joint9", "finger_" + suffix + "_joint10",
    "finger_" + suffix + "_joint11", "finger_" + suffix + "_joint12",
    "finger_" + suffix + "_joint13", "finger_" + suffix + "_joint14",
    "finger_" + suffix + "_joint15", "finger_" + suffix + "_joint16",
    "finger_" + suffix + "_joint17", "finger_" + suffix + "_joint18",
    "finger_" + suffix + "_joint19", "finger_" + suffix + "_joint20"
  };
}

std::vector<double> init_r_positions()
{
  return {
    0.297, -1.792, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0
  };
}

std::vector<double> init_l_positions()
{
  return {
    -0.15, 1.792, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0,
    0.0, 0.8, 0.0, 0.0
  };
}

std::vector<double> init_positions(const std::string & hand_side)
{
  if (normalize_hand_side(hand_side) == "left") {
    return init_l_positions();
  }
  return init_r_positions();
}

}  // namespace robotis_hand_playground
