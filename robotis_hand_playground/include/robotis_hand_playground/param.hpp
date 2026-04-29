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

#include "rclcpp/rclcpp.hpp"

namespace robotis_hand_playground
{

// Parameters used by tactile grasp controllers.
struct Params
{
  // Common control parameters
  double control_hz = 20.0;
  double trajectory_dt = 0.05;
  double close_step = 0.01;
  double contact_threshold = 30.0;
  double thumb_contact_ratio = 2.0;
  std::string hand_side = "right";
  std::vector<int> un_use_finger;

  // Tactile CoP correction parameters
  double y_center = 0.5;
  double x_center = 0.2;
  double min_force_correction = 10.0;
  double cost_thres = 0.1;

  // Force maintenance parameters
  double reactive_force = 1.2;
  std::string state = "IDLE";

  // Optimization grasping parameters
  double feedback_max_delta = 0.01;
  double regrasp_force = 3.0;
  std::array<double, 3> link_lengths = {0.0235, 0.0355, 0.0355};
};

/**
 * @brief Declare ROS 2 parameters.
 */
void declare_params(rclcpp::Node * node);
/**
 * @brief Load ROS 2 parameters.
 */
Params load_params(rclcpp::Node * node);

}  // namespace robotis_hand_playground
