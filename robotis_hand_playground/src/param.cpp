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

#include "robotis_hand_playground/param.hpp"


namespace robotis_hand_playground
{

void declare_params(rclcpp::Node * node)
{
  // Common control parameters
  node->declare_parameter<double>("control_hz", 20.0);
  node->declare_parameter<double>("trajectory_dt", 0.05);
  node->declare_parameter<double>("close_step", 0.01);
  node->declare_parameter<double>("contact_threshold", 30.0);
  node->declare_parameter<double>("thumb_contact_ratio", 2.0);
  node->declare_parameter<std::string>("hand_side", "right");
  node->declare_parameter<std::vector<int64_t>>("un_use_finger", std::vector<int64_t>{});

  // Force maintenance controller parameters
  node->declare_parameter<double>("reactive_force", 1.2);
  node->declare_parameter<std::string>("state", "IDLE");
}

Params load_params(rclcpp::Node * node)
{
  Params p;

  // Common control parameters
  node->get_parameter("control_hz", p.control_hz);
  node->get_parameter("trajectory_dt", p.trajectory_dt);
  node->get_parameter("close_step", p.close_step);
  node->get_parameter("contact_threshold", p.contact_threshold);
  node->get_parameter("thumb_contact_ratio", p.thumb_contact_ratio);
  node->get_parameter("hand_side", p.hand_side);

  std::vector<int64_t> un_use_finger_tmp{};
  node->get_parameter("un_use_finger", un_use_finger_tmp);
  p.un_use_finger.clear();
  for (const auto value : un_use_finger_tmp) {
    if (value == 0) {
      continue;  // NONE
    }
    const int finger_idx = static_cast<int>(value - 1);
    if (finger_idx >= 0 && finger_idx < 5) {
      p.un_use_finger.push_back(finger_idx);
    }
  }

  // Force maintenance controller parameters
  node->get_parameter("reactive_force", p.reactive_force);
  node->get_parameter("state", p.state);

  return p;
}

}  // namespace robotis_hand_playground
