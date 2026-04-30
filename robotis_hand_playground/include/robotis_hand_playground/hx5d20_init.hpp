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

#ifndef ROBOTIS_HAND_PLAYGROUND__HX5D20_INIT_HPP_
#define ROBOTIS_HAND_PLAYGROUND__HX5D20_INIT_HPP_

#include <string>
#include <vector>

#include "robotis_hand_playground/hx5d20_struct.hpp"


namespace robotis_hand_playground
{

/**
 * @brief Check a hand side string to "left" or "right".
 */
std::string check_hand_side(const std::string & hand_side);

/**
 * @brief Return the joint name suffix for the selected hand side.
 */
std::string hand_suffix(const std::string & hand_side);

/**
 * @brief Return the ROS namespace for the selected hand side.
 */
std::string hand_namespace(const std::string & hand_side);

/**
 * @brief Return the joint trajectory controller topic for the selected hand side.
 */
std::string hand_controller_topic(const std::string & hand_side);

/**
 * @brief Return thumb joint direction sign for the selected hand side.
 */
double thumb_joint_sign(const std::string & hand_side);

/**
 * @brief Initialize HX5-D20 finger joint configuration for the selected hand side.
 */
FingerArray init_fingers(const std::string & hand_side = "right");

/**
 * @brief Initialize ordered HX5-D20 joint names for the selected hand side.
 */
std::vector<std::string> init_joint_names(const std::string & hand_side = "right");

/**
 * @brief Initialize ordered HX5-D20 open joint positions for the right hand.
 */
std::vector<double> init_r_positions();

/**
 * @brief Initialize ordered HX5-D20 open joint positions for the left hand.
 */
std::vector<double> init_l_positions();

/**
 * @brief Select ordered HX5-D20 open joint positions for the selected hand side.
 */
std::vector<double> init_positions(const std::string & hand_side = "right");

}  // namespace robotis_hand_playground

#endif  // ROBOTIS_HAND_PLAYGROUND__HX5D20_INIT_HPP_
