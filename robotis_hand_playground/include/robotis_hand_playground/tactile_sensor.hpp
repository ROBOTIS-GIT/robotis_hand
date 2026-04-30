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

#ifndef ROBOTIS_HAND_PLAYGROUND__TACTILE_SENSOR_HPP_
#define ROBOTIS_HAND_PLAYGROUND__TACTILE_SENSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <robotis_interfaces/msg/hand_pressures.hpp>

#include "robotis_hand_playground/hx5d20_struct.hpp"


namespace robotis_hand_playground
{

/**
 * @brief Handles tactile sensor parsing, filtering, and baseline compensation.
 */
class TactileSensor {
public:
  using HandPressuresMsg = robotis_interfaces::msg::HandPressures;
  using HandPressuresPtr = HandPressuresMsg::SharedPtr;

  TactileSensor(const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock);

  /**
   * @brief Check tactile pressure message size.
   */
  bool check_msg(const HandPressuresPtr msg) const;

  /**
   * @brief Convert tactile pressure message into internal sensor array.
   */
  SensorArray parse_sensors(const HandPressuresPtr msg) const;

  /**
   * @brief Update tactile pressure, baseline, filtered force, and CoP state.
   */
  void update_pressure(FingerArray & fingers, bool & baseline, const SensorArray & sensors);

private:
  /**
   * @brief Collect and update tactile baseline values.
   */
  bool update_baseline(FingerArray & fingers, bool & baseline, const SensorArray & sensors);

  /**
   * @brief Apply baseline compensation and EMA filter to tactile pressure.
   */
  PressureArray filter_pressure(FingerData & finger, const Hx5d20SensorData & sensor) const;

  /**
   * @brief Calculate total tactile force from filtered pressure.
   */
  double calc_total_force(const PressureArray & pressure) const;

  /**
   * @brief Update one finger tactile state.
   */
  void update_finger_state(FingerData & finger, const Hx5d20SensorData & sensor) const;

private:
  // ROS interfaces
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Filtering and baseline
  double ema_alpha_{0.2};
  int baseline_sample_count_{30};
};

}  // namespace robotis_hand_playground

#endif  // ROBOTIS_HAND_PLAYGROUND__TACTILE_SENSOR_HPP_
