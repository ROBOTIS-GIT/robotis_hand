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

#include <algorithm>
#include <cstddef>
#include <numeric>

#include "robotis_hand_playground/tactile_sensor.hpp"


namespace robotis_hand_playground
{

TactileSensor::TactileSensor(const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock)
: logger_(logger), clock_(clock)
{
}

bool TactileSensor::check_msg(const HandPressuresPtr msg) const
{
  if (msg->sensors.size() != fingers_num) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "sensors size mismatch: %zu", msg->sensors.size());
    return false;
  }
  for (size_t i = 0; i < msg->sensors.size(); ++i) {
    if (msg->sensors[i].pressure_names.size() != tactiles_num) {
      RCLCPP_WARN_THROTTLE(logger_,
        *clock_,
        2000,
        "sensor[%zu] pressure_names size mismatch: %zu",
        i,
        msg->sensors[i].pressure_names.size());
      return false;
    }

    if (msg->sensors[i].pressure_values.size() != tactiles_num) {
      RCLCPP_WARN_THROTTLE(logger_,
        *clock_,
        2000,
        "sensor[%zu] pressure_values size mismatch: %zu",
        i,
        msg->sensors[i].pressure_values.size());
      return false;
    }
  }
  return true;
}

SensorArray TactileSensor::parse_sensors(const HandPressuresPtr msg) const
{
  SensorArray out{};

  // Copy tactile sensor names and 3x3 pressure values.
  for (size_t i = 0; i < fingers_num; ++i) {
    out[i].name = msg->sensors[i].sensor_name;

    for (size_t j = 0; j < tactiles_num; ++j) {
      out[i].labels[j] = msg->sensors[i].pressure_names[j];
      out[i].values[j] = static_cast<double>(msg->sensors[i].pressure_values[j]);
    }
  }
  return out;
}

bool TactileSensor::update_baseline(
  FingerArray & fingers, bool & baseline, const SensorArray & sensors)
{
  if (baseline) {
    return false;
  }
  // Accumulate baseline samples.
  for (int f = 0; f < fingers_num; ++f) {
    for (int t = 0; t < tactiles_num; ++t) {
      fingers[f].baseline_sum_tactiles[t] += sensors[f].values[t];
    }
    fingers[f].baseline_samples++;
  }
  // Wait until all fingers collect enough samples.
  bool ready = true;
  for (int f = 0; f < fingers_num; ++f) {
    if (fingers[f].baseline_samples < baseline_sample_count_) {
      ready = false;
      break;
    }
  }
  if (!ready) {
    return true;
  }

  // Store averaged baseline and reset EMA values.
  for (int f = 0; f < fingers_num; ++f) {
    const int count = std::max(1, fingers[f].baseline_samples);

    for (int t = 0; t < tactiles_num; ++t) {
      fingers[f].baseline_tactiles[t] =
        fingers[f].baseline_sum_tactiles[t] / static_cast<double>(count);
      fingers[f].ema_tactiles[t] = 0.0;
    }
  }
  baseline = true;
  RCLCPP_INFO(logger_, "Tactile-wise baseline ready.");

  return true;
}

PressureArray TactileSensor::filter_pressure(
  FingerData & finger, const Hx5d20SensorData & sensor) const
{
  PressureArray filtered{};

  for (int t = 0; t < tactiles_num; ++t) {
    double value = sensor.values[t] - finger.baseline_tactiles[t];

    // Ignore pressure values lower than baseline.
    if (value < 0.0) {
      value = 0.0;
    }

    // Apply exponential moving average filter.
    finger.ema_tactiles[t] = (1.0 - ema_alpha_) * finger.ema_tactiles[t] + ema_alpha_ * value;

    filtered[t] = finger.ema_tactiles[t];
  }

  return filtered;
}

double TactileSensor::calc_total_force(const PressureArray & pressure) const
{
  return std::accumulate(pressure.begin(), pressure.end(), 0.0);
}

void TactileSensor::update_finger_state(FingerData & finger, const Hx5d20SensorData & sensor) const
{
  const auto filtered = filter_pressure(finger, sensor);
  const double total_force = calc_total_force(filtered);

  // Update filtered total force.
  finger.filtered_force = (1.0 - ema_alpha_) * finger.filtered_force + ema_alpha_ * total_force;
}

void TactileSensor::update_pressure(
  FingerArray & fingers, bool & baseline, const SensorArray & sensors)
{
  if (update_baseline(fingers, baseline, sensors)) {
    return;
  }

  for (int f = 0; f < fingers_num; ++f) {
    update_finger_state(fingers[f], sensors[f]);
  }
}

}  // namespace robotis_hand_playground
