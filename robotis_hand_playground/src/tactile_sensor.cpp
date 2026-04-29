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

#include "robotis_hand_playground/tactile_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace robotis_hand_playground
{

TactileSensor::TactileSensor(const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr & clock)
: logger_(logger), clock_(clock)
{
  init_tactiles();
}

void TactileSensor::set_params(const robotis_hand_playground::Params & params)
{
  param = params;
}

void TactileSensor::init_tactiles()
{
  const double x_offset = tactile_x_ / 3.0;
  const double y_offset = tactile_y_ / 3.0;

  const std::array<double, 3> xs = {-x_offset, 0.0, x_offset};
  const std::array<double, 3> ys = {-y_offset, 0.0, y_offset};

  int idx = 0;
  for (double y : ys) {
    for (double x : xs) {
      tactile_xy_[idx++] = {x, y};
    }
  }
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

void TactileSensor::update_finger_state(
  int finger_idx, FingerData & finger, const Hx5d20SensorData & sensor) const
{
  const auto filtered = filter_pressure(finger, sensor);
  const double total_force = calc_total_force(filtered);

  // Update filtered total force.
  finger.filtered_force = (1.0 - ema_alpha_) * finger.filtered_force + ema_alpha_ * total_force;

  // Update center of pressure information.
  finger.cop = calc_cop(finger_idx, filtered);
}

void TactileSensor::update_pressure(
  FingerArray & fingers, bool & baseline, const SensorArray & sensors)
{
  if (update_baseline(fingers, baseline, sensors)) {
    return;
  }

  for (int f = 0; f < fingers_num; ++f) {
    update_finger_state(f, fingers[f], sensors[f]);
  }
}

CopInfo TactileSensor::calc_cop(int finger_idx, const PressureArray & pressure) const
{
  CopInfo info;
  info.pressure = pressure;
  info.total_force = calc_total_force(pressure);

  if (info.total_force <= 1e-9) {
    return info;
  }
  // Calculate weighted center of pressure.
  for (int i = 0; i < tactiles_num; ++i) {
    info.cop_x += pressure[i] * tactile_xy_[i].first;
    info.cop_y += pressure[i] * tactile_xy_[i].second;
  }
  info.cop_x /= info.total_force;
  info.cop_y /= info.total_force;

  // Normalize CoP to [-1, 1].
  // x: left(-) to right(+), y: top(-) to bottom(+)
  const double half_x = tactile_x_ * 0.5;
  const double half_y = tactile_y_ * 0.5;

  info.cop_x_ratio = clamp(info.cop_x / std::max(half_x, 1e-9), -1.0, 1.0);
  info.cop_y_ratio = clamp(info.cop_y / std::max(half_y, 1e-9), -1.0, 1.0);

  // Apply wider dead zone for thumb.
  const double thumb_ = 0.45;
  const double x_center = param.x_center + ((finger_idx == 0) ? thumb_ : 0.0);
  const double y_center = param.y_center + ((finger_idx == 0) ? thumb_ : 0.0);

  // Calculate left/right correction cost from x-axis CoP.
  const double abs_x_ratio = std::fabs(info.cop_x_ratio);

  if (abs_x_ratio > x_center) {
    const double raw_cost_x = (abs_x_ratio - x_center) / std::max(1.0 - x_center, 1e-9);
    const double cost_x = clamp(raw_cost_x, 0.0, 1.0);

    if (info.cop_x_ratio < 0.0) {
      info.x_left_cost = cost_x;
      info.x_right_cost = 0.0;
    } else {
      info.x_left_cost = 0.0;
      info.x_right_cost = cost_x;
    }
  }
  // Calculate top/bottom correction cost from y-axis CoP.
  const double abs_y_ratio = std::fabs(info.cop_y_ratio);

  if (abs_y_ratio > y_center) {
    const double raw_cost_y = (abs_y_ratio - y_center) / std::max(1.0 - y_center, 1e-9);
    const double cost_y = clamp(raw_cost_y, 0.0, 1.0);

    if (info.cop_y_ratio < 0.0) {
      info.y_top_cost = cost_y;
      info.y_bot_cost = 0.0;
    } else {
      info.y_top_cost = 0.0;
      info.y_bot_cost = cost_y;
    }
  }
  return info;
}

std::optional<CorrectionDecision> TactileSensor::pick_correction(const CopInfo & info) const
{
  if (info.total_force < param.min_force_correction) {
    return std::nullopt;
  }
  // Select left/right correction first.
  const double cost_x = std::max(info.x_left_cost, info.x_right_cost);
  if (cost_x >= param.cost_thres) {
    if (info.x_left_cost > info.x_right_cost) {
      return CorrectionDecision{CorrectionType::X_LEFT, info.x_left_cost};
    } else {
      return CorrectionDecision{CorrectionType::X_RIGHT, info.x_right_cost};
    }
  }
  // Select top/bottom correction.
  const double cost_y = std::max(info.y_top_cost, info.y_bot_cost);
  if (cost_y >= param.cost_thres) {
    if (info.y_top_cost > info.y_bot_cost) {
      return CorrectionDecision{CorrectionType::Y_TOP, info.y_top_cost};
    } else {
      return CorrectionDecision{CorrectionType::Y_BOT, info.y_bot_cost};
    }
  }
  return std::nullopt;
}

double TactileSensor::clamp(double v, double min_v, double max_v) const
{
  return std::max(min_v, std::min(v, max_v));
}

}  // namespace robotis_hand_playground
