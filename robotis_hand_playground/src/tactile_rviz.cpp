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

#include "robotis_hand_playground/tactile_rviz.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>


namespace
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

std::vector<std::string> finger_frame_names(const std::string & hand_side)
{
  const auto suffix = hand_suffix(hand_side);
  return {
    "finger_end_" + suffix + "_link1",
    "finger_end_" + suffix + "_link2",
    "finger_end_" + suffix + "_link3",
    "finger_end_" + suffix + "_link4",
    "finger_end_" + suffix + "_link5"};
}

}  // namespace

TactileRviz::TactileRviz()
: Node("tactile_rviz"), baseline_count_(0), baseline_ready_(false)
{
  const auto hand_side = normalize_hand_side(declare_parameter<std::string>("hand_side", "right"));

  // Declare input and output topics.
  topic_ = declare_parameter<std::string>(
    "topic", "/" + hand_side + "_hand/finger_pressures");
  marker_topic_ = declare_parameter<std::string>("marker_topic", "/tactile_force_markers");
  marker_ns_ = declare_parameter<std::string>("marker_ns", "tactile_force");

  // Declare tactile sensor configuration.
  sensor_prefix_ = declare_parameter<std::string>(
    "sensor_prefix", "finger_" + hand_suffix(hand_side) + "_sensor");
  num_fingers_ = declare_parameter<int>("num_fingers", 5);
  num_taxels_ = declare_parameter<int>("num_taxels", 9);

  finger_frames_ = declare_parameter<std::vector<std::string>>(
    "finger_frames", finger_frame_names(hand_side));

  // Declare ROS timing and QoS parameters.
  update_hz_ = declare_parameter<double>("update_hz", 20.0);
  use_best_effort_ = declare_parameter<bool>("use_best_effort", true);

  // Declare baseline and filtering parameters.
  baseline_seconds_ = declare_parameter<double>("baseline_seconds", 1.0);
  ema_alpha_ = declare_parameter<double>("ema_alpha", 0.25);
  deadband_ = declare_parameter<double>("deadband", 1.0);
  clip_negative_ = declare_parameter<bool>("clip_negative", true);

  // Declare tactile geometry and CoP parameters.
  center_region_ratio_ = declare_parameter<double>("center_region_ratio", 0.2);
  taxel_pitch_x_ = declare_parameter<double>("taxel_pitch_x", 0.003);
  taxel_pitch_y_ = declare_parameter<double>("taxel_pitch_y", 0.003);

  // Declare force vector scaling parameters.
  // Higher direction sensitivity increases lateral arrow response.
  lateral_gain_ = declare_parameter<double>("lateral_gain", 1.0);
  normal_gain_ = declare_parameter<double>("normal_gain", 0.3);
  normal_sign_ = declare_parameter<double>("normal_sign", 1.0);
  force_to_arrow_scale_ = declare_parameter<double>("force_to_arrow_scale", 0.0005);
  min_arrow_len_ = declare_parameter<double>("min_arrow_len", 0.01);
  max_arrow_len_ = declare_parameter<double>("max_arrow_len", 0.10);

  // Declare marker dimension parameters.
  shaft_diameter_ = declare_parameter<double>("shaft_diameter", 0.003);
  head_diameter_ = declare_parameter<double>("head_diameter", 0.006);
  head_length_ = declare_parameter<double>("head_length", 0.01);
  cop_marker_offset_ = declare_parameter<double>("cop_marker_offset", 0.01);
  cop_marker_scale_ = declare_parameter<double>("cop_marker_scale", 0.006);

  // Initialize pressure buffers.
  pressure_.assign(num_fingers_, std::vector<double>(num_taxels_, 0.0));
  ema_.assign(num_fingers_, std::vector<double>(num_taxels_, 0.0));
  baseline_.assign(num_fingers_, std::vector<double>(num_taxels_, 0.0));
  baseline_sum_.assign(num_fingers_, std::vector<double>(num_taxels_, 0.0));

  baseline_frames_ = std::max(1, static_cast<int>(update_hz_ * baseline_seconds_));
  baseline_samples_per_finger_.assign(num_fingers_, 0);

  init_taxel_positions();

  // Initialize ROS interfaces.
  auto qos =
    use_best_effort_ ? rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile() :
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  sub_ = create_subscription<HandPressures>(
    topic_, qos, std::bind(&TactileRviz::callback, this, std::placeholders::_1));

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  force_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/tactile_force", 10);
  hand_total_pub_ = create_publisher<std_msgs::msg::Float32>("/tactile_force/hand_total", 10);

  // Start marker publishing timer.
  auto period = std::chrono::duration<double>(1.0 / std::max(update_hz_, 1.0));
  timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&TactileRviz::publish_markers, this));
}

double TactileRviz::compute_total_force(const std::vector<double> & p) const
{
  return std::accumulate(p.begin(), p.end(), 0.0);
}

void TactileRviz::init_taxel_positions()
{
  taxel_xy_.clear();

  const std::array<double, 3> xs = {-taxel_pitch_x_, 0.0, taxel_pitch_x_};
  const std::array<double, 3> ys = {-taxel_pitch_y_, 0.0, taxel_pitch_y_};

  // Store 3x3 tactile coordinates in row-major order.
  for (double y : ys) {
    for (double x : xs) {
      taxel_xy_.push_back({x, y});
    }
  }
}

int TactileRviz::finger_index_from_sensor(const std::string & sensor_name) const
{
  if (sensor_name.rfind(sensor_prefix_, 0) != 0) {
    return -1;
  }

  const std::string suffix = sensor_name.substr(sensor_prefix_.size());
  if (suffix.empty()) {
    return -1;
  }

  for (char c : suffix) {
    if (!std::isdigit(c)) {
      return -1;
    }
  }

  const int idx = std::stoi(suffix) - 1;
  if (idx < 0 || idx >= num_fingers_) {
    return -1;
  }

  return idx;
}

std::vector<double> TactileRviz::extract_pressures(const TactileSensor & sensor_msg) const
{
  std::vector<double> vals(num_taxels_, 0.0);

  const size_t count =
    std::min(static_cast<size_t>(num_taxels_), sensor_msg.pressure_values.size());

  for (size_t i = 0; i < count; ++i) {
    vals[i] = static_cast<double>(sensor_msg.pressure_values[i]);
  }

  return vals;
}

void TactileRviz::callback(const HandPressures::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Update each finger pressure from tactile message.
  for (const auto & sensor : msg->sensors) {
    const int finger_idx = finger_index_from_sensor(sensor.sensor_name);
    if (finger_idx < 0) {
      continue;
    }

    const auto vals = extract_pressures(sensor);

    if (!baseline_ready_) {
      accumulate_baseline(finger_idx, vals);
    } else {
      update_pressure(finger_idx, vals);
    }
  }

  // Finalize baseline when all fingers have enough samples.
  if (!baseline_ready_) {
    bool ready = true;

    for (int f = 0; f < num_fingers_; ++f) {
      if (baseline_samples_per_finger_[f] < baseline_frames_) {
        ready = false;
        break;
      }
    }

    if (ready) {
      finalize_baseline();
    }
  }
}

void TactileRviz::accumulate_baseline(int finger_idx, const std::vector<double> & vals)
{
  for (int t = 0; t < num_taxels_; ++t) {
    baseline_sum_[finger_idx][t] += vals[t];
  }

  baseline_samples_per_finger_[finger_idx] += 1;
}

void TactileRviz::update_pressure(int finger_idx, const std::vector<double> & vals)
{
  for (int t = 0; t < num_taxels_; ++t) {
    double v = vals[t] - baseline_[finger_idx][t];

    if (clip_negative_ && v < 0.0) {
      v = 0.0;
    }

    if (v < deadband_) {
      v = 0.0;
    }

    // Apply exponential moving average filter.
    const double prev = ema_[finger_idx][t];
    ema_[finger_idx][t] = (1.0 - ema_alpha_) * prev + ema_alpha_ * v;
    pressure_[finger_idx][t] = ema_[finger_idx][t];
  }
}

void TactileRviz::finalize_baseline()
{
  for (int f = 0; f < num_fingers_; ++f) {
    const int count = std::max(baseline_samples_per_finger_[f], 1);

    for (int t = 0; t < num_taxels_; ++t) {
      baseline_[f][t] = baseline_sum_[f][t] / static_cast<double>(count);
      ema_[f][t] = 0.0;
      pressure_[f][t] = 0.0;
    }
  }

  baseline_ready_ = true;
  RCLCPP_INFO(get_logger(), "Baseline calibrated.");
}

std::array<double, 3> TactileRviz::map_sensor_vector_to_link(
  int finger_idx, double sx, double sy, double sn) const
{
  if (finger_idx == 0) {
    return {sx, -sy, sn};
  }

  return {sn, sx, -sy};
}

std::array<double, 3> TactileRviz::compute_force_vector(
  int finger_idx, const std::vector<double> & p) const
{
  const double total = compute_total_force(p);
  if (total <= 1e-6) {
    return {0.0, 0.0, 0.0};
  }

  // Compute tactile center of pressure.
  double cop_x = 0.0;
  double cop_y = 0.0;

  for (int i = 0; i < num_taxels_; ++i) {
    cop_x += p[i] * taxel_xy_[i].first;
    cop_y += p[i] * taxel_xy_[i].second;
  }

  cop_x /= total;
  cop_y /= total;

  // Normalize CoP and total force.
  const double cop_x_norm = cop_x / std::max(taxel_pitch_x_, 1e-9);
  const double cop_y_norm = cop_y / std::max(taxel_pitch_y_, 1e-9);
  const double total_norm = total / 100.0;

  // Reduce lateral gain for weak contact.
  double gain_scale = std::clamp((total - 20.0) / 80.0, 0.0, 1.0);
  gain_scale = gain_scale * gain_scale;

  const double effective_lateral_gain = lateral_gain_ * gain_scale;

  const double sx = effective_lateral_gain * cop_x_norm;
  const double sy = effective_lateral_gain * cop_y_norm;
  const double sn = normal_sign_ * normal_gain_ * total_norm;

  const auto v = map_sensor_vector_to_link(finger_idx, sx, sy, sn);

  const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (norm <= 1e-9) {
    return {0.0, 0.0, 0.0};
  }

  const double arrow_len =
    std::clamp(total * force_to_arrow_scale_, min_arrow_len_, max_arrow_len_);

  return {v[0] / norm * arrow_len, v[1] / norm * arrow_len, v[2] / norm * arrow_len};
}

TactileRviz::DirectionInfo TactileRviz::compute_direction_info(
  int finger_idx, const std::vector<double> & p) const
{
  DirectionInfo info;
  info.total_force = compute_total_force(p);

  if (info.total_force <= 1e-6) {
    info.region = CENTER;
    info.angle_rad = 0.0;
    info.vec = {0.0, 0.0, 0.0};
    return info;
  }

  // Compute tactile center of pressure.
  for (int i = 0; i < num_taxels_; ++i) {
    info.cop_x += p[i] * taxel_xy_[i].first;
    info.cop_y += p[i] * taxel_xy_[i].second;
  }

  info.cop_x /= info.total_force;
  info.cop_y /= info.total_force;

  // Classify CoP direction and force vector.
  info.region = 0;
  info.angle_rad = std::atan2(-info.cop_y, info.cop_x);
  info.vec = compute_force_vector(finger_idx, p);

  return info;
}

geometry_msgs::msg::Point TactileRviz::cop_point_in_frame(
  int finger_idx, const DirectionInfo & info) const
{
  geometry_msgs::msg::Point p;

  // Scale CoP offset for easier visualization.
  const double vis_cop_x = info.cop_x * 2.5;
  const double vis_cop_y = info.cop_y * 2.5;

  if (finger_idx == 0) {
    p.x = vis_cop_x;
    p.y = -vis_cop_y;
    p.z = cop_marker_offset_;
  } else {
    p.x = cop_marker_offset_;
    p.y = vis_cop_x;
    p.z = -vis_cop_y;
  }

  return p;
}

std::array<float, 4> TactileRviz::finger_color(int finger_idx) const
{
  static const ColorArray colors = {{
    {1.0f, 0.2f, 0.2f, 1.0f},    // Thumb
    {1.0f, 0.55f, 0.0f, 1.0f},   // Index
    {0.95f, 0.75f, 0.1f, 1.0f},  // Middle
    {0.2f, 0.9f, 0.2f, 1.0f},    // Ring
    {0.2f, 0.4f, 1.0f, 1.0f}     // Little
  }};

  return colors[finger_idx % colors.size()];
}

visualization_msgs::msg::Marker TactileRviz::make_arrow_marker(
  int finger_idx, const DirectionInfo & info) const
{
  visualization_msgs::msg::Marker m;
  m.header.stamp = now();
  m.header.frame_id = finger_frames_[finger_idx];
  m.ns = marker_ns_;
  m.id = finger_idx;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.frame_locked = true;

  geometry_msgs::msg::Point end = cop_point_in_frame(finger_idx, info);
  geometry_msgs::msg::Point start = end;

  start.x += info.vec[0];
  start.y += info.vec[1];
  start.z += info.vec[2];

  m.points.push_back(start);
  m.points.push_back(end);

  m.scale.x = shaft_diameter_;
  m.scale.y = head_diameter_;
  m.scale.z = head_length_;

  const auto c = finger_color(finger_idx);
  m.color.r = c[0];
  m.color.g = c[1];
  m.color.b = c[2];
  m.color.a = c[3];

  // Hide marker for weak contact.
  if (info.total_force <= 5.0) {
    m.color.a = 0.0f;
  }

  return m;
}

visualization_msgs::msg::Marker TactileRviz::make_cop_marker(
  int finger_idx, const DirectionInfo & info) const
{
  visualization_msgs::msg::Marker m;
  m.header.stamp = now();
  m.header.frame_id = finger_frames_[finger_idx];
  m.ns = marker_ns_ + "_cop";
  m.id = finger_idx;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.frame_locked = true;

  // Hide marker for weak contact.
  if (info.total_force <= 5.0) {
    m.scale.x = 0.001;
    m.scale.y = 0.001;
    m.scale.z = 0.001;
    m.color.a = 0.0;
    return m;
  }

  m.pose.position = cop_point_in_frame(finger_idx, info);
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 1.0;

  m.scale.x = cop_marker_scale_;
  m.scale.y = cop_marker_scale_;
  m.scale.z = cop_marker_scale_;

  const auto c = finger_color(finger_idx);
  m.color.r = c[0];
  m.color.g = c[1];
  m.color.b = c[2];
  m.color.a = c[3];

  return m;
}

void TactileRviz::publish_markers()
{
  visualization_msgs::msg::MarkerArray markers;
  std_msgs::msg::Float32MultiArray force_msg;
  std_msgs::msg::Float32 hand_total_msg;

  std::lock_guard<std::mutex> lock(mutex_);

  // Publish [region, angle_deg, total_force] for each finger.
  force_msg.data.reserve(num_fingers_ * 3);

  double hand_total_force = 0.0;

  for (int f = 0; f < num_fingers_; ++f) {
    const auto info = compute_direction_info(f, pressure_[f]);

    markers.markers.push_back(make_cop_marker(f, info));
    markers.markers.push_back(make_arrow_marker(f, info));

    const double angle_deg = info.angle_rad * 180.0 / M_PI;

    force_msg.data.push_back(static_cast<float>(info.region));
    force_msg.data.push_back(static_cast<float>(angle_deg));
    force_msg.data.push_back(static_cast<float>(info.total_force));

    hand_total_force += info.total_force;
  }

  hand_total_msg.data = static_cast<float>(hand_total_force);

  marker_pub_->publish(markers);
  force_pub_->publish(force_msg);
  hand_total_pub_->publish(hand_total_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TactileRviz>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
