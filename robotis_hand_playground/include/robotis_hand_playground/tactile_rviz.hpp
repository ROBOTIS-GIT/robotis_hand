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

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <mutex>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "robotis_interfaces/msg/tactile_sensor.hpp"

/**
 * @brief RViz marker publisher for tactile force and CoP visualization.
 */
class TactileRviz : public rclcpp::Node {
public:
  TactileRviz();

private:
  typedef robotis_interfaces::msg::HandPressures HandPressures;
  typedef robotis_interfaces::msg::TactileSensor TactileSensor;
  typedef std::array<std::array<float, 4>, 5> ColorArray;

  /**
   * @brief Direction region classified from tactile CoP.
   */
  enum DirectionRegion
  {
    CENTER = 0,
    UP = 1,
    DOWN = 2,
    LEFT = 3,
    RIGHT = 4
  };

  /**
   * @brief Tactile force direction information for one finger.
   */
  struct DirectionInfo
  {
    double total_force{0.0};
    double cop_x{0.0};
    double cop_y{0.0};
    double angle_rad{0.0};
    int region{CENTER};
    std::array<double, 3> vec{0.0, 0.0, 0.0};
  };

  /**
   * @brief Handle tactile pressure message.
   */
  void callback(const HandPressures::SharedPtr msg);

  /**
   * @brief Publish RViz markers and force summary topics.
   */
  void publish_markers();

  /**
   * @brief Initialize 3x3 tactile cell coordinates.
   */
  void init_taxel_positions();

  /**
   * @brief Convert tactile sensor name to finger index.
   */
  int finger_index_from_sensor(const std::string & sensor_name) const;

  /**
   * @brief Extract pressure values from tactile sensor message.
   */
  std::vector<double> extract_pressures(const TactileSensor & sensor_msg) const;

  /**
   * @brief Accumulate tactile baseline samples.
   */
  void accumulate_baseline(int finger_idx, const std::vector<double> & vals);

  /**
   * @brief Update filtered tactile pressure after baseline compensation.
   */
  void update_pressure(int finger_idx, const std::vector<double> & vals);

  /**
   * @brief Finalize tactile baseline values.
   */
  void finalize_baseline();

  /**
   * @brief Compute total tactile force from pressure values.
   */
  double compute_total_force(const std::vector<double> & p) const;

  /**
   * @brief Map sensor-frame vector to each finger link frame.
   */
  std::array<double, 3> map_sensor_vector_to_link(
    int finger_idx, double sx, double sy, double sn) const;

  /**
   * @brief Compute RViz force arrow vector from tactile pressure.
   */
  std::array<double, 3> compute_force_vector(int finger_idx, const std::vector<double> & p) const;

  /**
   * @brief Get marker color for each finger.
   */
  std::array<float, 4> finger_color(int finger_idx) const;

  /**
   * @brief Compute tactile direction information for one finger.
   */
  DirectionInfo compute_direction_info(int finger_idx, const std::vector<double> & p) const;

  /**
   * @brief Convert CoP position to marker point in finger frame.
   */
  geometry_msgs::msg::Point cop_point_in_frame(int finger_idx, const DirectionInfo & info) const;

  /**
   * @brief Create force arrow marker.
   */
  visualization_msgs::msg::Marker make_arrow_marker(
    int finger_idx, const DirectionInfo & info) const;

  /**
   * @brief Create CoP sphere marker.
   */
  visualization_msgs::msg::Marker make_cop_marker(int finger_idx, const DirectionInfo & info) const;

private:
  // Input and output topics
  std::string topic_;
  std::string marker_topic_;
  std::string marker_ns_;

  // Tactile sensor configuration
  std::string sensor_prefix_;
  int num_fingers_;
  int num_taxels_;
  std::vector<std::string> finger_frames_;

  // ROS timing and QoS
  double update_hz_;
  bool use_best_effort_;

  // Baseline and filtering
  double baseline_seconds_;
  double ema_alpha_;
  double deadband_;
  bool clip_negative_;

  // CoP and tactile geometry
  double center_region_ratio_;
  double taxel_pitch_x_;
  double taxel_pitch_y_;
  std::vector<std::pair<double, double>> taxel_xy_;

  // Force vector scaling
  double lateral_gain_;
  double normal_gain_;
  double normal_sign_;
  double force_to_arrow_scale_;
  double min_arrow_len_;
  double max_arrow_len_;

  // Marker dimensions
  double shaft_diameter_;
  double head_diameter_;
  double head_length_;
  double cop_marker_offset_;
  double cop_marker_scale_;

  // Tactile pressure buffers
  std::vector<std::vector<double>> pressure_;
  std::vector<std::vector<double>> ema_;
  std::vector<std::vector<double>> baseline_;
  std::vector<std::vector<double>> baseline_sum_;

  // Baseline state
  int baseline_count_;
  bool baseline_ready_;
  int baseline_frames_;
  std::vector<int> baseline_samples_per_finger_;

  // Thread lock
  std::mutex mutex_;

  // ROS interfaces
  rclcpp::Subscription<HandPressures>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr force_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr hand_total_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
