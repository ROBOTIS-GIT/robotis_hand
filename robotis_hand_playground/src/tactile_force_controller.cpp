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
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "robotis_hand_playground/tactile_force_controller.hpp"


using namespace std::chrono_literals;

namespace robotis_hand_tactile_force
{

TactileForceController::TactileForceController()
: Node("tactile_force_controller"), tactile_sensor_(this->get_logger(), this->get_clock())
{
  // Load parameters.
  declare_params(this);
  param = load_params(this);

  // Initialize hand model.
  fingers_ = init_fingers(param.hand_side);
  hand_joint_names_ = init_joint_names(param.hand_side);
  init_positions_ = init_positions(param.hand_side);

  // Initialize ROS interfaces.
  pressure_sub_ = this->create_subscription<robotis_interfaces::msg::HandPressures>(
    hand_namespace(param.hand_side) + "/finger_pressures",
    10,
    std::bind(&TactileForceController::pressure_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
    10,
    std::bind(&TactileForceController::joint_state_callback, this, std::placeholders::_1));

  grasp_start_sub_ = this->create_subscription<std_msgs::msg::Bool>("/grasp_start",
    10,
    std::bind(&TactileForceController::grasp_start_callback, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    hand_controller_topic(param.hand_side), 10);

  // Move unused fingers independently.
  unused_finger_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        close_unused_finger();
        publish_traj();
  });

  // Start control loop.
  const auto period = std::chrono::duration<double>(1.0 / param.control_hz);
  control_timer_ =
    this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TactileForceController::control_loop, this));

  // Initialize joint targets with open positions.
  for (auto & finger : fingers_) {
    for (int j = 0; j < 4; ++j) {
      finger.current_joint_targets[j] = get_open_pos(finger.joint_names[j]);
    }
  }
  RCLCPP_INFO(this->get_logger(), "TactileForceController initialized.");
}

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

std::string check_hand_side(const std::string & hand_side)
{
  if (hand_side == "left" || hand_side == "l") {
    return "left";
  }
  return "right";
}

std::string hand_suffix(const std::string & hand_side)
{
  return check_hand_side(hand_side) == "left" ? "l" : "r";
}

std::string hand_namespace(const std::string & hand_side)
{
  return "/" + check_hand_side(hand_side) + "_hand";
}

std::string hand_controller_topic(const std::string & hand_side)
{
  return "/leader/joint_trajectory_command_broadcaster_" + check_hand_side(hand_side) +
         "_hand/joint_trajectory";
}

double thumb_joint_sign(const std::string & hand_side)
{
  return check_hand_side(hand_side) == "left" ? -1.0 : 1.0;
}

robotis_hand_playground::FingerArray init_fingers(const std::string & hand_side)
{
  robotis_hand_playground::FingerArray fingers{};
  const auto suffix = hand_suffix(hand_side);
  const bool is_left = check_hand_side(hand_side) == "left";

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
  fingers[4].joint_max = {0.6, 1.5, 1.5, 1.5};

  for (auto & finger : fingers) {
    finger.current_joint_targets = {0.0, 0.0, 0.0, 0.0};
  }

  return fingers;
}

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
  if (check_hand_side(hand_side) == "left") {
    return init_l_positions();
  }
  return init_r_positions();
}

void TactileForceController::pressure_callback(
  const robotis_interfaces::msg::HandPressures::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!tactile_sensor_.check_msg(msg)) {
    return;
  }
  // Parse tactile data and update filtered pressure states.
  const auto sensors = tactile_sensor_.parse_sensors(msg);
  tactile_sensor_.update_pressure(fingers_, baseline_, sensors);
}

void TactileForceController::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Store current joint positions by joint name.
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    curr_joint_[msg->name[i]] = msg->position[i];
  }
  joint_state_received_ = true;
}

void TactileForceController::grasp_start_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Start grasping when /grasp_start receives true.
  if (msg->data) {
    if (state_ == State::IDLE) {
      reset_grasp();
      state_ = State::CLOSE;
      RCLCPP_INFO(this->get_logger(), "grasp_start=true received -> State = CLOSE");
    }
    return;
  }

  reset_to_init();
  publish_traj();
  RCLCPP_INFO(this->get_logger(), "grasp_start=false received -> State = IDLE");
}

void TactileForceController::control_loop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Run state-specific controller logic.
  switch (state_) {
    case State::IDLE:
      handle_idle();
      break;
    case State::CLOSE:
      handle_close();
      break;
    case State::HOLD:
      handle_hold();
      break;
    default:
      break;
  }
}

bool TactileForceController::unused_finger(int finger_idx) const
{
  return std::find(param.un_use_finger.begin(), param.un_use_finger.end(), finger_idx) !=
         param.un_use_finger.end();
}

void TactileForceController::close_unused_finger()
{
  for (int i = 1; i < fingers_num; ++i) {
    if (!unused_finger(i)) {
      continue;
    }
    auto & finger = fingers_[i];

    // Move unused fingers to the closed posture.
    for (int j = 1; j <= 3; ++j) {
      finger.current_joint_targets[j] += param.close_step * 3;
      finger.current_joint_targets[j] =
        clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
    }
  }
}

void TactileForceController::handle_idle()
{
  // IDLE
}

double TactileForceController::finger_contact_threshold(int finger_idx) const
{
  if (finger_idx == 0) {
    return param.contact_threshold * param.thumb_contact_ratio;
  }
  return param.contact_threshold;
}

void TactileForceController::handle_close()
{
  for (int i = 0; i < fingers_num; ++i) {
    auto & finger = fingers_[i];

    if (unused_finger(i)) {
      finger.contact_detected = true;
      desired_force_[i] = 0.0;
      continue;
    }

    if (!finger.contact_detected) {
      if (i == 0) {
        // Thumb closes using joint3 and joint4.
        const std::array<double, 4> weights = {0.5, 0.5, 0.5, 0.5};
        const double direction = thumb_joint_sign(param.hand_side);

        for (int j = 2; j <= 3; ++j) {
          finger.current_joint_targets[j] += direction * param.close_step * weights[j];
          finger.current_joint_targets[j] =
            clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      } else {
        // Other fingers close using joint2, joint3, and joint4.
        const std::array<double, 4> weights = {0.0, 0.5, 0.3, 0.2};

        for (int j = 1; j <= 3; ++j) {
          finger.current_joint_targets[j] += param.close_step * weights[j];
          finger.current_joint_targets[j] =
            clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
        }
      }
      // Detect initial tactile contact.
      if (finger.filtered_force >= finger_contact_threshold(i)) {
        finger.contact_detected = true;
        contact_force_[i] = finger.filtered_force;
        RCLCPP_INFO(this->get_logger(),
          "[%s] contact detected, force=%.3f",
          finger.name.c_str(),
          contact_force_[i]);
      }
    }
  }
  publish_traj();

  // Switch to HOLD when all fingers are contacted.
  if (all_contacted()) {
    set_desired_force();
    if (param.state == "HOLD") {
      state_ = State::HOLD;
      RCLCPP_INFO(this->get_logger(), "State -> HOLD");
    } else {
      state_ = State::IDLE;
      RCLCPP_INFO(this->get_logger(), "State -> IDLE");
    }
  }
}

void TactileForceController::handle_hold()
{
  for (int i = 0; i < fingers_num; ++i) {
    auto & finger = fingers_[i];

    // Compute force feedback command.
    double error = desired_force_[i] - finger.filtered_force;
    double dq_scalar = apply_deadband(error);

    if (dq_scalar != 0.0) {
      dq_scalar *= force_kp_;
    }
    dq_scalar = clamp(dq_scalar, -reactive_step_, reactive_step_);

    if (i == 0) {
      // Thumb force regulation using joint3 and joint4.
      const std::array<int, 2> joints = {2, 3};
      const std::array<double, 2> weights = {0.7, 0.3};
      const double direction = thumb_joint_sign(param.hand_side);

      for (int k = 0; k < 2; ++k) {
        const int j = joints[k];
        finger.current_joint_targets[j] += direction * dq_scalar * weights[k];
        finger.current_joint_targets[j] =
          clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
      }
    } else {
      // Other fingers force regulation using joint2, joint3, and joint4.
      const std::array<int, 3> joints = {1, 2, 3};
      const std::array<double, 3> weights = {0.5, 0.3, 0.2};

      for (int k = 0; k < 3; ++k) {
        const int j = joints[k];
        finger.current_joint_targets[j] += dq_scalar * weights[k];
        finger.current_joint_targets[j] =
          clamp(finger.current_joint_targets[j], finger.joint_min[j], finger.joint_max[j]);
      }
    }
  }

  publish_traj();
}

void TactileForceController::reset_grasp()
{
  for (int i = 0; i < fingers_num; ++i) {
    fingers_[i].contact_detected = false;
    contact_force_[i] = 0.0;
    desired_force_[i] = 0.0;
  }

  sync_targets();
}

void TactileForceController::reset_to_init()
{
  reset_grasp();
  state_ = State::IDLE;

  for (auto & finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      finger.current_joint_targets[j] = get_open_pos(finger.joint_names[j]);
    }
  }
}

void TactileForceController::set_desired_force()
{
  for (int i = 0; i < fingers_num; ++i) {
    desired_force_[i] =
      std::max(contact_force_[i] * param.reactive_force, finger_contact_threshold(i));

    RCLCPP_INFO(
      this->get_logger(), "[%s] desired_force=%.3f", fingers_[i].name.c_str(), desired_force_[i]);
  }
}

void TactileForceController::publish_traj()
{
  trajectory_msgs::msg::JointTrajectory traj_msg;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  traj_msg.header.stamp = this->now();
  traj_msg.joint_names = hand_joint_names_;

  // Fill trajectory point with current target values.
  for (const auto & joint_name : hand_joint_names_) {
    double position = 0.0;

    if (get_target(joint_name, position)) {
      point.positions.push_back(position);
    } else {
      point.positions.push_back(get_open_pos(joint_name));
    }
  }

  point.time_from_start = rclcpp::Duration::from_seconds(param.trajectory_dt);
  traj_msg.points.push_back(point);
  traj_pub_->publish(traj_msg);
}

void TactileForceController::sync_targets()
{
  if (!joint_state_received_) {
    return;
  }

  // Start from the current hand posture.
  for (auto & finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      finger.current_joint_targets[j] = get_joint_pos(finger.joint_names[j]);
    }
  }
}

bool TactileForceController::all_contacted() const
{
  for (const auto & finger : fingers_) {
    if (!finger.contact_detected) {
      return false;
    }
  }
  return true;
}

bool TactileForceController::get_target(const std::string & joint_name, double & target) const
{
  for (const auto & finger : fingers_) {
    for (size_t j = 0; j < finger.joint_names.size(); ++j) {
      if (finger.joint_names[j] == joint_name) {
        target = finger.current_joint_targets[j];
        return true;
      }
    }
  }
  return false;
}

double TactileForceController::get_open_pos(const std::string & joint_name) const
{
  for (size_t i = 0; i < hand_joint_names_.size(); ++i) {
    if (hand_joint_names_[i] == joint_name) {
      return init_positions_[i];
    }
  }
  return 0.0;
}

double TactileForceController::apply_deadband(double error) const
{
  if (error > -deadband_ && error < deadband_) {
    return 0.0;
  }
  return error;
}

double TactileForceController::clamp(double value, double min_v, double max_v) const
{
  return std::max(min_v, std::min(value, max_v));
}

double TactileForceController::get_joint_pos(const std::string & joint_name) const
{
  auto it = curr_joint_.find(joint_name);
  if (it != curr_joint_.end()) {
    return it->second;
  }
  return 0.0;
}

}  // namespace robotis_hand_tactile_force

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotis_hand_tactile_force::TactileForceController>());
  rclcpp::shutdown();
  return 0;
}
