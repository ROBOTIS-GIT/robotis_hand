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

#include "robotis_hand_playground/hx5d20_init.hpp"
#include "robotis_hand_playground/tactile_force_controller.hpp"


using namespace std::chrono_literals;

namespace robotis_hand_playground_force
{
TactileForceController::TactileForceController()
: Node("tactile_force_controller"), tactile_sensor_(this->get_logger(), this->get_clock())
{
  // Load parameters.
  robotis_hand_playground::declare_params(this);
  param = robotis_hand_playground::load_params(this);

  // Initialize hand model.
  fingers_ = robotis_hand_playground::init_fingers(param.hand_side);
  hand_joint_names_ = robotis_hand_playground::init_joint_names(param.hand_side);
  init_positions_ = robotis_hand_playground::init_positions(param.hand_side);

  // Initialize ROS interfaces.
  pressure_sub_ = this->create_subscription<robotis_interfaces::msg::HandPressures>(
    robotis_hand_playground::hand_namespace(param.hand_side) + "/finger_pressures",
    10,
    std::bind(&TactileForceController::pressure_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states",
    10,
    std::bind(&TactileForceController::joint_state_callback, this, std::placeholders::_1));

  grasp_start_sub_ = this->create_subscription<std_msgs::msg::Bool>("/grasp_start",
    10,
    std::bind(&TactileForceController::grasp_start_callback, this, std::placeholders::_1));

  traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    robotis_hand_playground::hand_controller_topic(param.hand_side), 10);

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
        const double direction = robotis_hand_playground::thumb_joint_sign(param.hand_side);

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
      const double direction = robotis_hand_playground::thumb_joint_sign(param.hand_side);

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

}  // namespace robotis_hand_playground_force

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotis_hand_playground_force::TactileForceController>());
  rclcpp::shutdown();
  return 0;
}
