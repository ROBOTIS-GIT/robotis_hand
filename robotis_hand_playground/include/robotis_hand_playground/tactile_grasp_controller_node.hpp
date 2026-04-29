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

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "robotis_interfaces/msg/hand_pressures.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "robotis_hand_playground/tactile_grasp_controller.hpp"
#include "robotis_hand_playground/tactile_sensor.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace robotis_hand_playground
{

typedef robotis_interfaces::msg::HandPressures HandPressuresMsg;
typedef robotis_interfaces::msg::HandPressures::SharedPtr HandPressuresPtr;
typedef sensor_msgs::msg::JointState JointStateMsg;
typedef sensor_msgs::msg::JointState::SharedPtr JointStatePtr;
typedef std_msgs::msg::Bool BoolMsg;
typedef std_msgs::msg::Bool::SharedPtr BoolPtr;
typedef trajectory_msgs::msg::JointTrajectory JointTrajectoryMsg;
typedef trajectory_msgs::msg::JointTrajectoryPoint JointTrajectoryPointMsg;

/**
 * @brief ROS 2 node wrapper for tactile CoP-based grasp control.
 */
class TactileGraspControllerNode : public TactileGraspController {
public:
  TactileGraspControllerNode();

private:
  /**
   * @brief Handle tactile pressure message.
   */
  void on_pressure(const HandPressuresPtr msg);

  /**
   * @brief Handle joint state message.
   */
  void on_joint_state(const JointStatePtr msg);

  /**
   * @brief Handle grasp start command.
   */
  void on_grasp_start(const BoolPtr msg);

  /**
   * @brief Main controller loop.
   */
  void control_loop();

  /**
   * @brief Publish current joint targets as JointTrajectory.
   */
  void publish_traj() override;

  /**
   * @brief Select correction direction using tactile sensor CoP information.
   */
  std::optional<CorrectionDecision> pick_correction(const CopInfo & info) const override;

private:
  // Thread lock
  std::mutex mutex_;

  // ROS interfaces
  rclcpp::Subscription<HandPressuresMsg>::SharedPtr pressure_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr grasp_start_sub_;
  rclcpp::Publisher<JointTrajectoryMsg>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr unused_finger_timer_;

  // Tactile processing
  TactileSensor tactile_sensor_;
  bool baseline_ = false;
};

}  // namespace robotis_hand_playground
