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

#ifndef ROBOTIS_HAND_PLAYGROUND__TACTILE_FORCE_CONTROLLER_HPP_
#define ROBOTIS_HAND_PLAYGROUND__TACTILE_FORCE_CONTROLLER_HPP_

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <robotis_interfaces/msg/hand_pressures.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "robotis_hand_playground/hx5d20_struct.hpp"
#include "robotis_hand_playground/tactile_sensor.hpp"


namespace robotis_hand_tactile_force
{

typedef robotis_interfaces::msg::HandPressures HandPressuresMsg;
typedef robotis_interfaces::msg::HandPressures::SharedPtr HandPressuresPtr;
typedef sensor_msgs::msg::JointState JointStateMsg;
typedef sensor_msgs::msg::JointState::SharedPtr JointStatePtr;
typedef std_msgs::msg::Bool BoolMsg;
typedef std_msgs::msg::Bool::SharedPtr BoolPtr;
typedef trajectory_msgs::msg::JointTrajectory JointTrajectoryMsg;
typedef robotis_hand_playground::FingerArray FingerArrayMsg;

/**
 * @brief ROS 2 parameters loaded from config/param.yaml for tactile force control.
 */
struct Params
{
  // Common control parameters
  double control_hz = 20.0;
  double trajectory_dt = 0.05;
  double close_step = 0.01;
  double contact_threshold = 30.0;
  double thumb_contact_ratio = 2.0;
  std::string hand_side = "right";
  std::vector<int> un_use_finger;

  // Force maintenance parameters
  double reactive_force = 1.2;
  std::string state = "IDLE";
};

/**
 * @brief Declare ROS 2 parameters.
 */
void declare_params(rclcpp::Node * node);

/**
 * @brief Load ROS 2 parameters.
 */
Params load_params(rclcpp::Node * node);

/**
 * @brief Check a hand side string to "left" or "right".
 */
std::string check_hand_side(const std::string & hand_side);

/**
 * @brief Return the joint name suffix for the selected hand side.
 */
std::string hand_suffix(const std::string & hand_side);

/**
 * @brief Initialize HX5-D20 finger joint configuration for the selected hand side.
 */
robotis_hand_playground::FingerArray init_fingers(const std::string & hand_side = "right");

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
 * @brief Force maintenance grasp controller using tactile feedback.
 */
class TactileForceController : public rclcpp::Node {
public:
  static constexpr int fingers_num = robotis_hand_playground::fingers_num;

  enum class State
  {
    IDLE,
    CLOSE,
    HOLD
  };

  TactileForceController();

private:
  /**
   * @brief Handle tactile pressure message.
   */
  void pressure_callback(const HandPressuresPtr msg);

  /**
   * @brief Handle joint state message.
   */
  void joint_state_callback(const JointStatePtr msg);

  /**
   * @brief Handle grasp start command.
   */
  void grasp_start_callback(const BoolPtr msg);

  /**
   * @brief Main controller loop.
   */
  void control_loop();

  /**
   * @brief IDLE state.
   */
  void handle_idle();

  /**
   * @brief Close fingers until tactile contact is detected.
   */
  void handle_close();

  /**
   * @brief Maintain desired grasp force using tactile feedback.
   */
  void handle_hold();

  /**
   * @brief Reset contact and target force states before grasping.
   */
  void reset_grasp();

  /**
   * @brief Return controller state and joint targets to the initial open posture.
   */
  void reset_to_init();

  /**
   * @brief Set desired force after initial contact.
   */
  void set_desired_force();

  /**
   * @brief Publish current joint targets as JointTrajectory.
   */
  void publish_traj();

  /**
   * @brief Synchronize target joint values with current joint states.
   */
  void sync_targets();

  /**
   * @brief Apply deadband to small force errors to prevent jitter.
   */
  double apply_deadband(double error) const;

  /**
   * @brief Check whether all fingers have detected contact.
   */
  bool all_contacted() const;

  /**
   * @brief Get contact threshold for each finger.
   */
  double finger_contact_threshold(int finger_idx) const;

  /**
   * @brief Check whether the finger is configured as unused.
   */
  bool unused_finger(int finger_idx) const;

  /**
   * @brief Move unused fingers to the closed posture.
   */
  void close_unused_finger();

  /**
   * @brief Get target joint value by joint name.
   */
  bool get_target(const std::string & joint_name, double & target) const;

  /**
   * @brief Get current joint position by joint name.
   */
  double get_joint_pos(const std::string & joint_name) const;

  /**
   * @brief Get initial open position by joint name.
   */
  double get_open_pos(const std::string & joint_name) const;

  /**
   * @brief Clamp value between minimum and maximum.
   */
  double clamp(double v, double min_v, double max_v) const;

private:
  rclcpp::Subscription<HandPressuresMsg>::SharedPtr pressure_sub_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<BoolMsg>::SharedPtr grasp_start_sub_;
  rclcpp::Publisher<JointTrajectoryMsg>::SharedPtr traj_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr unused_finger_timer_;

  // Parameters
  Params param;

  // Tactile processing
  robotis_hand_playground::TactileSensor tactile_sensor_;
  bool baseline_ = false;

  // Thread lock
  std::mutex mutex_;

  // Hand joint states
  FingerArrayMsg fingers_;
  std::vector<std::string> hand_joint_names_;
  std::vector<double> init_positions_;
  std::map<std::string, double> curr_joint_;

  // Force states
  std::array<double, fingers_num> contact_force_;
  std::array<double, fingers_num> desired_force_;

  // Controller state
  State state_{State::IDLE};
  bool joint_state_received_ = false;

  // Force control
  double force_kp_ = 0.002;  // Force feedback gain
  double deadband_ = 5.0;    // Deadband for small force errors
  double reactive_step_ = 0.02;
};

}  // namespace robotis_hand_tactile_force

#endif  // ROBOTIS_HAND_PLAYGROUND__TACTILE_FORCE_CONTROLLER_HPP_
