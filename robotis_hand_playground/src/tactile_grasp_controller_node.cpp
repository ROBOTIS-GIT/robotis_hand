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

#include "robotis_hand_playground/tactile_grasp_controller_node.hpp"

using namespace std::chrono_literals;

namespace robotis_hand_playground
{

TactileGraspControllerNode::TactileGraspControllerNode()
: TactileGraspController("tactile_grasp_controller"),
  tactile_sensor_(this->get_logger(), this->get_clock())
{
  // Load parameters.
  declare_params(this);
  param = load_params(this);
  configure_hand(param.hand_side);
  tactile_sensor_.set_params(param);

  // Initialize ROS subscriptions.
  pressure_sub_ = this->create_subscription<HandPressuresMsg>(
    hand_namespace(param.hand_side) + "/finger_pressures",
    10,
    std::bind(&TactileGraspControllerNode::on_pressure, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<JointStateMsg>("/joint_states",
    10,
    std::bind(&TactileGraspControllerNode::on_joint_state, this, std::placeholders::_1));

  grasp_start_sub_ = this->create_subscription<BoolMsg>("/grasp_start",
    10,
    std::bind(&TactileGraspControllerNode::on_grasp_start, this, std::placeholders::_1));

  // Initialize ROS publishers.
  traj_pub_ =
    this->create_publisher<JointTrajectoryMsg>(hand_controller_topic(param.hand_side), 10);

  // Start main control loop.
  const auto period = std::chrono::duration<double>(1.0 / std::max(param.control_hz, 1.0));
  control_timer_ =
    this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&TactileGraspControllerNode::control_loop, this));

  // Move unused fingers independently.
  unused_finger_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        close_unused_finger();
        publish_traj();
  });
  RCLCPP_INFO(this->get_logger(), "TactileGraspController initialized.");
}

void TactileGraspControllerNode::on_pressure(const HandPressuresPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!tactile_sensor_.check_msg(msg)) {
    return;
  }

  // Store current joint positions by joint name.
  const auto sensors = tactile_sensor_.parse_sensors(msg);
  tactile_sensor_.update_pressure(fingers_, baseline_, sensors);
}

void TactileGraspControllerNode::on_joint_state(const JointStatePtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // Store current joint positions by joint name.
  const size_t n = std::min(msg->name.size(), msg->position.size());
  for (size_t i = 0; i < n; ++i) {
    curr_joint_[msg->name[i]] = msg->position[i];
  }
  joint_received_ = true;
}

void TactileGraspControllerNode::on_grasp_start(const BoolPtr msg)
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

void TactileGraspControllerNode::control_loop()
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

void TactileGraspControllerNode::publish_traj()
{
  JointTrajectoryMsg traj_msg;
  JointTrajectoryPointMsg point;
  traj_msg.header.stamp = this->now();
  traj_msg.joint_names = hand_joint_names_;

  // Fill trajectory point with current target values.
  for (const auto & joint_name : hand_joint_names_) {
    double pos = 0.0;
    if (get_target(joint_name, pos)) {
      point.positions.push_back(pos);
    } else {
      point.positions.push_back(get_open_pos(joint_name));
    }
  }
  point.time_from_start = rclcpp::Duration::from_seconds(param.trajectory_dt);
  traj_msg.points.push_back(point);
  traj_pub_->publish(traj_msg);
}

std::optional<CorrectionDecision> TactileGraspControllerNode::pick_correction(
  const CopInfo & info) const
{
  return tactile_sensor_.pick_correction(info);
}

}  // namespace robotis_hand_playground

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robotis_hand_playground::TactileGraspControllerNode>());
  rclcpp::shutdown();
  return 0;
}
