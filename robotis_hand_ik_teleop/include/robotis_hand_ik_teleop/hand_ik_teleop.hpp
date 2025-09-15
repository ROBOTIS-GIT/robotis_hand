#ifndef ROBOTIS_HAND_IK_TELEOP__HAND_IK_TELEOP_HPP
#define ROBOTIS_HAND_IK_TELEOP__HAND_IK_TELEOP_HPP

#include <robotis_hand_ik_teleop/position_only_ik_vel_solver.hpp>

#include <memory>
// #include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

namespace hand_ik_teleop
{

class HandInverseKinematics : public rclcpp::Node
{
public:
  HandInverseKinematics();

protected:
  std::vector<std::string> left_joint_names_ = {
    "finger_l_joint1", "finger_l_joint2", "finger_l_joint3", "finger_l_joint4",
    "finger_l_joint5", "finger_l_joint6", "finger_l_joint7", "finger_l_joint8",
    "finger_l_joint9", "finger_l_joint10", "finger_l_joint11", "finger_l_joint12",
    "finger_l_joint13", "finger_l_joint14", "finger_l_joint15", "finger_l_joint16",
    "finger_l_joint17", "finger_l_joint18", "finger_l_joint19", "finger_l_joint20"
  };

  std::vector<std::string> right_joint_names_ = {
    "finger_r_joint1", "finger_r_joint2", "finger_r_joint3", "finger_r_joint4",
    "finger_r_joint5", "finger_r_joint6", "finger_r_joint7", "finger_r_joint8",
    "finger_r_joint9", "finger_r_joint10", "finger_r_joint11", "finger_r_joint12",
    "finger_r_joint13", "finger_r_joint14", "finger_r_joint15", "finger_r_joint16",
    "finger_r_joint17", "finger_r_joint18", "finger_r_joint19", "finger_r_joint20"
  };

  std::vector<float> min_joint_positions_ = {
    0.0, -0.3, 0.0, 0.0
  };

  std::vector<float> max_joint_positions_ = {
    2.2, 2.0, 1.57, 1.57
  };

  unsigned int n_joints_;
  unsigned int total_n_joints_;

  double thumb_length_offset = 0.02;
  float thumb_ik_x_offset = 0.01;
  float thumb_ik_y_offset = 0.0; //0.01;
  float thumb_ik_z_offset = 0.025; //0.025;

  KDL::Tree tree_;
  KDL::Chain left_thumb_chain_;
  KDL::Chain right_thumb_chain_;

  std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<PositionOnlyIKVelSolver> ik_vel_solver_;

  KDL::JntArray q_min_;
  KDL::JntArray q_max_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  // rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr vr_hand_right_thumb_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr vr_hand_left_thumb_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hand_pub_;

  void robot_description_callback(const std_msgs::msg::String& msg);
  bool setup_complete_ = false;

  void solve_ik(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg);

  geometry_msgs::msg::Quaternion quat_inverse(const geometry_msgs::msg::Quaternion& quat);
  geometry_msgs::msg::Quaternion quat_multiply(const geometry_msgs::msg::Quaternion& quat1, const geometry_msgs::msg::Quaternion& quat2);
  double get_roll_pitch_yaw(const geometry_msgs::msg::Quaternion& quat1, const geometry_msgs::msg::Quaternion& quat2, char cmd);
};

} // namespace hand_ik_teleop

#endif // ROBOTIS_HAND_IK_TELEOP__HAND_IK_TELEOP_HPP