#include <robotis_hand_ik_teleop/hand_ik_teleop.hpp>

namespace hand_ik_teleop
{

HandInverseKinematics::HandInverseKinematics()
: Node("hand_ik_teleop")
{
  // Subscriber
  robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/robot_description", rclcpp::QoS(1).transient_local(),
    std::bind(&HandInverseKinematics::robot_description_callback, this, std::placeholders::_1));

  // vr_hand_right_thumb_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
  //   "/vr_hand/right_thumb", 10,
  //   std::bind(&HandInverseKinematics::solve_ik, this, std::placeholders::_1));

  vr_hand_left_thumb_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/vr_hand/left_thumb", 10,
    std::bind(&HandInverseKinematics::solve_ik, this, std::placeholders::_1));

  // Publisher
  joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/left_thumb/joint_states", 10);
    // "/joint_states", 10);

  hand_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/hand_ik_pose", 10);
}

void HandInverseKinematics::robot_description_callback(const std_msgs::msg::String& msg)
{
  // Create KDL tree
  const std::string urdf = msg.data;
  if (!kdl_parser::treeFromString(urdf, tree_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create KDL tree from URDF");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "KDL tree created with %d segments", tree_.getNrOfSegments());

  // Extract chain from tree
  if (!tree_.getChain("rh_5_left_base", "finger_l_end_link1", left_thumb_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract left chain from %s to %s",
    "rh_5_left_base", "finger_l_end_link1");
  }

  // if (!tree_.getChain("rh_5_right_base", "finger_r_end_link1", right_thumb_chain_)) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to extract right chain from %s to %s",
  //   "rh_5_right_base", "finger_r_end_link1");
  // }

  RCLCPP_INFO(this->get_logger(), "KDL left chain created with %d joints", left_thumb_chain_.getNrOfJoints());
  // RCLCPP_INFO(this->get_logger(), "KDL right chain created with %d joints", right_thumb_chain_.getNrOfJoints());

  n_joints_ = left_thumb_chain_.getNrOfJoints();
  total_n_joints_ = tree_.getNrOfJoints();

  q_min_.resize(n_joints_);
  q_max_.resize(n_joints_);

  for (unsigned int i = 0; i < n_joints_; ++i) {
    q_min_(i) = min_joint_positions_[i];
    q_max_(i) = max_joint_positions_[i];
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(left_thumb_chain_);
  ik_vel_solver_ = std::make_unique<PositionOnlyIKVelSolver>(left_thumb_chain_);

  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
    left_thumb_chain_, q_min_, q_max_, *fk_solver_, *ik_vel_solver_, 300, 1e-6);

  setup_complete_ = true;

  RCLCPP_INFO(this->get_logger(), "KDL setup complete");
}

void HandInverseKinematics::solve_ik(const std::shared_ptr<geometry_msgs::msg::PoseArray> msg)
{
  if (setup_complete_) {
    geometry_msgs::msg::Quaternion target_quat = quat_multiply(quat_inverse(msg->poses[0].orientation), msg->poses[3].orientation);
    KDL::Frame target_pose;
    KDL::Rotation R_bw = KDL::Rotation::Quaternion(msg->poses[0].orientation.x,
                                                  msg->poses[0].orientation.y,
                                                  msg->poses[0].orientation.z,
                                                  msg->poses[0].orientation.w).Inverse();

    KDL::Vector ip_tip = KDL::Vector(msg->poses[3].position.x - msg->poses[2].position.x,
                                    msg->poses[3].position.y - msg->poses[2].position.y,
                                    msg->poses[3].position.z - msg->poses[2].position.z);

    double mag = ip_tip.Norm();

    if (mag == 0) {
      RCLCPP_WARN(this->get_logger(), "Invalid Norm ip-tip");
    // } else {
    //   RCLCPP_INFO(this->get_logger(), "Norm ip-tip : %f", mag);
    }

    ip_tip = ip_tip * (thumb_length_offset/mag);

    target_pose.p = KDL::Vector((msg->poses[2].position.x - msg->poses[0].position.x) + ip_tip.x(),
                                (msg->poses[2].position.y - msg->poses[0].position.y) + ip_tip.y(),
                                (msg->poses[2].position.z - msg->poses[0].position.z) + ip_tip.z());
    target_pose.M = KDL::Rotation::Quaternion(target_quat.x,
                                            target_quat.y,
                                            target_quat.z,
                                            target_quat.w);

    target_pose.p = R_bw * target_pose.p;
    target_pose.p = KDL::Vector(-target_pose.p(1)+thumb_ik_x_offset,
                                -target_pose.p(0)+thumb_ik_y_offset,
                                -target_pose.p(2)+thumb_ik_z_offset);

    // RCLCPP_INFO(this->get_logger(), "[Wrist] pos: %f, %f, %f", msg->poses[0].position.x, msg->poses[0].position.y, msg->poses[0].position.z);
    // RCLCPP_INFO(this->get_logger(), "[Thumb] pos: %f, %f, %f", msg->poses[1].position.x, msg->poses[1].position.y, msg->poses[1].position.z);
    RCLCPP_INFO(this->get_logger(), "pos: %f, %f, %f", target_pose.p(0), target_pose.p(1), target_pose.p(2));

    auto pub_msg = geometry_msgs::msg::PoseStamped();
    pub_msg.header.stamp = this->get_clock()->now();
    pub_msg.header.frame_id = "rh_5_left_base";
    pub_msg.pose.position.x = target_pose.p(0);
    pub_msg.pose.position.y = target_pose.p(1);
    pub_msg.pose.position.z = target_pose.p(2);
    // pub_msg.pose.orientation = target_quat;
    pub_msg.pose.orientation.x = 0.0;
    pub_msg.pose.orientation.y = 0.0;
    pub_msg.pose.orientation.z = 0.0;
    pub_msg.pose.orientation.w = 1.0;
    hand_pub_->publish(pub_msg);

    double thumb_joint_4 = -get_roll_pitch_yaw(quat_inverse(msg->poses[1].orientation), msg->poses[2].orientation, 'r');

    KDL::JntArray initial_joint_positions(thumb_chain_.getNrOfJoints());
    KDL::JntArray solution_joint_positions(thumb_chain_.getNrOfJoints());

    // Set initial joint positions for the ik solver
    for (unsigned int i = 0; i < n_joints_; ++i) {
      if (i == 1) {
        initial_joint_positions(i) = 1.57;
      } else {
        initial_joint_positions(i) = 0.0;
      }
    }
    // initial_joint_positions.data.setZero();

    int solver_status = ik_solver_->CartToJnt(initial_joint_positions,
                                            target_pose,
                                            solution_joint_positions);

    if ((solver_status < 0) and (solver_status != -5)) {
      RCLCPP_WARN(this->get_logger(), "❌ IK failed with status : %d", solver_status);
      RCLCPP_WARN(this->get_logger(), "Solution: %.3f, %.3f, %.3f, %.3f", solution_joint_positions(0), solution_joint_positions(1), solution_joint_positions(2), solution_joint_positions(3));
    } else if (solver_status == -5) {
      RCLCPP_WARN(this->get_logger(), "⚠️ IK solved with status : %d", solver_status);
      RCLCPP_WARN(this->get_logger(), "Solution: %.3f, %.3f, %.3f, %.3f", solution_joint_positions(0), solution_joint_positions(1), solution_joint_positions(2), solution_joint_positions(3));
      auto response = sensor_msgs::msg::JointState();
      response.header.stamp = this->get_clock()->now();
      response.name = joint_names_;
      response.position.resize(total_n_joints_);
      for (unsigned int i = 0; i < total_n_joints_; ++i)
      {
          if (i < 3) {
            response.position[i] = solution_joint_positions(i);
          } else if (i == 3) {
            response.position[i] = std::min(max_joint_positions_[i], std::max(min_joint_positions_[i], static_cast<float>(thumb_joint_4)));
          } else {
            response.position[i] = 0.0;
          }
      }
      // Publish the joint state
      joint_states_pub_->publish(response);
    } else {
      RCLCPP_INFO(this->get_logger(), "✅ IK solved");
      RCLCPP_INFO(this->get_logger(), "Solution: %.3f, %.3f, %.3f, %.3f", solution_joint_positions(0), solution_joint_positions(1), solution_joint_positions(2), solution_joint_positions(3));
      auto response = sensor_msgs::msg::JointState();
      response.header.stamp = this->get_clock()->now();
      response.name = joint_names_;
      response.position.resize(total_n_joints_);
      for (unsigned int i = 0; i < total_n_joints_; ++i)
      {
          if (i < 4) {
              response.position[i] = solution_joint_positions(i);
          } else {
              response.position[i] = 0.0;
          }
      }
      // Publish the joint state
      joint_states_pub_->publish(response);
    }
  }
}

geometry_msgs::msg::Quaternion HandInverseKinematics::quat_inverse(const geometry_msgs::msg::Quaternion& quat)
{
  double norm = quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w;
  if (norm == 0.0) {
    geometry_msgs::msg::Quaternion result;
    result.x = 0.0;
    result.y = 0.0;
    result.z = 0.0;
    result.w = 1.0;
    return result;
  }
  double inv_norm = 1.0 / norm;
  geometry_msgs::msg::Quaternion result;
  result.x = -quat.x * inv_norm;
  result.y = -quat.y * inv_norm;
  result.z = -quat.z * inv_norm;
  result.w = quat.w * inv_norm;
  return result;
}

geometry_msgs::msg::Quaternion HandInverseKinematics::quat_multiply(const geometry_msgs::msg::Quaternion& quat1, const geometry_msgs::msg::Quaternion& quat2)
{
  geometry_msgs::msg::Quaternion result;
  result.w = quat1.w * quat2.w - quat1.x * quat2.x - quat1.y * quat2.y - quat1.z * quat2.z;
  result.x = quat1.w * quat2.x + quat1.x * quat2.w + quat1.y * quat2.z - quat1.z * quat2.y;
  result.y = quat1.w * quat2.y - quat1.x * quat2.z + quat1.y * quat2.w + quat1.z * quat2.x;
  result.z = quat1.w * quat2.z + quat1.x * quat2.y - quat1.y * quat2.x + quat1.z * quat2.w;
  return result;
}

double HandInverseKinematics::get_roll_pitch_yaw(const geometry_msgs::msg::Quaternion& quat1, const geometry_msgs::msg::Quaternion& quat2, char cmd)
{
  geometry_msgs::msg::Quaternion quat_combined = quat_multiply(quat1, quat2);
  double w = quat_combined.w;
  double x = quat_combined.x;
  double y = quat_combined.y;
  double z = quat_combined.z;

  if (cmd == 'r') {
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    double roll = std::atan2(sinr_cosp, cosr_cosp);
    return roll;
  } else if (cmd == 'p') {
    double sinp = 2 * (w * y - z * x);
    sinp = std::min(1.0, std::max(-1.0, sinp));
    double pitch = std::asin(sinp);
    return pitch;
  } else {
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }
}

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hand_ik_teleop::HandInverseKinematics>();

  RCLCPP_INFO(node->get_logger(), "Hand IK teleop node started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
