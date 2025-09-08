#include <robotis_hand_ik_teleop/position_only_ik_vel_solver.hpp>

PositionOnlyIKVelSolver::PositionOnlyIKVelSolver(const KDL::Chain& chain)
{
  ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain);
}

int PositionOnlyIKVelSolver::CartToJnt(const KDL::JntArray& q_in, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out)
{
  // Create a new twist that only contains the linear velocity (position)
  KDL::Twist linear_v = v_in.GetTwist();
  linear_v.rot = KDL::Vector::Zero(); // Set rotational velocity to zero

  KDL::JntArray q_vel_out(q_in.rows());

  // Call the base solver with the new twist
  int status = ik_vel_solver_->CartToJnt(q_in, linear_v, q_vel_out);

  q_out.qdot = q_vel_out;

  return status;
}

// Implementation for the new overload.
int PositionOnlyIKVelSolver::CartToJnt(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out)
{
  KDL::Twist linear_v = v_in;
  linear_v.rot = KDL::Vector::Zero();
  return ik_vel_solver_->CartToJnt(q_in, linear_v, qdot_out);
}

void PositionOnlyIKVelSolver::updateInternalDataStructures()
{
  // This is a placeholder; it's required but may not need to do anything.
}