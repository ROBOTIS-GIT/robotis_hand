#ifndef POSITION_ONLY_IK_VEL_SOLVER_HPP
#define POSITION_ONLY_IK_VEL_SOLVER_HPP

#include <memory>
#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

class PositionOnlyIKVelSolver : public KDL::ChainIkSolverVel
{
public:
  PositionOnlyIKVelSolver(const KDL::Chain& chain);

  // This is the required implementation for the position solver (NR_JL)
  int CartToJnt(const KDL::JntArray& q_in, const KDL::FrameVel& v_in, KDL::JntArrayVel& q_out) override;

  // This is the required implementation for the base class
  int CartToJnt(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out) override;

  // Implement the required pure virtual function.
  void updateInternalDataStructures() override;

private:
  std::unique_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
};

#endif // POSITION_ONLY_IK_VEL_SOLVER_HPP