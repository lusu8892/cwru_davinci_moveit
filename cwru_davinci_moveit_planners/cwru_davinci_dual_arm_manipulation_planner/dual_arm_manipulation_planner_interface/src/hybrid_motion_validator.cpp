/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Case Western Reserve University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Su Lu <sxl924@case.edu>
 * Description: The class to do state motion validity check
 */

#include <dual_arm_manipulation_planner_interface/hybrid_motion_validator.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

using namespace dual_arm_manipulation_planner_interface;

HybridMotionValidator::HybridMotionValidator
(
const ompl::base::SpaceInformationPtr& si,
const robot_model::RobotModelConstPtr& pRobotModel,
const std::string& objectName
)
: ompl::base::MotionValidator(si),
  HybridStateValidityChecker(si,
                             pRobotModel,
                             objectName)
{
}

bool HybridMotionValidator::checkMotion
(
const ompl::base::State* s1,
const ompl::base::State* s2
) const
{
  bool result = false;
  if (!ompl::base::MotionValidator::si_->isValid(s2))
  {
    invalid_++;
  }
  else
  {
    const HybridObjectStateSpace::StateType* pHyState1 = dynamic_cast<const HybridObjectStateSpace::StateType*>(s1);
    const HybridObjectStateSpace::StateType* pHyState2 = dynamic_cast<const HybridObjectStateSpace::StateType*>(s2);

    if (!pHyState1 || !pHyState2)
    {
      return false;
    }

    const robot_state::RobotStatePtr start_state = std::make_shared<robot_state::RobotState>(kmodel_);
    if (!start_state || !hybridStateToRobotState(pHyState1, start_state))
    {
      return result;
    }
    const robot_state::RobotStatePtr goal_state = std::make_shared<robot_state::RobotState>(kmodel_);
    if (!goal_state || !hybridStateToRobotState(pHyState2, goal_state))
    {
      return result;
    }

    if (!start_state->hasAttachedBody(m_ObjectName) || !goal_state->hasAttachedBody(m_ObjectName))
    {
      start_state->clearAttachedBodies();
      goal_state->clearAttachedBodies();
      return result;
    }

    // active group is the arm supporting the needle
    const std::string supportGroupS1 = (pHyState1->armIndex().value == 1) ? "psm_one" : "psm_two";
    const std::string supportGroupS2 = (pHyState2->armIndex().value == 1) ? "psm_one" : "psm_two";

    switch (hyStateSpace_->checkStateDiff(pHyState1, pHyState2))
    {
      case StateDiff::AllSame:
        result = true;
        break;
      case StateDiff::PoseDiffArmAndGraspSame:
      {
        result = planObjectTransit(*start_state, *goal_state, supportGroupS1);
        break;
      }
      case StateDiff::ArmAndGraspDiffPoseSame:
        result = planHandoff(*start_state, *goal_state, supportGroupS1, supportGroupS2);
        break;
      default:
        // should not be there
        break;
    }

    result ? valid_++ : invalid_++;
    start_state->clearAttachedBodies();
    goal_state->clearAttachedBodies();
  }

  return result;
}

bool HybridMotionValidator::planHandoff
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state,
const std::string& ss_active_group,
const std::string& gs_active_group
) const
{
  bool able_to_grasp = false;
  bool able_to_release = false;
  bool able_to_handoff = false;

  // an intermediate state when needle is supporting by two grippers
  const robot_state::RobotStatePtr handoff_state = std::make_shared<robot_state::RobotState>(start_state);

  std::vector<double> gs_jt_position;
  goal_state.copyJointGroupPositions(gs_active_group, gs_jt_position);
  handoff_state->setJointGroupPositions(gs_active_group, gs_jt_position);
  setMimicJointPositions(handoff_state, gs_active_group);

  const moveit::core::AttachedBody* ss_needle_body = start_state.getAttachedBody(m_ObjectName);
  const moveit::core::AttachedBody* gs_needle_body = goal_state.getAttachedBody(m_ObjectName);

  std::set<std::string> touch_links = ss_needle_body->getTouchLinks();
  touch_links.insert(gs_needle_body->getTouchLinks().begin(), gs_needle_body->getTouchLinks().end());

  trajectory_msgs::JointTrajectory dettach_posture = ss_needle_body->getDetachPosture();
  trajectory_msgs::JointTrajectory gs_dettach_posture = gs_needle_body->getDetachPosture();

  dettach_posture.joint_names.insert(dettach_posture.joint_names.end(),
                                     gs_dettach_posture.joint_names.begin(),
                                     gs_dettach_posture.joint_names.end());

  dettach_posture.points.insert(dettach_posture.points.end(),
                                gs_dettach_posture.points.begin(),
                                gs_dettach_posture.points.end());


  handoff_state->attachBody(gs_needle_body->getName(), gs_needle_body->getShapes(),
                            gs_needle_body->getFixedTransforms(), touch_links,
                            gs_needle_body->getAttachedLinkName(), gs_needle_body->getDetachPosture());
  handoff_state->update();

  if (!noCollision(*handoff_state))
  {
    return able_to_handoff;
  }

  able_to_grasp = planNeedleGrasping(start_state, *handoff_state, gs_active_group);
  if (able_to_grasp)
  {
    able_to_release = planNeedleReleasing(*handoff_state, goal_state, ss_active_group);
    if (able_to_release)
      able_to_handoff = true;
  }

  return able_to_handoff;
}

bool HybridMotionValidator::planNeedleGrasping
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& handoff_state,
const std::string& gs_active_group
) const
{
  // planning in a back order fashion
  robot_state::RobotStatePtr pre_grasp_state = std::make_shared<robot_state::RobotState>(start_state);
  if (!planPreGraspStateToGraspedState(pre_grasp_state, handoff_state, gs_active_group))
    return false;
  if (!planSafeStateToPreGraspState(start_state, *pre_grasp_state, gs_active_group))
    return false;
  return true;
}

bool HybridMotionValidator::planNeedleReleasing
(
const robot_state::RobotState& handoff_state,
const robot_state::RobotState& goal_state,
const std::string& ss_active_group
) const
{
  robot_state::RobotStatePtr ungrasped_state = std::make_shared<robot_state::RobotState>(goal_state);
  if (!planGraspStateToUngraspedState(handoff_state, ungrasped_state, ss_active_group))
    return false;
  if (!planUngraspedStateToSafeState(*ungrasped_state, goal_state, ss_active_group))
    return false;
  return true;
}

bool HybridMotionValidator::planPreGraspStateToGraspedState
(
robot_state::RobotStatePtr& pre_grasp_state,
const robot_state::RobotState& handoff_state,
const std::string& planning_group
) const
{
  // make a pre_grasp_state
  const robot_state::JointModelGroup* arm_joint_group = pre_grasp_state->getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d pregrasp_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d unit_approach_dir(0.0, 0.0, 1.0);  // grasp approach along the +z-axis of tip frame

  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&isRobotStateValid,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group), _1, _2, _3);
  bool found_ik = false;
  double distance = 0.01;
  while (distance <= 0.015)
  {
    Eigen::Vector3d approach_dir = grasped_tool_tip_pose.linear() * (distance * unit_approach_dir);
    pregrasp_tool_tip_pose.translation() = grasped_tool_tip_pose.translation() - approach_dir;
    std::size_t attempts = 1;
    double timeout = 0.1;
    found_ik = pre_grasp_state->setFromIK(arm_joint_group, pregrasp_tool_tip_pose, attempts, timeout, stateValidityCallbackFn);
    if (found_ik)
      break;
    distance += 0.001;
  }

  if (!found_ik)
  {
    return found_ik;
  }

  const std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  pre_grasp_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (std::size_t i = 0; i < eef_joint_position.size(); ++i)
  {
    eef_joint_position[i] = 0.5;
  }
  pre_grasp_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  pre_grasp_state->update();

  std::vector<robot_state::RobotStatePtr> traj;
  double found_cartesian_path = computeCartesianPath(pre_grasp_state,
                                                     planning_group,
                                                     traj,
                                                     tip_link,
                                                     grasped_tool_tip_pose,
                                                     true,
                                                     0.001,
                                                     0.0);

  bool clear_path = false;
  if (!(fabs(found_cartesian_path - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  pre_grasp_state = std::move(traj[0]);  // move the ownership, do not copy
  // pre_grasp_state.reset(new robot_state::RobotState(*traj[0]));
  for (std::size_t i = 0; i < eef_joint_position.size(); ++i)
  {
    eef_joint_position[i] = 0.0;
  }
  pre_grasp_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  pre_grasp_state->update();
  clear_path = true;
  return clear_path;
}


bool HybridMotionValidator::planSafeStateToPreGraspState
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& pre_grasp_state,
const std::string& planning_group
) const
{
  const robot_state::JointModelGroup* arm_joint_group = pre_grasp_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_home = start_state.getGlobalLinkTransform(tip_link);
  std::vector<robot_state::RobotStatePtr> traj;

  // compute from pre-grasp state to safe state, back order fashion give higher succeeded rate
  double found_cartesian_path = computeCartesianPath(std::make_shared<robot_state::RobotState>(pre_grasp_state),
                                                     planning_group,
                                                     traj,
                                                     tip_link,
                                                     tool_tip_home,
                                                     true,
                                                     0.001,
                                                     0.0);

  bool clear_path = false;
  if (!((found_cartesian_path - 0.9) >= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planGraspStateToUngraspedState
(
const robot_state::RobotState& handoff_state,
robot_state::RobotStatePtr& ungrasped_state,
const std::string& planning_group
) const
{
  // make a ungrasped_state
  const robot_state::JointModelGroup* arm_joint_group = handoff_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d ungrasped_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d retreat_dir(0.0, 0.0, -1.0);  // ungrasp retreat along the -z-axis of tip frame
  double distance = 0.01;
  retreat_dir = grasped_tool_tip_pose.linear() * (distance * retreat_dir);
  ungrasped_tool_tip_pose.translation() += retreat_dir;

  std::vector<double> fromSupportGroupJntPosition;
  handoff_state.copyJointGroupPositions(planning_group, fromSupportGroupJntPosition);  // this will not copy mimic joints' value
  ungrasped_state->setJointGroupPositions(planning_group, fromSupportGroupJntPosition);

  const std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  ungrasped_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (std::size_t i = 0; i < eef_joint_position.size(); ++i)
  {
    eef_joint_position[i] = 0.5;
  }
  ungrasped_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  setMimicJointPositions(ungrasped_state, planning_group);  // this line cannot be removed, as copyJointGroupPosition does not copy mimic joints's value
  ungrasped_state->update();

  std::vector<robot_state::RobotStatePtr> traj;

  double found_cartesian_path = computeCartesianPath(ungrasped_state,
                                                     planning_group,
                                                     traj,
                                                     tip_link,
                                                     ungrasped_tool_tip_pose,
                                                     true,
                                                     0.001,
                                                     0.0);

  bool clear_path = false;

  // for (std::size_t i = 0; i < traj.size(); ++i)
  // {
  //   setMimicJointPositions(traj[i], planning_group);  // this is line is a must, as computeCartesianPath does not set mimic joints' value
  //   traj[i]->update();
  //   if (!noCollision(*traj[i]))  // check intermediate states
  //   {
  //     if (i > 0)
  //     {
  //       ungrasped_state.reset(new robot_state::RobotState(*(traj[i - 1])));
  //       // ungrasped_state->setToDefaultValues(ungrasped_state->getJointModelGroup(eef_group_name), eef_group_name + "_home");
  //       ungrasped_state->update();
  //       clear_path = true;
  //       return clear_path;
  //     }
  //     return clear_path;  // This happens when robot has collision at traj[0] false
  //   }
  // }

  if (traj.size() == 1)  // This happens when traj size is one and noCollision true
  {
    clear_path = true;
    return clear_path;
  }

  ungrasped_state = std::move(traj.back());
  // ungrasped_state.reset(new robot_state::RobotState(*traj.back()));
  ungrasped_state->setToDefaultValues(ungrasped_state->getJointModelGroup(eef_group_name), eef_group_name + "_home");
  setMimicJointPositions(ungrasped_state, planning_group);
  ungrasped_state->update();
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planUngraspedStateToSafeState
(
const robot_state::RobotState& ungrasped_state,
const robot_state::RobotState& goal_state,
const std::string& planning_group
) const
{
  const robot_state::JointModelGroup* arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;

  double found_cartesian_path = computeCartesianPath(std::make_shared<robot_state::RobotState>(ungrasped_state),
                                                     planning_group,
                                                     traj,
                                                     tip_link,
                                                     tool_tip_pose,
                                                     true,
                                                     0.001,
                                                     0.0);

  bool clear_path = false;
  if (!((found_cartesian_path - 0.9) >= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planObjectTransit
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state,
const std::string& planning_group
) const
{
  const robot_state::JointModelGroup* arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;

  double found_cartesian_path = computeCartesianPath(std::make_shared<robot_state::RobotState>(start_state),
                                                     planning_group,
                                                     traj,
                                                     tip_link,
                                                     tool_tip_pose,
                                                     true,
                                                     0.001,
                                                     0.0);

  bool clear_path = false;
  if (!(fabs(found_cartesian_path - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  clear_path = true;
  return clear_path;
}

void HybridMotionValidator::checkCollisionMultiThreads
(
bool & clear_path,
const std::vector<robot_state::RobotStatePtr>& traj,
const std::string& planning_group
) const
{
  const unsigned int min_per_thread = 2;
  const unsigned int length = traj.size();
  const unsigned int max_threads = (length + min_per_thread - 1) / min_per_thread;

  const unsigned int hardware_threads = std::thread::hardware_concurrency();
  const unsigned int num_threads = std::min((hardware_threads != 0) ? hardware_threads : 4, max_threads);

  const unsigned int block_size = length / num_threads;
  std::vector<uint8_t> results(num_threads, 0);
  std::vector<std::thread> threads(num_threads-1);

  std::vector<robot_state::RobotStatePtr>::const_iterator block_start = traj.begin();
  for (std::size_t i = 0; i < num_threads - 1; ++i)
  {
    std::vector<robot_state::RobotStatePtr>::const_iterator block_end = block_start;

    // advance the block_end iterator to the end of the current block
    std::advance(block_end, block_size);

    // launch a new thread to accumulate the results for the block
    threads[i] = std::thread(&HybridMotionValidator::noCollisionThread, this,
                                                                        std::ref(results[i]),
                                                                        block_start,
                                                                        block_end,
                                                                        std::cref(planning_group));

    block_start  = block_end;
  }

  std::vector<robot_state::RobotStatePtr>::const_iterator last = traj.end();
  noCollisionThread(results[num_threads-1], block_start, last, planning_group);

  // threads are still running you can not return by now, return must be called after join()
  // if (results[num_threads-1] != (length - (num_threads - 1) * block_size))
  // {
  //   clear_path = false;
  //   return;
  // }

  // Once accumulated the results for the last block,
  // we can wait for all the threads 
  std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));

  uint8_t sum = 0;
  for (uint8_t r : results)
    sum += r;

  if (sum == length)
    clear_path = true;
}

double HybridMotionValidator::computeCartesianPath
(
const robot_state::RobotStatePtr& rstate,
const std::string& planning_group,
std::vector<robot_state::RobotStatePtr>& traj,
const robot_state::LinkModel* link,
const Eigen::Affine3d& target,
bool global_reference_frame,
double max_step_d,
double jump_threshold_factor
) const
{
  const robot_state::MaxEEFStep max_step(max_step_d);
  const robot_state::JumpThreshold jump_threshold(jump_threshold_factor);
  const robot_state::JointModelGroup* group = rstate->getJointModelGroup(planning_group);

  const std::vector<const robot_state::JointModel*>& cjnt = group->getContinuousJointModels();
  // make sure that continuous joints wrap
  for (std::size_t i = 0; i < cjnt.size(); ++i)
    rstate->enforceBounds(cjnt[i]);

  // this is the Cartesian pose we start from, and we move in the direction indicated
  Eigen::Affine3d start_pose = rstate->getGlobalLinkTransform(link);

  // the target can be in the local reference frame (in which case we rotate it)
  Eigen::Affine3d rotated_target = global_reference_frame ? target : start_pose * target;

  Eigen::Quaterniond start_quaternion(start_pose.linear());
  Eigen::Quaterniond target_quaternion(rotated_target.linear());

  if (max_step.translation <= 0.0 && max_step.rotation <= 0.0)
  {
    ROS_ERROR_NAMED("robot_state",
                    "Invalid MaxEEFStep passed into computeCartesianPath. Both the MaxEEFStep.rotation and "
                    "MaxEEFStep.translation components must be non-negative and at least one component must be "
                    "greater than zero");
    return 0.0;
  }

  double rotation_distance = start_quaternion.angularDistance(target_quaternion);
  double translation_distance = (rotated_target.translation() - start_pose.translation()).norm();

  // decide how many steps we will need for this trajectory
  std::size_t translation_steps = 0;
  if (max_step.translation > 0.0)
    translation_steps = floor(translation_distance / max_step.translation);

  std::size_t rotation_steps = 0;
  if (max_step.rotation > 0.0)
    rotation_steps = floor(rotation_distance / max_step.rotation);

  // If we are testing for relative jumps, we always want at least MIN_STEPS_FOR_JUMP_THRESH steps
  std::size_t steps = std::max(translation_steps, rotation_steps) + 1;
  // if (jump_threshold.factor > 0 && steps < MIN_STEPS_FOR_JUMP_THRESH)
  //   steps = MIN_STEPS_FOR_JUMP_THRESH;

  const robot_state::RobotState initialRState = *rstate;

  const unsigned int min_per_thread = 4;
  const unsigned int length = steps;
  const unsigned int max_threads = (length + min_per_thread - 1) / min_per_thread;

  const unsigned int hardware_threads = std::thread::hardware_concurrency();
  const unsigned int num_threads = std::min((hardware_threads != 0) ? hardware_threads : 4, max_threads);

  const unsigned int block_size = length / num_threads;
  std::vector<uint8_t> results(num_threads, 0);
  std::vector<std::thread> threads(num_threads-1);
  std::vector<double> block_valid_percentage(num_threads, 0.0);
  std::vector<std::vector<robot_state::RobotStatePtr>> trajVec(num_threads);

  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&isRobotStateValid,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group), _1, _2, _3);
  int block_start = 1;
  for (std::size_t i = 0; i < num_threads - 1; ++i)
  {
    int block_end = block_start;
    block_end += block_size;
    // launch a new thread to accumulate the results for the block
    threads[i] = std::thread(&HybridMotionValidator::checkIKCollisionMultiThreads, this,
                                                                                   block_start,
                                                                                   block_end,
                                                                                   std::cref(steps),
                                                                                   std::cref(planning_group),
                                                                                   std::cref(start_quaternion),
                                                                                   std::cref(target_quaternion),
                                                                                   std::cref(rotated_target),
                                                                                   std::cref(start_pose),
                                                                                   std::cref(link),
                                                                                   std::cref(*rstate),
                                                                                   std::cref(stateValidityCallbackFn),
                                                                                   std::ref(trajVec[i]),
                                                                                   std::ref(block_valid_percentage[i]));

    block_start  = block_end;
  }

  checkIKCollisionMultiThreads(block_start, steps+1, steps, planning_group,
                               start_quaternion, target_quaternion, rotated_target,
                               start_pose, link, *rstate, stateValidityCallbackFn,
                               trajVec[num_threads-1], block_valid_percentage[num_threads-1]);
  std::for_each(threads.begin(), threads.end(), std::mem_fn(&std::thread::join));

  traj.clear();
  traj.push_back(std::make_shared<robot_state::RobotState>(initialRState));
  double last_valid_percentage = 0.0;

  for (std::size_t i = 0; i < num_threads; ++i)
  {
    last_valid_percentage += block_valid_percentage[i];

    if (trajVec[i].size() != block_size && trajVec[i].size() != (steps - block_start))
    {
      if (!trajVec[i].empty())
        std::move(std::begin(trajVec[i]), std::end(trajVec[i]), std::back_inserter(traj));
        // traj.insert(traj.end(), trajVec[i].begin(), trajVec[i].end());
      return last_valid_percentage;
    }

    std::move(std::begin(trajVec[i]), std::end(trajVec[i]), std::back_inserter(traj));
    // traj.insert(traj.end(), trajVec[i].begin(), trajVec[i].end());
  }

  return last_valid_percentage;
}

void HybridMotionValidator::checkIKCollisionMultiThreads
(
int first,
int last,
const int& steps,
const std::string& planning_group,
const Eigen::Quaterniond& start_quaternion,
const Eigen::Quaterniond& target_quaternion,
const Eigen::Affine3d& rotated_target,
const Eigen::Affine3d& start_pose,
const robot_state::LinkModel* link,
const robot_state::RobotState& initialRState,
const robot_state::GroupStateValidityCallbackFn& validCallback,
std::vector<robot_state::RobotStatePtr>& traj,
double& block_valid_percentage
) const
{
  // std::unique_lock<std::mutex> lock_guard(planning_scene_mutex_, std::defer_lock);
  const double current_percentage = (double)first / (double)steps;
  robot_state::RobotState rstate = initialRState;

  for (std::size_t i = first; i < last; ++i)
  {
    double percentage = (double)i / (double)steps;

    Eigen::Affine3d pose(start_quaternion.slerp(percentage, target_quaternion));
    pose.translation() = percentage * rotated_target.translation() + (1 - percentage) * start_pose.translation();

    // Explicitly use a single IK attempt only: We want a smooth trajectory.
    // Random seeding (of additional attempts) would probably create IK jumps.
    if (rstate.setFromIK(rstate.getJointModelGroup(planning_group), pose, link->getName(), 2, 0.0, validCallback))
    {
      // lock_guard.lock();
      traj.push_back(std::make_shared<robot_state::RobotState>(rstate));
      // lock_guard.unlock();
    }
    else
      break;

    block_valid_percentage = (double)(i+1-first) / (double)steps;
  }
}
