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

#include <moveit/dual_arm_manipulation_planner_interface/hybrid_motion_validator.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

// collision
#include <moveit/collision_detection/collision_tools.h>
//#include <interactivity/interactive_robot.h>

namespace dual_arm_manipulation_planner_interface
{

HybridMotionValidator::HybridMotionValidator(const ros::NodeHandle &node_priv,
                                             const std::string &robot_name,
                                             const std::string &object_name,
                                             const ompl::base::SpaceInformationPtr &si) :
  ompl::base::MotionValidator(si),
  node_priv_(node_priv),
  robot_model_loader_(robot_name),
  robot_name_(robot_name),
  object_name_(object_name),
  stateValidityChecker_(robot_name, object_name, si)
{
  defaultSettings();

  kmodel_ = robot_model_loader_.getModel();

//  initializePlannerPlugin();

//  initializeIKPlugin();

  planning_scene_.reset(new planning_scene::PlanningScene(kmodel_));

  hyStateSpace_->object_transit_planning_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->check_motion_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->hand_off_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->ik_solving_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->collision_checking_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->object_transit_motion_planner_num = 0;

  hyStateSpace_->check_motion_num = 0;

  hyStateSpace_->hand_off_planning_num = 0;

  hyStateSpace_->hand_off_failed_num = 0;

//  CommaInitFmt_ = Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", ", ", "", "");
  CommaInitFmt_ = Eigen::IOFormat(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

//  pMonitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_name_));

//  robot_state_publisher_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>("interactive_robot_state", 1);
}

std::ofstream HybridMotionValidator::outFile_ = std::ofstream("/home/sulu/failedPoses.txt");

bool HybridMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
  hyStateSpace_->check_motion_num += 1;
  auto start = std::chrono::high_resolution_clock::now();

  bool result = false;
  if (!si_->isValid(s2))
    invalid_++;
  else
  {
    const auto *hs1 = static_cast<const HybridObjectStateSpace::StateType *>(s1);
    const auto *hs2 = static_cast<const HybridObjectStateSpace::StateType *>(s2);

    // active group is the arm supporting the needle
    const std::string active_group_s1 = (hs1->armIndex().value == 1) ? "psm_one" : "psm_two";
    const std::string rest_group_s1 = (active_group_s1 == "psm_one") ? "psm_two" : "psm_one";

    const robot_state::RobotStatePtr start_state(new robot_state::RobotState(kmodel_));
    start_state->setJointGroupPositions(active_group_s1, hs1->jointVariables().values);
    start_state->setToDefaultValues(start_state->getJointModelGroup(rest_group_s1), rest_group_s1 + "_home");
    const std::string rest_group_s1_eef_name = start_state->getJointModelGroup(
      rest_group_s1)->getAttachedEndEffectorNames()[0];
    start_state->setToDefaultValues(start_state->getJointModelGroup(rest_group_s1_eef_name),
                                    rest_group_s1_eef_name + "_home");

    const std::string active_group_s2 = (hs2->armIndex().value == 1) ? "psm_one" : "psm_two";
    const std::string rest_group_s2 = (active_group_s2 == "psm_one") ? "psm_two" : "psm_one";

    const robot_state::RobotStatePtr goal_state(new robot_state::RobotState(kmodel_));
    goal_state->setJointGroupPositions(active_group_s2, hs2->jointVariables().values);
    goal_state->setToDefaultValues(goal_state->getJointModelGroup(rest_group_s2), rest_group_s2 + "_home");
    std::string rest_group_s2_eef_name = goal_state->getJointModelGroup(
      rest_group_s2)->getAttachedEndEffectorNames()[0];
    goal_state->setToDefaultValues(goal_state->getJointModelGroup(rest_group_s2_eef_name),
                                   rest_group_s2_eef_name + "_home");

    moveit::core::AttachedBody* s1_needle = stateValidityChecker_.createAttachedBody(active_group_s1,
                                                                                     object_name_,
                                                                                     hs1->graspIndex().value);
    moveit::core::AttachedBody* s2_needle = stateValidityChecker_.createAttachedBody(active_group_s2,
                                                                                     object_name_,
                                                                                     hs2->graspIndex().value);
    start_state->attachBody(s1_needle);
    start_state->update();
    goal_state->attachBody(s2_needle);
    goal_state->update();

    if (!start_state->hasAttachedBody(object_name_) || !goal_state->hasAttachedBody(object_name_))
    {
      start_state->clearAttachedBodies();
      goal_state->clearAttachedBodies();
      return result;
    }

    switch (hyStateSpace_->checkStateDiff(hs1, hs2))
    {
      case StateDiff::AllSame:
        result = true;
        break;
      case StateDiff::PoseDiffArmAndGraspSame:
      {
        result = planObjectTransit(*start_state, *goal_state, active_group_s1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        hyStateSpace_->object_transit_planning_duration_ += elapsed;
        break;
      }
      case StateDiff::ArmAndGraspDiffPoseSame:
        result = planHandoff(*start_state, *goal_state, active_group_s1, active_group_s2);
        if(!result)
          hyStateSpace_->hand_off_failed_num += 1;
        break;
      default:
        // should not be there
        break;
    }

    result ? valid_++ : invalid_++;
    start_state->clearAttachedBodies();
    goal_state->clearAttachedBodies();
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  hyStateSpace_->check_motion_duration_ += elapsed;

  return result;
}

void HybridMotionValidator::initializePlannerPlugin()
{
  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_priv_.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException &ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance_->initialize(kmodel_, node_priv_.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException &ex)
  {
    const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
}

bool HybridMotionValidator::planHandoff(const robot_state::RobotState &start_state,
                                        const robot_state::RobotState &goal_state,
                                        const std::string &ss_active_group,
                                        const std::string &gs_active_group) const
{
  hyStateSpace_->hand_off_planning_num += 1;
  auto start = std::chrono::high_resolution_clock::now();

  bool able_to_grasp = false;
  bool able_to_release = false;
  bool able_to_handoff = false;

  // an intermediate state when needle is supporting by two grippers
  const robot_state::RobotStatePtr handoff_state(new robot_state::RobotState(start_state));

  std::vector<double> gs_jt_position;
  goal_state.copyJointGroupPositions(gs_active_group, gs_jt_position);
  handoff_state->setJointGroupPositions(gs_active_group, gs_jt_position);

  const moveit::core::AttachedBody *ss_needle_body = start_state.getAttachedBody(object_name_);
  const moveit::core::AttachedBody *gs_needle_body = goal_state.getAttachedBody(object_name_);
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
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    hyStateSpace_->hand_off_duration_ += elapsed;
    return able_to_handoff;
  }

  able_to_grasp = planNeedleGrasping(start_state, *handoff_state, gs_active_group);
  if (able_to_grasp)
  {
    able_to_release = planNeedleReleasing(*handoff_state, goal_state, ss_active_group);
    if (able_to_release)
      able_to_handoff = true;
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  hyStateSpace_->hand_off_duration_ += elapsed;
  return able_to_handoff;
}

bool HybridMotionValidator::planNeedleGrasping(const robot_state::RobotState &start_state,
                                               const robot_state::RobotState &handoff_state,
                                               const std::string &gs_active_group) const
{
  // planning in a back order fashion
  robot_state::RobotStatePtr pre_grasp_state(new robot_state::RobotState(start_state));
// pre_grasp_state->clearAttachedBody(object_name_);
  if (!planPreGraspStateToGraspedState(pre_grasp_state, handoff_state, gs_active_group))
    return false;
  if (!planSafeStateToPreGraspState(start_state, *pre_grasp_state, gs_active_group))
    return false;
  return true;
}

bool HybridMotionValidator::planNeedleReleasing(const robot_state::RobotState &handoff_state,
                                                const robot_state::RobotState &goal_state,
                                                const std::string &ss_active_group) const
{
  robot_state::RobotStatePtr ungrasped_state(new robot_state::RobotState(goal_state));

  planGraspStateToUngraspedState(handoff_state, ungrasped_state, ss_active_group);
  if (!planUngraspedStateToSafeState(*ungrasped_state, goal_state, ss_active_group))
    return false;
  return true;
}

bool HybridMotionValidator::planPreGraspStateToGraspedState(moveit::core::RobotStatePtr pre_grasp_state,
                                                            const robot_state::RobotState &handoff_state,
                                                            const std::string &planning_group) const
{
  // make a pre_grasp_state
//  publishRobotState(handoff_state);
  const robot_state::JointModelGroup *arm_joint_group = pre_grasp_state->getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d pregrasp_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d unit_approach_dir(0.0, 0.0, 1.0);  // grasp approach along the +z-axis of tip frame

  auto start_ik = std::chrono::high_resolution_clock::now();
  outFile_ << sep;
  outFile_ << "grasped_tool_tip pose: " << "\n";
  outFile_ << grasped_tool_tip_pose.matrix().format(CommaInitFmt_) << "\n";
  outFile_ << "\n" << "Pre_grasp_state before call setFromIK" << "\n";
  outFile_ << pre_grasp_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";

  bool found_ik = false;
  double distance = 0.008;
  while (distance <= 0.013)
  {
    Eigen::Vector3d approach_dir = grasped_tool_tip_pose.linear() * (distance * unit_approach_dir);
    pregrasp_tool_tip_pose.translation() = grasped_tool_tip_pose.translation() - approach_dir;
    std::size_t attempts = 1;
    double timeout = 0.1;
    found_ik = pre_grasp_state->setFromIK(arm_joint_group, pregrasp_tool_tip_pose, attempts, timeout);
//    found_ik = setFromIK(pre_grasp_state, arm_joint_group, planning_group, tip_link->getName(), pregrasp_tool_tip_pose);
    if (found_ik)
      break;
    distance += 0.001;
  }
  if (!found_ik)
  {
    outFile_ << "\n" << "Failed Pre_grasp_state after call setFromIK with pose pre_grasp_tool_tip_pose" << "\n";
    outFile_ << pre_grasp_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";
    outFile_ << sep;

    auto finish_ik = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_ik - start_ik;
    hyStateSpace_->ik_solving_duration_ += elapsed;
    return found_ik;
  }
  pre_grasp_state->update();

  outFile_ << "\n" << "Succeeded Pre_grasp_state after call setFromIK with pose pre_grasp_tool_tip_pose" << "\n";
  outFile_ << pre_grasp_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";

  std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  pre_grasp_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (int i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 0.5;
  }
  pre_grasp_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  pre_grasp_state->update();
//  publishRobotState(pre_grasp_state);

  std::vector<robot_state::RobotStatePtr> traj;
  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
//  double jt_revolute = 0.0, jt_prismatic = 0.0, jump_threshold_factor = 0.1;
  moveit::core::JumpThreshold jump_threshold;
  double found_cartesian_path = pre_grasp_state->computeCartesianPath(arm_joint_group,
                                                                      traj,
                                                                      tip_link,
                                                                      grasped_tool_tip_pose,
                                                                      true,
                                                                      max_step,
                                                                      jump_threshold);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

//  const Eigen::Affine3d current_tip_pose = pre_grasp_state.getGlobalLinkTransform(tip_link);
//  bool is_same = current_tip_pose.isApprox(grasped_tool_tip_pose, 0.0001);

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
  {
    outFile_ << "\n" << "Failed Pre_grasp_state after call computeCartesianPath" << "\n";
    outFile_ << pre_grasp_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";
    outFile_ << sep;
    return clear_path;
  }

  pre_grasp_state->update();
//  publishRobotState(pre_grasp_state);
  const moveit::core::AttachedBody *hdof_needle_body = handoff_state.getAttachedBody(object_name_);

  traj.back()->attachBody(hdof_needle_body->getName(), hdof_needle_body->getShapes(),
                          hdof_needle_body->getFixedTransforms(), hdof_needle_body->getTouchLinks(),
                          hdof_needle_body->getAttachedLinkName(), hdof_needle_body->getDetachPosture());

  for (int i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 0.0;
  }
  traj.back()->setJointGroupPositions(eef_group_name,eef_joint_position);
  traj.back()->update();

//  if (!planPathFromTwoStates(*traj.back(), handoff_state, planning_group + "_gripper"))  // // check last two states
//    return false;

  for (int i = 0; i < traj.size(); i++)
  {
    traj[i]->update();
    if (!noCollision(*traj[i]))  // check intermediate states
      return clear_path;
  }
  pre_grasp_state = traj[0];
  pre_grasp_state->update();

  outFile_ << "\n" << "Succeeded Pre_grasp_state after call computeCartesianPath" << "\n";
  outFile_ << pre_grasp_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";
  outFile_ << sep;

//  publishRobotState(pre_grasp_state);
  clear_path = true;
  return clear_path;
}


bool HybridMotionValidator::planSafeStateToPreGraspState(const robot_state::RobotState &start_state,
                                                         const robot_state::RobotState &pre_grasp_state,
                                                         const std::string &planning_group) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(start_state));
//  publishRobotState(cp_start_state);
  const robot_state::JointModelGroup *arm_joint_group = pre_grasp_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d pre_grasp_tool_tip_pose = pre_grasp_state.getGlobalLinkTransform(tip_link);

  outFile_ << sep;
  outFile_ << "\n" << "Pregrasp tool tip pose: " << "\n";
  outFile_ << pre_grasp_tool_tip_pose.matrix().format(CommaInitFmt_) << "\n";
  outFile_ << "\n" << "cp_start_state before call computeCartesianPath with pose pre_grasp_tool_tip_pose" << "\n";
  outFile_ << cp_start_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";

  std::vector<robot_state::RobotStatePtr> traj;
  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     pre_grasp_tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0);
  cp_start_state->update();
//  publishRobotState(cp_start_state);
//  const Eigen::Affine3d current_tip_pose = cp_start_state.getGlobalLinkTransform(tip_link);
//  bool is_same = current_tip_pose.isApprox(pre_grasp_tool_tip_pose, 0.0001);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
  {
    outFile_ << "\n" << "Failed cp_start_state tool tip pose" << "\n";
    outFile_ << cp_start_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";
    outFile_ << sep;
    return clear_path;
  }

  outFile_ << "\n" << "Succeeded cp_start_state tool tip pose" << "\n";
  outFile_ << cp_start_state->getGlobalLinkTransform(tip_link).matrix().format(CommaInitFmt_) << "\n";
  outFile_ << sep;

  for (int i = 0; i < traj.size(); ++i)
  {
    traj[i]->update();
    if (!noCollision(*traj[i]))
    {
//      publishRobotState(*traj[i]);
      return clear_path;
    }
  }
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planGraspStateToUngraspedState(const robot_state::RobotState &handoff_state,
                                                           moveit::core::RobotStatePtr ungrasped_state,
                                                           const std::string &planning_group) const
{
  // make a ungrasped_state
  const robot_state::JointModelGroup *arm_joint_group = handoff_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d ungrasped_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d retreat_dir(0.0, 0.0, -1.0);  // ungrasp retreat along the -z-axis of tip frame
  double distance = 0.01;
  retreat_dir = grasped_tool_tip_pose.linear() * (distance * retreat_dir);
  ungrasped_tool_tip_pose.translation() += retreat_dir;

  auto start_ik = std::chrono::high_resolution_clock::now();

  std::size_t attempts = 1;
  double timeout = 0.1;
  bool found_ik = ungrasped_state->setFromIK(arm_joint_group, grasped_tool_tip_pose, attempts, timeout);
//  bool found_ik = setFromIK(ungrasped_state, arm_joint_group, planning_group, tip_link->getName(), grasped_tool_tip_pose);

  if (!found_ik)
  {
    auto finish_ik = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_ik - start_ik;
    hyStateSpace_->ik_solving_duration_ += elapsed;
    return found_ik;
  }

  ungrasped_state->update();
  std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  ungrasped_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (int i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 0.5;
  }
  ungrasped_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  ungrasped_state->update();
//  publishRobotState(ungrasped_state);
  if (!noCollision(*ungrasped_state))
  {
//    publishRobotState(ungrasped_state);
    return false;
  }

  std::vector<robot_state::RobotStatePtr> traj;
  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
//  double jt_revolute = 0.0, jt_prismatic = 0.0, jump_threshold_factor = 0.1;
  moveit::core::JumpThreshold jump_threshold;
  double found_cartesian_path = ungrasped_state->computeCartesianPath(arm_joint_group,
                                                                      traj,
                                                                      tip_link,
                                                                      ungrasped_tool_tip_pose,
                                                                      true,
                                                                      max_step,
                                                                      jump_threshold);

  ungrasped_state->update();
//  const Eigen::Affine3d current_tip_pose = ungrasped_state.getGlobalLinkTransform(tip_link);
//  bool is_same = current_tip_pose.isApprox(ungrasped_tool_tip_pose, 0.0001);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
    return clear_path;

  ungrasped_state->setToDefaultValues(ungrasped_state->getJointModelGroup(eef_group_name), eef_group_name + "_home");
  ungrasped_state->update();
  traj.back() = ungrasped_state;

  for (int i = 0; i < traj.size(); i++)
  {
    traj[i]->update();
    if (!noCollision(*traj[i]))  // check intermediate states
      return clear_path;

  }
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planUngraspedStateToSafeState(const robot_state::RobotState &ungrasped_state,
                                                          const robot_state::RobotState &goal_state,
                                                          const std::string &planning_group) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(ungrasped_state));
//  publishRobotState(ungrasped_state);
  const robot_state::JointModelGroup *arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0);
  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  cp_start_state->update();
// publishRobotState(cp_start_state);
//  const Eigen::Affine3d current_tip_pose = cp_start_state.getGlobalLinkTransform(tip_link);
//  bool is_same = current_tip_pose.isApprox(tool_tip_pose, 0.0001);

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
    return clear_path;

  for (int i = 0; i < traj.size(); ++i)
  {
    traj[i]->update();
    if (!noCollision(*traj[i]))
    {
//      publishRobotState(*traj[i]);
      return clear_path;
    }
  }
  clear_path = true;
  return clear_path;

//  return planPathFromTwoStates(ungrasped_state, goal_state, planning_group);
}

bool HybridMotionValidator::planObjectTransit(const robot_state::RobotState &start_state,
                                              const robot_state::RobotState &goal_state,
                                              const std::string &planning_group) const
{
  hyStateSpace_->object_transit_motion_planner_num += 1;
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(start_state));
  const robot_state::JointModelGroup *arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0);
  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  cp_start_state->update();
//  publishRobotState(cp_start_state);
//  const Eigen::Affine3d current_tip_pose = cp_start_state.getGlobalLinkTransform(tip_link);
//  bool is_same = current_tip_pose.isApprox(tool_tip_pose, 0.0001);
  bool clear_path = false;
  if (found_cartesian_path != 1.0)
    return clear_path;

  for (int i = 0; i < traj.size(); ++i)
  {
    traj[i]->update();
    if (!noCollision(*traj[i]))
    {
      return clear_path;
    }
  }

  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::noCollision(const robot_state::RobotState& rstate) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  planning_scene_->setCurrentState(rstate);
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_detection::CollisionResult collision_result;
  planning_scene_->checkCollision(collision_request, collision_result, rstate);
  bool no_collision = !collision_result.collision;

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->collision_checking_duration_ += elapsed;

//  if(collision_result.collision)
//  {
//    ROS_INFO("Invalid State: Robot state is in collision with planning scene. \n");
//    collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
//    for(collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin(); it != contactMap.end(); ++it)
//    {
//      ROS_INFO("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
//    }
//  }
  return no_collision;
}

bool HybridMotionValidator::planPathFromTwoStates(const robot_state::RobotState &start_state,
                                                  const robot_state::RobotState &goal_state,
                                                  const std::string &planning_group) const
{
  auto start = std::chrono::high_resolution_clock::now();

  planning_scene_->setCurrentState(start_state);

  const robot_state::JointModelGroup *joint_model_group = goal_state.getJointModelGroup(planning_group);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  // create a motion plan request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  moveit_msgs::RobotState start_state_msg;
  moveit::core::robotStateToRobotStateMsg(start_state, start_state_msg);

  req.start_state = start_state_msg;
  req.group_name = planning_group;
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  planning_interface::PlanningContextPtr context = planner_instance_->getPlanningContext(planning_scene_,
                                                                                         req,
                                                                                         res.error_code_);
  if (context->solve(res))
  {
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    hyStateSpace_->object_transit_planning_duration_ += elapsed;
    return true;
  }
  else
  {
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    hyStateSpace_->object_transit_planning_duration_ += elapsed;

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_ERROR("Could not compute plan successfully, the error code is %d", res.error_code_.val);
      return false;
    }
  }
}

void HybridMotionValidator::defaultSettings()
{
  hyStateSpace_ = si_->getStateSpace().get()->as<HybridObjectStateSpace>();
  if (hyStateSpace_ == nullptr)
    throw ompl::Exception("No state space for motion validator");
}

void HybridMotionValidator::publishRobotState(const robot_state::RobotState& rstate) const
{
  moveit_msgs::DisplayRobotState drstate;
  robot_state_publisher_.publish(drstate);
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  moveit::core::robotStateToRobotStateMsg(rstate, drstate.state);

  robot_state_publisher_.publish(drstate);
  ros::spinOnce();
  ros::Duration(1.0).sleep();
}

void HybridMotionValidator::initializeIKPlugin()
{
  psm_one_kinematics_solver_ = NULL;
  psm_two_kinematics_solver_ = NULL;
  kinematics_loader_.reset(
    new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
  std::string plugin_name = "davinci_moveit_kinematics/DavinciMoveitKinematicsPlugin";

  psm_one_kinematics_solver_ = kinematics_loader_->createInstance(plugin_name);
  psm_two_kinematics_solver_ = kinematics_loader_->createInstance(plugin_name);

  double search_discretization = 0.025;
  psm_one_kinematics_solver_->initialize(robot_name_, "psm_one", "PSM1_psm_base_link", "PSM1_tool_tip_link", search_discretization);
  psm_two_kinematics_solver_->initialize(robot_name_, "psm_two", "PSM2_psm_base_link", "PSM2_tool_tip_link", search_discretization);
}

bool HybridMotionValidator::setFromIK(robot_state::RobotState &rstate,
                                      const robot_state::JointModelGroup *arm_joint_group,
                                      const std::string &planning_group,
                                      const std::string &tip_frame,
                                      const Eigen::Affine3d &tip_pose_wrt_world) const
{
  bool found_ik = false;
  std::string base_frame = (planning_group == "psm_one") ? "PSM1_psm_base_link" : "PSM2_psm_base_link";

  boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
  if (planning_group == "psm_one")
  {
    kinematics_solver_ = psm_one_kinematics_solver_;
    if (kinematics_solver_->getGroupName() != planning_group)
      return found_ik;
  }
  else
  {
    kinematics_solver_ = psm_two_kinematics_solver_;
    if (kinematics_solver_->getGroupName() != planning_group)
      return found_ik;
  }

  std::vector<double> seed(kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<double> solution(kinematics_solver_->getJointNames().size(), 0.0);
  double timeout = 2.0;
  Eigen::Affine3d affine_base_wrt_world = rstate.getFrameTransform(base_frame);
  Eigen::Affine3d affine_tip_pose_wrt_base = affine_base_wrt_world.inverse() * tip_pose_wrt_world;
  geometry_msgs::Pose tip_pose_wrt_base;
  tf::poseEigenToMsg(affine_tip_pose_wrt_base, tip_pose_wrt_base);
  moveit_msgs::MoveItErrorCodes error_code;

  found_ik = kinematics_solver_->searchPositionIK(tip_pose_wrt_base, seed, timeout, solution, error_code);

  if (found_ik)
  {
    rstate.setJointGroupPositions(arm_joint_group, solution);
    rstate.update();
  }
  else
  {
    ROS_INFO("No IK solution in CheckMotion");
  }

  return found_ik;
}


}  // namespace
