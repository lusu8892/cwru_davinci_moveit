/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Case Western Reserve University
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Su Lu */

#include <ros/ros.h>
#include <dual_arm_manipulation_planner_interface/hybrid_motion_validator.h>
#include <gtest/gtest.h>

using namespace dual_arm_manipulation_planner_interface;

namespace hybrid_planner_test
{
class HybridMotionValidatorTester : public HybridMotionValidator
{
public:
  HybridMotionValidatorTester
  (
  const ompl::base::SpaceInformationPtr& si,
  const robot_model::RobotModelConstPtr& pRobotModel,
  const std::string& objectName
  )
  : HybridMotionValidator(si, pRobotModel, objectName)
  {}

  ~HybridMotionValidatorTester()
  {}

  robot_state::RobotStatePtr sampleRobotState
  (
  );

  bool testComputeCartesianPath
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& goal_state
  );

  bool testMultiTreadComputeCartesianPathWithSingleThreaded
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& goal_state,
  const robot_state::GroupStateValidityCallbackFn& = robot_state::GroupStateValidityCallbackFn()
  );

  bool testMultiTreadComputeCartesianPathWithCollCheck
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& goal_state
  );

  bool testMultiTreadComputeCartesianPath
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& goal_state
  );

  bool testSinglereadComputeCartesianPath
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& goal_state
  );

  inline const robot_model::RobotModelConstPtr getRobotModel() const
  {
    return kmodel_;
  }

  inline int getSucceededNum()
  {
    return succeeded_num_;
  }

  inline int getPercentageNum()
  {
    return same_percentage_;
  }

private:
  double eps = 1e-7;

  std::string planning_group_ = "psm_one";

  int succeeded_num_ = 0;
  int test_num_ = 0;
  int same_percentage_ = 0;
};

bool HybridMotionValidatorTester::testComputeCartesianPath
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state
)
{
  test_num_ += 1;
  ROS_INFO("This is %dth test", test_num_);

  const moveit::core::LinkModel* tip_link = goal_state.getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d goal_tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  bool first = false;
  int first_traj_size = 0;
  double path_percent_1 = 0.0;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_1 = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group_),
                                                          traj,
                                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                                          goal_tool_tip_pose,
                                                          true,
                                                          0.001,
                                                          0.0);
    first = (fabs(path_percent_1 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);

    std::vector<double> start_state_jnt_value;
    start_state.copyJointGroupPositions(planning_group_, start_state_jnt_value);

    std::vector<double> first_traj_jnt_value;
    traj[0]->copyJointGroupPositions(planning_group_, first_traj_jnt_value);

    for (std::size_t j = 0; j < start_state_jnt_value.size(); ++j)
    {
      EXPECT_EQ(start_state_jnt_value[j], first_traj_jnt_value[j]);
    }

    std::vector<double> goal_state_jnt_value;
    goal_state.copyJointGroupPositions(planning_group_, goal_state_jnt_value);

    if (first)
    {
      std::vector<double> last_traj_jnt_value;
      traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value);
      for (std::size_t j = 0; j < start_state_jnt_value.size(); ++j)
      {
        EXPECT_NEAR(goal_state_jnt_value[j], last_traj_jnt_value[j], 1e-3);
      }

      traj.back()->update();
      EXPECT_TRUE(goal_tool_tip_pose.isApprox(traj.back()->getGlobalLinkTransform(tip_link), 1e-3));
      first_traj_size = traj.size();
    }
  }

  std::size_t variable_count = start_state.getVariableCount();

  bool same_count_var = (variable_count == start_state.getVariableNames().size()) ? true : false;
  EXPECT_TRUE(same_count_var);

  std::vector<double> rstate_home_position(variable_count);
  for (std::size_t i = 0; i < variable_count; ++i)
  {
    rstate_home_position[i] = start_state.getVariablePosition(start_state.getVariableNames()[i]);
  }

  bool second = false;
  int second_traj_size = 0;
  double path_percent_2 = 0.0;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_2 = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group_),
                                                          traj,
                                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                                          goal_tool_tip_pose,
                                                          true,
                                                          0.001,
                                                          0.0);

    second = (fabs(path_percent_2 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    std::vector<double> start_state_jnt_value;
    start_state.copyJointGroupPositions(planning_group_, start_state_jnt_value);

    std::vector<double> first_traj_jnt_value;
    traj[0]->copyJointGroupPositions(planning_group_, first_traj_jnt_value);

    for (std::size_t j = 0; j < start_state_jnt_value.size(); ++j)
    {
      EXPECT_EQ(start_state_jnt_value[j], first_traj_jnt_value[j]);
    }

    std::vector<double> goal_state_jnt_value;
    goal_state.copyJointGroupPositions(planning_group_, goal_state_jnt_value);

    if (second)
    {
      std::vector<double> last_traj_jnt_value;
      traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value);
      for (std::size_t j = 0; j < start_state_jnt_value.size(); ++j)
      {
        EXPECT_NEAR(goal_state_jnt_value[j], last_traj_jnt_value[j], 1e-3);
      }

      traj.back()->update();
      EXPECT_TRUE(goal_tool_tip_pose.isApprox(traj.back()->getGlobalLinkTransform(tip_link), 1e-3));
      second_traj_size = traj.size();
    }
  }

  same_count_var = (variable_count == start_state.getVariableNames().size()) ? true : false;
  EXPECT_TRUE(same_count_var);

  for (std::size_t i = 0; i < variable_count; ++i)
  {
    EXPECT_EQ(rstate_home_position[i], start_state.getVariablePosition(start_state.getVariableNames()[i]));
  }

  if (fabs(path_percent_1-path_percent_2) <= 1e-8)
  {
    same_percentage_ += 1;
    EXPECT_EQ(path_percent_1, path_percent_2);
    EXPECT_EQ(first_traj_size, second_traj_size);
  }

  if (first && second)
  {
    succeeded_num_ += 1;
    EXPECT_EQ(first_traj_size, second_traj_size);
    ROS_INFO("Times of succeeded %d", succeeded_num_);
    return true;
  }

  return false;
}

bool HybridMotionValidatorTester::testMultiTreadComputeCartesianPathWithSingleThreaded
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state,
const robot_state::GroupStateValidityCallbackFn& stateValidityCallbackFn
)
{
  test_num_ += 1;
  ROS_INFO("This is %dth test", test_num_);

  const moveit::core::LinkModel* tip_link = goal_state.getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d goal_tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  bool first = false;
  int first_traj_size = 0;
  double path_percent_1 = 0.0;
  std::vector<double> last_traj_jnt_value_1;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_1 = computeCartesianPath(cp_start_state,
                                          planning_group_,
                                          traj,
                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                          goal_tool_tip_pose,
                                          true,
                                          0.001,
                                          0.0);
    first = (fabs(path_percent_1 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    first_traj_size = traj.size();
    traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value_1);
  }

  bool second = false;
  int second_traj_size = 0;
  double path_percent_2 = 0.0;
  std::vector<double> last_traj_jnt_value_2;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_2 = computeCartesianPath(cp_start_state,
                                          planning_group_,
                                          traj,
                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                          goal_tool_tip_pose,
                                          true,
                                          0.001,
                                          0.0);
    second = (fabs(path_percent_2 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    second_traj_size = traj.size();
    traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value_2);
  }

  bool third = false;
  int third_traj_size = 0;
  double path_percent_3 = 0.0;
  std::vector<double> last_traj_jnt_value_3;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_3 = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group_),
                                                          traj,
                                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                                          goal_tool_tip_pose,
                                                          true,
                                                          0.001,
                                                          0.0,
                                                          stateValidityCallbackFn);

    third = (fabs(path_percent_3 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    third_traj_size = traj.size();
    traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value_3);
  }

  if (fabs(path_percent_1-path_percent_2) <= 1e-8 && fabs(path_percent_1-path_percent_3) <= 1e-8 && fabs(path_percent_2-path_percent_3) <= 1e-8)
  {
    same_percentage_ += 1;
    EXPECT_NEAR(path_percent_1, path_percent_2, 1e-8);
    EXPECT_NEAR(path_percent_2, path_percent_3, 1e-8);
    EXPECT_EQ(first_traj_size, second_traj_size);
    // EXPECT_EQ(second_traj_size, third_traj_size);
    for (std::size_t j = 0; j < 6; ++j)
    {
      EXPECT_NEAR(last_traj_jnt_value_1[j], last_traj_jnt_value_2[j], 1e-3);
      EXPECT_NEAR(last_traj_jnt_value_2[j], last_traj_jnt_value_3[j], 1e-3);
    }
  }

  if (first && second && third)
  {
    succeeded_num_ += 1;
    EXPECT_EQ(first_traj_size, second_traj_size);
    // EXPECT_EQ(second_traj_size, third_traj_size);
    for (std::size_t j = 0; j < 6; ++j)
    {
      EXPECT_NEAR(last_traj_jnt_value_1[j], last_traj_jnt_value_2[j], 1e-3);
      EXPECT_NEAR(last_traj_jnt_value_2[j], last_traj_jnt_value_3[j], 1e-3);
    }

    ROS_INFO("Times of succeeded %d", succeeded_num_);
    return true;
  }

  return false;
}

bool HybridMotionValidatorTester::testMultiTreadComputeCartesianPathWithCollCheck
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state
)
{
  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&isRobotStateValid,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group_), _1, _2, _3);
  return testMultiTreadComputeCartesianPathWithSingleThreaded(start_state, goal_state, stateValidityCallbackFn);
}

bool HybridMotionValidatorTester::testMultiTreadComputeCartesianPath
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state
)
{
  test_num_ += 1;
  ROS_INFO("This is %dth test", test_num_);

  const moveit::core::LinkModel* tip_link = goal_state.getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d goal_tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  bool first = false;
  int first_traj_size = 0;
  double path_percent_1 = 0.0;
  std::vector<double> last_traj_jnt_value_1;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_1 = computeCartesianPath(cp_start_state,
                                          planning_group_,
                                          traj,
                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                          goal_tool_tip_pose,
                                          true,
                                          0.001,
                                          0.0);
    first = (fabs(path_percent_1 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    first_traj_size = traj.size();
    traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value_1);
  }

  bool second = false;
  int second_traj_size = 0;
  double path_percent_2 = 0.0;
  std::vector<double> last_traj_jnt_value_2;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_2 = computeCartesianPath(cp_start_state,
                                          planning_group_,
                                          traj,
                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                          goal_tool_tip_pose,
                                          true,
                                          0.001,
                                          0.0);
    second = (fabs(path_percent_2 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    second_traj_size = traj.size();
    traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value_2);
  }

  if (fabs(path_percent_1-path_percent_2) <= 1e-8)
  {
    same_percentage_ += 1;
    EXPECT_EQ(first_traj_size, second_traj_size);
    for (std::size_t j = 0; j < 6; ++j)
    {
      EXPECT_NEAR(last_traj_jnt_value_1[j], last_traj_jnt_value_2[j], 1e-3);
    }
  }

  if (first && second)
  {
    succeeded_num_ += 1;
    EXPECT_EQ(first_traj_size, second_traj_size);
    for (std::size_t j = 0; j < 6; ++j)
    {
      EXPECT_NEAR(last_traj_jnt_value_1[j], last_traj_jnt_value_2[j], 1e-3);
    }

    ROS_INFO("Times of succeeded %d", succeeded_num_);
    return true;
  }

  return false;
}

bool HybridMotionValidatorTester::testSinglereadComputeCartesianPath
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state
)
{
  test_num_ += 1;
  ROS_INFO("This is %dth test", test_num_);

  const moveit::core::LinkModel* tip_link = goal_state.getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d goal_tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  bool first = false;
  int first_traj_size = 0;
  double path_percent_1 = 0.0;
  std::vector<double> last_traj_jnt_value_1;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_1 = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group_),
                                                          traj,
                                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                                          goal_tool_tip_pose,
                                                          true,
                                                          0.001,
                                                          0.0);

    first = (fabs(path_percent_1 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    first_traj_size = traj.size();
    traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value_1);
  }

  bool second = false;
  int second_traj_size = 0;
  double path_percent_2 = 0.0;
  std::vector<double> last_traj_jnt_value_2;
  {
    const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
    std::vector<robot_state::RobotStatePtr> traj;
    path_percent_2 = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group_),
                                                          traj,
                                                          cp_start_state->getJointModelGroup(planning_group_)->getOnlyOneEndEffectorTip(),
                                                          goal_tool_tip_pose,
                                                          true,
                                                          0.001,
                                                          0.0);

    second = (fabs(path_percent_2 - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 1);
    second_traj_size = traj.size();
    traj.back()->copyJointGroupPositions(planning_group_, last_traj_jnt_value_2);
  }

  if (fabs(path_percent_1-path_percent_2) <= 1e-8)
  {
    same_percentage_ += 1;
    EXPECT_EQ(first_traj_size, second_traj_size);
    for (std::size_t j = 0; j < 6; ++j)
    {
      EXPECT_NEAR(last_traj_jnt_value_1[j], last_traj_jnt_value_2[j], 1e-3);
    }
  }

  if (first && second)
  {
    succeeded_num_ += 1;
    EXPECT_EQ(first_traj_size, second_traj_size);
    for (std::size_t j = 0; j < 6; ++j)
    {
      EXPECT_NEAR(last_traj_jnt_value_1[j], last_traj_jnt_value_2[j], 1e-3);
    }

    ROS_INFO("Times of succeeded %d", succeeded_num_);
    return true;
  }

  return false;
}


robot_state::RobotStatePtr HybridMotionValidatorTester::sampleRobotState()
{
  robot_state::RobotStatePtr pRState = std::make_shared<robot_state::RobotState>(kmodel_);
  pRState->setToDefaultValues();
  const robot_state::JointModelGroup* selected_joint_model_group = pRState->getJointModelGroup(planning_group_);
  pRState->setToRandomPositions(selected_joint_model_group);
  pRState->update();
  return pRState;
}
}
