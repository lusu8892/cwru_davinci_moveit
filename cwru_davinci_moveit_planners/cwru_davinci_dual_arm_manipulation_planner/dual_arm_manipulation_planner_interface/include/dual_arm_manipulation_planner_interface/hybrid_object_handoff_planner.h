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
 * Description: This class is to do object handoff planning with given initial state and
 *              goal state, the result is robot joint trajectories for davinci dual-arm manipulators
 */

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_HANDOFF_PLANNER_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_HANDOFF_PLANNER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>

namespace dual_arm_manipulation_planner_interface
{

typedef std::vector<std::vector<double>> JointTrajectory;
typedef std::map<std::string, JointTrajectory> PlanningGroupJointTrajectory;
typedef std::vector<PlanningGroupJointTrajectory> SolutionPathJointTrajectory;

class HybridObjectHandoffPlanner
{
public:
  HybridObjectHandoffPlanner
  (
  const ompl::base::State *start,
  const ompl::base::State *goal,
  const double se3BoundXAxisMin,
  const double se3BoundXAxisMax,
  const double se3BoundYAxisMin,
  const double se3BoundYAxisMax,
  const double se3BoundZAxisMin,
  const double se3BoundZAxisMax,
  const int armIdxLwBd,
  const int armIdxUpBd,
  const int graspIdxLwBd,
  const int graspIdxUpBd,
  const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
  const robot_model::RobotModelConstPtr& pRobotModel,
  const std::string &objectName,
  const double maxDistance,
  bool verbose = true
  );

  ~HybridObjectHandoffPlanner(){}

  ompl::base::PlannerStatus::StatusType solve
  (
  const double solveTime
  );

  bool getSolutionPathJointTrajectory
  (
  SolutionPathJointTrajectory& wholePathJntTraj
  );

protected:
  HybridObjectStateSpacePtr                    m_pHyStateSpace;

  ompl::base::SpaceInformationPtr              m_pSpaceInfor;

  ompl::base::ProblemDefinitionPtr             m_pProblemDef;

  std::shared_ptr<ompl::geometric::RRTConnect> m_pRRTConnectPlanner;

  ompl::base::PlannerStatus                    m_solved;

  bool                                         m_verbose;
protected:
  void setupSpaceInformation
  (
  const HybridObjectStateSpacePtr& pHyStateSpace,
  const robot_model::RobotModelConstPtr& pRobotModel,
  const std::string &objectName
  );

  void setupProblemDefinition
  (
  const ompl::base::State *start,
  const ompl::base::State *goal
  );

  void setupPlanner
  (
  const double maxDistance
  );

  bool connectStates
    (
    const ompl::base::State *pFromState,
    const ompl::base::State *pToState,
    SolutionPathJointTrajectory &jntTrajectoryBtwStates
    );
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_HANDOFF_PLANNER_H
