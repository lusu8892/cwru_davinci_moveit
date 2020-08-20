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
 * Description: The main function to do needle handoff calculation then to control robot to
 *              execute trajectories
 */

#include <dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution_manager.h>
#include <std_srvs/SetBool.h>
#include <cmath>

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_needle_as_defined");

  ros::NodeHandle nodeHandle;
  ros::NodeHandle nodeHandlePriv("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(1.0).sleep();

  ros::ServiceClient psmOneStyFgClient = nodeHandle.serviceClient<std_srvs::SetBool>("/sticky_finger/PSM1_tool_wrist_sca_ee_link_1");
  ros::ServiceClient psmTwoStyFgClient = nodeHandle.serviceClient<std_srvs::SetBool>("/sticky_finger/PSM2_tool_wrist_sca_ee_link_1");

  std_srvs::SetBool graspCommand;
  graspCommand.request.data = false;

  if (!nodeHandlePriv.hasParam("num_test"))
  {
    ROS_ERROR_STREAM("Handoff perturbation test inputs parameter `num_test` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }
  int numTest;
  nodeHandlePriv.getParam("num_test", numTest);

  if (!nodeHandlePriv.hasParam("object_name"))
  {
    ROS_ERROR_STREAM("Handoff perturbation test inputs parameter `object_name` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }
  std::string objectName;
  nodeHandlePriv.getParam("object_name", objectName);

  if (!nodeHandlePriv.hasParam("initial_support_arm"))
  {
    ROS_ERROR_STREAM("Handoff perturbation test inputs parameter `initial_support_arm` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }
  std::string initialSupportArm;
  nodeHandlePriv.getParam("initial_support_arm", initialSupportArm);

  if (!nodeHandlePriv.hasParam("planning_time"))
  {
    ROS_ERROR_STREAM("Handoff perturbation test inputs parameter `planning_time` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }
  double planningTime;
  nodeHandlePriv.getParam("planning_time", planningTime);

  DummyNeedleModifier needlePoseCoordinator(nodeHandle, nodeHandlePriv);

  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr pSimpleGrasp =
  boost::make_shared<cwru_davinci_grasp::DavinciSimpleNeedleGrasper>(nodeHandle,
                                                                     nodeHandlePriv,
                                                                     initialSupportArm,
                                                                     objectName);

  std::vector<cwru_davinci_grasp::GraspInfo> graspPoses = pSimpleGrasp->getAllPossibleNeedleGrasps(false);

  DavinciNeedleHandoffExecutionManager needleHandoffExecutor(nodeHandle,
                                                             nodeHandlePriv,
                                                             graspPoses,
                                                             objectName);

  if (!needleHandoffExecutor.initializePlannerWithoutState())
  {
    return -1;
  }

  const std::vector<double> psmHomeConfig{0, 0, 0.05, 0, 0, 0, 0};

  PSMInterfacePtr psmOnePtr = std::make_unique<psm_interface>(1, nodeHandle);
  PSMInterfacePtr psmTwoPtr = std::make_unique<psm_interface>(2, nodeHandle);

  std::uniform_real_distribution<double> uniformRealDistribution(-1*M_PI, M_PI);
  std::random_device                     rdv;

  const ompl::base::SpaceInformationPtr spaceInforPtr = needleHandoffExecutor.plannerSpaceInformation();
  ob::StateSamplerPtr stateSampler = spaceInforPtr->allocStateSampler();
  ob::ScopedState<HybridObjectStateSpace> start(spaceInforPtr);
  bool grasped = false;
  while (!grasped)
  {
    bool is_ss_valid = false;
    while (!is_ss_valid)
    {
      stateSampler->sampleUniform(start.get());
      Eigen::Matrix3d m;

      m = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(uniformRealDistribution(rdv), Eigen::Vector3d::UnitZ());

      Eigen::Quaterniond qua(m);
      start->se3State().rotation().w = qua.w();
      start->se3State().rotation().x = qua.x();
      start->se3State().rotation().y = qua.y();
      start->se3State().rotation().z = qua.z();
      start->graspIndex().value = 147;
      start->armIndex().value = 1;
      is_ss_valid = spaceInforPtr->isValid(start.get());
      if (start->se3State().getX() > 0.01)
        is_ss_valid = false;
    }

    // rest robot to home position
    psmOnePtr->go(psmHomeConfig, 1.0);
    psmTwoPtr->go(psmHomeConfig, 1.0);
    // rest stick finger
    psmOneStyFgClient.call(graspCommand);
    psmTwoStyFgClient.call(graspCommand);

    needlePoseCoordinator.setNeedlePose(start->se3State().getX(),
                                        start->se3State().getY(),
                                        start->se3State().getZ(),
                                        start->se3State().rotation().w,
                                        start->se3State().rotation().x,
                                        start->se3State().rotation().y,
                                        start->se3State().rotation().z);
    ros::Duration(2.0).sleep();
    ros::spinOnce();

    // go grasp it
    const std::string& initialArm = (start->armIndex().value == 1) ? "psm_one" : "psm_two";
    pSimpleGrasp->changePlanningGroup(initialArm);

    // execute needle grasping first
    if (!pSimpleGrasp->selectPickNeedle(objectName, graspPoses[start->graspIndex().value]))
    {
      ROS_INFO("%s: did not pick needle up in SELECT way", nodeHandle.getNamespace().c_str());
    }
    else
    {
      ROS_INFO("%s: needle is picked up in SELECT way", nodeHandle.getNamespace().c_str());
      grasped = true;
    }
  }

  ros::shutdown();
  return 0;
}
