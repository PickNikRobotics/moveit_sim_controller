/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Simulates a robot using ros_control controllers with a default position loaded from MoveIt!
*/

#include <moveit_sim_controller/moveit_sim_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_sim_controller
{
MoveItSimHWInterface::MoveItSimHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
  : ros_control_boilerplate::SimHWInterface(nh, urdf_model), name_("moveit_sim_hw_interface")
{
  // Load rosparams
  ros::NodeHandle rpnh(nh_, name_);
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(name_, rpnh, "joint_model_group", joint_model_group_);
  error += !rosparam_shortcuts::get(name_, rpnh, "joint_model_group_pose", joint_model_group_pose_);
  rosparam_shortcuts::shutdownIfError(name_, error);
}

void MoveItSimHWInterface::init()
{
  // Call parent class version of this function
  SimHWInterface::init();

  // Load the loader
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION));

  // Load default joint values
  loadDefaultJointValues();

  ROS_INFO_STREAM_NAMED(name_, "MoveItSimHWInterface Ready.");
}

void MoveItSimHWInterface::loadDefaultJointValues()
{
  // Load the robot model
  robot_model::RobotModelPtr robot_model = robot_model_loader_->getModel();  // Get a shared pointer to the robot

  // Check for existance of joint model group
  if (!robot_model->hasJointModelGroup(joint_model_group_))
  {
    ROS_WARN_STREAM_NAMED(name_, "Unable to find joint model group '" << joint_model_group_
                          << "' for the fake controller manager");
    return;
  }

  moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(joint_model_group_);

  // Load a robot state
  moveit::core::RobotState robot_state(robot_model);

  // First set whole robot default values to ensure there are no 'nan's
  robot_state.setToDefaultValues();

  // Attempt to set pose
  if (!robot_state.setToDefaultValues(jmg, joint_model_group_pose_))
  {
    ROS_WARN_STREAM_NAMED(name_, "Unable to find pose " << joint_model_group_pose_ << " for the fake controller "
                                                                                      "manager");
  }
  else {
    ROS_INFO_STREAM_NAMED(name_, "Set joints to pose " << joint_model_group_pose_);
  }

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    const moveit::core::JointModel* jm = robot_state.getJointModel(joint_names_[i]);

    // Error check
    if (!jm)
    {
      ROS_WARN_STREAM_NAMED(name_, "Unable to find joint model group: " << joint_names_[i]);
      continue;
    }
    if (jm->getVariableCount() != 1)
    {
      ROS_WARN_STREAM_NAMED(name_, "Fake joint controller does not currently accept more than 1 "
                                   "variable per joint");
      continue;
    }

    // Set position from SRDF
    joint_position_[i] = robot_state.getJointPositions(jm)[0];
    joint_position_command_[i] = joint_position_[i];

    if (std::isnan(joint_position_[i]))
    {
      ROS_ERROR_STREAM_NAMED(name_, "NaN found");
      std::cout << std::endl;
      std::cout << "i: " << i << " name: " << jm->getName() << std::endl;
      std::cout << "joint_model_group: " << jmg->getName() << std::endl;
      std::cout << "getVariableCount(): " << jm->getVariableCount() << std::endl;
      std::cout << "joint_position_[i]: " << joint_position_[i] << std::endl;
      std::cout << "joint_position_command_[i]: " << joint_position_command_[i] << std::endl;
      robot_state.printStateInfo();
      exit(-1);
    }
  }
}

}  // namespace moveit_sim_controller
