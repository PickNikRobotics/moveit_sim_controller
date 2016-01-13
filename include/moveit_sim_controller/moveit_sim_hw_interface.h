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
   Desc:   Simulates a robot using ros_control controllers
*/

#ifndef MOVEIT_SIM_CONTROLLER_MOVEIT_SIM_HW_INTERFACE_H
#define MOVEIT_SIM_CONTROLLER_MOVEIT_SIM_HW_INTERFACE_H

// C++
#include <string>

// ROS
#include <ros/ros.h>
#include <ros_control_boilerplate/sim_hw_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace moveit_sim_controller
{
static const std::string ROBOT_DESCRIPTION = "robot_description";

class MoveItSimHWInterface : public ros_control_boilerplate::SimHWInterface
{
public:
  /**
   * \brief Constructor
   */
  explicit MoveItSimHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

  /** \brief Initialize the robot hardware interface */
  void init();

  void loadDefaultJointValues();

private:
  std::string name_;

  std::string joint_model_group_;
  std::string joint_model_group_pose_;

  // Note: this doesn't need to be a member variable (only used once) but there are warnings about
  // unloading shared objects so this is a work around at least for now
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
};  // class

// Create boost pointers for this class
typedef boost::shared_ptr<MoveItSimHWInterface> MoveItSimHWInterfacePtr;
typedef boost::shared_ptr<const MoveItSimHWInterface> MoveItSimHWInterfaceConstPtr;

}  // namespace moveit_sim_controller

#endif  // MOVEIT_SIM_CONTROLLER_MOVEIT_SIM_HW_INTERFACE_H
