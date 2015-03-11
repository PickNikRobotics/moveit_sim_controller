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

// ROS
#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hardware_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace moveit_sim_controller
{

static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string JOINT_MODEL_GROUP = "whole_body";
static const std::string JOINT_MODEL_GROUP_POSE = "home";

class SimHardwareInterface: public ros_control_boilerplate::GenericHardwareInterface
{
public:

  /**
   * \brief Constructor
   * \param verbose - run in debug mode
   */
  SimHardwareInterface(ros::NodeHandle& nh, bool verbose)
    : ros_control_boilerplate::GenericHardwareInterface(nh)
    , nh_(nh)
    , verbose_(verbose)
  {

    // Load default joint values
    loadDefaultJointValues();

    ROS_INFO_STREAM_NAMED("sim_hardware_interface","SimHardwareInterface Ready.");
  }

  /**
   * \brief Destructor
   */
  ~SimHardwareInterface()
  {
  }

  void loadDefaultJointValues()
  {
    // Load the robot loader
    robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
    // Load the robot model
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel(); // Get a shared pointer to the robot

    if (robot_model->hasJointModelGroup(JOINT_MODEL_GROUP))
    {
      moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("whole_body");

      // Load a robot state
      moveit::core::RobotState robot_state(robot_model);

      // Check for existance of joint model group
      if (robot_state.setToDefaultValues(jmg, JOINT_MODEL_GROUP_POSE))
      {
        ROS_INFO_STREAM_NAMED("loadDefaultJointValues","Set joints to pose " << JOINT_MODEL_GROUP_POSE);

        for (std::size_t i = 0; i < joint_names_.size(); ++i)
        {
          const moveit::core::JointModel* jm = robot_state.getJointModel(joint_names_[i]);

          // Error check
          if (!jm)
          {
            ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Unable to find joint model group: " << joint_names_[i]);
            continue;
          }
          if (jm->getVariableCount() != 1)
          {
            ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Fake joint controller does not currently accept more than 1 variable per joint");
            continue;
          }

          // Set position from SRDF
          joint_position_[i] = robot_state.getJointPositions(jm)[0];
          joint_position_command_[i] = robot_state.getJointPositions(jm)[0];
        }
      }
      else
        ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Unable to find pose " << JOINT_MODEL_GROUP_POSE << " for the fake controller manager");
    }
    else
      ROS_WARN_STREAM_NAMED("loadDefaultJointValues","Unable to find joint model group " << JOINT_MODEL_GROUP << " for the fake controller manager");
  }

private:

  // A shared node handle
  ros::NodeHandle nh_;

  // Show more visual and console output, with general slower run time.
  bool verbose_;

}; // end class

// Create boost pointers for this class
typedef boost::shared_ptr<SimHardwareInterface> SimHardwareInterfacePtr;
typedef boost::shared_ptr<const SimHardwareInterface> SimHardwareInterfaceConstPtr;

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_hardware_interface");
  ros::NodeHandle nh;
  ROS_INFO_STREAM_NAMED("main", "Starting SimHardwareInterface...");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
        verbose = true;
        continue;
      }
    }
  }

  moveit_sim_controller::SimHardwareInterface server(nh, verbose);

  ros::spin();

  return 0;
}
