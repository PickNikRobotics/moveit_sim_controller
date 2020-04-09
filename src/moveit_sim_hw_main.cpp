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

/* Author: Dave Coleman
   Desc:   MoveIt! robot simulator using ros_control
*/

#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <moveit_sim_controller/moveit_sim_hw_interface.h>

// Support released kinetic & melodic versions of GenericHWControlLoop
// can be dropped once kinetic is not supported anymore
template <typename ...> using void_t = void;
template <typename T, typename = void> struct WithRun : public T { using T::T; void run(){ ros::waitForShutdown(); } };
template<typename T> struct WithRun<T, void_t<decltype(&T::run)> > : public T { using T::T; };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_sim_hw_main");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Create the hardware interface specific to your robot
  boost::shared_ptr<moveit_sim_controller::MoveItSimHWInterface> moveit_sim_hw_iface(
      new moveit_sim_controller::MoveItSimHWInterface(nh));
  moveit_sim_hw_iface->init();

  // Start the control loop
  WithRun<ros_control_boilerplate::GenericHWControlLoop> control_loop(nh, moveit_sim_hw_iface);
  control_loop.run(); // blocks until shutdown is signaled

  return 0;
}
