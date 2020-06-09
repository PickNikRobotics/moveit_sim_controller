# MoveIt Simulator Controller

A simulation interface for a hardware interface for ros_control, and loads default joint values from SRDF

Intended to replace ``moveit_fake_controller_manager`` - this repo almost exactly replicates a ros_control hardware setup, and has the new feature of being able to specify an inital position.

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

This open source project developed at [PickNik Robotics](https://picknik.ai/). Need professional ROS development and consulting? Contact us at projects@picknik.ai for a free consultation.

## Status:

* [![Build Status](https://travis-ci.org/PickNikRobotics/moveit_sim_controller.svg)](https://travis-ci.org/ros-planning/moveit_sim_controller) Travis CI
* [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__moveit_sim_controller__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__moveit_sim_controller__ubuntu_xenial_amd64__binary/) ROS Buildfarm - AMD64 Xenial Debian Build - Ubuntu 16.04 LTS
* [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__moveit_sim_controller__ubuntu_xenial_amd64)](http://build.ros.org/view/Kdev/job/Kdev__moveit_sim_controller__ubuntu_xenial_amd64/) ROS Buildfarm - AMD64 Xenial Devel Build - Ubuntu 16.04 LTS
* [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__moveit_sim_controller__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__moveit_sim_controller__ubuntu_bionic__source/) ROS Buildfarm - AMD64 Bionic Source Build - Ubuntu 18.04 LTS
* [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__moveit_sim_controller__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__moveit_sim_controller__ubuntu_bionic_amd64/) ROS Buildfarm - AMD64 Bionic Devel Build - Ubuntu 18.04 LTS

![](resources/screenshot.png)

## Install

### Ubuntu Debian

```
sudo apt-get install ros-melodic-moveit-sim-controller
```

## Code API

See [Class Reference](http://docs.ros.org/melodic/api/moveit_sim_controller/html/)

## Quick Start

Our example uses the UR5 robot:

    sudo apt-get install ros-melodic-ur5-moveit-config
    roslaunch moveit_sim_controller ur5_rviz.launch
    roslaunch moveit_sim_controller ur5_sim_controller.launch

You should see the robot launch in Rviz with the arm oriented straight up, which is not the zero/home position. You can change the start position by editing in ``config/ur5_controllers.yaml`` the value ``joint_model_group_pose``.

With this simulator you should also be able to use ``rostopic echo`` to see the ``joint_states`` and ``tf`` it is publishing - essentially you have just simulated a full ros_control-based robot without needing hardware. To test with an example joint trajectory, see the demo code in [ros_control_boilerplate](https://github.com/davetcoleman/ros_control_boilerplate).

## Usage

To set your robot's initial simulated position, create a *planning group* in your SRDF using the *MoveIt Setup Assistant* named something like ``whole_body`` or ``arm`` that contains all of your robot's joints. Then, create a *pose* for the planning group that is your start position, and name it something like ``home``.

Then load this node with the following ROS params (yaml is suggested use):

    # MoveIt-specific simulation settings:
    moveit_sim_hw_interface:
      joint_model_group: arm
      joint_model_group_pose: home

See [ros_control_boilerplate](https://github.com/davetcoleman/ros_control_boilerplate) for more detailed instructions about using ros_control to visualize your robot - this package simply inherits from that package and adds some MoveIt! dependencies that can parse SRDFs for your initial state.

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
