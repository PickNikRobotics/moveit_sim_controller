# MoveIt Simulator Controller

A simulation interface for a hardware interface for ros_control, and loads default joint values from SRDF

Intended to replace moveit_fake_controller_manager - this supports simulated trajectory following and loading an inital position

## Use

To set your robot's initial simulated position, create a *planning group* in your SRDF using the *MoveIt Setup Assistant* that is named ``whole_body`` that contains all of your robot's joints. Then, create a *pose* for the ``whole_body`` planning group that is your start position, and name it ``home``.

See [ros_control_boilerplate](https://github.com/davetcoleman/ros_control_boilerplate) for more detailed instructions about using ros_control to visualize your robot - this package simply inherits from that package and adds some MoveIt! dependencies that can parse SRDFs for your initial state.
