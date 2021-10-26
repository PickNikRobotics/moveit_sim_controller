^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_sim_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix shared pointer conversion bug (`#9 <https://github.com/PickNikRobotics/moveit_sim_controller/issues/9>`_)
* Fixup SFINAE detection of boilerplate API (`#6 <https://github.com/PickNikRobotics/moveit_sim_controller/issues/6>`_)
* Cleanup & Fix main executable (`#4 <https://github.com/PickNikRobotics/moveit_sim_controller/issues/4>`_)
  * if the pose does not exist, still initialize joints Otherwise this warning should be fatal...
  * conditionally call loop only when available
* Contributors: Dave Coleman, Henning Kayser, Jafar Abdi, Michael Görner, Rick Staa

0.2.0 (2019-09-18)
------------------
* Cleanup UR5 example (`#2 <https://github.com/PickNikRobotics/moveit_sim_controller/issues/2>`_)
* Update README
* Add example simulated robot with UR5
* Contributors: Dave Coleman

0.1.0 (2016-10-20)
------------------
* Add C++11 support
* Fixed occational Nan
* Minor formatting
* added roslint and roslint changes
* Contributors: Dave Coleman

0.0.5 (2016-01-13)
------------------
* Fixed deprecated API for rosparam_shortcuts
* Contributors: Dave Coleman

0.0.4 (2015-12-27)
------------------
* Improved rosparam_shortcuts api
* Fixed initialization of default joint positions
* Fix travis
* Contributors: Dave Coleman

0.0.3 (2015-12-10)
------------------
* Fix missing dep
* Contributors: Dave Coleman

0.0.2 (2015-12-10)
------------------
* Initial release
* Contributors: Dave Coleman
