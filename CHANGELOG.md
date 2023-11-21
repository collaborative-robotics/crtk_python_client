Change log
==========

1.2.0 (2023-11-21)
==================

* :warning: Use `PoseStamped` instead of `TransformStamped` for CRTK `_cp` commands (see collaborative-robotics/documentation#1)
* Documentation ported to ReadTheDocs: https://crtk-robotics.readthedocs.io
* Added `crtk.ral` (ROS Abstraction Layer) so Python scripts can be written for either ROS 1 or ROS 2
  * `ral.check_connections` allows you to check if there are subscribers/publishers connected to your publishers/subscriber
  * `ral.spin` starts the thread for ROS spin (no op on ROS 1)
  * `ral.parse_argv` parses and removed ROS specific arguments
  * ...
* `utils.py`:
  * Fixed bug in `measured_cp`, was returning `setpoint_cp`
  * Fixed `wait` on move handle
  * Add `hold`, `free`, `forward_kinematics`, `inverse_kinematics`, `servo_cv`
  * Added `joystick_button` to wrap foot pedals and other buttons
