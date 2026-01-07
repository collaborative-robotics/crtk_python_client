Change log
==========

1.4.0 (2025-12-18)
==================

* crtk_simple_orientation_registration.py: print result as 4x4 json
* Added simple orientation registration script for any device with measured_cp
* Fix wait_for_busy last busy time check
* bumped cmake minimum required to 3.16
* Update CRTK examples for latest API updates
* updated some examples
* utils.py: using time.time_ns to detect timeout on message passing, it doesn't make sense to use the ros time
* added create_time for RAL ROS1, commenented out some type hints for older python
* utils: all methods return tuples, added some type hints
* ral_ros2: added method to wrap executor.spin with try/except
* ROS1 ral modified so shutdown throws an exception (like ROS2)
* Using ral for duration
* Now raises TimeoutError if messages are too old

1.3.1 (2025-01-17)
==================

* ROS2 ral spin uses SingleThreadedExecutor.  MultiThreadedExecutor lead to strange bugs on Ubuntu 24.04 with ROS Jazzy

1.3.0 (2024-08-09)
==================

* Updated code, `package.xml` and `CMakeLists.txt` so the same repository can be used for both ROS1 and ROS2

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
