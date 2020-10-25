# CRTK Python client library

This Python package provides some tools to facilitate the development of a CRTK compatible client over ROS.
It can be used to create a "proxy" class to communicate with an existing CRTK compatible ROS device.

CRTK specifications can be found on the [CRTK github page](https://github.com/collaborative-robotics/documentation/wiki/Robot-API).

CRTK devices with a CRTK/ROS interface include:
* [Raven II](https://github.com/uw-biorobotics/raven2/tree/crtk)
* [dVRK](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki)

# Build

To build this package, we recommend to use the catkin build tools, i.e. `catkin build`.
You will need to clone this repository as well as the repository with CRTK specific ROS messages:
```bash
cd ~/catkin_ws/src   # or wherever your catkin workspace is
git clone https://github.com/collaborative-robotics/crtk_msgs
git clone https://github.com/collaborative-robotics/crtk_python_client
catkin build
```

Once these packages are built, you should source your `setup.bash`: `source ~/catkin_ws/devel/setup.bash`.
At that point, you should be able to import the crtk python package in Python using `import crtk`.

# Usage

The main class in the CRTK Python client library is `crtk.utils`.  It can be used to quickly populate an existing class by adding CRTK like methods.
These methods will handle the following for you:
* declare all required ROS publishers and wrap publisher calls in methods to send data to the device
* declare all required ROS subscriber and provide callbacks to receive the data from the device
* convert ROS messages to more convenient Python data types, i.e. numpy arrays for joint values and PyKDL types for cartesian data 


## Base class and `crtk.utils`

```python
class crtk_move_cp_example:

    # configuration
    def configure(self, device_namespace):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('crtk_move_cp_example', anonymous = True, log_level = rospy.WARN)

        print(rospy.get_caller_id() + ' -> configuring crtk_device_test for: ' + device_namespace)
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, device_namespace)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_move_cp()
```

## Motion Commands

## Operating States

# Examples

https://github.com/jhu-dvrk/dvrk-ros/blob/devel/dvrk_python/src/dvrk/arm.py
