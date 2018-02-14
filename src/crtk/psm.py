#  Author(s):  Anton Deguet
#  Created on: 2016-05

#   (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

from dvrk.arm import *

class psm(arm):
    """Simple robot API wrapping around ROS messages
    """
    # initialize the robot
    def __init__(self, psm_name, ros_namespace = '/dvrk/'):
        # first call base class constructor
        self._arm__init_arm(psm_name, ros_namespace)

        # jaw states
        self.__servoed_jp_jaw = 0.0
        self.__servoed_jf_jaw = 0.0
        self.__measured_jp_jaw = 0.0
        self.__measured_jv_jaw = 0.0
        self.__measured_jf_jaw = 0.0

        # publishers
        self.__servo_jp_jaw_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                  + '/servo_jp_jaw',
                                                  JointState, latch = True, queue_size = 1)
        self.__move_jp_jaw_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                 + '/move_jp_jaw',
                                                 JointState, latch = True, queue_size = 1)
        self.__set_tool_present_pub = rospy.Publisher(self._arm__full_ros_namespace
                                                      + '/set_tool_present',
                                                      Bool, latch = True, queue_size = 1)

        self._arm__pub_list.extend([self.__servo_jp_jaw_pub,
                               self.__move_jp_jaw_pub,
                               self.__set_tool_present_pub])
        # subscribers
        self._arm__sub_list.extend([
        rospy.Subscriber(self._arm__full_ros_namespace + '/state_jaw_desired',
                         JointState, self.__state_jaw_desired_cb),
        rospy.Subscriber(self._arm__full_ros_namespace + '/state_jaw_current',
                         JointState, self.__state_jaw_current_cb)])


    def __state_jaw_desired_cb(self, data):
        if (len(data.position) == 1):
            self.__servoed_jp_jaw = data.position[0]
            self.__servoed_jf_jaw = data.effort[0]


    def __state_jaw_current_cb(self, data):
        if (len(data.position) == 1):
            self.__measured_jp_jaw = data.position[0]
            self.__measured_jv_jaw = data.velocity[0]
            self.__measured_jf_jaw = data.effort[0]


    def measured_jp_jaw(self):
        "get the current angle of the jaw"
        return self.__measured_jp_jaw

    def measured_jv_jaw(self):
        "get the current angular velocity of the jaw"
        return self.__measured_jv_jaw

    def measured_jf_jaw(self):
        "get the current torque applied to the jaw"
        return self.__measured_jf_jaw


    def servoed_jp_jaw(self):
        "get the desired angle of the jaw"
        return self.__servoed_jp_jaw

    def servoed_jf_jaw(self):
        "get the desired torque to be applied to the jaw"
        return self.__servoed_jf_jaw


    def close_jaw(self, interpolate = True, blocking = True):
        "Close the tool jaw"
        return self.move_jaw(-20.0 * math.pi / 180.0, interpolate, blocking)

    def open_jaw(self, interpolate = True, blocking = True):
        "Open the tool jaw"
        return self.move_jaw(80.0 * math.pi / 180.0, interpolate, blocking)

    def move_jaw(self, angle_radian, interpolate = True, blocking = True):
        "Set the jaw tool to set_jaw in radians"
        # create payload
        joint_state = JointState()
        joint_state.position.append(angle_radian)
        # check for interpolation
        if interpolate:
            if blocking:
                self._arm__goal_reached_event.clear()
                self._arm__goal_reached = False
            self.__move_jp_jaw_pub.publish(joint_state)
            if blocking:
                self._arm__goal_reached_event.wait(20)
                if not self._arm__goal_reached:
                    return False
            return True
        else:
            return self.__servo_jp_jaw_pub.publish(joint_state)

    def insert_tool(self, depth, interpolate = True, blocking = True):
        "insert the tool, by moving it to an absolute depth"
        return self.move_joint_one(depth, 2, interpolate, blocking)


    def dinsert_tool(self, depth, interpolate = True, blocking = True):
        "insert the tool, by moving it an additional depth"
        return self.dmove_joint_one(depth, 2, interpolate, blocking)


    def set_tool_present(self, tool_present):
        "Set tool inserted.  To be used only for custom tools that can't be detected automatically"
        ti = Bool()
        ti.data = tool_present
        self.__set_tool_present_pub.publish(ti)
