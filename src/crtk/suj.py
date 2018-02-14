#  Author(s):  Anton Deguet
#  Created on: 2016-05

# (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import rospy
import numpy
import PyKDL

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf_conversions import posemath

class suj(object):
    """Simple arm API wrapping around ROS messages
    """

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = '/dvrk/SUJ/'):
        """Constructor.  This initializes a few data members.It
        requires a arm name, this will be used to find the ROS
        topics for the arm being controlled.  For example if the
        user wants `PSM1`, the ROS topics will be from the namespace
        `/dvrk/SUJ/PSM1`"""
        # data members, event based
        self.__arm_name = arm_name
        self.__ros_namespace = ros_namespace

        # continuous publish from dvrk_bridge
        self.__servoed_cp = PyKDL.Frame()
        self.__measured_cp = PyKDL.Frame()
        self.__servoed_cp_local = PyKDL.Frame()
        self.__measured_cp_local = PyKDL.Frame()

        # publishers
        self.__full_ros_namespace = self.__ros_namespace + self.__arm_name
        self.__set_position_joint_pub = rospy.Publisher(self.__full_ros_namespace
                                                        + '/set_position_joint',
                                                        JointState, latch = False, queue_size = 1)

        # subscribers
        rospy.Subscriber(self.__full_ros_namespace + '/servoed_cp',
                         PoseStamped, self.__servoed_cp_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/measured_cp',
                         PoseStamped, self.__measured_cp_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/local/servoed_cp',
                         PoseStamped, self.__servoed_cp_local_cb)
        rospy.Subscriber(self.__full_ros_namespace + '/local/measured_cp',
                         PoseStamped, self.__measured_cp_local_cb)

        # create node
        if not rospy.get_node_uri():
            rospy.init_node('suj_api', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')

    def __state_joint_current_cb(self, data):
        """Callback for the current joint position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_current"""
        self.__position_joint_current.resize(len(data.position))
        self.__position_joint_current.flat[:] = data.position

    def __servoed_cp_cb(self, data):
        """Callback for the cartesian desired position.

        :param data: the cartesian position desired"""
        self.__servoed_cp = posemath.fromMsg(data.pose)

    def __measured_cp_cb(self, data):
        """Callback for the current cartesian position.

        :param data: The cartesian position current."""
        self.__measured_cp = posemath.fromMsg(data.pose)

    def __servoed_cp_local_cb(self, data):
        """Callback for the cartesian_local desired position.

        :param data: the cartesian_local position desired"""
        self.__servoed_cp_local = posemath.fromMsg(data.pose)

    def __measured_cp_local_cb(self, data):
        """Callback for the current cartesian_local position.

        :param data: The cartesian_local position current."""
        self.__measured_cp_local = posemath.fromMsg(data.pose)

    def measured_cp(self):
        """Get the :ref:`current cartesian position <currentvdesired>` of the arm.

        :returns: the current position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__measured_cp

    def servoed_cp(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the arm.

        :returns: the desired position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__servoed_cp

    def measured_cp_local(self):
        """Get the :ref:`current cartesian position <currentvdesired>` of the arm.

        :returns: the current position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__measured_cp_local

    def servoed_cp_local(self):
        """Get the :ref:`desired cartesian position <currentvdesired>` of the arm.

        :returns: the desired position of the arm in cartesian space
        :rtype: `PyKDL.Frame <http://docs.ros.org/diamondback/api/kdl/html/python/geometric_primitives.html>`_"""
        return self.__servoed_cp_local
