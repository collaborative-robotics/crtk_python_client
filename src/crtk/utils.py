#  Author(s):  Anton Deguet
#  Created on: 2018-02-15

# (C) Copyright 2018-2019 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import threading
import time

import rospy
import numpy
import PyKDL
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import crtk_msgs.msg

def TransformFromMsg(t):
    """
    :param p: input pose
    :type p: :class:`geometry_msgs.msg.Pose`
    :return: New :class:`PyKDL.Frame` object

    Convert a pose represented as a ROS Pose message to a :class:`PyKDL.Frame`.
    """
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.rotation.x,
                                                 t.rotation.y,
                                                 t.rotation.z,
                                                 t.rotation.w),
                       PyKDL.Vector(t.translation.x,
                                    t.translation.y,
                                    t.translation.z))

def TransformToMsg(f):
    """
    :param f: input pose
    :type f: :class:`PyKDL.Frame`

    Return a ROS Pose message for the Frame f.

    """
    m = geometry_msgs.msg.TransformStamped()
    t = m.transform
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = f.M.GetQuaternion()
    t.translation.x = f.p[0]
    t.translation.y = f.p[1]
    t.translation.z = f.p[2]
    return m



class utils:
    def __init__(self, class_instance, ros_namespace):
        self.__class_instance = class_instance
        self.__ros_namespace = ros_namespace
        self.__subscribers = []
        self.__publishers = []
        self.__attributes = []
        # internal data for subscriber callbacks
        self.__setpoint_jp_data = numpy.array(0, dtype = numpy.float)
        self.__setpoint_jf_data = numpy.array(0, dtype = numpy.float)
        self.__setpoint_cp_data = PyKDL.Frame()
        self.__measured_jp_data = numpy.array(0, dtype = numpy.float)
        self.__measured_jv_data = numpy.array(0, dtype = numpy.float)
        self.__measured_jf_data = numpy.array(0, dtype = numpy.float)
        self.__measured_cp_data = PyKDL.Frame()
        self.__measured_cv_data = numpy.zeros(6, dtype = numpy.float)
        self.__measured_cf_data = numpy.zeros(6, dtype = numpy.float)
        # thread event for blocking commands
        self.__operating_state_event = threading.Event()
        self.__is_busy_event = threading.Event()


    def __del__(self):
        print("del called")
        self.remove_all()


    def remove_all(self):
        for sub in self.__subscribers:
            sub.unregister()
        for pub in self.__publishers:
            pub.unregister()
        for attr in self.__attributes:
            dir(self.__class_instance)
            delattr(self.__class_instance, attr)
            dir(self.__class_instance)


    # internal methods to manage state
    def __operating_state_cb(self, msg):
        # crtk operating state contains state as well as homed and busy
        self.__operating_state_data = msg
        # busy
        self.__is_busy_data = msg.is_busy
        # then when all data is saved, release "lock"
        self.__operating_state_event.set()

    def __operating_state(self):
        return self.__operating_state_data.state

    def __wait_for_operating_state(self, expected_state, timeout):
        if timeout < 0.0:
            return False
        start_time = time.time()
        in_time = self.__operating_state_event.wait(timeout)
        if in_time:
            # within timeout and result we expected
            if self.__operating_state_data.state == expected_state:
                return True
            else:
                # wait a bit more
                elapsed_time = time.time() - start_time
                self.__operating_state_event.clear()
                return self.__operating_state_wait(state, timeout - elapsed_time)
        # past timeout
        return False

    def __operating_state_command(self, state):
        # clear timeout
        self.__operating_state_event.clear()
        # convert to ROS msg and publish
        msg = std_msgs.msg.String()
        msg.data = state
        # publish and wait
        self.__operating_state_command_publisher.publish(msg)

    def __enable(self, timeout = 0):
        self.__operating_state_event.clear()
        self.__operating_state_command("enable")
        return self.__wait_for_operating_state('ENABLED', timeout)

    def __disable(self, timeout = 0):
        self.__operating_state_event.clear()
        self.__operating_state_command("disable")
        return self.__wait_for_operating_state('DISABLED', timeout)

    def __is_busy(self):
        return self.__is_busy_data

    def __is_busy_wait(self, timeout):
        start_time = time.time()
        while True:
            self.__operating_state_event.clear()
            self.__operating_state_event.wait(timeout)
            # if not busy we're good
            if not self.__is_busy_data:
                break
            # otherwise, keep waiting a bit more
            timeout = timeout - (time.time() - start_time)
            if timeout <= 0.0:
                break
        return not self.__is_busy_data

    def add_operating_state(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'operating_state'):
            raise RuntimeWarning('operating_state already exists')
        # create the subscriber/publisher and keep in list
        self.__operating_state_data = crtk_msgs.msg.robot_state()
        self.__is_busy_data = False
        self.__operating_state_subscriber = rospy.Subscriber(self.__ros_namespace + '/operating_state',
                                                             crtk_msgs.msg.robot_state, self.__operating_state_cb)
        self.__subscribers.append(self.__operating_state_subscriber)
        self.__operating_state_command_publisher = rospy.Publisher(self.__ros_namespace + '/state_command',
                                                                   std_msgs.msg.String,
                                                                   latch = True, queue_size = 1)
        self.__publishers.append(self.__operating_state_command_publisher)
        # add attributes to class instance
        self.__class_instance.operating_state = self.__operating_state
        self.__class_instance.wait_for_operating_state = self.__wait_for_operating_state
        self.__class_instance.operating_state_command = self.__operating_state_command
        self.__class_instance.enable = self.__enable
        self.__class_instance.disable = self.__disable
        self.__class_instance.is_busy = self.__is_busy
        self.__class_instance.is_busy_wait = self.__is_busy_wait


    # internal methods for setpoint_js
    def __setpoint_js_cb(self, msg):
        self.__setpoint_jp_data.resize(len(msg.position))
        self.__setpoint_jf_data.resize(len(msg.effort))
        self.__setpoint_jp_data.flat[:] = msg.position
        self.__setpoint_jf_data.flat[:] = msg.effort

    def __setpoint_jp(self):
        return self.__setpoint_jp_data

    def __setpoint_jf(self):
        return self.__setpoint_jf_data

    def add_setpoint_js(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'setpoint_jp'):
            raise RuntimeWarning('setpoint_js already exists')
        # create the subscriber and keep in list
        self.__setpoint_js_subscriber = rospy.Subscriber(self.__ros_namespace + '/setpoint_js',
                                                         sensor_msgs.msg.JointState,
                                                         self.__setpoint_js_cb)
        self.__subscribers.append(self.__setpoint_js_subscriber)
        # add attributes to class instance
        self.__class_instance.setpoint_jp = self.__setpoint_jp
        self.__class_instance.setpoint_jf = self.__setpoint_jf


    # internal methods for setpoint_cp
    def __setpoint_cp_cb(self, msg):
        self.__setpoint_cp_data = TransformFromMsg(msg.transform)

    def __setpoint_cp(self):
        return self.__setpoint_cp_data

    def add_setpoint_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'setpoint_cp'):
            raise RuntimeWarning('setpoint_cp already exists')
        # create the subscriber and keep in list
        self.__setpoint_cp_subscriber = rospy.Subscriber(self.__ros_namespace + '/setpoint_cp',
                                                         geometry_msgs.msg.TransformStamped,
                                                         self.__setpoint_cp_cb)
        self.__subscribers.append(self.__setpoint_cp_subscriber)
        # add attributes to class instance
        self.__class_instance.setpoint_cp = self.__setpoint_cp


    # internal methods for measured_js
    def __measured_js_cb(self, msg):
        self.__measured_jp_data.resize(len(msg.position))
        self.__measured_jv_data.resize(len(msg.position))
        self.__measured_jf_data.resize(len(msg.effort))
        self.__measured_jp_data.flat[:] = msg.position
        self.__measured_jv_data.flat[:] = msg.velocity
        self.__measured_jf_data.flat[:] = msg.effort

    def __measured_jp(self):
        return self.__measured_jp_data

    def __measured_jv(self):
        return self.__measured_jv_data

    def __measured_jf(self):
        return self.__measured_jf_data

    def add_measured_js(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_jp'):
            raise RuntimeWarning('measured_js already exists')
        # create the subscriber and keep in list
        self.__measured_js_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_js',
                                                         sensor_msgs.msg.JointState,
                                                         self.__measured_js_cb)
        self.__subscribers.append(self.__measured_js_subscriber)

        # add attributes to class instance
        self.__class_instance.measured_jp = self.__measured_jp
        self.__class_instance.measured_jv = self.__measured_jv
        self.__class_instance.measured_jf = self.__measured_jf


    # internal methods for measured_cp
    def __measured_cp_cb(self, msg):
        self.__measured_cp_data = TransformFromMsg(msg.transform)

    def __measured_cp(self):
        return self.__measured_cp_data

    def add_measured_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cp'):
            raise RuntimeWarning('measured_cp already exists')
        # create the subscriber and keep in list
        self.__measured_cp_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cp',
                                                         geometry_msgs.msg.TransformStamped,
                                                         self.__measured_cp_cb)
        self.__subscribers.append(self.__measured_cp_subscriber)
        # add attributes to class instance
        self.__class_instance.measured_cp = self.__measured_cp


    # internal methods for measured_cv
    def __measured_cv_cb(self, msg):
        self.__measured_cv_data[0] = msg.twist.linear.x
        self.__measured_cv_data[1] = msg.twist.linear.y
        self.__measured_cv_data[2] = msg.twist.linear.z
        self.__measured_cv_data[3] = msg.twist.angular.x
        self.__measured_cv_data[4] = msg.twist.angular.y
        self.__measured_cv_data[5] = msg.twist.angular.z

    def __measured_cv(self):
        return self.__measured_cv_data

    def add_measured_cv(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cv'):
            raise RuntimeWarning('measured_cv already exists')
        # create the subscriber and keep in list
        self.__measured_cv_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cv',
                                                         geometry_msgs.msg.TwistStamped,
                                                         self.__measured_cv_cb)
        self.__subscribers.append(self.__measured_cv_subscriber)
        # add attributes to class instance
        self.__class_instance.measured_cv = self.__measured_cv


    # internal methods for measured_cf
    def __measured_cf_cb(self, msg):
        self.__measured_cf_data[0] = msg.wrench.force.x
        self.__measured_cf_data[1] = msg.wrench.force.y
        self.__measured_cf_data[2] = msg.wrench.force.z
        self.__measured_cf_data[3] = msg.wrench.torque.x
        self.__measured_cf_data[4] = msg.wrench.torque.y
        self.__measured_cf_data[5] = msg.wrench.torque.z

    def __measured_cf(self):
        return self.__measured_cf_data

    def add_measured_cf(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cf'):
            raise RuntimeWarning('measured_cf already exists')
        # create the subscriber and keep in list
        self.__measured_cf_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cf',
                                                         geometry_msgs.msg.TwistStamped,
                                                         self.__measured_cf_cb)
        self.__subscribers.append(self.__measured_cf_subscriber)
        # add attributes to class instance
        self.__class_instance.measured_cf = self.__measured_cf


    # internal methods for servo_jp
    def __servo_jp(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position[:] = setpoint.flat
        self.__servo_jp_publisher.publish(msg)

    def add_servo_jp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_jp'):
            raise RuntimeWarning('servo_jp already exists')
        # create the subscriber and keep in list
        self.__servo_jp_publisher = rospy.Publisher(self.__ros_namespace + '/servo_jp',
                                                    sensor_msgs.msg.JointState,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_jp_publisher)
        # add attributes to class instance
        self.__class_instance.servo_jp = self.__servo_jp


    # internal methods for servo_cp
    def __servo_cp(self, setpoint):
        # convert to ROS msg and publish
        msg = TransformToMsg(setpoint)
        self.__servo_cp_publisher.publish(msg)

    def add_servo_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_cp'):
            raise RuntimeWarning('servo_cp already exists')
        # create the subscriber and keep in list
        self.__servo_cp_publisher = rospy.Publisher(self.__ros_namespace + '/servo_cp',
                                                    geometry_msgs.msg.TransformStamped,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_cp_publisher)
        # add attributes to class instance
        self.__class_instance.servo_cp = self.__servo_cp


    # internal methods for servo_jf
    def __servo_jf(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.effort[:] = setpoint.flat
        self.__servo_jf_publisher.publish(msg)

    def add_servo_jf(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_jf'):
            raise RuntimeWarning('servo_jf already exists')
        # create the subscriber and keep in list
        self.__servo_jf_publisher = rospy.Publisher(self.__ros_namespace + '/servo_jf',
                                                    sensor_msgs.msg.JointState,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_jf_publisher)
        # add attributes to class instance
        self.__class_instance.servo_jf = self.__servo_jf


    # internal methods for servo_cf
    def __servo_cf(self, setpoint):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.WrenchStamped()
        msg.wrench.force.x = setpoint[0]
        msg.wrench.force.y = setpoint[1]
        msg.wrench.force.z = setpoint[2]
        msg.wrench.torque.x = setpoint[3]
        msg.wrench.torque.y = setpoint[4]
        msg.wrench.torque.z = setpoint[5]
        self.__servo_cf_publisher.publish(msg)

    def add_servo_cf(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_cf'):
            raise RuntimeWarning('servo_cf already exists')
        # create the subscriber and keep in list
        self.__servo_cf_publisher = rospy.Publisher(self.__ros_namespace + '/servo_cf',
                                                    geometry_msgs.msg.WrenchStamped,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_cf_publisher)
        # add attributes to class instance
        self.__class_instance.servo_cf = self.__servo_cf


    # internal methods for move_jp
    def __move_jp(self, setpoint, blocking = True, timeout = 300.0):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position[:] = setpoint.flat
        self.__move_jp_publisher.publish(msg)
        if blocking:
            self.__is_busy_wait(timeout)

    def add_move_jp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'move_jp'):
            raise RuntimeWarning('move_jp already exists')
        # create the subscriber and keep in list
        self.__move_jp_publisher = rospy.Publisher(self.__ros_namespace + '/move_jp',
                                                   sensor_msgs.msg.JointState,
                                                   latch = True, queue_size = 1)
        self.__publishers.append(self.__move_jp_publisher)
        # add attributes to class instance
        self.__class_instance.move_jp = self.__move_jp


    # internal methods for move_cp
    def __move_cp(self, goal, blocking = True, timeout = 300.0):
        # convert to ROS msg and publish
        msg = TransformToMsg(goal)
        self.__move_cp_publisher.publish(msg)
        if blocking:
            self.__is_busy_wait(timeout)

    def add_move_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'move_cp'):
            raise RuntimeWarning('move_cp already exists')
        # create the subscriber and keep in list
        self.__move_cp_publisher = rospy.Publisher(self.__ros_namespace + '/move_cp',
                                                    geometry_msgs.msg.TransformStamped,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__move_cp_publisher)
        # add attributes to class instance
        self.__class_instance.move_cp = self.__move_cp
