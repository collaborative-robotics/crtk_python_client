#  Author(s):  Anton Deguet
#  Created on: 2018-02-15
#
# Copyright (c) 2018-2022 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import threading
import time

import rospy
import numpy
import PyKDL
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf_conversions.posemath
import crtk_msgs.msg
import crtk.wait_move_handle

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


def TwistFromMsg(t):
    return numpy.array([t.linear.x,
                        t.linear.y,
                        t.linear.z,
                        t.angular.x,
                        t.angular.y,
                        t.angular.z])


def WrenchFromMsg(w):
    return numpy.array([w.force.x,
                        w.force.y,
                        w.force.z,
                        w.torque.x,
                        w.torque.y,
                        w.torque.z])

class utils:
    """Class containing methods used to populate the interface
    (dictionary) of an existing Python object with CRTK compatile
    methods.  These methods will hide all the ROS publishers and
    subscribers.  They will also convert the ROS messages into useful
    types: numpy arrays for vector and matrices, PyKDL for 3D vectors
    and frames.  Finally, this methods use threading events to help
    synchronize the client and device/server (i.e. wait for state or end of
    move command).

    class_instance : object that will be populated
    ros_namespace : ROS namespace for the CRTK commands used by the device
    expected_interval : expected interval at which the device sends its motion state (measured, setpoint, goal)
    """
    def __init__(self,
                 class_instance,
                 ros_namespace,
                 expected_interval = 0.02,
                 operating_state_instance = None):
        self.__class_instance = class_instance
        self.__operating_state_instance = operating_state_instance
        self.__ros_namespace = ros_namespace
        self.__expected_interval = expected_interval
        self.__subscribers = []
        self.__publishers = []
        self.__attributes = []
        rospy.on_shutdown(self.__ros_shutdown)

    def __del__(self):
        self.remove_all()


    def __ros_shutdown(self):
        if hasattr(self, '_utils__operating_state_event'):
            self.__operating_state_event.set()


    def remove_all(self):
        for sub in self.__subscribers:
            sub.unregister()
        for pub in self.__publishers:
            pub.unregister()
        for attr in self.__attributes:
            dir(self.__class_instance)
            delattr(self.__class_instance, attr)
            dir(self.__class_instance)


    def __wait_for_valid_data(self, data, event, age, wait):
        event.clear()
        if age == None:
            age = self.__expected_interval
        if wait == None:
            wait = self.__expected_interval
        # check if user accepts cached data
        if age != 0.0:
            data_age = rospy.Time.now() - data.header.stamp
            if data_age <= rospy.Duration(age):
                return True
        if wait != 0.0:
            if event.wait(wait):
                return True
        return False


    # internal methods to manage state
    def __operating_state_cb(self, msg):
        # crtk operating state contains state as well as homed and busy
        self.__operating_state_data = msg

        # then when all data is saved, release "lock"
        self.__operating_state_event.set()

    def __operating_state(self, extra = None):
        if not extra:
            return self.__operating_state_data.state
        else:
            return [self.__operating_state_data.state,
                    self.__operating_state_data.header.stamp.to_sec()]

    def __wait_for_operating_state(self, expected_state, timeout):
        if timeout < 0.0:
            return False
        start_time = time.time()
        in_time = self.__operating_state_event.wait(timeout)
        if rospy.is_shutdown():
            return False;
        if in_time:
            # within timeout and result we expected
            if self.__operating_state_data.state == expected_state:
                return True
            else:
                # wait a bit more
                elapsed_time = time.time() - start_time
                self.__operating_state_event.clear()
                return self.__wait_for_operating_state(expected_state, timeout - elapsed_time)
        # past timeout
        return False

    def __state_command(self, state):
        # clear timeout
        self.__operating_state_event.clear()
        # convert to ROS msg and publish
        msg = crtk_msgs.msg.StringStamped()
        msg.string = state
        # publish and wait
        self.__state_command_publisher.publish(msg)

    def __is_enabled(self):
        return self.__operating_state_data.state == 'ENABLED'

    def __enable(self, timeout = 0):
        if self.__is_enabled():
            self.__state_command("enable")
            return True
        self.__operating_state_event.clear()
        self.__state_command("enable")
        return self.__wait_for_operating_state('ENABLED', timeout)

    def __is_disabled(self):
        return self.__operating_state_data.state == 'DISABLED'

    def __disable(self, timeout = 0):
        if self.__is_disabled():
            self.__state_command("disable")
            return True
        self.__operating_state_event.clear()
        self.__state_command("disable")
        return self.__wait_for_operating_state('DISABLED', timeout)

    def __is_homed(self, extra = None):
        if not extra:
            return self.__operating_state_data.is_homed
        else:
            return [self.__operating_state_data.is_homed,
                    self.__operating_state_data.header.stamp.to_sec()]

    def __wait_for_homed(self, timeout, expected_homed):
        if timeout < 0.0:
            return False
        start_time = time.time()
        self.__operating_state_event.clear()
        in_time = self.__operating_state_event.wait(timeout)
        if rospy.is_shutdown():
            return False;
        if in_time:
            # within timeout and result we expected
            if (self.__operating_state_data.is_homed == expected_homed) and (not self.__operating_state_data.is_busy):
                return True
            else:
                # wait a bit more
                elapsed_time = time.time() - start_time
                return self.__wait_for_homed(timeout - elapsed_time, expected_homed)
        # past timeout
        return False

    def __home(self, timeout = 0):
        if self.__is_homed():
            self.__state_command("home")
            return True
        self.__operating_state_event.clear()
        self.__state_command("home")
        return self.__wait_for_homed(timeout, True)

    def __unhome(self, timeout = 0):
        if not self.__is_homed():
            self.__state_command("unhome")
            return True
        self.__operating_state_event.clear()
        self.__state_command("unhome")
        return self.__wait_for_homed(timeout, False)

    def __is_busy(self,
                  start_time = rospy.Time(0.0),
                  extra = None):
        result = True
        if (self.__operating_state_data.header.stamp > start_time):
            result = self.__operating_state_data.is_busy
        if not extra:
            return result
        else:
            return [result,
                    self.__operating_state_data.header.stamp.to_sec()]

    def __wait_for_busy(self,
                        is_busy = False,
                        start_time = rospy.Time(0.0),
                        timeout = 30.0):
        # if timeout is negative, not waiting
        if timeout < 0.0:
            return False
        # if start_time 0.0, user provided a start time and we should
        # check if an event arrived after start_time
        if start_time > rospy.Time(0.0):
            if (self.__operating_state_data.header.stamp > start_time
                and self.__operating_state_data.is_busy == is_busy):
                return True
        # other cases, waiting for an operating_state event
        _start_time = time.time()
        self.__operating_state_event.clear()
        in_time = self.__operating_state_event.wait(timeout)
        if rospy.is_shutdown():
            return False;
        if in_time:
            # within timeout and result we expected
            if self.__operating_state_data.is_busy == is_busy:
                return True
            else:
                # wait a bit more
                elapsed_time = time.time() - _start_time
                return self.__wait_for_busy(is_busy = is_busy,
                                            start_time = start_time,
                                            timeout = (timeout - elapsed_time))
        # past timeout
        return False

    def add_operating_state(self, optional_ros_namespace = None):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'operating_state'):
            raise RuntimeWarning('operating_state already exists')
        # data
        self.__operating_state_data = crtk_msgs.msg.operating_state()
        self.__operating_state_event = threading.Event()

        # determine namespace to use
        if optional_ros_namespace == None:
            namespace_to_use = self.__ros_namespace
        else:
            namespace_to_use = optional_ros_namespace

        # create the subscriber/publisher and keep in list
        self.__operating_state_subscriber = rospy.Subscriber(namespace_to_use + '/operating_state',
                                                             crtk_msgs.msg.operating_state, self.__operating_state_cb)
        self.__subscribers.append(self.__operating_state_subscriber)
        self.__state_command_publisher = rospy.Publisher(namespace_to_use + '/state_command',
                                                                   crtk_msgs.msg.StringStamped,
                                                                   latch = True, queue_size = 1)
        self.__publishers.append(self.__state_command_publisher)
        # add attributes to class instance
        self.__class_instance.operating_state = self.__operating_state
        self.__class_instance.wait_for_operating_state = self.__wait_for_operating_state
        self.__class_instance.state_command = self.__state_command
        self.__class_instance.is_enabled = self.__is_enabled
        self.__class_instance.enable = self.__enable
        self.__class_instance.is_disabled = self.__is_disabled
        self.__class_instance.disable = self.__disable
        self.__class_instance.home = self.__home
        self.__class_instance.unhome = self.__unhome
        self.__class_instance.is_homed = self.__is_homed
        self.__class_instance.is_busy = self.__is_busy
        self.__class_instance.wait_for_busy = self.__wait_for_busy
        if not self.__operating_state_instance:
            self.__operating_state_instance = self.__class_instance
        else:
            raise RuntimeWarning('over writting operating state for ' + self.__ros_namespace)

    # internal methods for setpoint_js
    def __setpoint_js_cb(self, msg):
        self.__setpoint_js_data = msg
        self.__setpoint_js_event.set()

    def __setpoint_js(self, age = None, wait = None):
        if self.__wait_for_valid_data(self.__setpoint_js_data,
                                      self.__setpoint_js_event,
                                      age, wait):
            return [numpy.array(self.__setpoint_js_data.position),
                    numpy.array(self.__setpoint_js_data.velocity),
                    numpy.array(self.__setpoint_js_data.effort),
                    self.__setpoint_js_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get setpoint_js')

    def __setpoint_jp(self, age = None, wait = None, extra = None):
        """Joint Position Setpoint.  Default age and wait are set to
        expected_interval.  Age determines maximum age of already
        received data considered valid.  If age is set to 0, any data
        already received is considered valid.  Wait is the amount of
        time user is willing to wait if there's no valid data already
        received.  The method will not wait if wait is set to 0.
        """
        if self.__wait_for_valid_data(self.__setpoint_js_data,
                                      self.__setpoint_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__setpoint_js_data.position)
            else:
                return [numpy.array(self.__setpoint_js_data.position),
                        self.__setpoint_js_data.header.stamp.to_sec()]

        raise RuntimeWarning('unable to get setpoint_jp in namespace ' + self.__ros_namespace)

    def __setpoint_jv(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__setpoint_js_data,
                                      self.__setpoint_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__setpoint_js_data.velocity)
            else:
                return [numpy.array(self.__setpoint_js_data.velocity),
                        self.__setpoint_js_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get setpoint_jv')

    def __setpoint_jf(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__setpoint_js_data,
                                      self.__setpoint_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__setpoint_js_data.effort)
            else:
                return [numpy.array(self.__setpoint_js_data.effort),
                        self.__setpoint_js_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get setpoint_jf')

    def add_setpoint_js(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'setpoint_js'):
            raise RuntimeWarning('setpoint_js already exists')
        # data
        self.__setpoint_js_data = sensor_msgs.msg.JointState()
        self.__setpoint_js_event = threading.Event()
        # create the subscriber and keep in list
        self.__setpoint_js_subscriber = rospy.Subscriber(self.__ros_namespace + '/setpoint_js',
                                                         sensor_msgs.msg.JointState,
                                                         self.__setpoint_js_cb)
        self.__subscribers.append(self.__setpoint_js_subscriber)
        # add attributes to class instance
        self.__class_instance.setpoint_js = self.__setpoint_js
        self.__class_instance.setpoint_jp = self.__setpoint_jp
        self.__class_instance.setpoint_jv = self.__setpoint_jv
        self.__class_instance.setpoint_jf = self.__setpoint_jf


    # internal methods for setpoint_cp
    def __setpoint_cp_cb(self, msg):
        self.__setpoint_cp_lock = True
        self.__setpoint_cp_data = msg
        self.__setpoint_cp_lock = False
        self.__setpoint_cp_event.set()

    def __setpoint_cp(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__setpoint_cp_data,
                                      self.__setpoint_cp_event,
                                      age, wait):
            if not extra:
                return tf_conversions.posemath.fromMsg(self.__setpoint_cp_data.pose)
            else:
                return [tf_conversions.posemath.fromMsg(self.__setpoint_cp_data.pose),
                        self.__setpoint_cp_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get setpoint_cp')

    def add_setpoint_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'setpoint_cp'):
            raise RuntimeWarning('setpoint_cp already exists')
        # data
        self.__setpoint_cp_data = geometry_msgs.msg.PoseStamped()
        self.__setpoint_cp_event = threading.Event()
        self.__setpoint_cp_lock = False
        # create the subscriber and keep in list
        self.__setpoint_cp_subscriber = rospy.Subscriber(self.__ros_namespace + '/setpoint_cp',
                                                         geometry_msgs.msg.PoseStamped,
                                                         self.__setpoint_cp_cb)
        self.__subscribers.append(self.__setpoint_cp_subscriber)
        # add attributes to class instance
        self.__class_instance.setpoint_cp = self.__setpoint_cp


    # internal methods for measured_js
    def __measured_js_cb(self, msg):
        self.__measured_js_data = msg
        self.__measured_js_event.set()

    def __measured_js(self, age = None, wait = None):
        if self.__wait_for_valid_data(self.__measured_js_data,
                                      self.__measured_js_event,
                                      age, wait):
            return [numpy.array(self.__measured_js_data.position),
                    numpy.array(self.__measured_js_data.velocity),
                    numpy.array(self.__measured_js_data.effort),
                    self.__measured_js_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get measured_js')

    def __measured_jp(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_js_data,
                                      self.__measured_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__measured_js_data.position)
            else:
                return [numpy.array(self.__measured_js_data.position),
                        self.__measured_js_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get measured_jp')

    def __measured_jv(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_js_data,
                                      self.__measured_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__measured_js_data.velocity)
            else:
                return [numpy.array(self.__measured_js_data.velocity),
                        self.__measured_js_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get measured_jv')

    def __measured_jf(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_js_data,
                                      self.__measured_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__measured_js_data.effort)
            else:
                return [numpy.array(self.__measured_js_data.effort),
                        self.__measured_js_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get measured_jf')

    def add_measured_js(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_js'):
            raise RuntimeWarning('measured_js already exists')
        # data
        self.__measured_js_data = sensor_msgs.msg.JointState()
        self.__measured_js_event = threading.Event()
        # create the subscriber and keep in list
        self.__measured_js_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_js',
                                                         sensor_msgs.msg.JointState,
                                                         self.__measured_js_cb)
        self.__subscribers.append(self.__measured_js_subscriber)
        # add attributes to class instance
        self.__class_instance.measured_js = self.__measured_js
        self.__class_instance.measured_jp = self.__measured_jp
        self.__class_instance.measured_jv = self.__measured_jv
        self.__class_instance.measured_jf = self.__measured_jf


    # internal methods for measured_cp
    def __measured_cp_cb(self, msg):
        self.__measured_cp_data = msg
        self.__measured_cp_event.set()

    def __measured_cp(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_cp_data,
                                      self.__measured_cp_event,
                                      age, wait):
            if not extra:
                return tf_conversions.posemath.fromMsg(self.__measured_cp_data.pose)
            else:
                return [tf_conversions.posemath.fromMsg(self.__measured_cp_data.pose),
                        self.__measured_cp_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get measured_cp')

    def add_measured_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cp'):
            raise RuntimeWarning('measured_cp already exists')
        # data
        self.__measured_cp_data = geometry_msgs.msg.PoseStamped()
        self.__measured_cp_event = threading.Event()
        # create the subscriber and keep in list
        self.__measured_cp_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cp',
                                                         geometry_msgs.msg.PoseStamped,
                                                         self.__measured_cp_cb)
        self.__subscribers.append(self.__measured_cp_subscriber)
        # add attributes to class instance
        self.__class_instance.measured_cp = self.__measured_cp


    # internal methods for measured_cv
    def __measured_cv_cb(self, msg):
        self.__measured_cv_data = msg
        self.__measured_cv_event.set()

    def __measured_cv(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_cv_data,
                                      self.__measured_cv_event,
                                      age, wait):
            if not extra:
                return TwistFromMsg(self.__measured_cv_data.twist)
            else:
                return [TwistFromMsg(self.__measured_cv_data.twist),
                        self.__measured_cv_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get measured_cv')

    def add_measured_cv(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cv'):
            raise RuntimeWarning('measured_cv already exists')
        # data
        self.__measured_cv_data = geometry_msgs.msg.TwistStamped()
        self.__measured_cv_event = threading.Event()
        # create the subscriber and keep in list
        self.__measured_cv_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cv',
                                                         geometry_msgs.msg.TwistStamped,
                                                         self.__measured_cv_cb)
        self.__subscribers.append(self.__measured_cv_subscriber)
        # add attributes to class instance
        self.__class_instance.measured_cv = self.__measured_cv


    # internal methods for measured_cf
    def __measured_cf_cb(self, msg):
        self.__measured_cf_data = msg
        self.__measured_cf_event.set()

    def __measured_cf(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_cf_data,
                                      self.__measured_cf_event,
                                      age, wait):
            if not extra:
                return WrenchFromMsg(self.__measured_cf_data.wrench)
            else:
                return [WrenchFromMsg(self.__measured_cf_data.wrench),
                        self.__measured_cf_data.header.stamp.to_sec()]
        raise RuntimeWarning('unable to get measured_cf')

    def add_measured_cf(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cf'):
            raise RuntimeWarning('measured_cf already exists')
        # data
        self.__measured_cf_data = geometry_msgs.msg.WrenchStamped()
        self.__measured_cf_event = threading.Event()
        # create the subscriber and keep in list
        self.__measured_cf_subscriber = rospy.Subscriber(self.__ros_namespace + '/measured_cf',
                                                         geometry_msgs.msg.WrenchStamped,
                                                         self.__measured_cf_cb)
        self.__subscribers.append(self.__measured_cf_subscriber)
        # add attributes to class instance
        self.__class_instance.measured_cf = self.__measured_cf


    # internal methods for jacobian
    def __jacobian_cb(self, msg):
        self.__jacobian_data = msg
        self.__jacobian_event.set()

    def __jacobian(self):
        jacobian = numpy.asarray(self.__jacobian_data.data)
        jacobian.shape = self.__jacobian_data.layout.dim[0].size, self.__jacobian_data.layout.dim[1].size
        return jacobian

    def add_jacobian(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'jacobian'):
            raise RuntimeWarning('jacobian already exists')
        # data
        self.__jacobian_data = std_msgs.msg.Float64MultiArray()
        self.__jacobian_event = threading.Event()
        # create the subscriber and keep in list
        self.__jacobian_subscriber = rospy.Subscriber(self.__ros_namespace + '/jacobian',
                                                      std_msgs.msg.Float64MultiArray,
                                                      self.__jacobian_cb)
        self.__subscribers.append(self.__jacobian_subscriber)
        # add attributes to class instance
        self.__class_instance.jacobian = self.__jacobian



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


    # internal methods for servo_jr
    def __servo_jr(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position[:] = setpoint.flat
        self.__servo_jr_publisher.publish(msg)

    def add_servo_jr(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_jr'):
            raise RuntimeWarning('servo_jr already exists')
        # create the subscriber and keep in list
        self.__servo_jr_publisher = rospy.Publisher(self.__ros_namespace + '/servo_jr',
                                                    sensor_msgs.msg.JointState,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__servo_jr_publisher)
        # add attributes to class instance
        self.__class_instance.servo_jr = self.__servo_jr


    # internal methods for servo_cp
    def __servo_cp(self, setpoint):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.PoseStamped()
        msg.pose = tf_conversions.posemath.toMsg(setpoint)
        self.__servo_cp_publisher.publish(msg)

    def add_servo_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_cp'):
            raise RuntimeWarning('servo_cp already exists')
        # create the subscriber and keep in list
        self.__servo_cp_publisher = rospy.Publisher(self.__ros_namespace + '/servo_cp',
                                                    geometry_msgs.msg.PoseStamped,
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
    def __move_jp(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position[:] = setpoint.flat
        handle = crtk.wait_move_handle(self.__operating_state_instance)
        self.__move_jp_publisher.publish(msg)
        return handle

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


    # internal methods for move_jr
    def __move_jr(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position[:] = setpoint.flat
        handle = crtk.wait_move_handle(self.__operating_state_instance)
        self.__move_jr_publisher.publish(msg)
        return handle

    def add_move_jr(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'move_jr'):
            raise RuntimeWarning('move_jr already exists')
        # create the subscriber and keep in list
        self.__move_jr_publisher = rospy.Publisher(self.__ros_namespace + '/move_jr',
                                                   sensor_msgs.msg.JointState,
                                                   latch = True, queue_size = 1)
        self.__publishers.append(self.__move_jr_publisher)
        # add attributes to class instance
        self.__class_instance.move_jr = self.__move_jr


    # internal methods for move_cp
    def __move_cp(self, goal):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.PoseStamped()
        msg.pose = tf_conversions.posemath.toMsg(goal)
        handle = crtk.wait_move_handle(self.__operating_state_instance);
        self.__move_cp_publisher.publish(msg)
        return handle

    def add_move_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'move_cp'):
            raise RuntimeWarning('move_cp already exists')
        # create the subscriber and keep in list
        self.__move_cp_publisher = rospy.Publisher(self.__ros_namespace + '/move_cp',
                                                    geometry_msgs.msg.PoseStamped,
                                                    latch = True, queue_size = 1)
        self.__publishers.append(self.__move_cp_publisher)
        # add attributes to class instance
        self.__class_instance.move_cp = self.__move_cp
