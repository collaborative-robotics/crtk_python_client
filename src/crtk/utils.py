#  Author(s):  Anton Deguet
#  Created on: 2018-02-15
#
# Copyright (c) 2018-2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import threading

import numpy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import crtk_msgs.msg
import crtk_msgs.srv
import crtk.wait_move_handle

import crtk.msg_conversions as msg_conv


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
    ral : ral object for the device namespace
    expected_interval : expected interval at which the device sends its motion state (measured, setpoint, goal)
    """
    def __init__(self,
                 class_instance,
                 ral,
                 expected_interval = 0.02,
                 operating_state_instance = None):
        self.__class_instance = class_instance
        self.__operating_state_instance = operating_state_instance
        self.__ral = ral
        self.__expected_interval = expected_interval

        self.__ral.on_shutdown(self.__ros_shutdown)

    def __ros_shutdown(self):
        if hasattr(self, '_utils__operating_state_event'):
            self.__operating_state_event.set()

    def __wait_for_valid_data(self, data, event, age, wait):
        event.clear()
        if age is None:
            age = self.__expected_interval
        if wait is None:
            wait = self.__expected_interval
        # check if user accepts cached data
        if age != 0.0:
            data_age = self.__ral.now() - self.__ral.get_timestamp(data)
            if data_age <= self.__ral.create_duration(age):
                return True
        if wait != 0.0:
            if event.wait(wait):
                return True
        return False

    # internal methods to manage state
    def __operating_state_cb(self, msg):
        event_time = self.__ral.get_timestamp(msg)
        last_event_time = self.__ral.get_timestamp(self.__operating_state_data)

        # update last_busy_time
        if msg.is_busy and event_time > self.__last_busy_time:
            self.__last_busy_time = event_time

        if event_time > last_event_time:
            # crtk operating state contains state as well as homed and busy
            self.__operating_state_data = msg

        # then when all data is saved, release "lock"
        self.__operating_state_event.set()

    def __operating_state(self, extra = None):
        if not extra:
            return self.__operating_state_data.state
        else:
            return [self.__operating_state_data.state,
                    self.__ral.to_sec(self.__operating_state_data)]

    def __wait_for_operating_state(self, expected_state, timeout):
        if timeout < 0.0:
            return False
        start_time = self.__ral.now()
        in_time = self.__operating_state_event.wait(timeout)
        if self.__ral.is_shutdown():
            return False

        if in_time:
            # within timeout and result we expected
            if self.__operating_state_data.state == expected_state:
                return True
            else:
                # wait a bit more
                elapsed_time = self.__ral.to_sec(self.__ral.now() - start_time)
                self.__operating_state_event.clear()
                return self.__wait_for_operating_state(expected_state = expected_state,
                                                       timeout = timeout - elapsed_time)
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
                    self.__ral.to_sec(self.__operating_state_data)]

    def __wait_for_homed(self, timeout, expected_homed):
        if timeout < 0.0:
            return False
        start_time = self.__ral.now()
        self.__operating_state_event.clear()
        in_time = self.__operating_state_event.wait(timeout)
        if self.__ral.is_shutdown():
            return False

        if in_time:
            # within timeout and result we expected
            if (self.__operating_state_data.is_homed == expected_homed) and (not self.__operating_state_data.is_busy):
                return True
            else:
                # wait a bit more
                elapsed_time = self.__ral.to_sec(self.__ral.now() - start_time)
                self.__operating_state_event.clear()
                return self.__wait_for_homed(expected_homed = expected_homed,
                                             timeout = timeout - elapsed_time)
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
                  start_time = None,
                  extra = None):
        # set start time to now if not specified
        if start_time is None:
            start_time = self.__ral.now()
        result = True
        if self.__ral.get_timestamp(self.__operating_state_data) > start_time:
            result = self.__operating_state_data.is_busy
        if not extra:
            return result
        else:
            return [result,
                    self.__ral.to_sec(self.__operating_state_data)]

    def __wait_for_busy(self,
                        is_busy = False,
                        start_time = None,
                        timeout = 30.0):
        # set start_time if not provided
        start_time = start_time or self.__ral.now()
        elapsed = self.__ral.to_sec(self.__ral.now() - start_time)
        if elapsed > timeout:
            return False # past timeout, wait failed

        # check if wait condition has been satisfied
        if is_busy:
            if self.__last_busy_time > start_time:
                return True # waiting for busy and currently busy
        else:
            was_busy = self.__last_busy_time > start_time
            no_longer_busy = not self.__operating_state_data.is_busy
            if was_busy and no_longer_busy:
                return True # waiting for idle, and have recent transition to idle

        # wait for next operating state event (or timeout)
        self.__operating_state_event.clear()
        in_time = self.__operating_state_event.wait(timeout - elapsed)

        # check for ROS shutdown or event time-out
        if self.__ral.is_shutdown() or not in_time:
            return False

        # recurse
        return self.__wait_for_busy(is_busy = is_busy,
                                    start_time = start_time,
                                    timeout = timeout)

    def add_operating_state(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'operating_state'):
            raise RuntimeWarning('operating_state already exists')
        # data
        self.__operating_state_data = crtk_msgs.msg.OperatingState()
        self.__operating_state_event = threading.Event()
        self.__last_busy_time = self.__ral.now()

        # create the subscriber/publisher
        self.__operating_state_subscriber = self.__ral.subscriber(
            'operating_state',
            crtk_msgs.msg.OperatingState,
            self.__operating_state_cb,
            queue_size = 10
        )

        self.__state_command_publisher = self.__ral.publisher(
            'state_command',
            crtk_msgs.msg.StringStamped,
            latch = True, queue_size = 10
        )

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
            raise RuntimeWarning('over writting operating state for ' + self.__ral.namespace())

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
                    self.__ral.to_sec(self.__setpoint_js_data)]
        raise RuntimeWarning('unable to get setpoint_js ({})'.format(self.__ral.get_topic_name(self.__setpoint_js_subscriber)))

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
                        self.__ral.to_sec(self.__setpoint_js_data)]
        raise RuntimeWarning('unable to get setpoint_jp ({})'.format(self.__ral.get_topic_name(self.__setpoint_js_subscriber)))

    def __setpoint_jv(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__setpoint_js_data,
                                      self.__setpoint_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__setpoint_js_data.velocity)
            else:
                return [numpy.array(self.__setpoint_js_data.velocity),
                        self.__ral.to_sec(self.__setpoint_js_data)]
        raise RuntimeWarning('unable to get setpoint_jv ({})'.format(self.__ral.get_topic_name(self.__setpoint_js_subscriber)))

    def __setpoint_jf(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__setpoint_js_data,
                                      self.__setpoint_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__setpoint_js_data.effort)
            else:
                return [numpy.array(self.__setpoint_js_data.effort),
                        self.__ral.to_sec(self.__setpoint_js_data)]
        raise RuntimeWarning('unable to get setpoint_jf ({})'.format(self.__ral.get_topic_name(self.__setpoint_js_subscriber)))

    def add_setpoint_js(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'setpoint_js'):
            raise RuntimeWarning('setpoint_js already exists')
        # data
        self.__setpoint_js_data = sensor_msgs.msg.JointState()
        self.__setpoint_js_event = threading.Event()
        # create the subscriber
        self.__setpoint_js_subscriber = self.__ral.subscriber(
            'setpoint_js',
            sensor_msgs.msg.JointState,
            self.__setpoint_js_cb,
            queue_size = 10
        )
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
                return msg_conv.FrameFromPoseMsg(self.__setpoint_cp_data.pose)
            else:
                return [msg_conv.FrameFromPoseMsg(self.__setpoint_cp_data.pose),
                        self.__ral.to_sec(self.__setpoint_cp_data)]
        raise RuntimeWarning('unable to get setpoint_cp ({})'.format(self.__ral.get_topic_name(self.__setpoint_cp_subscriber)))

    def add_setpoint_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'setpoint_cp'):
            raise RuntimeWarning('setpoint_cp already exists')
        # data
        self.__setpoint_cp_data = geometry_msgs.msg.PoseStamped()
        self.__setpoint_cp_event = threading.Event()
        self.__setpoint_cp_lock = False
        # create the subscriber
        self.__setpoint_cp_subscriber = self.__ral.subscriber(
            'setpoint_cp',
            geometry_msgs.msg.PoseStamped,
            self.__setpoint_cp_cb,
            queue_size = 10
        )
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
                    self.__ral.to_sec(self.__measured_js_data)]
        raise RuntimeWarning('unable to get measured_js ({})'.format(self.__ral.get_topic_name(self.__measured_js_subscriber)))

    def __measured_jp(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_js_data,
                                      self.__measured_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__measured_js_data.position)
            else:
                return [numpy.array(self.__measured_js_data.position),
                        self.__ral.to_sec(self.__measured_js_data)]
        raise RuntimeWarning('unable to get measured_jp ({})'.format(self.__ral.get_topic_name(self.__measured_js_subscriber)))

    def __measured_jv(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_js_data,
                                      self.__measured_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__measured_js_data.velocity)
            else:
                return [numpy.array(self.__measured_js_data.velocity),
                        self.__ral.to_sec(self.__measured_js_data)]
        raise RuntimeWarning('unable to get measured_jv ({})'.format(self.__ral.get_topic_name(self.__measured_js_subscriber)))

    def __measured_jf(self, age = None, wait = None, extra = None):
        if self.__wait_for_valid_data(self.__measured_js_data,
                                      self.__measured_js_event,
                                      age, wait):
            if not extra:
                return numpy.array(self.__measured_js_data.effort)
            else:
                return [numpy.array(self.__measured_js_data.effort),
                        self.__ral.to_sec(self.__measured_js_data)]
        raise RuntimeWarning('unable to get measured_jf ({})'.format(self.__ral.get_topic_name(self.__measured_js_subscriber)))

    def add_measured_js(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_js'):
            raise RuntimeWarning('measured_js already exists')
        # data
        self.__measured_js_data = sensor_msgs.msg.JointState()
        self.__measured_js_event = threading.Event()
        # create the subscriber
        self.__measured_js_subscriber = self.__ral.subscriber(
            'measured_js',
            sensor_msgs.msg.JointState,
            self.__measured_js_cb
        )
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
                return msg_conv.FrameFromPoseMsg(self.__measured_cp_data.pose)
            else:
                return [msg_conv.FrameFromPoseMsg(self.__measured_cp_data.pose),
                        self.__ral.to_sec(self.__measured_cp_data)]
        raise RuntimeWarning('unable to get measured_cp ({})'.format(self.__ral.get_topic_name(self.__measured_cp_subscriber)))

    def add_measured_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cp'):
            raise RuntimeWarning('measured_cp already exists')
        # data
        self.__measured_cp_data = geometry_msgs.msg.PoseStamped()
        self.__measured_cp_event = threading.Event()
        # create the subscriber
        self.__measured_cp_subscriber = self.__ral.subscriber(
            'measured_cp',
            geometry_msgs.msg.PoseStamped,
            self.__measured_cp_cb,
            queue_size = 10
        )
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
                return msg_conv.ArrayFromTwistMsg(self.__measured_cv_data.twist)
            else:
                return [msg_conv.ArrayFromTwistMsg(self.__measured_cv_data.twist),
                        self.__ral.to_sec(self.__measured_cv_data)]
        raise RuntimeWarning('unable to get measured_cv ({})'.format(self.__ral.get_topic_name(self.__measured_cv_subscriber)))

    def add_measured_cv(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cv'):
            raise RuntimeWarning('measured_cv already exists')
        # data
        self.__measured_cv_data = geometry_msgs.msg.TwistStamped()
        self.__measured_cv_event = threading.Event()
        # create the subscriber
        self.__measured_cv_subscriber = self.__ral.subscriber(
            'measured_cv',
            geometry_msgs.msg.TwistStamped,
            self.__measured_cv_cb,
            queue_size = 10
        )
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
                return msg_conv.ArrayFromWrenchMsg(self.__measured_cf_data.wrench)
            else:
                return [msg_conv.ArrayFromWrenchMsg(self.__measured_cf_data.wrench),
                        self.__ral.to_sec(self.__measured_cf_data)]
        raise RuntimeWarning('unable to get measured_cf ({})'.format(self.__ral.get_topic_name(self.__measured_cf_subscriber)))

    def add_measured_cf(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'measured_cf'):
            raise RuntimeWarning('measured_cf already exists')
        # data
        self.__measured_cf_data = geometry_msgs.msg.WrenchStamped()
        self.__measured_cf_event = threading.Event()
        # create the subscriber
        self.__measured_cf_subscriber = self.__ral.subscriber(
            'measured_cf',
            geometry_msgs.msg.WrenchStamped,
            self.__measured_cf_cb,
            queue_size = 10
        )
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
        # create the subscriber
        self.__jacobian_subscriber = self.__ral.subscriber(
            'jacobian',
            std_msgs.msg.Float64MultiArray,
            self.__jacobian_cb,
            queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.jacobian = self.__jacobian


    # internal methods for hold
    def __hold(self):
        # convert to ROS msg and publish
        msg = std_msgs.msg.Empty()
        self.__hold_publisher.publish(msg)

    def add_hold(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'hold'):
            raise RuntimeWarning('hold already exists')
        # create the subscriber
        self.__hold_publisher = self.__ral.publisher(
            'hold',
            std_msgs.msg.Empty,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.hold = self.__hold


    # internal methods for free
    def __free(self):
        # convert to ROS msg and publish
        msg = std_msgs.msg.Empty()
        self.__free_publisher.publish(msg)

    def add_free(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'free'):
            raise RuntimeWarning('free already exists')
        # create the subscriber
        self.__free_publisher = self.__ral.publisher(
            'free',
            std_msgs.msg.Empty,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.free = self.__free


    # internal methods for servo_jp
    def __servo_jp(self, setpoint_p, setpoint_v = numpy.array([])):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position = setpoint_p.tolist()
        msg.velocity = setpoint_v.tolist()
        self.__servo_jp_publisher.publish(msg)

    def add_servo_jp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_jp'):
            raise RuntimeWarning('servo_jp already exists')
        # create the subscriber
        self.__servo_jp_publisher = self.__ral.publisher(
            'servo_jp',
            sensor_msgs.msg.JointState,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.servo_jp = self.__servo_jp


    # internal methods for servo_jr
    def __servo_jr(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position = setpoint.tolist()
        self.__servo_jr_publisher.publish(msg)

    def add_servo_jr(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_jr'):
            raise RuntimeWarning('servo_jr already exists')
        # create the subscriber
        self.__servo_jr_publisher = self.__ral.publisher(
            'servo_jr',
            sensor_msgs.msg.JointState,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.servo_jr = self.__servo_jr


    # internal methods for servo_cp
    def __servo_cp(self, setpoint):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.PoseStamped()
        msg.pose = msg_conv.FrameToPoseMsg(setpoint)
        self.__servo_cp_publisher.publish(msg)

    def add_servo_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_cp'):
            raise RuntimeWarning('servo_cp already exists')
        # create the subscriber
        self.__servo_cp_publisher = self.__ral.publisher(
            'servo_cp',
            geometry_msgs.msg.PoseStamped,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.servo_cp = self.__servo_cp


    # internal methods for servo_jf
    def __servo_jf(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.effort = setpoint.tolist()
        self.__servo_jf_publisher.publish(msg)

    def add_servo_jf(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_jf'):
            raise RuntimeWarning('servo_jf already exists')
        # create the subscriber
        self.__servo_jf_publisher = self.__ral.publisher(
            'servo_jf',
            sensor_msgs.msg.JointState,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.servo_jf = self.__servo_jf


    # internal methods for servo_cf
    def __servo_cf(self, setpoint):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.WrenchStamped()
        msg.wrench = msg_conv.ArrayToWrenchMsg(setpoint)
        self.__servo_cf_publisher.publish(msg)

    def add_servo_cf(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_cf'):
            raise RuntimeWarning('servo_cf already exists')
        # create the subscriber
        self.__servo_cf_publisher = self.__ral.publisher(
            'servo_cf',
            geometry_msgs.msg.WrenchStamped,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.servo_cf = self.__servo_cf


    # internal methods for servo_cv
    def __servo_cv(self, setpoint):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.TwistStamped()
        msg.twist = msg_conv.ArrayToTwistMsg(setpoint)
        self.__servo_cv_publisher.publish(msg)

    def add_servo_cv(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'servo_cv'):
            raise RuntimeWarning('servo_cv already exists')
        # create the subscriber
        self.__servo_cv_publisher = self.__ral.publisher(
            'servo_cv',
            geometry_msgs.msg.TwistStamped,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.servo_cv = self.__servo_cv


    # internal methods for move_jp
    def __move_jp(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position = setpoint.tolist()
        handle = crtk.wait_move_handle(self.__operating_state_instance, self.__ral)
        self.__move_jp_publisher.publish(msg)
        return handle

    def add_move_jp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'move_jp'):
            raise RuntimeWarning('move_jp already exists')
        # create the subscriber
        self.__move_jp_publisher = self.__ral.publisher(
            'move_jp',
            sensor_msgs.msg.JointState,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.move_jp = self.__move_jp


    # internal methods for move_jr
    def __move_jr(self, setpoint):
        # convert to ROS msg and publish
        msg = sensor_msgs.msg.JointState()
        msg.position = setpoint.tolist()
        handle = crtk.wait_move_handle(self.__operating_state_instance, self.__ral)
        self.__move_jr_publisher.publish(msg)
        return handle

    def add_move_jr(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'move_jr'):
            raise RuntimeWarning('move_jr already exists')
        # create the subscriber
        self.__move_jr_publisher = self.__ral.publisher(
            'move_jr',
            sensor_msgs.msg.JointState,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.move_jr = self.__move_jr


    # internal methods for move_cp
    def __move_cp(self, goal):
        # convert to ROS msg and publish
        msg = geometry_msgs.msg.PoseStamped()
        msg.pose = msg_conv.FrameToPoseMsg(goal)
        handle = crtk.wait_move_handle(self.__operating_state_instance, self.__ral)
        self.__move_cp_publisher.publish(msg)
        return handle

    def add_move_cp(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'move_cp'):
            raise RuntimeWarning('move_cp already exists')
        # create the subscriber
        self.__move_cp_publisher = self.__ral.publisher(
            'move_cp',
            geometry_msgs.msg.PoseStamped,
            latch = False, queue_size = 10
        )
        # add attributes to class instance
        self.__class_instance.move_cp = self.__move_cp


    # internal methods for forward_kinematics
    def __forward_kinematics(self, jp, extra = None):
        # convert to ROS msg and publish
        if self.__ral.ros_version() == 1:
            request = crtk_msgs.srv.QueryForwardKinematicsRequest()
        else:
            request = crtk_msgs.srv.QueryForwardKinematics.Request()
        request.jp = jp.tolist()
        response = self.__forward_kinematics_service.call(request)
        if not extra:
            return msg_conv.FrameFromPoseMsg(response.cp)
        else:
            return [msg_conv.FrameFromPoseMsg(response.cp), response.result, response.message]

    def add_forward_kinematics(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'forward_kinematics'):
            raise RuntimeWarning('forward_kinematics already exists')
        # create the service
        self.__forward_kinematics_service = self.__ral.service_client(
            '/forward_kinematics', crtk_msgs.srv.QueryForwardKinematics)
        # add attributes to class instance
        self.__class_instance.forward_kinematics = self.__forward_kinematics


    # internal methods for inverse_kinematics
    def __inverse_kinematics(self, jp, cp, extra = None):
        # convert to ROS msg and publish
        if self.__ral.ros_version() == 1:
            request = crtk_msgs.srv.QueryInverseKinematicsRequest()
        else:
            request = crtk_msgs.srv.QueryInverseKinematics.Request()
        request.jp = jp.tolist()
        request.cp = msg_conv.FrameToPoseMsg(cp)
        response = self.__inverse_kinematics_service.call(request)
        if not extra:
            return numpy.array(response.jp)
        else:
            return [numpy.array(response.jp), response.result, response.message]

    def add_inverse_kinematics(self):
        # throw a warning if this has alread been added to the class,
        # using the callback name to test
        if hasattr(self.__class_instance, 'inverse_kinematics'):
            raise RuntimeWarning('inverse_kinematics already exists')
        # create the service
        self.__inverse_kinematics_service = self.__ral.service_client(
            '/inverse_kinematics',
            crtk_msgs.srv.QueryInverseKinematics)
        # add attributes to class instance
        self.__class_instance.inverse_kinematics = self.__inverse_kinematics
