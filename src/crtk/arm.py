#  Author(s):  Anton Deguet
#  Created on: 2018-02-15

# (C) Copyright 2018 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

import crtk.utils
import rospy

class arm(object):

    # initialize the arm
    def __init__(self, arm_name, ros_namespace = '/dvrk/'):
        self.__init_arm(arm_name, ros_namespace)

    def __del__(self):
        print("del arm")
        del(self.__crtk_utils)

    def __init_arm(self, arm_name, ros_namespace = '/dvrk/'):
        self.__crtk_utils = crtk.utils(self, ros_namespace + arm_name)

        # add crtk features that we need and are supported by the dVRK
        self.__crtk_utils.add_device_state(self)
        self.__crtk_utils.add_setpoint_js(self)
        self.__crtk_utils.add_setpoint_cp(self)
        self.__crtk_utils.add_measured_js(self)
        self.__crtk_utils.add_measured_cp(self)
        self.__crtk_utils.add_measured_cv(self)
        self.__crtk_utils.add_measured_cf(self)
        self.__crtk_utils.add_servo_jp(self)
        self.__crtk_utils.add_servo_cp(self)

        if not rospy.get_node_uri():
            rospy.init_node('arm_api', anonymous = True, log_level = rospy.WARN)
        else:
            rospy.logdebug(rospy.get_caller_id() + ' -> ROS already initialized')


    def cleanup(self):
        self.__crtk_utils.remove_all()
