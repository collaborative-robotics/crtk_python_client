#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2015-02-22

# (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file> -c crtk_alpha

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun crtk_python_client crtk_arm_test.py <arm-name>

import crtk
import math
import sys
import rospy
import numpy
import PyKDL


# example of application using device.py
class crtk_servo_cp_example:

    # configuration
    def configure(self, device_namespace):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('crtk_servo_cp_example', anonymous = True, log_level = rospy.WARN)

        print(rospy.get_caller_id() + ' -> configuring crtk_device_test for: ' + device_namespace)
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, device_namespace)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_servo_cp()
        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 500    # aiming for 200 Hz
        self.samples = self.duration * self.rate

    def run_servo_cp(self):
        if not self.enable(60):
            print("Unable to enable the device, make sure it is connected.")
            return

        # create a new goal starting with current position
        start_cp = PyKDL.Frame()
        start_cp.p = self.measured_cp().p
        start_cp.M = self.measured_cp().M
        goal = PyKDL.Frame()
        goal.p = self.measured_cp().p
        goal.M = self.measured_cp().M
        amplitude = 0.01 # 2 centimeters
        for i in xrange(self.samples):
            goal.p[0] =  start_cp.p[0] + amplitude *  math.sin(i * math.radians(360.0) / self.samples)
            goal.p[1] =  start_cp.p[1] + amplitude *  math.sin(i * math.radians(360.0) / self.samples)
            self.servo_cp(goal)
            rospy.sleep(1.0 / self.rate)

# use the class now, i.e. main program
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
        else:
            example = crtk_servo_cp_example()
            example.configure(sys.argv[1])
            example.run_servo_cp()

    except rospy.ROSInterruptException:
        pass
