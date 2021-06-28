#!/usr/bin/env python

# Author: Anton Deguet
# Created on: 2015-02-22
#
# Copyright (c) 2015-2021 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

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
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_servo_cp()
        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 200    # aiming for 200 Hz
        self.samples = self.duration * self.rate

    def run_servo_cp(self):
        if not self.enable(60):
            print("Unable to enable the device, make sure it is connected.")
            return

        # create a new goal starting with current position
        start= PyKDL.Frame()
        start.p = self.setpoint_cp().p
        start.M = self.setpoint_cp().M
        goal = PyKDL.Frame()
        goal.p = self.setpoint_cp().p
        goal.M = self.setpoint_cp().M
        amplitude = 0.01 # 2 centimeters
        for i in range(self.samples):
            goal.p[0] =  start.p[0] + amplitude * (1.0 - math.cos(i * math.radians(360.0) / self.samples))
            goal.p[1] =  start.p[1] + amplitude * (1.0 - math.cos(i * math.radians(360.0) / self.samples))
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
