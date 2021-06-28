#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2018-09-29
#
# Copyright (c) 2018-2021 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start your crtk compatible device first!
# dVRK example:
# > rosrun dvrk_robot dvrk_console_json -j <console-file>
# Phantom Omni example:
# > rosrun sensable_phantom_ros sensable_phantom -j sawSensablePhantomRight.json

# To communicate with the device using ROS topics, see the python based example:
# > rosrun crtk_python_client crtk_haptic_example <device-namespace>

import crtk
import math
import sys
import rospy
import numpy
import PyKDL

if sys.version_info.major < 3:
    input = raw_input

# example of application using device.py
class crtk_haptic_example:

    # configuration
    def configure(self, device_namespace):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('crtk_haptic_example', anonymous = True, log_level = rospy.WARN)

        print(rospy.get_caller_id() + ' -> configuring crtk_device_test for: ' + device_namespace)
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, device_namespace)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_measured_cv()
        self.crtk_utils.add_servo_cf()
        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 500    # aiming for 500 Hz
        self.samples = self.duration * self.rate

    # main loop
    def run(self):
        if not self.enable(60):
            print("Unable to enable the device, make sure it is connected.")
            return

        self.running = True
        while (self.running):
            print ('\n- q: quit\n- p: print position, velocity\n- b: virtual box around current position with linear forces (10s)\n- v: viscosity (10s)')
            answer = input('Enter your choice and [enter] to continue\n')
            if answer == 'q':
                self.running = False
                self.disable()
            elif answer == 'p':
                self.run_print()
            elif answer == 'b':
                self.run_box()
            elif answer == 'v':
                self.run_viscosity()
            else:
                print('Invalid choice\n')

    # print current position
    def run_print(self):
        print(self.measured_cp())
        print(self.measured_cv())

    # virtual box
    def run_box(self):
        # save current position
        dim = 0.01 # 2 cm cube
        p_gain = -500.0
        center = PyKDL.Frame()
        center.p = self.measured_cp().p
        for i in range(self.samples):
            wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # foreach d dimension x, y, z
            for d in range(3):
                distance = self.measured_cp().p[d] - center.p[d]
                if (distance > dim):
                    wrench[d] = p_gain * (distance - dim)
                elif  (distance < -dim):
                    wrench[d] = p_gain * (distance + dim)
            self.servo_cf(wrench)
            rospy.sleep(1.0 / self.rate)
        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_cf(wrench)

    # viscosity
    def run_viscosity(self):
        d_gain = -10.0
        for i in range(self.samples):
            wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # foreach d dimension x, y, z
            for d in range(3):
                wrench[d] = d_gain * self.measured_cv()[d]
            self.servo_cf(wrench)
            rospy.sleep(1.0 / self.rate)
        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_cf(wrench)

# use the class now, i.e. main program
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
        else:
            example = crtk_haptic_example()
            example.configure(sys.argv[1])
            example.run()

    except rospy.ROSInterruptException:
        pass
