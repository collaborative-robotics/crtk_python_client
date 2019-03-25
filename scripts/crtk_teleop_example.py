#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2018-09-29

# (C) Copyright 2019 Johns Hopkins University (JHU), All Rights Reserved.

# --- begin cisst license - do not edit ---

# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.

# --- end cisst license ---

# Start your crtk compatible device first!
# dVRK example:
# > rosrun dvrk_robot dvrk_console_json -j <console-file> -c crtk_alpha
# Phantom Omni example:
# > rosrun sensable_phantom_ros sensable_phantom -j sawSensablePhantomRight.json

# To communicate with the device using ROS topics, see the python based example:
# > rosrun crtk_python_client crtk_teleop_example <device-namespace>

import crtk
import math
import sys
import rospy
import numpy
import PyKDL

# example of application using device.py
class crtk_teleop_example:

    # populate master with the ROS topics we need
    class Master:
        def __init__(self, namespace):
            self.crtk = crtk.utils(self, namespace)
            self.crtk.add_operating_state()
            self.crtk.add_measured_cp()

    # populate puppet with the ROS topics we need
    class Puppet:
        def __init__(self, namespace):
            self.crtk = crtk.utils(self, namespace)
            self.crtk.add_operating_state()
            self.crtk.add_setpoint_cp()
            self.crtk.add_servo_cp()

    # configuration
    def configure(self, master_namespace, puppet_namespace):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('crtk_teleop_example', anonymous = True, log_level = rospy.WARN)

        self.master = self.Master(master_namespace)
        self.puppet = self.Puppet(puppet_namespace)

        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 500    # aiming for 200 Hz
        self.samples = self.duration * self.rate

    # main loop
    def run(self):
        if not self.master.enable(5.0):
            print("Unable to enable the master device, make sure it is connected.")
            return
        if not self.puppet.enable(5.0):
            print("Unable to enable the puppet device, make sure it is connected.")
            return

        self.running = True
        while (self.running):
            print ('\n- q: quit\n- p: print position, velocity\n- t: position based teleop (10s)')
            answer = raw_input('Enter your choice and [enter] to continue\n')
            if answer == 'q':
                self.running = False
                self.master.disable()
                self.puppet.disable()
            elif answer == 'p':
                self.run_print()
            elif answer == 't':
                self.run_teleop()
            else:
                print('Invalid choice\n')

    # print positions
    def run_print(self):
        print('master')
        print(self.master.measured_cp().p)
        print('puppet')
        print(self.puppet.setpoint_cp().p)

    # position based teleop
    def run_teleop(self):
        # save current position
        scale = 0.5
        # record where we started, only positions
        start_master = PyKDL.Frame()
        start_master.p = self.master.measured_cp().p
        start_puppet = PyKDL.Frame()
        start_puppet.p = self.puppet.setpoint_cp().p
        # create the target goal for the puppet, use current orientation
        goal_puppet = PyKDL.Frame()
        goal_puppet.M = self.puppet.setpoint_cp().M

        # this should be defined usingros::param
        rotation = PyKDL.Rotation()
        # rotation for master Omni, puppet Falcon
        rotation.DoRotX(math.pi / 2.0)
        rotation.DoRotY(math.pi / 2.0)
        # loop
        for i in xrange(self.samples):
            goal_puppet.p = start_puppet.p + scale * (rotation * (self.master.measured_cp().p - start_master.p))
            self.puppet.servo_cp(goal_puppet)
            rospy.sleep(1.0 / self.rate)

# use the class now, i.e. main program
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 3):
            print(sys.argv[0], ' requires two arguments, i.e. master and puppet namespaces')
        else:
            example = crtk_teleop_example()
            example.configure(sys.argv[1], sys.argv[2])
            example.run()

    except rospy.ROSInterruptException:
        pass
