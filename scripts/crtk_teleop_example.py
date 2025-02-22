#!/usr/bin/env python

# Author: Anton Deguet
# Date: 2018-09-29
#
# Copyright (c) 2018-2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start your crtk compatible device first!
# dVRK example:
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# Phantom Omni example:
# > rosrun sensable_phantom_ros sensable_phantom -j sawSensablePhantomRight.json

# To communicate with the device using ROS topics, see the python based example:
# > rosrun crtk_python_client crtk_teleop_example <master device namespace> <puppet device namespace>

import argparse
import crtk
import math
import PyKDL
import sys
import time

# example of application using device.py
class crtk_teleop_example:

    class Master:
        def __init__(self, ral):
            # populate master with the ROS topics we need
            self.crtk = crtk.utils(self, ral)
            self.crtk.add_measured_cp()

    class Puppet:
        def __init__(self, ral):
            # populate puppet with the ROS topics we need
            self.crtk = crtk.utils(self, ral)
            self.crtk.add_setpoint_cp()
            self.crtk.add_servo_cp()

    class Gripper:
        def __init__(self, ral):
            self.crtk = crtk.utils(self, ral)
            self.crtk.add_measured_js()

    class Jaw:
        def __init__(self, ral):
            self.crtk = crtk.utils(self, ral)
            self.crtk.add_setpoint_js()
            self.crtk.add_servo_jp()

    def __init__(self, ral, master_namespace, puppet_namespace, gripper_namespace, jaw_namespace):
        self.master = self.Master(ral.create_child(master_namespace))
        self.puppet = self.Puppet(ral.create_child(puppet_namespace))
        self.has_gripper = False

        if ((gripper_namespace != '') and (jaw_namespace != '')):
            self.has_gripper = True
            self.gripper = self.Gripper(ral.create_child(gripper_namespace))
            self.jaw = self.Jaw(ral.create_child(jaw_namespace))

        self.duration = 10 # seconds
        self.rate = 500    # aiming for 500 Hz
        self.sleep_rate = ral.create_rate(self.rate)
        self.samples = self.duration * self.rate

    # main loop
    def run(self):
        self.running = True
        while (self.running):
            print ('\n- q: quit\n- p: print position, velocity\n- t: position based teleop (10s)')
            answer = input('Enter your choice and [enter] to continue\n')
            if answer == 'q':
                self.running = False
            elif answer == 'p':
                self.run_print()
            elif answer == 't':
                self.run_teleop()
            else:
                print('Invalid choice\n')

    # print positions
    def run_print(self):
        print('master')
        m_cp, _ = self.master.measured_cp()
        print(m_cp.p)
        print('puppet')
        p_cp, _ = self.puppet.setpoint_cp()
        print(p_cp.p)

    # position based teleop
    def run_teleop(self):

        # registration between master and puppet.  Ideally we don't
        # use this, the devices should be able to set a base frame
        registration_rotation = PyKDL.Frame()
        # rotation from master Omni to puppet Falcon
        # registration_rotation.M.DoRotX(math.pi / 2.0)
        # registration_rotation.M.DoRotY(math.pi / 2.0)

        # rotation from master Omni to puppet PSM
        registration_rotation.M.DoRotZ(math.pi)
        registration_rotation.M.DoRotX(math.pi / 2.0)

        # scale (should be using ros::param)
        scale = 0.1
        # record where we started, only positions
        m_cp, _ = self.master.measured_cp()
        start_master = PyKDL.Frame(registration_rotation.Inverse() * m_cp)
        start_puppet, _ = self.puppet.setpoint_cp()

        # create the target goal for the puppet, use current orientation
        goal_puppet = PyKDL.Frame()

        # start only when gripper is close to jaw angle
        gripper_started = False

        # loop
        for i in range(self.samples):
            # current master in puppet orientation
            m_cp, _ = self.master.measured_cp()
            current_master = PyKDL.Frame(registration_rotation.Inverse() * m_cp)
            goal_puppet.p = start_puppet.p + scale * (current_master.p - start_master.p)
            goal_puppet.M = current_master.M * start_master.M.Inverse() * start_puppet.M # this is not working yet!
            self.puppet.servo_cp(goal_puppet)
            # gripper
            if (self.has_gripper):
                if (gripper_started):
                    g_jp, _ = self.gripper.measured_jp()
                    self.jaw.servo_jp(g_jp)
                else:
                    g_jp, _ = self.gripper.measured_jp()
                    j_jp, _ = self.jaw.setpoint_jp()
                    if (abs(g_jp[0] - j_jp()[0]) < math.radians(5.0)):
                        gripper_started = True

            self.sleep_rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('master', type = str, help = 'ROS namespace for master CRTK device')
    parser.add_argument('puppet', type = str, help = 'ROS namespace for puppet CRTK device')
    parser.add_argument('-g', '--gripper', type = str, default = '', help = 'absolute ROS namespace for (optional) master gripper')
    parser.add_argument('-j', '--jaw', type = str, default = '', help = 'absolute ROS namespace for (optional) puppet jaw')
    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args)

    example_name = type(crtk_teleop_example).__name__
    ral = crtk.ral(example_name)

    example = crtk_teleop_example(ral, args.master, args.puppet, args.gripper, args.jaw)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
