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
# > rosrun crtk_python_client crtk_teleop_example <device-namespace>

import crtk
import math
import PyKDL
import rospy
import sys


if sys.version_info.major < 3:
    input = raw_input


# example of application using device.py
class crtk_teleop_example:

    class Puppeteer:
        def __init__(self, ral):
            # populate puppeteer with the ROS topics we need
            self.crtk = crtk.utils(self, ral)
            self.crtk.add_operating_state()
            self.crtk.add_measured_cp()

    class Puppet:
        def __init__(self, ral):
            # populate puppet with the ROS topics we need
            self.crtk = crtk.utils(self, ral)
            self.crtk.add_operating_state()
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

    def __init__(self, ral, puppeteer_namespace, puppet_namespace, gripper_namespace, jaw_namespace):
        self.puppeteer = self.Puppeteer(ral.create_child(puppeteer_namespace))
        self.puppet = self.Puppet(ral.create_child(puppet_namespace))
        self.has_gripper = False

        if ((gripper_namespace != '') and (jaw_namespace != '')):
            self.has_gripper = True
            self.gripper = self.Gripper(ral.create_child(gripper_namespace))
            self.jaw = self.Jaw(ral.create_child(jaw_namespace))

        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 500    # aiming for 200 Hz
        self.sleep_rate = ral.create_rate(self.rate)
        self.samples = self.duration * self.rate

    # main loop
    def run(self):
        if not self.puppeteer.enable(5.0):
            print("Unable to enable the puppeteer device, make sure it is connected.")
            return
        if not self.puppet.enable(5.0):
            print("Unable to enable the puppet device, make sure it is connected.")
            return

        self.running = True
        while (self.running):
            print ('\n- q: quit\n- p: print position, velocity\n- t: position based teleop (10s)')
            answer = input('Enter your choice and [enter] to continue\n')
            if answer == 'q':
                self.running = False
                self.puppeteer.disable()
                self.puppet.disable()
            elif answer == 'p':
                self.run_print()
            elif answer == 't':
                self.run_teleop()
            else:
                print('Invalid choice\n')

    # print positions
    def run_print(self):
        print('puppeteer')
        print(self.puppeteer.measured_cp().p)
        print('puppet')
        print(self.puppet.setpoint_cp().p)

    # position based teleop
    def run_teleop(self):

        # registration between puppeteer and puppet.  Ideally we don't
        # use this, the devices should be able to set a base frame
        registration_rotation = PyKDL.Frame()
        # rotation from puppeteer Omni to puppet Falcon
        # registration_rotation.M.DoRotX(math.pi / 2.0)
        # registration_rotation.M.DoRotY(math.pi / 2.0)

        # rotation from puppeteer Omni to puppet PSM
        registration_rotation.M.DoRotZ(math.pi)
        registration_rotation.M.DoRotX(math.pi / 2.0)

        # scale (should be using ros::param)
        scale = 0.5
        # record where we started, only positions
        start_puppeteer = PyKDL.Frame(registration_rotation.Inverse() * self.puppeteer.measured_cp())
        start_puppet = PyKDL.Frame(self.puppet.setpoint_cp())

        # create the target goal for the puppet, use current orientation
        goal_puppet = PyKDL.Frame()

        # start only when gripper is close to jaw angle
        gripper_started = False

        # loop
        for i in range(self.samples):
            # current puppeteer in puppet orientation
            current_puppeteer = PyKDL.Frame(registration_rotation.Inverse() * self.puppeteer.measured_cp())
            goal_puppet.p = start_puppet.p + scale * (current_puppeteer.p - start_puppeteer.p)
            goal_puppet.M = current_puppeteer.M * start_puppeteer.M.Inverse() * start_puppet.M # this is not working yet!
            self.puppet.servo_cp(goal_puppet)
            # gripper
            if (self.has_gripper):
                if (gripper_started):
                    self.jaw.servo_jp(self.gripper.measured_jp())
                else:
                    if (abs(self.gripper.measured_jp()[0] - self.jaw.setpoint_jp()[0]) < math.radians(5.0)):
                        gripper_started = True
            rospy.sleep(1.0 / self.rate)


def main():
    example_name = type(crtk_teleop_example).__name__
    ral = crtk.ral('crtk_teleop_example')

    if (len(sys.argv) == 3):
        puppeteer_namespace = sys.argv[1]
        puppet_namespace = sys.argv[2]
        gripper_namespace = ''
        jaw_namespace = ''
    elif (len(sys.argv) == 5):
        puppeteer_namespace = sys.argv[1]
        puppet_namespace = sys.argv[2]
        gripper_namespace = sys.argv[3]
        jaw_namespace = sys.argv[4]
    else:
        print(sys.argv[0], ' requires two or four arguments, i.e. puppeteer and puppet namespaces [puppeteer gripper and pupper jaw namespaces]')
        return
    
    example = crtk_teleop_example(ral, puppeteer_namespace, puppet_namespace, gripper_namespace, jaw_namespace)
    ral.spin_and_execute(example)

if __name__ == '__main__':
    main()
