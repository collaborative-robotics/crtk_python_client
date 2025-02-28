#! /usr/bin/env python3

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

# Make sure to enable/home the robot if needed

# To communicate with the device using ROS topics, see the python based example:
# > rosrun crtk_python_client crtk_teleop_example <master device namespace> <puppet device namespace>

# Note: for a dVRK teleop example, see dvrk_python/scripts/dvrk_teleoperation.py

import argparse
import crtk
import math
import PyKDL
import sys

class crtk_teleop_example:
    """Example of simple teleop using CRTK"""

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
        self.ral = ral
        self.master = self.Master(ral.create_child(master_namespace))
        self.puppet = self.Puppet(ral.create_child(puppet_namespace))
        self.has_gripper = False

        if ((gripper_namespace != '') and (jaw_namespace != '')):
            self.has_gripper = True
            self.gripper = self.Gripper(ral.create_child(gripper_namespace))
            self.jaw = self.Jaw(ral.create_child(jaw_namespace))

        self.rate = 500    # aiming for 500 Hz

    # position based teleop
    def run(self):
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
        m_cp, _ = self.master.measured_cp(wait_timeout=1.0)
        start_master = PyKDL.Frame(registration_rotation.Inverse() * m_cp)
        start_puppet, _ = self.puppet.setpoint_cp(wait_timeout=1.0)

        # create the target goal for the puppet, use current orientation
        goal_puppet = PyKDL.Frame()

        # start only when gripper is close to jaw angle
        gripper_started = False

        run_rate = self.ral.create_rate(self.rate)
        while not self.ral.is_shutdown():
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
                    if (abs(g_jp[0] - j_jp[0]) < math.radians(5.0)):
                        gripper_started = True

            run_rate.sleep()


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

    teleop = crtk_teleop_example(ral, args.master, args.puppet, args.gripper, args.jaw)
    ral.spin_and_execute(teleop.run)

if __name__ == '__main__':
    main()
