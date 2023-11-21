#!/usr/bin/env python

# Author: Hisashi Ishida, Anton Deguet
# Created on: 2023-04-01
#
# Copyright (c) 2015-2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun crtk_python_client crtk_servo_cv_example.py <arm-name>

import argparse
import crtk
import numpy as np
import sys


class crtk_servo_cv_example:
    def __init__(self, ral):
        self.ral = ral

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_servo_cv()

        self.duration = 10 # seconds

    def run(self):
        self.ral.check_connections()

        if not self.enable(30):
            print("Unable to enable the device, make sure it is connected.")
            return

        if not self.home(30):
            print('Unable to home the device, make sure it is connected.')
            return

        # create a new goal with constant speed
        vel = np.array([1e-3, 0.0, 0.0, 0.0, 0.0, 0.0]) # move 1 mm/sec along x direction
        self.servo_cv(vel)
        self.ral.create_rate(1/self.duration).sleep() # sleep once for 'duration' seconds

        # command zero velcity to stop the robot
        vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.servo_cv(vel)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('namespace', type = str, help = 'ROS namespace for CRTK device')
    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args) 

    example_name = type(crtk_servo_cv_example).__name__
    ral = crtk.ral(example_name, args.namespace)
    example = crtk_servo_cv_example(ral)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
