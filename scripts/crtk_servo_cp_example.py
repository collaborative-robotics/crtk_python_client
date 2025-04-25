#! /usr/bin/env python3

# Author: Anton Deguet
# Created on: 2015-02-22
#
# Copyright (c) 2015-2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start a single arm using
# > ros2 run dvrk_robot dvrk_console_json -j <console-file>

# Make sure to enable/home the robot if needed

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > ros2 run crtk_python_client crtk_servo_cp_example.py <arm-name>

import argparse
import crtk
import math
import sys
import time

class crtk_servo_cp_example:
    def __init__(self, ral):
        self.ral = ral

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_servo_cp()

        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 200    # aiming for 200 Hz
        self.samples = self.duration * self.rate

    def run(self):
        self.ral.check_connections()
        start, ts = self.setpoint_cp(wait_timeout=1.0)
        while ts == 0:
            start, ts = self.setpoint_cp()
            time.sleep(0.01)
            print('waiting for data')

        goal, _ = self.setpoint_cp()
        amplitude = 0.02 # 2 centimeter total

        sleep_rate = self.ral.create_rate(self.rate)
        for i in range(self.samples):
            goal.p[0] =  start.p[0] + amplitude * (1.0 - math.cos(i * math.radians(360.0) / self.samples))
            goal.p[1] =  start.p[1] + amplitude * (1.0 - math.cos(i * math.radians(360.0) / self.samples))
            self.servo_cp(goal)
            sleep_rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('namespace', type = str, help = 'ROS namespace for CRTK device')
    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args)

    example_name = type(crtk_servo_cp_example).__name__
    ral = crtk.ral(example_name, args.namespace)
    example = crtk_servo_cp_example(ral)
    ral.spin_and_execute(example.run)


if __name__ == '__main__':
    main()
