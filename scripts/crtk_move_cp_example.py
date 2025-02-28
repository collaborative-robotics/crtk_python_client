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
# > ros2 run crtk_python_client crtk_move_cp_example.py <arm-name>

import argparse
import crtk
import PyKDL
import sys


class crtk_move_cp_example:
    def __init__(self, ral):
        self.ral = ral

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state() # so we can wait until move is complete
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_move_cp()

    def run(self):
        self.ral.check_connections()

        # create a new goal starting with current position
        start_cp = PyKDL.Frame()
        setpoint, timestamp = self.setpoint_cp(wait_timeout=1.0)
        start_cp.p = setpoint.p
        start_cp.M = setpoint.M

        goal = PyKDL.Frame()
        goal.p = setpoint.p
        goal.M = setpoint.M

        amplitude = 0.02 # 2 centimeters

        # first move - increase x, y by amplitude
        goal.p[0] = start_cp.p[0] + amplitude
        goal.p[1] = start_cp.p[1] + amplitude
        goal.p[2] = start_cp.p[2]
        handle = self.move_cp(goal)
        handle.wait()

        # second move
        goal.p[0] = start_cp.p[0] - amplitude
        goal.p[1] = start_cp.p[1] - amplitude
        self.move_cp(goal).wait()

        # back to starting point
        goal.p[0] = start_cp.p[0]
        goal.p[1] = start_cp.p[1]
        self.move_cp(goal).wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('namespace', type = str, help = 'ROS namespace for CRTK device')
    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args) 

    example_name = type(crtk_move_cp_example).__name__
    ral = crtk.ral(example_name, args.namespace)
    example = crtk_move_cp_example(ral)
    ral.spin_and_execute(example.run)


if __name__ == '__main__':
    main()
