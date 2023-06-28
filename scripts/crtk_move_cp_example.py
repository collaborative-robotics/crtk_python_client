#!/usr/bin/env python

# Author: Anton Deguet
# Created on: 2015-02-22
#
# Copyright (c) 2015-2021 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun crtk_python_client crtk_move_cp_example.py <arm-name>

import crtk
import PyKDL
import sys


class crtk_move_cp_example:
    def __init__(self, ral):
         # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_move_cp()

    def run(self):
        if not self.enable(30):
            print("Unable to enable the device, make sure it is connected.")
            return

        if not self.home(30):
            print('Unable to home the device, make sure it is connected.')
            return

        # create a new goal starting with current position
        start_cp = PyKDL.Frame()
        start_cp.p = self.setpoint_cp().p
        start_cp.M = self.setpoint_cp().M
        goal = PyKDL.Frame()
        goal.p = self.setpoint_cp().p
        goal.M = self.setpoint_cp().M
        amplitude = 0.02 # 2 centimeters

        # first move
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
    if (len(sys.argv) != 2):
        print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
        return

    example_name = type(crtk_move_cp_example).__name__
    device_namespace = sys.argv[1]
    ral = crtk.ral(example_name, device_namespace)
    example = crtk_move_cp_example(ral)
    ral.spin_and_execute(example.run)


if __name__ == '__main__':
    main()
