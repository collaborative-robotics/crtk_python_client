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

        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 200    # aiming for 200 Hz
        self.samples = self.duration * self.rate

    def run(self):
        if not self.enable(60):
            print("Unable to enable the device, make sure it is connected.")
            return

        # create a new goal with constant speed
        sleep_rate = self.ral.rate(self.rate)
        for i in range(self.samples):
            vel = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0]) # move 5 cm/sec along x direction
            self.servo_cv(vel)
            sleep_rate.sleep()

        # send the zero velcity to stop the robot
        vel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) 
        self.servo_cv(vel)


def main():
    if (len(sys.argv) != 2):
        print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
        return
    
    example_name = type(crtk_servo_cv_example).__name__
    device_namespace = sys.argv[1]
    ral = crtk.ral(example_name, device_namespace)
    example = crtk_servo_cv_example(ral)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
