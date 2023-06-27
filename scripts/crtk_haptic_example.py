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
import PyKDL
import sys


if sys.version_info.major < 3:
    input = raw_input


class crtk_haptic_example:
    def __init__(self, ral):
        self.ral = ral

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_measured_cv()
        self.crtk_utils.add_servo_cf()

        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 500 # aiming for 500 Hz
        self.samples = self.duration * self.rate

    # main loop
    def run(self):
        if not self.enable(60):
            print('Unable to enable the device, make sure it is connected.')
            return

        self.running = True
        while (self.running):
            msg = ('\n'
                   '- q: quit\n'
                   '- p: print position, velocity\n'
                   '- b: virtual box around current position with linear forces ({}s)\n'
                   '- v: viscosity ({}s)\n'
            )
            print(msg.format(self.duration, self.duration))
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

        sleep_rate = self.ral.create_rate(self.rate)
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
            sleep_rate.sleep()

        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_cf(wrench)

    # viscosity
    def run_viscosity(self):
        d_gain = -10.0
        sleep_rate = self.ral.create_rate(self.rate)
        for i in range(self.samples):
            wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            # foreach d dimension x, y, z
            for d in range(3):
                wrench[d] = d_gain * self.measured_cv()[d]
            self.servo_cf(wrench)
            self.servo_cf(wrench)
            sleep_rate.sleep()

        wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.servo_cf(wrench)


def main():
    if (len(sys.argv) != 2):
        print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
        return
    
    example_name = type(crtk_haptic_example).__name__
    device_namespace = sys.argv[1]
    ral = crtk.ral(example_name, device_namespace)
    example = crtk_haptic_example(ral)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
