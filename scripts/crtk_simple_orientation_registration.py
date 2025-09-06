#! /usr/bin/env python3

# Author: Anton Deguet
# Created on: 2025-09-04
#
# Copyright (c) 2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import argparse
import sys
import time
import crtk
import PyKDL
import math

class input_device:
    def __init__(self, ral):
        self.ral = ral

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_measured_cp()

# extract ros arguments (e.g. __ns:= for namespace)
argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-d', '--device', type = str, required = True,
                    help = 'name of the CRTK compatible device.  The device must provide measured_cp')

args = parser.parse_args(argv)

ral = crtk.ral('crtk_simple_orientation_registration')
device = input_device(ral.create_child(args.device))
ral.spin()

print('Checking topics')
time.sleep(1)
ral.check_connections()
print('> All good\n')

print('Starting with right to left, place your device to the right')
input('-> Press "Enter" to collect x1')
x1, ts = device.measured_cp()
print(f'   x1 = {x1.p}\n')
print('Move to the left, keep other directions as constant as possible')
input('-> Press "Enter" to collect x2')
x2, ts = device.measured_cp()
print(f'   x2 = {x2.p}\n')

print('For bottom to top, place your device at the bottom')
input('-> Press "Enter" to collect y1')
y1, ts = device.measured_cp()
print(f'   y1 = {y1.p}\n')
print('Move up, keep other directions as constant as possible')
input('-> Press "Enter" to collect y2)')
y2, ts = device.measured_cp()
print(f'   y2 = {y2.p}\n')

print('For front to back, place your device close to you')
input('-> Press "Enter" to collect z1')
z1, ts = device.measured_cp()
print(f'   z1 = {z1.p}\n')
print('Move away, keep other directions as constant as possible')
input('-> Press "Enter" to collect z2)')
z2, ts = device.measured_cp()
print(f'   z2 = {z2.p}\n')

# lets create a nasty orientation matrix
x = x2.p - x1.p
x.Normalize()

y = y2.p - y1.p
y.Normalize()

z = z2.p - z1.p
z.Normalize()

bad_rotation = PyKDL.Rotation(x, y, z)
q = bad_rotation.GetQuaternion()
qx = q[1]; qy = q[2]; qz = q[3]; qw = q[0]
q_norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
print(f'-> Norm of quaternion found {q_norm}\n')
good_rotation = PyKDL.Rotation.Quaternion(qx, qy, qz, qw)

print('-> Rotation:\n')
print(good_rotation)

print('-> Inverse:\n')
print(good_rotation.Inverse())

ral.shutdown()
