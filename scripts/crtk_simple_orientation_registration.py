#! /usr/bin/env python3

# Author: Anton Deguet
# Created on: 2025-09-04
#
# Copyright (c) 2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import argparse
import json
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

def rotation_to_4x4(rotation_kdl):
    return [[rotation_kdl[0, 0], rotation_kdl[0, 1], rotation_kdl[0, 2], 0.0],
            [rotation_kdl[1, 0], rotation_kdl[1, 1], rotation_kdl[1, 2], 0.0],
            [rotation_kdl[2, 0], rotation_kdl[2, 1], rotation_kdl[2, 2], 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def angle_degrees_between(v1, v2):
    dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def print_json_block(name, value):
    print(f'-> {name}:\n')
    print(json.dumps(value, indent = 4))
    print('')


# extract ros arguments (e.g. __ns:= for namespace)
argv = crtk.ral.parse_argv(sys.argv[1:]) # skip argv[0], script name

# parse arguments
parser = argparse.ArgumentParser()
parser.add_argument('-d', '--device', type = str, required = True,
                    help = 'name of the CRTK compatible device.  The device must provide measured_cp')
parser.add_argument('-r', '--reference-frame', type = str, default = 'user',
                    help = 'reference frame name to use in the dVRK base_frame snippet')

args = parser.parse_args(argv)

ral = crtk.ral('crtk_simple_orientation_registration')
device = input_device(ral.create_child(args.device))
ral.spin()

print('Checking topics')
time.sleep(1)
ral.check_connections()
print('> All good\n')

print("This script assumes X is right to left, Y is low to high, Z is near to far from the user's perspective.\n")

print('Starting with right to left, place your device to the right')
input('-> Press "Enter" to collect x1')
x1, ts = device.measured_cp()
print(f'   x1 = {x1.p}\n')
print('Move to the left, keep other directions as constant as possible')
input('-> Press "Enter" to collect x2')
x2, ts = device.measured_cp()
print(f'   x2 = {x2.p}\n')

print('For low to high, place your device at the bottom')
input('-> Press "Enter" to collect y1')
y1, ts = device.measured_cp()
print(f'   y1 = {y1.p}\n')
print('Move up, keep other directions as constant as possible')
input('-> Press "Enter" to collect y2)')
y2, ts = device.measured_cp()
print(f'   y2 = {y2.p}\n')

print('For near to far, place your device close to you')
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

xy_angle = angle_degrees_between(x, y)
xz_angle = angle_degrees_between(x, z)
yz_angle = angle_degrees_between(y, z)

xCrossY = x * y  # PyKDL cross product
handedness = xCrossY[0] * z[0] + xCrossY[1] * z[1] + xCrossY[2] * z[2]  # (x×y)·z

print('-> Orthogonality metrics\n')
print(f'   angle(x, y) = {xy_angle:0.2f} deg, error to 90 = {abs(xy_angle - 90.0):0.2f} deg')
print(f'   angle(x, z) = {xz_angle:0.2f} deg, error to 90 = {abs(xz_angle - 90.0):0.2f} deg')
print(f'   angle(y, z) = {yz_angle:0.2f} deg, error to 90 = {abs(yz_angle - 90.0):0.2f} deg')
print(f'   handedness (x×y)·z = {handedness:+.4f} (should be close to +1.0 for a right-handed frame)\n')

bad_rotation = PyKDL.Rotation(x, y, z)
qx, qy, qz, qw = bad_rotation.GetQuaternion()  # returns (x, y, z, w)
q_norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
print(f'-> Norm of quaternion found {q_norm}\n')
good_rotation = PyKDL.Rotation.Quaternion(qx, qy, qz, qw)
base_frame_transform = rotation_to_4x4(good_rotation.Inverse())
base_frame_snippet = {
    'base_frame': {
        'reference_frame': args.reference_frame,
        'transform': base_frame_transform
    }
}

print_json_block('Rotation', rotation_to_4x4(good_rotation))
print_json_block('Inverse', base_frame_transform)
print_json_block('dVRK base_frame snippet', base_frame_snippet)

ral.shutdown()
