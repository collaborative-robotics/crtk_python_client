#! /usr/bin/env python3

# Author: Anton Deguet
# Created on: 2026-05-31
#
# Copyright (c) 2015-2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start a single arm using
# > ros2 run dvrk_robot dvrk_console_json -j <console-file>

# Make sure to enable/home the robot if needed

# Interactive jog script: press x, y, or z to move the robot in the
# corresponding positive direction by the specified amplitude, then
# return to the original position.  Press q to quit.

# To communicate with the arm using ROS topics:
# > ros2 run crtk_python_client crtk_xyz.py <arm-name>

import argparse
import crtk
import PyKDL
import sys
import termios
import tty

class crtk_xyz:
    def __init__(self, ral, amplitude):
        self.ral = ral
        self.amplitude = amplitude

        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_operating_state()  # so we can wait until move is complete
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_move_cp()

    def _get_key_nonblocking(self, fd, old_settings):
        """Read a single character from stdin without blocking (returns '' if none)."""
        import select
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            return sys.stdin.read(1)
        return ''

    def _jog(self, start_cp, axis):
        """Move along the given axis (0=x, 1=y, 2=z) by amplitude, then return."""
        goal = PyKDL.Frame()
        goal.p = PyKDL.Vector(start_cp.p[0], start_cp.p[1], start_cp.p[2])
        goal.M = start_cp.M

        # move forward
        goal.p[axis] = start_cp.p[axis] + self.amplitude
        self.move_cp(goal).wait()

        # return to start
        goal.p[axis] = start_cp.p[axis]
        self.move_cp(goal).wait()

    def run(self):
        self.ral.check_connections()

        # capture current position as the home position
        setpoint, timestamp = self.setpoint_cp(wait_timeout=1.0)
        start_cp = PyKDL.Frame()
        start_cp.p = setpoint.p
        start_cp.M = setpoint.M

        axis_names = {
            'x': (0, '+X'),
            'y': (1, '+Y'),
            'z': (2, '+Z'),
        }

        print(f'  Amplitude: {self.amplitude * 1000:.1f} mm')
        print('  Keys:')
        print('    x  - jog in +X direction')
        print('    y  - jog in +Y direction')
        print('    z  - jog in +Z direction')
        print('    q  - quit')

        # save terminal settings and switch to raw / non-canonical mode
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)

            while True:
                key = self._get_key_nonblocking(fd, old_settings)

                if key == 'q':
                    print('\r\nQuitting.\r\n')
                    break

                if key in axis_names:
                    axis_idx, label = axis_names[key]
                    # restore normal terminal output for the duration of the move
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    print(f'Moving in {label} direction by {self.amplitude * 1000:.1f} mm ...')
                    self._jog(start_cp, axis_idx)
                    print('Returned to start position. Waiting for key press ...')
                    tty.setraw(fd)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    parser = argparse.ArgumentParser(
        description='Interactive XYZ jog: press x/y/z to move, q to quit.'
    )
    parser.add_argument('namespace', type=str,
                        help='ROS namespace for CRTK device')
    parser.add_argument('--amplitude', type=float, default=0.010,
                        help='jog amplitude in meters (default: 0.010 = 10 mm)')
    app_args = crtk.ral.parse_argv(sys.argv[1:])  # process and remove ROS args
    args = parser.parse_args(app_args)

    example_name = type(crtk_xyz).__name__
    ral = crtk.ral(example_name, args.namespace)
    example = crtk_xyz(ral, args.amplitude)
    ral.spin_and_execute(example.run)


if __name__ == '__main__':
    main()
