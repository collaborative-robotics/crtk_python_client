#! /usr/bin/env python3

# Author: Anton Deguet
# Created on: 2025-09-04
#
# Copyright (c) 2025 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

import argparse
import itertools
import json
import sys
import PyKDL
import math
import numpy as np


class MotionGuide:
    COLORS = ('#c53d43', '#25834d', '#2869b1')
    AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1))

    def __init__(self):
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            raise RuntimeError(str(error)) from error
        self.root.title('CRTK orientation registration')
        self.root.resizable(False, False)
        self.closed = False
        self.mode = 'translation'
        self.axis = 0
        self.phase = 0.0
        self.ready = tk.BooleanVar(self.root, False)
        panel = ttk.Frame(self.root, padding=16)
        panel.pack()
        self.title = ttk.Label(panel, font=('TkDefaultFont', 14, 'bold'))
        self.title.pack(anchor='w')
        self.instruction = ttk.Label(panel, wraplength=600)
        self.instruction.pack(anchor='w', pady=(8, 0))
        self.canvas = tk.Canvas(panel, width=620, height=350, background='white',
                                highlightthickness=0)
        self.canvas.pack(pady=12)
        ttk.Label(panel, text='Your viewpoint: +X left, +Y up, +Z away from you').pack()
        self.feedback = ttk.Label(panel, wraplength=600)
        self.feedback.pack(anchor='w', pady=8)
        self.capture = ttk.Button(panel, text='Capture pose', command=lambda: self.ready.set(True))
        self.capture.pack(anchor='e')
        self.root.bind('<Return>', lambda event: self.ready.set(True))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(40, self.animate)

    def close(self):
        self.closed = True
        self.ready.set(True)

    @staticmethod
    def project(point):
        x, y, z = point
        return (310 + 110 * (-x + 0.55 * z), 210 - 110 * (y + 0.35 * z))

    @classmethod
    def rotation_point(cls, axis, angle):
        # The plane basis satisfies u cross v = positive rotation axis.
        u = cls.AXES[(axis + 1) % 3]
        v = cls.AXES[(axis + 2) % 3]
        return tuple(math.cos(angle) * a + math.sin(angle) * b for a, b in zip(u, v))

    def draw(self):
        canvas = self.canvas
        canvas.delete('all')
        origin = self.project((0, 0, 0))
        for index, axis in enumerate(self.AXES):
            tip = self.project(tuple(1.7 * value for value in axis))
            canvas.create_line(*origin, *tip, fill=self.COLORS[index], width=2, arrow='last')
            canvas.create_text(tip[0], tip[1] - 14, text=(' +X left', '+Y up', '+Z away')[index],
                               fill=self.COLORS[index])
        color = self.COLORS[self.axis]
        if self.mode == 'translation':
            direction = self.AXES[self.axis]
            start = self.project(tuple(-value for value in direction))
            end = self.project(direction)
            canvas.create_line(*start, *end, fill=color, width=7, arrow='last')
            point = self.project(tuple((-1 + 2 * self.phase) * value for value in direction))
            canvas.create_oval(point[0] - 6, point[1] - 6, point[0] + 6, point[1] + 6,
                               fill=color, outline='white')
        else:
            points = [self.project(self.rotation_point(self.axis, math.radians(angle)))
                      for angle in range(0, 301, 5)]
            canvas.create_line(*[coordinate for point in points for coordinate in point],
                               fill=color, width=4, arrow='last')
            point = self.project(self.rotation_point(self.axis, self.phase * 2 * math.pi))
            canvas.create_line(*origin, *point, fill='#333333', width=5)
            canvas.create_oval(point[0] - 6, point[1] - 6, point[0] + 6, point[1] + 6,
                               fill=color, outline='white')

    def animate(self):
        if not self.closed:
            self.phase = (self.phase + 0.012) % 1.0
            self.draw()
            self.root.after(40, self.animate)

    def prompt(self, title, instruction, axis, mode='translation', button='Capture pose'):
        self.title.configure(text=title)
        self.instruction.configure(text=instruction)
        self.axis, self.mode, self.phase = axis, mode, 0.0
        self.capture.configure(text=button)
        self.ready.set(False)
        self.draw()
        self.root.wait_variable(self.ready)
        if self.closed:
            raise SystemExit(0)

    def report(self, text):
        self.feedback.configure(text=text)
        self.root.update_idletasks()

    def show_results(self, report):
        from tkinter import ttk

        window = self.tk.Toplevel(self.root)
        window.title('Registration results')
        window.geometry('800x650')
        panel = ttk.Frame(window, padding=12)
        panel.pack(fill='both', expand=True)
        scrollbar = ttk.Scrollbar(panel)
        scrollbar.pack(side='right', fill='y')
        text = self.tk.Text(panel, wrap='none', font='TkFixedFont',
                            yscrollcommand=scrollbar.set)
        text.pack(fill='both', expand=True)
        scrollbar.configure(command=text.yview)
        text.insert('1.0', report)
        text.configure(state='disabled')
        ttk.Button(window, text='Finish', command=window.destroy).pack(pady=(0, 12))
        window.transient(self.root)
        window.grab_set()
        self.root.wait_window(window)


class InputDevice:
    def __init__(self, ral):
        import crtk
        self.crtk_utils = crtk.utils(self, ral)
        self.crtk_utils.add_measured_cp()


def unit_vector(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if vector.shape != (3,) or not np.isfinite(vector).all() or length < 1e-9:
        raise ValueError('An axis has zero length or invalid coordinates; repeat collection.')
    return vector / length


def axis_angle_degrees(first, second):
    return math.degrees(math.acos(float(np.clip(np.dot(unit_vector(first), unit_vector(second)), -1, 1))))


def rotation_angle_degrees(rotation):
    return math.degrees(math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1))))


def rotation_axis_diagnostics(axis_samples):
    """Describe raw rotation axes without applying frame acceptance thresholds."""
    samples = [[unit_vector(sample) for sample in axis] for axis in axis_samples]
    pairs = ((0, 1), (0, 2), (1, 2))

    def frame_metrics(axes):
        return {
            'pair_angles_XY_XZ_YZ_deg': [axis_angle_degrees(axes[a], axes[b]) for a, b in pairs],
            'handedness': float(np.linalg.det(np.column_stack(axes))),
        }

    means = [np.mean(axis, axis=0) for axis in samples]
    # Opposite repeat directions have no defined average axis; still report trials.
    average = (frame_metrics([unit_vector(axis) for axis in means])
               if all(np.linalg.norm(axis) >= 1e-9 for axis in means) else None)
    return {
        'axis_lengths_per_trial': [[float(np.linalg.norm(sample)) for sample in axis]
                                   for axis in axis_samples],
        'trials': [frame_metrics(axes) for axes in zip(*samples)],
        'normalized_average': average,
        'repeat_differences_X_Y_Z_deg': [
            max((axis_angle_degrees(a, b) for a, b in itertools.combinations(axis, 2)), default=0.0)
            for axis in samples
        ],
    }


def report_rotation_axis_diagnostics(axis_samples):
    diagnostics = rotation_axis_diagnostics(axis_samples)
    print('\nRotation axes: raw measurement quality (before tolerance checks)', flush=True)
    for name, lengths in zip('XYZ', diagnostics['axis_lengths_per_trial']):
        print(f'  {name} axis lengths: ' + ', '.join(f'{length:.6f}' for length in lengths))
    print(f'{"":12s} {"X-Y (deg)":>12s} {"X-Z (deg)":>12s} {"Y-Z (deg)":>12s} {"Handedness":>12s}')
    rows = [(f'Trial {index}', values) for index, values in enumerate(diagnostics['trials'], 1)]
    if diagnostics['normalized_average'] is not None:
        rows.append(('Averaged', diagnostics['normalized_average']))
    for label, values in rows:
        angles = values['pair_angles_XY_XZ_YZ_deg']
        print(f'{label:12s}' + ''.join(f'{angle:12.2f}' for angle in angles)
              + f'{values["handedness"]:12.4f}')
    if diagnostics['normalized_average'] is None:
        print('Averaged axes undefined: opposing trials cancel at least one axis.')
    print('  Repeat differences: ' + ', '.join(
        f'{name}={angle:.2f} deg' for name, angle in zip('XYZ', diagnostics['repeat_differences_X_Y_Z_deg'])))
    print('  Ideal: unit lengths, pair angles 90 deg, handedness +1, repeat differences 0 deg.\n',
          flush=True)
    return diagnostics


def fit_frame(axis_samples, tolerance=30.0):
    """Fit columns X/Y/Z expressed in the measured reference frame to SO(3)."""
    samples = [[unit_vector(sample) for sample in axis] for axis in axis_samples]
    if len(samples) != 3 or any(not axis for axis in samples):
        raise ValueError('Three nonempty axis sample sets are required.')
    means = [np.mean(axis, axis=0) for axis in samples]
    cancelled = [index for index, axis in enumerate(means) if np.linalg.norm(axis) < 1e-9]
    measured = np.column_stack([unit_vector(axis) if index not in cancelled else np.zeros(3)
                                for index, axis in enumerate(means)])
    handedness = float(np.linalg.det(measured))
    pair_angles = [axis_angle_degrees(measured[:, a], measured[:, b])
                   if a not in cancelled and b not in cancelled else None
                   for a, b in ((0, 1), (0, 2), (1, 2))]
    warnings = []
    if handedness <= 0.0:
        warnings.append(f'Raw axes are left-handed or degenerate (det={handedness:.4f}).')
    if any(angle is not None and abs(angle - 90) > tolerance for angle in pair_angles):
        warnings.append(f'Orthogonality error exceeds {tolerance:.1f} deg.')
    # Equal weight per physical axis; the polar factor is the nearest proper rotation.
    u, singular_values, vt = np.linalg.svd(measured)
    if singular_values[-1] < 1e-6:
        warnings.append('Axes do not span a unique frame; the fitted rotation is underdetermined.')
    if cancelled:
        warnings.append('Opposing samples cancel the mean for axes: '
                        + ', '.join('XYZ'[index] for index in cancelled) + '.')
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    fit_errors = [[axis_angle_degrees(sample, rotation[:, index]) for sample in axis]
                  for index, axis in enumerate(samples)]
    repeat_errors = [max((axis_angle_degrees(a, b) for a, b in itertools.combinations(axis, 2)),
                         default=0.0) for axis in samples]
    if max(error for axis in fit_errors for error in axis) > tolerance or max(repeat_errors) > tolerance:
        warnings.append(f'Fit or repeat error exceeds {tolerance:.1f} deg.')
    return rotation, {
        'normalized_measured_axes_columns': measured.tolist(),
        'pair_angles_XY_XZ_YZ_deg': pair_angles,
        'raw_handedness': handedness,
        'fit_errors_per_axis_deg': fit_errors,
        'repeat_errors_per_axis_deg': repeat_errors,
        'singular_values': singular_values.tolist(),
        'warning_threshold_deg': tolerance,
        'warnings': warnings,
    }


def nearest_axis_mapping(rotation):
    """Find the closest of the 24 proper signed axis permutations."""
    candidates = []
    for permutation in itertools.permutations(range(3)):
        for signs in itertools.product((-1, 1), repeat=3):
            candidate = np.eye(3)[:, permutation] @ np.diag(signs)
            if np.linalg.det(candidate) > 0:
                candidates.append(candidate)
    nearest = min(candidates, key=lambda candidate: rotation_angle_degrees(candidate.T @ rotation))
    labels = []
    for index, name in enumerate('XYZ'):
        row = int(np.argmax(np.abs(nearest[:, index])))
        sign = '+' if nearest[row, index] > 0 else '-'
        labels.append(f'orientation +{name} -> position {sign}{"XYZ"[row]}')
    return nearest, rotation_angle_degrees(nearest.T @ rotation), labels


def homogeneous(rotation):
    transform = np.eye(4)
    transform[:3, :3] = rotation
    return transform.tolist()


def compare_frames(position, orientation):
    # Both column sets describe the SAME user axes, in their respective references.
    correction = position @ orientation.T
    nearest, residual, labels = nearest_axis_mapping(correction)
    axis_disagreements = [axis_angle_degrees(position[:, index], orientation[:, index])
                          for index in range(3)]
    return {
        'position_from_orientation': homogeneous(correction),
        'angle_from_identity_deg': rotation_angle_degrees(correction),
        'rpy_deg': [math.degrees(value) for value in PyKDL.Rotation(*correction.ravel()).GetRPY()],
        'rpy_convention': 'C = Rz(yaw) @ Ry(pitch) @ Rx(roll)',
        'nearest_90_degree_mapping': homogeneous(nearest),
        'nearest_mapping_axis_directions': labels,
        'nearest_mapping_residual_deg': residual,
        'axis_disagreements_before_correction_deg': axis_disagreements,
    }


def kdl_vector(vector):
    return np.array([vector[index] for index in range(3)])


def format_results(result):
    lines = []
    for title, key in (('Frame from translations (P)', 'position_frame'),
                       ('Frame from rotations (O)', 'orientation_frame'),
                       ('Rotation between frames: C = P @ O.T', 'position_from_orientation')):
        lines.append(title)
        lines.extend('  ' + ' '.join(f'{value: .6f}' for value in row[:3])
                     for row in result[key][:3])
        lines.append('')
    lines.extend([
        'Mapping direction: v_position = C @ v_orientation',
        f'Angle from identity: {result["angle_from_identity_deg"]:.2f} deg',
        'Roll / pitch / yaw: ' + ' / '.join(f'{value:.2f}' for value in result['rpy_deg']) + ' deg',
        result['rpy_convention'],
        f'Nearest 90-degree mapping residual: {result["nearest_mapping_residual_deg"]:.2f} deg',
        *result['nearest_mapping_axis_directions'],
    ])
    for title, key in (('Translation quality', 'position_quality'), ('Rotation quality', 'orientation_quality')):
        quality = result[key]
        pairs = ', '.join(f'{value:.2f}' if value is not None else 'undefined'
                          for value in quality['pair_angles_XY_XZ_YZ_deg'])
        lines.extend(['', title, f'  XY / XZ / YZ angles: {pairs} deg',
                      f'  Raw handedness: {quality["raw_handedness"]:.4f}'])
        for name, errors, repeat in zip('XYZ', quality['fit_errors_per_axis_deg'],
                                        quality['repeat_errors_per_axis_deg']):
            lines.append(f'  {name}: fit errors ' + ', '.join(f'{error:.2f}' for error in errors)
                         + f' deg; repeat difference {repeat:.2f} deg')
        lines.extend('  Warning: ' + warning for warning in quality['warnings'])
    lines.extend(['', 'Thresholds flag quality only; no measurements were rejected.',
                  'Origin offsets were not measured. Device calibration was not changed.'])
    return '\n'.join(lines)


def collect_pose(device, guide, title, instruction, axis, mode='translation'):
    guide.prompt(title, instruction, axis, mode)
    pose, _ = device.measured_cp()
    values = [pose.p[index] for index in range(3)]
    values.extend(pose.M[row, col] for row in range(3) for col in range(3))
    if not np.isfinite(values).all():
        raise ValueError('Received a non-finite pose.')
    return pose


def collect_translations(device, guide):
    samples = []
    starts = ('to your RIGHT', 'LOW', 'NEAR YOU')
    directions = ('LEFT', 'UP', 'AWAY FROM YOU')
    for index, name in enumerate('XYZ'):
        while True:
            start = collect_pose(device, guide, f'Translation +{name}: start',
                                 f'Place the stylus {starts[index]} and hold still.', index)
            end = collect_pose(device, guide, f'Translation +{name}: end',
                               f'Move {directions[index]} along the arrow, then hold still. '
                               'Keep the other directions constant.', index)
            delta = kdl_vector(end.p - start.p)
            distance = np.linalg.norm(delta)
            if distance >= 0.01:
                break
            guide.report(f'Please retry: translation was {1000 * distance:.1f} mm; use at least 10 mm.')
        samples.append([delta])
        print(f'Translation +{name}: {1000 * distance:.1f} mm, direction {unit_vector(delta)}', flush=True)
    return samples


def collect_rotations(device, guide):
    samples = []
    for index, name in enumerate('XYZ'):
        trials = []
        for trial in (1, 2):
            while True:
                start = collect_pose(device, guide, f'Rotation +{name}: trial {trial}, start',
                                     'Choose a comfortable wrist pose; for trial 2, change it substantially. '
                                     'The arc shows the upcoming turn about the fixed physical axis.',
                                     index, 'rotation')
                end = collect_pose(device, guide, f'Rotation +{name}: trial {trial}, end',
                                   f'Turn 30 to 60 degrees about fixed +{name}, following the arc. '
                                   'Point your right thumb along the positive axis; curl your fingers '
                                   'in the turn direction. Keep position approximately fixed.', index, 'rotation')
                # Spatial increment, expressed in the orientation reference rather than the body frame.
                angle, axis = (end.M * start.M.Inverse()).GetRotAngle()
                degrees = math.degrees(angle)
                if 15 <= degrees <= 120:
                    break
                guide.report(f'Please retry: measured {degrees:.1f} deg; turn 30 to 60 deg.')
            trials.append(kdl_vector(axis))
            movement = 1000 * (end.p - start.p).Norm()
            guide.report(f'+{name} trial {trial}: {degrees:.1f} deg turn, {movement:.1f} mm position change.')
            print(f'Rotation +{name}, trial {trial}: {degrees:.1f} deg, axis {trials[-1]}, '
                  f'position change {movement:.1f} mm', flush=True)
        samples.append(trials)
    return samples


def run_registration(device, guide, reference_frame, tolerance):
    guide.prompt('Position and orientation registration',
                 'Disable teleoperation. First translate along three fixed axes, then rotate '
                 'about those same axes. No neutral stylus orientation is required.', 0, button='Begin')
    position, position_quality = fit_frame(collect_translations(device, guide), tolerance)
    print('Position frame quality:\n' + json.dumps(position_quality, indent=4), flush=True)
    guide.report('Position frame fitted. Next: wrist rotations about the same fixed axes.')
    rotation_samples = collect_rotations(device, guide)
    rotation_diagnostics = report_rotation_axis_diagnostics(rotation_samples)
    orientation, orientation_quality = fit_frame(rotation_samples, tolerance)
    comparison = compare_frames(position, orientation)
    result = {
        'convention': 'Frame columns are physical user +X left, +Y up, +Z away, expressed '
                      'in each measured reference. position_from_orientation maps orientation '
                      'axis vectors into the position reference: v_position = C @ v_orientation. '
                      'Zero translation is a placeholder; origin offsets were not measured.',
        'position_frame': homogeneous(position),
        'position_quality': position_quality,
        'orientation_frame': homogeneous(orientation),
        'orientation_quality': orientation_quality,
        'rotation_axis_diagnostics': rotation_diagnostics,
        'base_frame_from_translation': {
            'reference_frame': reference_frame, 'transform': homogeneous(position.T),
        },
        'orientation_to_user': homogeneous(orientation.T),
        **comparison,
    }
    print('\nRegistration results:\n' + json.dumps(result, indent=4), flush=True)
    mismatch = comparison['angle_from_identity_deg']
    residual = comparison['nearest_mapping_residual_deg']
    summary = (f'Position/orientation difference: {mismatch:.2f} deg. '
               f'Nearest 90-degree mapping residual: {residual:.2f} deg.\n'
               + '\n'.join(comparison['nearest_mapping_axis_directions']))
    guide.report(summary)
    report = format_results(result)
    print('\n' + report, flush=True)
    guide.show_results(report)
    return result


def main():
    parser = argparse.ArgumentParser(description='GUI registration from translations and wrist rotations.')
    parser.add_argument('-d', '--device', required=True, help='CRTK device providing measured_cp')
    parser.add_argument('-r', '--reference-frame', default='user')
    parser.add_argument('--axis-tolerance', type=float, default=30.0,
                        help='warning threshold for orthogonality, fit and repeat errors (default: 30 degrees)')
    import crtk
    args = parser.parse_args(crtk.ral.parse_argv(sys.argv[1:]))
    if not math.isfinite(args.axis_tolerance) or not 0 < args.axis_tolerance < 90:
        parser.error('--axis-tolerance must be finite and between 0 and 90 degrees')
    ral = crtk.ral('crtk_orientation_registration_gui')
    guide = None
    try:
        device = InputDevice(ral.create_child(args.device))
        ral.spin()
        ral.check_connections()
        guide = MotionGuide()
        try:
            run_registration(device, guide, args.reference_frame, args.axis_tolerance)
        except ValueError as error:
            print(f'Registration failed: {error}', file=sys.stderr, flush=True)
            guide.report(str(error))
            guide.prompt('Registration failed', 'Repeat collection with the indicated axes corrected.',
                         0, button='Close')
            return 1
    except (ImportError, RuntimeError) as error:
        print(f'Unable to start registration: {error}', file=sys.stderr)
        return 1
    finally:
        if guide is not None:
            guide.closed = True
            guide.root.destroy()
        ral.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
