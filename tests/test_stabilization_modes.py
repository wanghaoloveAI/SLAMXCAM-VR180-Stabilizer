from math import cos, radians, sin

import numpy as np
import pytest

from slam_stabilizer.core.quaternion import Quat
from slam_stabilizer.core.stabilization import SmoothParams, build_frame_stabilization
from slam_stabilizer.cpu_renderer import _render_sbs_frame
from slam_stabilizer.pipeline import StabilizationJob


def rotation(angle, axis):
    values = [cos(radians(angle) / 2), 0.0, 0.0, 0.0]
    values[axis + 1] = sin(radians(angle) / 2)
    return Quat.from_iter(values)


@pytest.mark.parametrize("fps", [30, 50])
def test_fast_roll_is_fully_cancelled_at_video_timestamps(fps):
    times = np.arange(201).tolist()
    times = [t / 200 for t in times]
    raw = [rotation(120 * sin(2 * np.pi * 3 * t), 2) for t in times]
    frames = build_frame_stabilization(times, raw, fps, fps,
                                      stabilization_mode="horizon-lock")
    for frame in frames:
        target = Quat.from_iter(frame.raw_wxyz).mul(Quat.from_iter(frame.correction_wxyz))
        assert target.angular_distance_deg(Quat.identity()) < 1e-5
    assert StabilizationJob.__dataclass_fields__["max_correction_velocity_deg_s"].default == 0


def test_fixed_orientation_and_stereo_reprojection_share_target():
    times = [i / 200 for i in range(201)]
    raw = [rotation(20 * sin(9 * t), 0).mul(rotation(30 * t, 1)).mul(
        rotation(15 * sin(13 * t), 2)) for t in times]
    frames = build_frame_stabilization(times, raw, 50, 50,
                                      stabilization_mode="orientation-lock")
    anchor = Quat.from_iter(frames[0].smooth_wxyz)
    eye = np.arange(48 * 48 * 3, dtype=np.uint8).reshape(48, 48, 3)
    source = np.concatenate([eye, eye], axis=1)
    for frame in frames:
        actual = Quat.from_iter(frame.raw_wxyz).mul(Quat.from_iter(frame.correction_wxyz))
        assert actual.angular_distance_deg(anchor) < 1e-5
    result = _render_sbs_frame(source, np.asarray(frames[25].correction_matrix3, dtype=np.float32))
    np.testing.assert_array_equal(result[:, :48], result[:, 48:])


def test_three_axis_smoothing_reduces_mixed_axis_jitter():
    times = [i / 200 for i in range(601)]
    raw = [rotation(4 * sin(40 * t), 0).mul(rotation(4 * sin(33 * t), 1)).mul(
        rotation(4 * sin(47 * t), 2)) for t in times]
    frames = build_frame_stabilization(times, raw, 90, 30,
                                      params=SmoothParams(max_correction_deg=30),
                                      stabilization_mode="normal")
    raw_path = [Quat.from_iter(f.raw_wxyz) for f in frames]
    target_path = [Quat.from_iter(f.smooth_wxyz) for f in frames]
    def travel(path):
        return sum(a.angular_distance_deg(b) for a, b in zip(path, path[1:]))
    assert travel(target_path) < travel(raw_path) * 0.25


def test_invalid_mode_does_not_silently_fall_back():
    with pytest.raises(ValueError, match="Unknown stabilization"):
        build_frame_stabilization([0, 1], [Quat.identity()] * 2, 30, 30,
                                  stabilization_mode="typo")
