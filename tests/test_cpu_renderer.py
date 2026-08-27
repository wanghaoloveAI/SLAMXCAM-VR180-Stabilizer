from __future__ import annotations

from math import cos, radians, sin

import numpy as np

from slam_stabilizer.core.quaternion import Quat
from slam_stabilizer.cpu_renderer import (
    _build_eye_maps,
    _render_sbs_frame,
    build_cfr_frame_slots,
)


def _synthetic_sbs(eye_size: int = 96) -> np.ndarray:
    yy, xx = np.indices((eye_size, eye_size), dtype=np.uint8)
    eye = np.stack(
        [
            xx,
            yy,
            np.bitwise_xor(xx, yy),
        ],
        axis=-1,
    )
    return np.concatenate([eye, eye], axis=1)


def test_non_identity_correction_changes_rendered_pixels() -> None:
    source = _synthetic_sbs()
    identity = np.eye(3, dtype=np.float32)
    roll = Quat(cos(radians(8.0)), 0.0, 0.0, sin(radians(8.0)))

    uncorrected = _render_sbs_frame(source, identity)
    corrected = _render_sbs_frame(source, np.array(roll.to_matrix3(), dtype=np.float32))

    changed = np.count_nonzero(np.any(uncorrected != corrected, axis=2))
    assert changed > source.shape[0] * source.shape[1] * 0.25


def test_per_row_rolling_shutter_changes_rendered_pixels() -> None:
    source = _synthetic_sbs()
    eye_size = source.shape[0]
    identity = np.eye(3, dtype=np.float32)
    row_matrices = np.empty((eye_size, 3, 3), dtype=np.float32)
    for row in range(eye_size):
        angle = radians(-2.0 + 4.0 * row / (eye_size - 1))
        row_matrices[row] = np.array(
            [
                [cos(angle), -sin(angle), 0.0],
                [sin(angle), cos(angle), 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    without_rs = _render_sbs_frame(source, identity)
    with_rs = _render_sbs_frame(source, identity, rolling_shutter_matrices=row_matrices)

    changed = np.count_nonzero(np.any(without_rs != with_rs, axis=2))
    assert changed > source.shape[0] * source.shape[1] * 0.1


def test_second_rolling_shutter_iteration_refines_source_map() -> None:
    eye_size = 96
    identity = np.eye(3, dtype=np.float32)
    row_matrices = np.empty((eye_size, 3, 3), dtype=np.float32)
    for row in range(eye_size):
        angle = radians(-8.0 + 16.0 * row / (eye_size - 1))
        row_matrices[row] = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, cos(angle), -sin(angle)],
                [0.0, sin(angle), cos(angle)],
            ],
            dtype=np.float32,
        )

    one_x, one_y, _ = _build_eye_maps(
        eye_size,
        identity,
        rolling_shutter_matrices=row_matrices,
        rolling_shutter_iterations=1,
    )
    two_x, two_y, _ = _build_eye_maps(
        eye_size,
        identity,
        rolling_shutter_matrices=row_matrices,
        rolling_shutter_iterations=2,
    )

    refined_pixels = np.count_nonzero(
        (np.abs(one_x - two_x) > 1e-4) | (np.abs(one_y - two_y) > 1e-4)
    )
    assert refined_pixels > eye_size * eye_size * 0.01


def test_cfr_slots_preserve_a_dropped_capture_interval() -> None:
    frame_rate, slots = build_cfr_frame_slots(
        [0.0, 1.0 / 30.0, 2.0 / 30.0, 4.0 / 30.0, 5.0 / 30.0],
        fallback_frame_rate=29.0,
    )

    assert abs(frame_rate - 30.0) < 1e-6
    assert slots == [0, 1, 2, 4, 5]
