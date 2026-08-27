from __future__ import annotations

from math import cos, radians, sin

import numpy as np
import pytest

from slam_stabilizer.cpu_renderer import _render_sbs_frame
from slam_stabilizer.gpu_renderer import GpuRendererUnavailable, OpenGlSbsRenderer


def test_opengl_renderer_matches_cpu_reference() -> None:
    eye_size = 96
    yy, xx = np.indices((eye_size, eye_size), dtype=np.uint8)
    eye = np.stack([xx, yy, np.bitwise_xor(xx, yy)], axis=-1)
    source = np.concatenate([eye, eye], axis=1)
    angle = radians(4.0)
    correction = np.array(
        [
            [cos(angle), -sin(angle), 0.0],
            [sin(angle), cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float32,
    )
    row_matrices = np.repeat(np.eye(3, dtype=np.float32)[None, :, :], eye_size, axis=0)

    expected = _render_sbs_frame(
        source,
        correction,
        rolling_shutter_matrices=row_matrices,
    )
    try:
        renderer = OpenGlSbsRenderer(
            eye_size * 2,
            calibration=None,
            distortion_correction=True,
            field_of_view_deg=180.0,
        )
    except GpuRendererUnavailable as exc:
        pytest.skip(str(exc))

    try:
        actual = renderer.render(source, correction, row_matrices)
    finally:
        renderer.close()

    difference = np.abs(expected.astype(np.int16) - actual.astype(np.int16))
    assert np.max(difference) <= 1
