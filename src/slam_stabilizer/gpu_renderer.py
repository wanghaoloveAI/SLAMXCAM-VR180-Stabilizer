from __future__ import annotations

from array import array
from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Any

import numpy as np
from PySide6.QtCore import QSize
from PySide6.QtGui import (
    QGuiApplication,
    QImage,
    QOffscreenSurface,
    QOpenGLContext,
    QSurfaceFormat,
)
from PySide6.QtOpenGL import (
    QOpenGLFramebufferObject,
    QOpenGLFramebufferObjectFormat,
    QOpenGLShader,
    QOpenGLShaderProgram,
    QOpenGLTexture,
    QOpenGLVertexArrayObject,
)

from .core.quaternion import Quat
from .core.stabilization import FrameStabilization
from .cpu_renderer import CpuRenderOptions, build_cfr_frame_slots
from .process import hidden_subprocess_kwargs


class GpuRendererUnavailable(RuntimeError):
    pass


@dataclass(frozen=True)
class GpuRenderResult:
    renderer_name: str
    api: str = "OpenGL 3.3"


_VERTEX_SHADER = """
#version 330 core
void main() {
    vec2 position = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    gl_Position = vec4(position * 2.0 - 1.0, 0.0, 1.0);
}
"""


_FRAGMENT_SHADER = """
#version 330 core

uniform sampler2D u_input;
uniform sampler2D u_rowMatrices;
uniform float u_eyeSize;
uniform mat3 u_frameRotation;
uniform mat3 u_leftLensRotation;
uniform mat3 u_rightLensRotation;
uniform vec4 u_leftLens;
uniform vec4 u_rightLens;
uniform vec4 u_leftDistortion;
uniform vec4 u_rightDistortion;
uniform float u_halfFov;
uniform int u_hasRollingShutter;
uniform int u_rowCount;

out vec4 fragColor;

vec2 projectFisheye(vec3 direction, vec4 lens, vec4 distortion) {
    float theta = acos(clamp(direction.z, -1.0, 1.0));
    float theta2 = theta * theta;
    float theta4 = theta2 * theta2;
    float theta6 = theta4 * theta2;
    float theta8 = theta4 * theta4;
    float thetaD = theta * (
        1.0
        + distortion.x * theta2
        + distortion.y * theta4
        + distortion.z * theta6
        + distortion.w * theta8
    );
    float phi = atan(direction.y, direction.x);
    return vec2(
        lens.z + lens.x * thetaD * cos(phi),
        lens.w - lens.y * thetaD * sin(phi)
    );
}

vec3 applyRowMatrix(int row, vec3 direction) {
    int selectedRow = clamp(row, 0, u_rowCount - 1);
    vec3 r0 = texelFetch(u_rowMatrices, ivec2(0, selectedRow), 0).rgb;
    vec3 r1 = texelFetch(u_rowMatrices, ivec2(1, selectedRow), 0).rgb;
    vec3 r2 = texelFetch(u_rowMatrices, ivec2(2, selectedRow), 0).rgb;
    return vec3(dot(r0, direction), dot(r1, direction), dot(r2, direction));
}

void main() {
    bool leftEye = gl_FragCoord.x < u_eyeSize;
    float eyeOffset = leftEye ? 0.0 : u_eyeSize;
    float pixelX = gl_FragCoord.x - 0.5 - eyeOffset;
    float pixelY = u_eyeSize - gl_FragCoord.y - 0.5;
    float center = (u_eyeSize - 1.0) * 0.5;
    float radius = u_eyeSize * 0.5;
    float dx = (pixelX - center) / radius;
    float dy = (center - pixelY) / radius;
    float normalizedRadius = length(vec2(dx, dy));
    if (normalizedRadius > 1.0) {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    float phi = atan(dy, dx);
    float theta = normalizedRadius * u_halfFov;
    float sinTheta = sin(theta);
    vec3 outputDirection = vec3(
        sinTheta * cos(phi),
        sinTheta * sin(phi),
        cos(theta)
    );
    vec3 baseSourceDirection = u_frameRotation * outputDirection;
    vec3 sourceDirection = baseSourceDirection;
    mat3 lensRotation = leftEye ? u_leftLensRotation : u_rightLensRotation;
    vec4 lens = leftEye ? u_leftLens : u_rightLens;
    vec4 distortion = leftEye ? u_leftDistortion : u_rightDistortion;

    if (u_hasRollingShutter != 0) {
        for (int iteration = 0; iteration < 2; ++iteration) {
            vec2 lookup = projectFisheye(lensRotation * sourceDirection, lens, distortion);
            int sourceRow = int(floor(lookup.y + 0.5));
            sourceDirection = applyRowMatrix(sourceRow, baseSourceDirection);
        }
    }

    sourceDirection = lensRotation * sourceDirection;
    float sourceTheta = acos(clamp(sourceDirection.z, -1.0, 1.0));
    vec2 sourcePixel = projectFisheye(sourceDirection, lens, distortion);
    bool valid = (
        sourceTheta <= u_halfFov
        && sourcePixel.x >= 0.0
        && sourcePixel.x < u_eyeSize - 1.0
        && sourcePixel.y >= 0.0
        && sourcePixel.y < u_eyeSize - 1.0
    );
    if (!valid) {
        fragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }

    vec2 inputSize = vec2(u_eyeSize * 2.0, u_eyeSize);
    vec2 textureCoordinate = vec2(
        (sourcePixel.x + eyeOffset + 0.5) / inputSize.x,
        (sourcePixel.y + 0.5) / inputSize.y
    );
    fragColor = vec4(texture(u_input, textureCoordinate).rgb, 1.0);
}
"""


class OpenGlSbsRenderer:
    def __init__(
        self,
        output_width: int,
        calibration: dict[str, Any] | None,
        distortion_correction: bool,
        field_of_view_deg: float,
    ) -> None:
        self.output_width = output_width
        self.eye_size = output_width // 2
        self.calibration = calibration or {}
        self.distortion_correction = distortion_correction
        self.field_of_view_deg = field_of_view_deg
        self._owned_app: QGuiApplication | None = None
        if QGuiApplication.instance() is None:
            self._owned_app = QGuiApplication([])

        surface_format = QSurfaceFormat()
        surface_format.setRenderableType(QSurfaceFormat.OpenGL)
        surface_format.setVersion(3, 3)
        surface_format.setProfile(QSurfaceFormat.CoreProfile)

        self.context = QOpenGLContext()
        self.context.setFormat(surface_format)
        if not self.context.create() or not self.context.isValid():
            raise GpuRendererUnavailable("OpenGL 3.3 context creation failed.")

        self.surface = QOffscreenSurface()
        self.surface.setFormat(self.context.format())
        self.surface.create()
        if not self.surface.isValid() or not self.context.makeCurrent(self.surface):
            raise GpuRendererUnavailable("OpenGL offscreen surface creation failed.")

        self.functions = self.context.extraFunctions()
        renderer = self.functions.glGetString(0x1F01)
        self.renderer_name = str(renderer or "Unknown OpenGL GPU")
        software_tokens = ("gdi generic", "llvmpipe", "microsoft basic render")
        if any(token in self.renderer_name.lower() for token in software_tokens):
            self.context.doneCurrent()
            raise GpuRendererUnavailable(f"Only a software OpenGL renderer is available: {self.renderer_name}")

        self.program = QOpenGLShaderProgram()
        if not self.program.addShaderFromSourceCode(QOpenGLShader.Vertex, _VERTEX_SHADER):
            raise GpuRendererUnavailable(f"GPU vertex shader compilation failed: {self.program.log()}")
        if not self.program.addShaderFromSourceCode(QOpenGLShader.Fragment, _FRAGMENT_SHADER):
            raise GpuRendererUnavailable(f"GPU fragment shader compilation failed: {self.program.log()}")
        if not self.program.link():
            raise GpuRendererUnavailable(f"GPU shader linking failed: {self.program.log()}")
        self.uniforms = {
            name: self.program.uniformLocation(name)
            for name in (
                "u_input",
                "u_rowMatrices",
                "u_eyeSize",
                "u_frameRotation",
                "u_leftLensRotation",
                "u_rightLensRotation",
                "u_leftLens",
                "u_rightLens",
                "u_leftDistortion",
                "u_rightDistortion",
                "u_halfFov",
                "u_hasRollingShutter",
                "u_rowCount",
            )
        }

        self.vertex_array = QOpenGLVertexArrayObject()
        if not self.vertex_array.create():
            raise GpuRendererUnavailable("OpenGL vertex array creation failed.")
        self.vertex_array.bind()

        self.input_texture = QOpenGLTexture(QOpenGLTexture.Target2D)
        self.input_texture.setFormat(QOpenGLTexture.RGB8_UNorm)
        self.input_texture.setSize(self.output_width, self.eye_size)
        self.input_texture.allocateStorage(QOpenGLTexture.RGB, QOpenGLTexture.UInt8)
        self.input_texture.setMinificationFilter(QOpenGLTexture.Linear)
        self.input_texture.setMagnificationFilter(QOpenGLTexture.Linear)
        self.input_texture.setWrapMode(QOpenGLTexture.ClampToEdge)

        self.row_texture = QOpenGLTexture(QOpenGLTexture.Target2D)
        self.row_texture.setFormat(QOpenGLTexture.RGB32F)
        self.row_texture.setSize(3, self.eye_size)
        self.row_texture.allocateStorage(QOpenGLTexture.RGB, QOpenGLTexture.Float32)
        self.row_texture.setMinificationFilter(QOpenGLTexture.Nearest)
        self.row_texture.setMagnificationFilter(QOpenGLTexture.Nearest)
        self.row_texture.setWrapMode(QOpenGLTexture.ClampToEdge)

        framebuffer_format = QOpenGLFramebufferObjectFormat()
        framebuffer_format.setInternalTextureFormat(0x8058)  # GL_RGBA8
        self.framebuffer = QOpenGLFramebufferObject(
            QSize(self.output_width, self.eye_size),
            framebuffer_format,
        )
        if not self.framebuffer.isValid():
            raise GpuRendererUnavailable("OpenGL framebuffer creation failed.")

        self.functions.glPixelStorei(0x0CF5, 1)  # GL_UNPACK_ALIGNMENT
        self.program.bind()
        self.functions.glUniform1i(self.uniforms["u_input"], 0)
        self.functions.glUniform1i(self.uniforms["u_rowMatrices"], 1)
        self.functions.glUniform1f(self.uniforms["u_eyeSize"], float(self.eye_size))
        self.functions.glUniform1f(
            self.uniforms["u_halfFov"],
            float(np.deg2rad(field_of_view_deg) * 0.5),
        )
        self.functions.glUniform1i(self.uniforms["u_rowCount"], self.eye_size)
        self._set_lens_uniforms()
        self.program.release()
        self.context.doneCurrent()

    def _lens_values(
        self,
        side: str,
    ) -> tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        np.ndarray,
    ]:
        lens = self.calibration.get(side) or {}
        image_size = self.calibration.get("image_size") or {}
        source_width = float(image_size.get("width", self.eye_size) or self.eye_size)
        source_height = float(image_size.get("height", self.eye_size) or self.eye_size)
        scale_x = self.eye_size / source_width
        scale_y = self.eye_size / source_height
        default_focal = self.eye_size / max(1e-9, np.deg2rad(self.field_of_view_deg))
        values = (
            float(lens.get("fx", default_focal)) * scale_x,
            float(lens.get("fy", default_focal)) * scale_y,
            float(lens.get("cx", (source_width - 1.0) / 2.0)) * scale_x,
            float(lens.get("cy", (source_height - 1.0) / 2.0)) * scale_y,
        )
        coefficients = (
            [float(value) for value in lens.get("distortion", [])[:4]]
            if self.distortion_correction
            else []
        )
        coefficients += [0.0] * (4 - len(coefficients))
        distortion = tuple(coefficients)
        rotation_values = lens.get("output_to_lens_rotation")
        rotation = np.asarray(
            rotation_values
            if rotation_values
            else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        ).reshape(3, 3)
        return values, distortion, rotation

    def _set_matrix_uniform(self, name: str, values: np.ndarray) -> None:
        matrix = np.asarray(values, dtype=np.float32).reshape(3, 3)
        column_major = array("f", matrix.T.reshape(-1).tolist())
        self.functions.glUniformMatrix3fv(
            self.uniforms[name],
            1,
            False,
            column_major,
        )

    def _set_vector4_uniform(
        self,
        name: str,
        values: tuple[float, float, float, float],
    ) -> None:
        self.functions.glUniform4f(self.uniforms[name], *values)

    def _set_lens_uniforms(self) -> None:
        left_lens, left_distortion, left_rotation = self._lens_values("left")
        right_lens, right_distortion, right_rotation = self._lens_values("right")
        self._set_vector4_uniform("u_leftLens", left_lens)
        self._set_vector4_uniform("u_rightLens", right_lens)
        self._set_vector4_uniform("u_leftDistortion", left_distortion)
        self._set_vector4_uniform("u_rightDistortion", right_distortion)
        self._set_matrix_uniform("u_leftLensRotation", left_rotation)
        self._set_matrix_uniform("u_rightLensRotation", right_rotation)

    def render(
        self,
        frame: np.ndarray,
        correction_matrix: np.ndarray,
        row_matrices: np.ndarray | None,
    ) -> np.ndarray:
        if frame.shape != (self.eye_size, self.output_width, 3):
            raise ValueError(
                f"GPU input frame shape {frame.shape} does not match "
                f"{(self.eye_size, self.output_width, 3)}."
            )
        if not self.context.makeCurrent(self.surface):
            raise GpuRendererUnavailable("The OpenGL context could not be made current.")

        contiguous_frame = np.ascontiguousarray(frame, dtype=np.uint8)
        self.input_texture.setData(
            QOpenGLTexture.RGB,
            QOpenGLTexture.UInt8,
            contiguous_frame.data,
        )
        has_rolling_shutter = row_matrices is not None and len(row_matrices) > 0
        if has_rolling_shutter:
            rows = np.ascontiguousarray(row_matrices, dtype=np.float32)
            if rows.shape != (self.eye_size, 3, 3):
                raise ValueError(
                    f"GPU rolling-shutter matrix shape {rows.shape} does not match "
                    f"{(self.eye_size, 3, 3)}."
                )
            self.row_texture.setData(
                QOpenGLTexture.RGB,
                QOpenGLTexture.Float32,
                rows.data,
            )

        self.framebuffer.bind()
        self.functions.glViewport(0, 0, self.output_width, self.eye_size)
        self.program.bind()
        self.vertex_array.bind()
        self.input_texture.bind(0)
        self.row_texture.bind(1)
        self._set_matrix_uniform("u_frameRotation", correction_matrix)
        self.functions.glUniform1i(
            self.uniforms["u_hasRollingShutter"],
            int(has_rolling_shutter),
        )
        self.functions.glDrawArrays(0x0004, 0, 3)  # GL_TRIANGLES
        self.functions.glFinish()

        image = self.framebuffer.toImage(True).convertToFormat(QImage.Format_RGB888)
        bytes_per_line = image.bytesPerLine()
        output = np.frombuffer(
            image.bits(),
            dtype=np.uint8,
            count=image.sizeInBytes(),
        ).reshape(self.eye_size, bytes_per_line)[:, : self.output_width * 3]
        output = output.reshape(self.eye_size, self.output_width, 3).copy()
        self.framebuffer.release()
        self.program.release()
        self.context.doneCurrent()
        return output

    def close(self) -> None:
        if self.context.makeCurrent(self.surface):
            self.input_texture.destroy()
            self.row_texture.destroy()
            self.vertex_array.destroy()
            self.context.doneCurrent()


def render_stabilized_sbs_gpu(
    input_video: Path,
    output_video: Path,
    frame_plan: list[FrameStabilization],
    frame_rate: float,
    options: CpuRenderOptions,
    calibration: dict[str, Any] | None = None,
    rolling_shutter_plan: list[np.ndarray] | None = None,
    progress=None,
) -> GpuRenderResult:
    output_width = max(640, int(options.output_width))
    if output_width % 2:
        output_width += 1
    output_height = output_width // 2
    frame_size = output_width * output_height * 3
    output_frame_rate, frame_slots = build_cfr_frame_slots(
        [frame.time_s for frame in frame_plan],
        frame_rate,
    )
    output_frame_count = frame_slots[-1] + 1 if frame_slots else len(frame_plan)
    output_video.parent.mkdir(parents=True, exist_ok=True)
    renderer = OpenGlSbsRenderer(
        output_width,
        calibration,
        options.distortion_correction,
        options.field_of_view_deg,
    )
    if progress:
        progress(82, f"OpenGL GPU Renderer active: {renderer.renderer_name}")

    decode_args = [
        "ffmpeg",
        "-v",
        "error",
        "-i",
        str(input_video),
        "-an",
        "-vf",
        f"scale={output_width}:{output_height}",
        "-vsync",
        "0",
        "-pix_fmt",
        "rgb24",
        "-f",
        "rawvideo",
        "-",
    ]
    encode_args = [
        "ffmpeg",
        "-y",
        "-v",
        "error",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "rgb24",
        "-s",
        f"{output_width}x{output_height}",
        "-r",
        f"{output_frame_rate:.6f}",
        "-i",
        "-",
        "-i",
        str(input_video),
        "-map",
        "0:v:0",
        "-map",
        "1:a?",
        "-c:v",
        "libx264",
        "-preset",
        options.preset,
        "-crf",
        str(options.crf),
        "-pix_fmt",
        "yuv420p",
        "-c:a",
        "copy",
        "-movflags",
        "+faststart",
        "-t",
        f"{output_frame_count / max(1e-9, output_frame_rate):.6f}",
        "-metadata:s:v:0",
        "stereo_mode=left_right",
        "-metadata:s:v:0",
        "projection=fisheye",
        "-metadata:s:v:0",
        "spherical=true",
        "-metadata:s:v:0",
        "vr180=true",
        str(output_video),
    ]
    decode = subprocess.Popen(
        decode_args,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        **hidden_subprocess_kwargs(),
    )
    encode = subprocess.Popen(
        encode_args,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        **hidden_subprocess_kwargs(),
    )
    assert decode.stdout is not None
    assert encode.stdin is not None

    try:
        total = len(frame_plan)
        previous_rendered: np.ndarray | None = None
        next_output_slot = 0
        rendered_count = 0
        for index, frame_stab in enumerate(frame_plan):
            raw = decode.stdout.read(frame_size)
            if len(raw) < frame_size:
                break
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((output_height, output_width, 3))
            row_matrices = (
                rolling_shutter_plan[index]
                if rolling_shutter_plan is not None and index < len(rolling_shutter_plan)
                else None
            )
            rendered = renderer.render(
                frame,
                np.asarray(frame_stab.correction_matrix3, dtype=np.float32),
                row_matrices,
            )
            target_slot = frame_slots[index] if index < len(frame_slots) else next_output_slot
            while next_output_slot < target_slot:
                filler = previous_rendered if previous_rendered is not None else rendered
                encode.stdin.write(filler.tobytes())
                next_output_slot += 1
            encode.stdin.write(rendered.tobytes())
            next_output_slot += 1
            previous_rendered = rendered
            rendered_count += 1
            if progress:
                percent = 82 + int((index + 1) * 16 / max(1, total))
                correction_deg = Quat.identity().angular_distance_deg(
                    Quat.from_iter(frame_stab.correction_wxyz)
                )
                progress(
                    min(98, percent),
                    (
                        f"GPU rendering frame {index + 1}/{total} | "
                        f"video={frame_stab.time_s:.6f}s | correction={correction_deg:.3f}deg"
                        f"{' | rolling-shutter rows active' if rolling_shutter_plan else ''}"
                    ),
                )
    finally:
        renderer.close()
        try:
            encode.stdin.close()
        except Exception:
            pass
        decode_stderr = decode.stderr.read().decode("utf-8", errors="replace") if decode.stderr else ""
        encode_stdout = encode.stdout.read() if encode.stdout else b""
        encode_stderr = encode.stderr.read().decode("utf-8", errors="replace") if encode.stderr else ""
        decode_code = decode.wait()
        encode_code = encode.wait()

    if rendered_count != len(frame_plan):
        raise RuntimeError(
            f"GPU renderer received {rendered_count} frames but expected {len(frame_plan)}."
        )
    if decode_code != 0:
        raise RuntimeError(f"FFmpeg decode failed: {decode_stderr}")
    if encode_code != 0:
        raise RuntimeError(f"FFmpeg encode failed: {encode_stderr}")
    return GpuRenderResult(renderer_name=renderer.renderer_name)
