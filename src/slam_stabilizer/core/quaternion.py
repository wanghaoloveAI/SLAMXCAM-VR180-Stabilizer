from __future__ import annotations

from dataclasses import dataclass
from math import acos, cos, degrees, sin, sqrt


@dataclass(frozen=True)
class Quat:
    """Unit quaternion in w, x, y, z order."""

    w: float
    x: float
    y: float
    z: float

    @classmethod
    def identity(cls) -> "Quat":
        return cls(1.0, 0.0, 0.0, 0.0)

    @classmethod
    def from_iter(cls, values: tuple[float, float, float, float] | list[float]) -> "Quat":
        return cls(float(values[0]), float(values[1]), float(values[2]), float(values[3])).normalized()

    @classmethod
    def from_matrix3(cls, matrix: list[list[float]]) -> "Quat":
        m = matrix
        trace = m[0][0] + m[1][1] + m[2][2]
        if trace > 0.0:
            s = sqrt(trace + 1.0) * 2.0
            return cls(
                0.25 * s,
                (m[2][1] - m[1][2]) / s,
                (m[0][2] - m[2][0]) / s,
                (m[1][0] - m[0][1]) / s,
            ).normalized()
        if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
            s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
            return cls(
                (m[2][1] - m[1][2]) / s,
                0.25 * s,
                (m[0][1] + m[1][0]) / s,
                (m[0][2] + m[2][0]) / s,
            ).normalized()
        if m[1][1] > m[2][2]:
            s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
            return cls(
                (m[0][2] - m[2][0]) / s,
                (m[0][1] + m[1][0]) / s,
                0.25 * s,
                (m[1][2] + m[2][1]) / s,
            ).normalized()
        s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        return cls(
            (m[1][0] - m[0][1]) / s,
            (m[0][2] + m[2][0]) / s,
            (m[1][2] + m[2][1]) / s,
            0.25 * s,
        ).normalized()

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.w, self.x, self.y, self.z)

    def norm(self) -> float:
        return sqrt(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)

    def normalized(self) -> "Quat":
        n = self.norm()
        if n <= 1e-12:
            return Quat.identity()
        return Quat(self.w / n, self.x / n, self.y / n, self.z / n)

    def conjugate(self) -> "Quat":
        return Quat(self.w, -self.x, -self.y, -self.z)

    def mul(self, other: "Quat") -> "Quat":
        aw, ax, ay, az = self.w, self.x, self.y, self.z
        bw, bx, by, bz = other.w, other.x, other.y, other.z
        return Quat(
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ).normalized()

    def dot(self, other: "Quat") -> float:
        return self.w * other.w + self.x * other.x + self.y * other.y + self.z * other.z

    def slerp(self, other: "Quat", t: float) -> "Quat":
        t = max(0.0, min(1.0, t))
        a = self.normalized()
        b = other.normalized()
        dot = a.dot(b)
        if dot < 0.0:
            b = Quat(-b.w, -b.x, -b.y, -b.z)
            dot = -dot
        if dot > 0.9995:
            return Quat(
                a.w + t * (b.w - a.w),
                a.x + t * (b.x - a.x),
                a.y + t * (b.y - a.y),
                a.z + t * (b.z - a.z),
            ).normalized()
        theta_0 = acos(max(-1.0, min(1.0, dot)))
        theta = theta_0 * t
        sin_theta = sin(theta)
        sin_theta_0 = sin(theta_0)
        s0 = cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return Quat(
            s0 * a.w + s1 * b.w,
            s0 * a.x + s1 * b.x,
            s0 * a.y + s1 * b.y,
            s0 * a.z + s1 * b.z,
        ).normalized()

    def angular_distance_deg(self, other: "Quat") -> float:
        rel = other.mul(self.conjugate())
        angle = 2.0 * acos(max(0.0, min(1.0, abs(rel.w))))
        return degrees(angle)

    def rotate_vector(self, vector: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
        p = Quat(0.0, float(vector[0]), float(vector[1]), float(vector[2]))
        rotated = self.mul(p).mul(self.conjugate())
        return (rotated.x, rotated.y, rotated.z)

    def to_matrix3(self) -> list[list[float]]:
        q = self.normalized()
        w, x, y, z = q.w, q.x, q.y, q.z
        return [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]

