
from __future__ import annotations

import numpy as np
import warp as wp

from newtonian.modules.params import ParameterError


def convert_pose_to_transform(pose: list[float] | None, where: str = "pose") -> wp.transform:
    """
    Returns '[x, y, z, roll, pitch, yaw]' as a transform, and raises 'ParameterError' if it does not have 6 values.
    """
    pose = pose or [0.0] * 6
    if len(pose) != 6:
        raise ParameterError(f"{where} expected [x, y, z, roll, pitch, yaw] but got {pose!r}")
    x, y, z, roll, pitch, yaw = (float(value) for value in pose)
    q = wp.quat_rpy(roll=roll, pitch=pitch, yaw=yaw)
    return wp.transform(wp.vec3(x, y, z), q)


def quat_rotate(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """
    Returns 'vec' rotated by 'quat'.
    """
    u, w = quat[..., :3], quat[..., 3:4]
    t = 2.0 * np.cross(u, vec)
    return vec + w * t + np.cross(u, t)


def quat_rotate_inv(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """
    Returns 'vec' rotated by the inverse of 'quat'.
    """
    q = quat.copy()
    q[..., :3] *= -1.0
    return quat_rotate(quat=q, vec=vec)


def quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Returns the product of the quaternions 'a' and 'b'.
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def tf_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Returns the transform 'a' applied to the transform 'b'.
    """
    p = a[:3] + quat_rotate(quat=a[3:], vec=b[:3])
    return np.concatenate([p, quat_mul(a=a[3:], b=b[3:])])


_R = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
CONVERT_ROS_TO_GL = np.array(wp.quat_from_matrix(wp.mat33(*_R.flatten())), dtype=np.float64)    # pyright: ignore[reportCallIssue, reportArgumentType]
TRANSFORM_ROS_TO_GL = np.concatenate([np.zeros(3), CONVERT_ROS_TO_GL])
