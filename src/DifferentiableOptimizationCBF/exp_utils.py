import numpy as np
import numpy.linalg as LA
from scipy.spatial.transform import Rotation


def change_quat_format(q):
    """
    change from q = [x y z w] to quat = [w x y z]
    """
    quat = np.zeros(4)
    quat[0] = q[3]
    quat[1] = q[0]
    quat[2] = q[1]
    quat[3] = q[2]

    return quat


def get_Q_mat(q):
    """
    q = [x y z w]
    """
    qx, qy, qz, qw = q[0], q[1], q[2], q[3]

    Q = np.array([[-qx, -qy, -qz], [qw, -qz, qy], [qz, qw, -qx], [-qy, qx, qw]])

    return Q


def get_link_config(link_idx, info):
    """
    Get the position and orientation of the link from pybullet
    and change the rotation format from pybullet to DifferentiableCollisions.jl
    """
    link_r = info[f"P_LINK{link_idx}"]
    link_R = info[f"R_LINK{link_idx}"] @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    link_q = Rotation.from_matrix(link_R).as_quat()

    return link_r, link_q


def axis_angle_from_rot_mat(rot_mat):
    rotation = Rotation.from_matrix(rot_mat)
    axis_angle = rotation.as_rotvec()
    angle = LA.norm(axis_angle)
    axis = axis_angle / angle

    return axis, angle


def get_R_end_from_start(x_ang, y_ang, z_ang, R_start):
    """Get target orientation based on initial orientation"""
    _R_end = (
        Rotation.from_euler("x", x_ang, degrees=True).as_matrix()
        @ Rotation.from_euler("y", y_ang, degrees=True).as_matrix()
        @ Rotation.from_euler("z", z_ang, degrees=True).as_matrix()
        @ R_start
    )
    R_end = Rotation.from_matrix(_R_end).as_matrix()

    return R_end
