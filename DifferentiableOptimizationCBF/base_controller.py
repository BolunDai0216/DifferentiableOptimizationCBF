import copy

import FR3Env
from julia import Main
import numpy as np
import pinocchio as pin
from FR3CBFSim.controllers.utils import axis_angle_from_rot_mat, get_R_end_from_start
from pinocchio.robot_wrapper import RobotWrapper
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation

from DifferentiableOptimizationCBF.cbfqp_solver import CBFQPSolver
from DifferentiableOptimizationCBF.exp_utils import (
    change_quat_format,
    get_link_config,
    get_Q_mat,
)


class BaseController:
    def __init__(self, crude_type="ellipsoid"):
        # load URDF file
        self.fr3env_dir = FR3Env.getDataPath()
        self.fr3_urdf = self.fr3env_dir + "/robots/fr3_crude.urdf"

        # build pin_robot
        self.robot = RobotWrapper.BuildFromURDF(self.fr3_urdf, self.fr3env_dir)

        # get Jacobian frame
        self.jacobian_frame = pin.ReferenceFrame.LOCAL_WORLD_ALIGNED

        # get fr3 frame ids
        self.FR3_LINK3_FRAME_ID = 8
        self.FR3_LINK4_FRAME_ID = 10
        self.FR3_LINK5_FRAME_ID = 12
        self.FR3_LINK6_FRAME_ID = 14
        self.FR3_LINK7_FRAME_ID = 16
        self.FR3_HAND_FRAME_ID = 20

        # save the frame ids in a list
        self.frame_ids = [
            self.FR3_LINK3_FRAME_ID,
            self.FR3_LINK4_FRAME_ID,
            self.FR3_LINK5_FRAME_ID,
            self.FR3_LINK5_FRAME_ID,
            self.FR3_LINK6_FRAME_ID,
            self.FR3_LINK7_FRAME_ID,
            self.FR3_HAND_FRAME_ID,
        ]

        self.frame_names = [
            "LINK3",
            "LINK4",
            "LINK5_1",
            "LINK5_2",
            "LINK6",
            "LINK7",
            "HAND",
        ]

        # get the bounding primitive rotation offsets
        self.R_offset = {}
        for name in self.frame_names:
            self.R_offset[name] = np.eye(3)

        # get the bounding primitive position offsets
        self.P_offset = {
            "LINK3": np.array([[0.0], [0.0], [-0.145]]),
            "LINK4": np.array([[0.0], [0.0], [0.0]]),
            "LINK5_1": np.array([[0.0], [0.0], [-0.26]]),
            "LINK5_2": np.array([[0.0], [0.08], [-0.13]]),
            "LINK6": np.array([[0.0], [0.0], [-0.03]]),
            "LINK7": np.array([[0.0], [0.0], [0.01]]),
            "HAND": np.array([[0.0], [0.0], [0.06]]),
        }

        # set nominal joint angles
        self.q_nominal = np.array(
            [
                [0.0],
                [-np.pi / 4],
                [0.0],
                [-3 * np.pi / 4],
                [0.0],
                [np.pi / 2],
                [np.pi / 4],
                [0.001],
                [0.001],
            ]
        )

        # load Julia functions
        if crude_type == "ellipsoid":
            create_arm = Main.include("dc_utils/create_arm_ellipsoid.jl")
        elif crude_type == "capsule":
            create_arm = Main.include("dc_utils/create_arm_capsule.jl")

        exp_setup = Main.include("dc_utils/three_blocks_exp_setup.jl")
        self.get_cbf = Main.include("dc_utils/get_cbf_three_blocks.jl")

        # setup everything in Julia
        create_arm()
        exp_setup()

        # define solver
        self.solver = CBFQPSolver(9, 0, 21)

        self.initialized = False

    def controller(self, t, q, dq):
        self.update_pinocchio(q, dq)
        info = self.get_info(q, dq)

        if not self.initialized:
            self.initialize_trajectory(
                t,
                info["P_HAND"],
                info["R_HAND"],
                np.array([[0.7], [0.05], [0.1]]),
                (0.0, 0.0, 0.0),
            )

        # get end-effector position
        p_current = info["P_HAND"][:, np.newaxis]

        # get end-effector orientation
        R_current = info["R_HAND"]

        # get Jacobians from info
        pinv_jac = info["pJ_HAND"]
        jacobian = info["J_HAND"]

        # compute joint-centering joint acceleration
        dq_nominal = 0.5 * (self.q_nominal - q[:, np.newaxis])

        # compute error rotation matrix
        R_err = self.R_end @ R_current.T

        # compute orientation error in axis-angle form
        rotvec_err = Rotation.from_matrix(R_err).as_rotvec()

        # compute EE position error
        p_error = np.zeros((6, 1))
        p_error[:3] = self.p_end - p_current
        p_error[3:] = rotvec_err[:, np.newaxis]

        # compute EE velocity target
        dp_target = np.zeros((6, 1))

        # update link position and oriention in DifferentiableCollisions
        rs, qs = self.compute_rs_qs(info)

        # compute α's and J's
        _αs, Js = self.get_cbf(rs, qs)

        # compute α's and J's
        αs = []
        Cs = []

        for k, link in enumerate(self.frame_names):
            _Q_mat_link = get_Q_mat(info[f"q_{link}"])
            Q_mat_link = block_diag(np.eye(3), 0.5 * _Q_mat_link)

            for j in range(3):
                α, J_link = _αs[j][k], np.array(Js[j][k])
                αs.append(copy.deepcopy(α))
                Cs.append(
                    J_link[-1, 7:][np.newaxis, :] @ Q_mat_link @ info[f"J_{link}"]
                )

        lb = -5.0 * (np.array(αs)[:, np.newaxis] - 1.03)
        C = np.concatenate(Cs, axis=0)

        params = {
            "Jacobian": jacobian,
            "p_error": p_error,
            "p_current": p_current,
            "dp_target": dp_target,
            "Kp": 0.1 * np.eye(6),
            "dq_nominal": dq_nominal,
            "nullspace_proj": np.eye(9) - pinv_jac @ jacobian,
            "lb": lb,
            "C": C,
        }

        # solver for target joint velocity
        self.solver.solve(params)
        dq_target = self.solver.qp.results.x

        info["αs"] = _αs
        info["Js"] = Js

        return dq_target, info

    def get_info(self, q, dq):
        info = {}

        for idx, name in zip(self.frame_ids, self.frame_names):
            info[f"J_{name}"] = self.robot.getFrameJacobian(idx, self.jacobian_frame)

            # compute the position and rotation of the crude models
            (
                info[f"P_{name}"],
                info[f"R_{name}"],
                info[f"q_{name}"],
            ) = self.compute_crude_location(
                self.R_offset[name], self.P_offset[name], idx
            )

        # Get pseudo-inverse of hand Jacobian
        info["pJ_HAND"] = np.linalg.pinv(info["J_HAND"])
        info["q"], info["dq"] = q, dq

        return info

    def update_pinocchio(self, q, dq):
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)

    def compute_crude_location(self, R_offset, p_offset, frame_id):
        # get link transformation matrix
        T = self.robot.data.oMf[frame_id].homogeneous

        # compute link offset transformation matrix
        TB = pin.SE3(R_offset, p_offset)

        # get transformation matrix
        T_mat = pin.SE3(T @ TB)

        # compute crude model location
        p = T_mat.translation

        # compute crude model orientation
        Rot = T_mat.rotation

        # quaternion
        q = Rotation.from_matrix(Rot).as_quat()

        return p, Rot, q

    def initialize_trajectory(
        self,
        t,
        end_effector_pos,
        end_effector_rot,
        target_end_effector_pos,
        target_relative_end_effector_rpy,
    ):
        # get initial rotation and position
        self.R_start, _p_start = end_effector_rot, end_effector_pos
        self.p_start = _p_start[:, np.newaxis]

        # get target position
        self.p_end = target_end_effector_pos

        # get target rotation
        roll, pitch, yaw = target_relative_end_effector_rpy
        self.R_end = get_R_end_from_start(roll, pitch, yaw, self.R_start)

        # compute R_error, ω_error, θ_error
        self.R_error = self.R_end @ self.R_start.T
        self.ω_error, self.θ_error = axis_angle_from_rot_mat(self.R_error)

        self.initial_time = copy.deepcopy(t)
        self.initialized = True

    def compute_rs_qs(self, info):
        # update link position and oriention in DifferentiableCollisions
        link_rs = []
        link_qs = []

        for idx in ["3", "4", "5_1", "5_2", "6", "7"]:
            _link_r, _link_q = get_link_config(idx, info)
            link_rs.append(copy.deepcopy(_link_r))
            link_qs.append(copy.deepcopy(_link_q))

        # update hand configuration
        link_rs.append(info["P_HAND"])
        link_qs.append(change_quat_format(info["q_HAND"]))

        rs = np.concatenate(link_rs)
        qs = np.concatenate(link_qs)

        return rs, qs
