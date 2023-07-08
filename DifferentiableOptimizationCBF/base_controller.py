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

# load Julia functions
create_arm = Main.include("dc_utils/create_arm_ellipsoid.jl")
three_blocks_exp_setup = Main.include("dc_utils/three_blocks_exp_setup.jl")
get_cbf_three_blocks = Main.include("dc_utils/get_cbf_three_blocks.jl")


class BaseController:
    def __init__(self):
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
        self.EE_FRAME_ID = 26

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

        # define the polytopes
        three_blocks_exp_setup()

        # define the robot links
        create_arm()

        # define solver
        self.solver = CBFQPSolver(9, 0, 21)

        self.initialized = False

    def controller(self, t, q, dq):
        self.update_pinocchio(q, dq)
        info = self.get_info(q, dq)

        if not self.initialized:
            self.initialize_trajectory(
                t,
                info["P_EE"],
                info["R_EE"],
                np.array([[0.7], [0.05], [0.1]]),
                (0.0, 0.0, 0.0),
            )

        # get end-effector position
        p_current = info["P_EE"][:, np.newaxis]

        # get end-effector orientation
        R_current = info["R_EE"]

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

        _αs, Js = get_cbf_three_blocks(rs, qs)

        # compute α's and J's
        αs = []
        Cs = []

        for k, link in enumerate(
            ["link3", "link4", "link5_1", "link5_2", "link6", "link7", "hand"]
        ):
            _Q_mat_link = get_Q_mat(info[f"q_{link.upper()}"])
            Q_mat_link = block_diag(np.eye(3), 0.5 * _Q_mat_link)

            for j in range(3):
                α, J_link = _αs[j][k], np.array(Js[j][k])
                αs.append(copy.deepcopy(α))

                if link == "hand":
                    Cs.append(J_link[-1, 7:][np.newaxis, :] @ Q_mat_link @ jacobian)
                else:
                    Cs.append(
                        J_link[-1, 7:][np.newaxis, :]
                        @ Q_mat_link
                        @ info[f"J_{link.upper()}"]
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
        # Get Jacobians, preprocessing is done in update_pinocchio()
        jacobian_link3 = self.robot.getFrameJacobian(
            self.FR3_LINK3_FRAME_ID, self.jacobian_frame
        )
        jacobian_link4 = self.robot.getFrameJacobian(
            self.FR3_LINK4_FRAME_ID, self.jacobian_frame
        )
        jacobian_link5_1 = self.robot.getFrameJacobian(
            self.FR3_LINK5_FRAME_ID, self.jacobian_frame
        )
        jacobian_link5_2 = self.robot.getFrameJacobian(
            self.FR3_LINK5_FRAME_ID, self.jacobian_frame
        )
        jacobian_link6 = self.robot.getFrameJacobian(
            self.FR3_LINK6_FRAME_ID, self.jacobian_frame
        )
        jacobian_link7 = self.robot.getFrameJacobian(
            self.FR3_LINK7_FRAME_ID, self.jacobian_frame
        )
        jacobian_hand = self.robot.getFrameJacobian(
            self.EE_FRAME_ID, self.jacobian_frame
        )

        # Get pseudo-inverse of hand Jacobian
        pinv_jacobian_hand = np.linalg.pinv(jacobian_hand)

        # compute the position and rotation of the crude models
        p_link3, R_link3, q_LINK3 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [-0.145])), self.FR3_LINK3_FRAME_ID
        )

        p_link4, R_link4, q_LINK4 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [0.0])), self.FR3_LINK4_FRAME_ID
        )

        p_link5_1, R_link5_1, q_LINK5_1 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [-0.26])), self.FR3_LINK5_FRAME_ID
        )

        p_link5_2, R_link5_2, q_LINK5_2 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.08], [-0.13])), self.FR3_LINK5_FRAME_ID
        )

        p_link6, R_link6, q_LINK6 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [-0.03])), self.FR3_LINK6_FRAME_ID
        )

        p_link7, R_link7, q_LINK7 = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [0.01])), self.FR3_LINK7_FRAME_ID
        )

        p_hand, R_hand, q_HAND = self.compute_crude_location(
            np.eye(3), np.array(([0.0], [0.0], [0.06])), self.FR3_HAND_FRAME_ID
        )

        info = {
            "q": q,
            "dq": dq,
            "R_LINK3": copy.deepcopy(R_link3),
            "P_LINK3": copy.deepcopy(p_link3),
            "q_LINK3": copy.deepcopy(q_LINK3),
            "J_LINK3": jacobian_link3,
            "R_LINK4": copy.deepcopy(R_link4),
            "P_LINK4": copy.deepcopy(p_link4),
            "q_LINK4": copy.deepcopy(q_LINK4),
            "J_LINK4": jacobian_link4,
            "R_LINK5_1": copy.deepcopy(R_link5_1),
            "P_LINK5_1": copy.deepcopy(p_link5_1),
            "q_LINK5_1": copy.deepcopy(q_LINK5_1),
            "J_LINK5_1": jacobian_link5_1,
            "R_LINK5_2": copy.deepcopy(R_link5_2),
            "P_LINK5_2": copy.deepcopy(p_link5_2),
            "q_LINK5_2": copy.deepcopy(q_LINK5_2),
            "J_LINK5_2": jacobian_link5_2,
            "R_LINK6": copy.deepcopy(R_link6),
            "P_LINK6": copy.deepcopy(p_link6),
            "q_LINK6": copy.deepcopy(q_LINK6),
            "J_LINK6": jacobian_link6,
            "R_LINK7": copy.deepcopy(R_link7),
            "P_LINK7": copy.deepcopy(p_link7),
            "q_LINK7": copy.deepcopy(q_LINK7),
            "J_LINK7": jacobian_link7,
            "R_HAND": copy.deepcopy(R_hand),
            "P_HAND": copy.deepcopy(p_hand),
            "q_HAND": copy.deepcopy(q_HAND),
            "J_HAND": jacobian_hand,
            "pJ_HAND": pinv_jacobian_hand,
            "R_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].rotation),
            "P_EE": copy.deepcopy(self.robot.data.oMf[self.EE_FRAME_ID].translation),
        }

        return info

    def update_pinocchio(self, q, dq):
        self.robot.computeJointJacobians(q)
        self.robot.framesForwardKinematics(q)
        self.robot.centroidalMomentum(q, dq)

    def compute_crude_location(self, R_offset, p_offset, frame_id):
        # get link orientation and position
        _p = self.robot.data.oMf[frame_id].translation
        _Rot = self.robot.data.oMf[frame_id].rotation

        # compute link transformation matrix
        _T = np.hstack((_Rot, _p[:, np.newaxis]))
        T = np.vstack((_T, np.array([[0.0, 0.0, 0.0, 1.0]])))

        # compute link offset transformation matrix
        _TB = np.hstack((R_offset, p_offset))
        TB = np.vstack((_TB, np.array([[0.0, 0.0, 0.0, 1.0]])))

        # get transformation matrix
        T_mat = T @ TB

        # compute crude model location
        p = (T_mat @ np.array([[0.0], [0.0], [0.0], [1.0]]))[:3, 0]

        # compute crude model orientation
        Rot = T_mat[:3, :3]

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
