import copy

import numpy as np
from julia import Main
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation

from DifferentiableOptimizationCBF.base_controller import BaseController
from DifferentiableOptimizationCBF.cbfqp_solver import CBFQPSolver
from DifferentiableOptimizationCBF.exp_utils import get_Q_mat


class TwoWallsController(BaseController):
    def __init__(self, crude_type="capsule"):
        super().__init__(crude_type=crude_type)

        exp_setup = Main.include("dc_utils/two_wall_exp_setup.jl")
        self.get_cbf = Main.include("dc_utils/get_cbf_two_walls.jl")

        exp_setup()

        # define solver
        self.solver = CBFQPSolver(9, 0, 28)

    def controller(self, t, q, dq):
        self.update_pinocchio(q, dq)
        info = self.get_info(q, dq)

        if not self.initialized:
            self.initialize_trajectory(
                t,
                info["P_HAND"],
                info["R_HAND"],
                np.array([[1.2], [0], [0.35]]),
                (0.0, -90.0, 0.0),
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

            for j in range(4):
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
