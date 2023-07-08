import numpy as np
import proxsuite
from scipy.linalg import block_diag


class CBFQPSolver:
    def __init__(self, n, n_eq, n_ieq):
        self.n = n
        self.n_eq = n_eq
        self.n_ieq = n_ieq
        self.qp = proxsuite.proxqp.dense.QP(self.n, self.n_eq, self.n_ieq)
        self.initialized = False

    def solve(self, params):
        self.H, self.g, self.C, self.lb = self.compute_params(params)

        if not self.initialized:
            self.qp.init(H=self.H, g=self.g, C=self.C, l=self.lb)
            self.qp.settings.eps_abs = 1.0e-6
            self.initialized = True
        else:
            self.qp.update(H=self.H, g=self.g, C=self.C, l=self.lb)

        self.qp.solve()

    def compute_params(self, params):
        H = (
            2 * params["Jacobian"].T @ params["Jacobian"]
            + 2 * params["nullspace_proj"].T @ params["nullspace_proj"]
        )

        a = params["Kp"] @ params["p_error"] + params["dp_target"]

        g = (
            -2
            * (
                a.T @ params["Jacobian"]
                + params["dq_nominal"].T
                @ params["nullspace_proj"].T
                @ params["nullspace_proj"]
            )[0, :]
        )

        C = params["C"]
        lb = params["lb"]

        return H, g, C, lb
