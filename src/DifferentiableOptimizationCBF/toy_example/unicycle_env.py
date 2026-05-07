import math
from typing import TYPE_CHECKING

import numpy as np
import pinocchio as pin

if TYPE_CHECKING:
    from numpy.typing import NDArray


class UnicycleEnv:
    def __init__(self, dt: float = 0.01) -> None:
        self.state: NDArray = None
        self.robot_r: NDArray = None
        self.robot_q: pin.Quaternion = None
        self.dt: float = dt

    def reset(self, set_init_state: "NDArray | None" = None) -> None:
        """
        The robot state is [x, y, θ]
        """

        if set_init_state is not None:
            self.state = np.array([set_init_state[0], set_init_state[1], set_init_state[2]])
        else:
            self.state = np.array([-1.0, -3.0, np.pi / 4])

        self.update_robot_rq()

    def step(self, action: "NDArray") -> None:
        """safe_control
        The system dynamics is:
        dx = vcos(θ)
        dy = vsin(θ)
        dθ = ω

        The control action is [v, ω].
        """
        if action.shape == (2,):
            action = action[:, np.newaxis]

        F_mat = np.array([[np.cos(self.state[2]), 0.0], [np.sin(self.state[2]), 0.0], [0.0, 1.0]])
        dstate = (F_mat @ action)[:, 0]

        nextstate = self.state + dstate * self.dt
        nextstate[2] = math.remainder(nextstate[2], 2 * math.pi)
        self.state = nextstate

        self.update_robot_rq()

    def update_robot_rq(self) -> None:
        """
        Robot is only moving within the x-y plane, thus the z coordinate is always 0.0.
        Rotation is only about the z-axis.
        """
        self.robot_r = np.array([self.state[0], self.state[1], 0.0])
        self.robot_q = pin.Quaternion(pin.rpy.rpyToMatrix(0.0, 0.0, self.state[2]))
