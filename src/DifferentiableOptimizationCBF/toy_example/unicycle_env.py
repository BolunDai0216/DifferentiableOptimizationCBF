from __future__ import annotations

import math
import sys
from typing import TYPE_CHECKING

import numpy as np

from DifferentiableOptimizationCBF.toy_example.unicycle_dynamics import get_F_mat

if TYPE_CHECKING:
    from numpy.typing import NDArray


class UnicycleState:
    """Unicycle state [x, y, yaw] backed by a numpy array.

    Construct with either an array or x/y/yaw keyword args:
        UnicycleState(np.array([1.0, 2.0, 0.5]))
        UnicycleState(x=1.0, y=2.0, yaw=0.5)
    """

    def __init__(
        self,
        array: NDArray | None = None,
        *,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
    ) -> None:
        if array is not None:
            self.array = np.asarray(array, dtype=float)
            self.x, self.y, self.yaw = self.array
        elif x is not None and y is not None and yaw is not None:
            self.x, self.y, self.yaw = x, y, yaw
            self.array = np.array([x, y, yaw], dtype=float)
        else:
            raise ValueError(
                f"{type(self).__name__} requires either an array or all of x, y, yaw"
            )

        if self.array.shape != (3,):
            raise ValueError(
                f"{type(self).__name__} expects an array of shape (3,), got {self.array.shape}"
            )


class UnicycleEnv:
    def __init__(self, dt: float = 0.01) -> None:
        self.state: UnicycleState | None = None
        self.dt: float = dt

    def reset(self, init_state: UnicycleState | None = None) -> None:
        """
        The robot state is [x, y, yaw]
        """

        if init_state is not None:
            self.state = init_state
        else:
            self.state = UnicycleState(x=-1.0, y=-3.0, yaw=np.pi / 4)

    def step(self, action: NDArray) -> None:
        """safe_control
        The system dynamics is:
        dx = v cos(yaw)
        dy = v sin(yaw)
        dyaw = ω

        The control action is [v, ω].
        """
        if self.state is None:
            self._raise_state_unset_error()

        if action.shape == (2,):
            action = action[:, np.newaxis]

        dstate = (get_F_mat(self.state) @ action)[:, 0]

        self.state = UnicycleState(
            x=self.state.x + dstate[0] * self.dt,
            y=self.state.y + dstate[1] * self.dt,
            yaw=math.remainder(self.state.yaw + dstate[2] * self.dt, 2 * math.pi),
        )

    def _raise_state_unset_error(self) -> str:
        fn = sys._getframe(1).f_code.co_name
        raise ValueError(
            f"{type(self).__name__}.{fn}(): self.state is None. Did you forget to call reset() before {fn}()?"
        )
