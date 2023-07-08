import pybullet as p

from DifferentiableOptimizationCBF import getDataPath
from DifferentiableOptimizationCBF.envs.fr3_base_env import FR3BaseEnv


class TwoWallsEnv(FR3BaseEnv):
    def __init__(
        self,
        render_mode="human",
        record_path=None,
        crude_model=False,
        dt=1 / 1000,
        crude_type="capsule",
    ):
        super().__init__(
            render_mode=render_mode,
            record_path=record_path,
            crude_model=crude_model,
            dt=dt,
            crude_type=crude_type,
        )

        datapath = getDataPath()
        p.setAdditionalSearchPath(datapath + "/robots")

        # load obstacle
        wall_front_upper = p.loadURDF("wall_front_upper.urdf", useFixedBase=True)
        wall_front_upper_pos = [0.55, 0, 0.8]
        p.resetBasePositionAndOrientation(
            wall_front_upper, wall_front_upper_pos, [0, 0, 0, 1]
        )

        wall_front_lower = p.loadURDF("wall_front_lower.urdf", useFixedBase=True)
        wall_front_lower_pos = [0.55, 0, 0.1]
        p.resetBasePositionAndOrientation(
            wall_front_lower, wall_front_lower_pos, [0, 0, 0, 1]
        )

        wall_back_upper = p.loadURDF("wall_back_upper.urdf", useFixedBase=True)
        wall_back_upper_pos = [0.85, 0, 0.85]
        p.resetBasePositionAndOrientation(
            wall_back_upper, wall_back_upper_pos, [0, 0, 0, 1]
        )

        wall_back_lower = p.loadURDF("wall_back_lower.urdf", useFixedBase=True)
        wall_back_lower_pos = [0.85, 0, 0.15]
        p.resetBasePositionAndOrientation(
            wall_back_lower, wall_back_lower_pos, [0, 0, 0, 1]
        )
