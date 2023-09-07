import pybullet as p

from DifferentiableOptimizationCBF import getDataPath
from DifferentiableOptimizationCBF.envs.fr3_base_env import FR3BaseEnv


class ThreeBlocksEnv(FR3BaseEnv):
    def __init__(
        self,
        render_mode="human",
        record_path=None,
        dt=1 / 1000,
        crude_type="capsule",
    ):
        super().__init__(
            render_mode=render_mode,
            record_path=record_path,
            dt=dt,
            crude_type=crude_type,
        )

        datapath = getDataPath()
        p.setAdditionalSearchPath(datapath + "/robots")

        # load obstacle
        block1 = p.loadURDF("square_obstacle.urdf", useFixedBase=True)
        block1_pos = [0.55, 0.1, 0.7]
        p.resetBasePositionAndOrientation(block1, block1_pos, [0, 0, 0, 1])

        block2 = p.loadURDF("square_obstacle.urdf", useFixedBase=True)
        block2_pos = [0.65, -0.1, 0.3]
        p.resetBasePositionAndOrientation(block2, block2_pos, [0, 0, 0, 1])

        block3 = p.loadURDF("square_obstacle.urdf", useFixedBase=True)
        block3_pos = [0.75, 0.0, 0.1]
        p.resetBasePositionAndOrientation(block3, block3_pos, [0, 0, 0, 1])
