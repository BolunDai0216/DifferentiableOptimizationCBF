import FR3Env
import pybullet as p
from DifferentiableOptimizationCBF import getDataPath
from FR3Env.fr3_bounding_box_env import FR3BoundingBoxSim


class FR3BaseEnv(FR3BoundingBoxSim):
    def __init__(
        self,
        render_mode="human",
        record_path=None,
        dt=1 / 1000,
        crude_type="capsule",
    ):
        super().__init__(render_mode=render_mode, record_path=record_path)

        # set time step
        self.dt = dt
        p.setTimeStep(self.dt)

        package_directory = FR3Env.getDataPath()
        urdf_search_path = package_directory + "/robots"
        p.setAdditionalSearchPath(urdf_search_path)

        datapath = getDataPath()
        p.setAdditionalSearchPath(datapath + "/robots")

        # load robot bounding primitives
        self.link3 = p.loadURDF(f"fr3_link3_{crude_type}.urdf", useFixedBase=True)
        p.resetBasePositionAndOrientation(self.link3, [2, 2, 2], [0, 0, 0, 1])

        self.link4 = p.loadURDF(f"fr3_link4_{crude_type}.urdf", useFixedBase=True)
        p.resetBasePositionAndOrientation(self.link4, [2, 2, 2], [0, 0, 0, 1])

        self.link5_1 = p.loadURDF(f"fr3_link5_1_{crude_type}.urdf", useFixedBase=True)
        p.resetBasePositionAndOrientation(self.link5_1, [2, 2, 2], [0, 0, 0, 1])

        self.link5_2 = p.loadURDF(f"fr3_link5_2_{crude_type}.urdf", useFixedBase=True)
        p.resetBasePositionAndOrientation(self.link5_2, [2, 2, 2], [0, 0, 0, 1])

        self.link6 = p.loadURDF(f"fr3_link6_{crude_type}.urdf", useFixedBase=True)
        p.resetBasePositionAndOrientation(self.link6, [2, 2, 2], [0, 0, 0, 1])

        self.link7 = p.loadURDF(f"fr3_link7_{crude_type}.urdf", useFixedBase=True)
        p.resetBasePositionAndOrientation(self.link7, [2, 2, 2], [0, 0, 0, 1])

        # load end-effector
        self.hand = p.loadURDF("fr3_hand_collision.urdf", useFixedBase=True)
        p.resetBasePositionAndOrientation(self.hand, [2, 2, 2], [0, 0, 0, 1])

    def step(self, τ):
        # send joint commands to motor
        info = super().step(τ)

        # update bounding primitives' configuration
        p.resetBasePositionAndOrientation(
            self.link3, info["P_LINK3"].tolist(), info["q_LINK3"].tolist()
        )
        p.resetBasePositionAndOrientation(
            self.link4, info["P_LINK4"].tolist(), info["q_LINK4"].tolist()
        )
        p.resetBasePositionAndOrientation(
            self.link5_1, info["P_LINK5_1"].tolist(), info["q_LINK5_1"].tolist()
        )
        p.resetBasePositionAndOrientation(
            self.link5_2, info["P_LINK5_2"].tolist(), info["q_LINK5_2"].tolist()
        )
        p.resetBasePositionAndOrientation(
            self.link6, info["P_LINK6"].tolist(), info["q_LINK6"].tolist()
        )
        p.resetBasePositionAndOrientation(
            self.link7, info["P_LINK7"].tolist(), info["q_LINK7"].tolist()
        )
        p.resetBasePositionAndOrientation(
            self.hand, info["P_HAND"].tolist(), info["q_HAND"].tolist()
        )

        return info
