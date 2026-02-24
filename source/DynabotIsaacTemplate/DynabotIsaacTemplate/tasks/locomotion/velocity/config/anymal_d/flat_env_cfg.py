from isaaclab.utils import configclass

from .rough_env_cfg import AnymalDRoughEnvCfg


@configclass
class AnymalDFlatEnvCfg(AnymalDRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # override rewards
        self.rewards.flat_orientation_l2.weight = -0.36390231323872757
        self.rewards.dof_torques_l2.weight = 0 #-2.5e-5
        self.rewards.feet_air_time.weight = 0.4
        self.rewards.feet_air_time.params["threshold"] = 0.22783556227609314
        self.rewards.action_rate_l2.weight = -0.08350033980165854
        self.rewards.joint_deviation.weight = -0.03735789059384906
        self.rewards.ang_vel_xy_l2.weight = -0.0659719844413108
        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None


class AnymalDFlatEnvCfg_PLAY(AnymalDFlatEnvCfg):
    def __post_init__(self) -> None:
        # post init of parent
        super().__post_init__()

        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
        # remove random pushing
        self.events.base_external_force_torque = None
        self.events.push_robot = None
