"""Locomotion environments with velocity-tracking commands.

These environments are based on the `legged_gym` environments provided by Rudin et al.

Reference:
    https://github.com/leggedrobotics/legged_gym
"""


from gymnasium.envs.registration import register

register(
    id="Dyna1-Rough-D",  # match the name used in train.py
    entry_point="DynabotIsaacTemplate.tasks.locomotion.velocity.env:AnymalDRoughEnvCfg",
    kwargs={
        "env_cfg_entry_point": "DynabotIsaacTemplate.tasks.locomotion.velocity.config.anymal_d.rough_env_cfg:AnymalDRoughEnvCfg",
        "agent_cfg_entry_point": "DynabotIsaacTemplate.tasks.locomotion.velocity.config.anymal_d.agents:AnymalDRoughEnvCfg",
    },
)

