"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import torch

from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import DirectMARLEnv, multi_agent_to_single_agent
from isaaclab.utils.dict import print_dict
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx
from isaaclab_tasks.utils import get_checkpoint_path, parse_env_cfg

# Import extensions to set up environment tasks
#import ext_template.tasks  # noqa: F401
import DynabotIsaacTemplate.tasks  # noqa: F401

def main():
    """Play with RSL-RL agent."""
    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    agent_cfg: RslRlOnPolicyRunnerCfg = cli_args.parse_rsl_rl_cfg(args_cli.task, args_cli)

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
    log_dir = os.path.dirname(resume_path)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    ppo_runner.load(resume_path)

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)
    # ppo_runner.alg.policy
    # export policy to onnx/jit
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(
        ppo_runner.alg.policy, ppo_runner.obs_normalizer, path=export_model_dir, filename="policy.pt"
    )
    export_policy_as_onnx(
        ppo_runner.alg.policy, normalizer=ppo_runner.obs_normalizer, path=export_model_dir, filename="policy.onnx"
    )


# 
# /workspace/isaaclab/_isaac_sim/kit/python/lib/python3.10/site-packages/rsl_rl/modules/actor_critic.py
# workspace/isaaclab/source/isaaclab/isaaclab/envs/mdp/actions/joint_actions.py
# /workspace/isaaclab/_isaac_sim/kit/python/lib/python3.10/site-packages/rsl_rl/modules/actor_critic.py(128)act_inference()
# /workspace/isaaclab/source/isaaclab/isaaclab/envs/mdp/actions/joint_actions.py

    # reset environment
    obs, _ = env.get_observations()
    timestep = 0
    i=0
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            
            #for i in range(10):
            #    obs[i][8]=0.0
            #    obs[i][9]=1.0
            #    obs[i][10]=0.0

            
            actions = policy(obs) 
            #print("Raw policy actions:", actions)
            #print("Scaled applied actions:", env.action_manager.get_applied_actions())

            """
            +-----------+---------------------------------+-----------+
            |   Index   | Name                            |   Shape   |
            +-----------+---------------------------------+-----------+
            |     0     | base_lin_vel                    |    (3,)   |  0 1 2
            |     1     | base_ang_vel                    |    (3,)   |  3 4 5
            |     2     | pitch_roll                      |    (2,)   |  6 7
            |     3     | velocity_commands               |    (3,)   |  8 9 10 
            |     4     | joint_pos                       |   (12,)   |  11 12 13 14 15 16 17 18 19 20 21 22
            |     5     | joint_vel                       |   (12,)   |  23 24,25 26 27 28 29, 30, 31, 32, 33, 34 
            |     6     | actions                         |   (12,)   |  
            +-----------+---------------------------------+-----------+
            """


            """actions[0][0]  = 0.0     # back_left_shoulder  11
            actions[0][1]  = 0.0                                       # back right_shoulder 12
            actions[0][2]  = 0.0                                        # front_left_shoulder 13 
            actions[0][3]  = 0.0                                        # front_right_shoulder 14
            actions[0][4]  = 0.0                                      # back_left_arm   15 
            actions[0][5]  = 0.0                                        # back_right_arm 
            actions[0][6]  = 0.0                                        # front_left_arm
            actions[0][7]  = 0.0                     # front_right_arm
            actions[0][8]  = 0.0                     # back_left_foot
            actions[0][9]  = 0.0                     # back_right_foot
            actions[0][10] = 0.0                     # front_left_foot
            actions[0][11] = 0.0                    # front_right_foot
            # env stepping
            actions[0][8]  = 1.0                     # back_left_foot
            actions[0][9]  = 0.0                     # back_right_foot
            actions[0][10] = 0.0  """
            obs, _, _, _ = env.step(actions)

            #print("Obs-acciones: ", obs[23:])
            #for i in range(12):
            #    print(f"joint {i}: {obs[0][i+11]}")
            #print("Action: ",f"{actions[0][4]}")
            #print(f"{float(obs[0][15])-0.79}")
            #i += 1

        if args_cli.video:
            timestep += 1
            # Exit the play loop after recording one video
            if timestep == args_cli.video_length:
                break

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
