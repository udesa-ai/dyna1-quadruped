# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Single-trial training script for the Optuna sweep.

This script is launched as a subprocess by sweep_rewards.py.
It reads reward parameters from a JSON file, applies them to the env config,
trains for N iterations, and writes the result metric to a JSON file.

NOT intended to be called directly — use sweep_rewards.py instead.
"""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train a single sweep trial.")
parser.add_argument("--num_envs", type=int, default=4096, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Dyna1-Flat-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=42, help="Seed used for the environment.")
parser.add_argument("--max_iterations", type=int, default=500, help="Training iterations.")
parser.add_argument("--params_file", type=str, required=True, help="JSON file with trial parameters.")
parser.add_argument("--result_file", type=str, required=True, help="JSON file to write result metric.")
parser.add_argument("--log_dir", type=str, required=True, help="Directory for training logs.")
parser.add_argument("--metric_window", type=int, default=100, help="Last N iterations to average.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
sys.argv = [sys.argv[0]] + [arg for arg in hydra_args if not arg.startswith("--/")]

"""Rest everything follows."""

import gymnasium as gym
import os
import json
import torch
from datetime import datetime

from rsl_rl.runners import OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.io import dump_pickle, dump_yaml
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper
from isaaclab_tasks.utils.hydra import hydra_task_config

# Import extensions to set up environment tasks
import DynabotIsaacTemplate.tasks  # noqa: F401

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False


def apply_trial_params(env_cfg, trial_params: dict):
    """Apply the sweep parameters to the env config."""
    env_cfg.rewards.action_rate_l2.weight = trial_params["action_rate_l2_weight"]
    env_cfg.rewards.joint_deviation.weight = trial_params["joint_deviation_weight"]
    env_cfg.rewards.feet_air_time.params["threshold"] = trial_params["feet_air_time_threshold"]
    env_cfg.rewards.ang_vel_xy_l2.weight = trial_params["ang_vel_xy_l2_weight"]
    env_cfg.rewards.flat_orientation_l2.weight = trial_params["flat_orientation_l2_weight"]


def extract_mean_reward(log_dir: str, metric_window: int) -> float:
    """Extract mean reward from TensorBoard logs or fallback to log files.

    Reads the TensorBoard events file to get the 'Episode/mean_reward' or
    'Train/mean_reward' scalar, and averages the last `metric_window` values.
    """
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

        # Find the events file
        event_files = [f for f in os.listdir(log_dir) if f.startswith("events.out.tfevents")]
        if not event_files:
            print("[WARNING] No TensorBoard events file found.")
            return float("-inf")

        ea = EventAccumulator(log_dir)
        ea.Reload()

        # Try different tag names that RSL-RL might use
        possible_tags = [
            "Episode/Return/mean",
            "Episode/mean_reward",
            "Train/mean_reward",
            "Episode_Return/mean",
            "Reward/mean",
        ]

        available_tags = ea.Tags().get("scalars", [])
        print(f"  Available TensorBoard tags: {available_tags}")

        for tag in possible_tags:
            if tag in available_tags:
                events = ea.Scalars(tag)
                if events:
                    values = [e.value for e in events]
                    window = min(metric_window, len(values))
                    mean_reward = sum(values[-window:]) / window
                    print(f"  Using tag '{tag}': {len(values)} data points, last {window} avg = {mean_reward:.4f}")
                    return mean_reward

        # If no known tag found, try the first available scalar tag
        if available_tags:
            # Find any reward-related tag
            reward_tags = [t for t in available_tags if "rew" in t.lower() or "return" in t.lower()]
            if reward_tags:
                tag = reward_tags[0]
                events = ea.Scalars(tag)
                values = [e.value for e in events]
                window = min(metric_window, len(values))
                mean_reward = sum(values[-window:]) / window
                print(f"  Using fallback tag '{tag}': avg = {mean_reward:.4f}")
                return mean_reward

        print(f"[WARNING] No reward tags found. Available: {available_tags}")
        return float("-inf")

    except ImportError:
        print("[WARNING] tensorboard not installed. Cannot read metrics.")
        return float("-inf")
    except Exception as e:
        print(f"[WARNING] Error reading TensorBoard logs: {e}")
        return float("-inf")


@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    """Train a single trial with modified reward weights."""

    # --- 1. Read trial parameters ---
    with open(args_cli.params_file, "r") as f:
        trial_params = json.load(f)

    print("\n" + "=" * 70)
    print("  SINGLE TRIAL TRAINING")
    print("=" * 70)
    for k, v in trial_params.items():
        print(f"  {k}: {v:.6f}")
    print("=" * 70 + "\n")

    # --- 2. Apply CLI overrides ---
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    agent_cfg.max_iterations = args_cli.max_iterations
    env_cfg.seed = args_cli.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # --- 3. Apply sweep parameters ---
    apply_trial_params(env_cfg, trial_params)

    # --- 4. Set up logging ---
    log_dir = os.path.abspath(args_cli.log_dir)
    os.makedirs(log_dir, exist_ok=True)

    # --- 5. Create environment ---
    env = gym.make(args_cli.task, cfg=env_cfg)

    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    env = RslRlVecEnvWrapper(env)

    # --- 6. Create runner and train ---
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)

    # Dump configs
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
    dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)

    # Train
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    # --- 7. Extract metric ---
    mean_reward = extract_mean_reward(log_dir, args_cli.metric_window)

    print(f"\n  RESULT: mean_reward = {mean_reward:.4f}\n")

    # --- 8. Write result ---
    result = {
        "mean_reward": mean_reward,
        "params": trial_params,
        "status": "completed",
    }
    with open(args_cli.result_file, "w") as f:
        json.dump(result, f, indent=2)

    # --- 9. Cleanup ---
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
