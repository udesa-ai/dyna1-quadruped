#!/usr/bin/env python3
# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

"""Optuna-based reward weight sweep for smooth locomotion.

This script orchestrates multiple training runs using Optuna to automatically
search for the best reward weights. Each trial launches a separate Isaac Sim
process via train_trial.py to avoid simulator state issues.

Usage:
    python scripts/rsl_rl/sweep_rewards.py \
        --task=Dyna1-Flat-v0 \
        --num_envs=4096 \
        --n_trials=15 \
        --max_iterations=500

Resume a previous sweep:
    python scripts/rsl_rl/sweep_rewards.py \
        --task=Dyna1-Flat-v0 \
        --num_envs=4096 \
        --n_trials=10 \
        --max_iterations=500 \
        --study_name=reward_sweep   # same name as before
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import optuna


# ---------------------------------------------------------------------------
# CLI arguments (no Isaac Sim — this is just the orchestrator)
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(
    description="Optuna sweep for reward weights (orchestrator).",
    formatter_class=argparse.RawDescriptionHelpFormatter,
    epilog=__doc__,
)
parser.add_argument("--task", type=str, default="Dyna1-Flat-v0", help="Gym task name.")
parser.add_argument("--num_envs", type=int, default=4096, help="Number of environments.")
parser.add_argument("--seed", type=int, default=42, help="Base seed.")
parser.add_argument("--max_iterations", type=int, default=500, help="Training iterations per trial.")
parser.add_argument("--n_trials", type=int, default=15, help="Number of Optuna trials.")
parser.add_argument("--study_name", type=str, default="reward_sweep", help="Optuna study name (for resumability).")
parser.add_argument("--metric_window", type=int, default=100, help="Last N iterations to average for objective.")
parser.add_argument("--device", type=str, default="cuda:0", help="Device for training.")
args = parser.parse_args()


# ---------------------------------------------------------------------------
# Sweep configuration: parameters and ranges
# ---------------------------------------------------------------------------

SWEEP_PARAMS = {
    "action_rate_l2_weight": {
        "low": -0.1,
        "high": -0.005,
        "log": False,
        "description": "Penalizes rapid changes between consecutive actions",
    },
    "joint_deviation_weight": {
        "low": -0.3,
        "high": -0.03,
        "log": True,
        "description": "Penalizes deviation from default joint positions",
    },
    "feet_air_time_threshold": {
        "low": 0.2,
        "high": 0.6,
        "log": False,
        "description": "Minimum air time to reward (seconds)",
    },
    "ang_vel_xy_l2_weight": {
        "low": -1.0,
        "high": -0.05,
        "log": True,
        "description": "Penalizes body roll/pitch angular velocity",
    },
    "flat_orientation_l2_weight": {
        "low": -2.0,
        "high": -0.1,
        "log": True,
        "description": "Penalizes body tilting away from flat",
    },
}


def suggest_params(trial: optuna.Trial) -> dict:
    """Use Optuna to suggest parameter values for a trial."""
    params = {}
    for name, config in SWEEP_PARAMS.items():
        if config["log"]:
            # Log-scale requires positive values, so negate range, sample, re-negate
            val = -trial.suggest_float(name, -config["high"], -config["low"], log=True)
        else:
            val = trial.suggest_float(name, config["low"], config["high"])
        params[name] = val
    return params


def convert_optuna_params(optuna_params: dict) -> dict:
    """Convert Optuna's stored params (positive for log-scale) back to actual values."""
    actual = {}
    for k, v in optuna_params.items():
        if k in SWEEP_PARAMS and SWEEP_PARAMS[k]["log"]:
            actual[k] = -v  # re-negate log-scale params
        else:
            actual[k] = v
    return actual


def run_trial(trial: optuna.Trial) -> float:
    """Optuna objective: launch a subprocess to train and return mean reward."""

    # --- 1. Suggest parameters ---
    trial_params = suggest_params(trial)

    print("\n" + "=" * 70)
    print(f"  TRIAL {trial.number}")
    print("=" * 70)
    for k, v in trial_params.items():
        print(f"  {k}: {v:.6f}")
    print("=" * 70 + "\n")

    # --- 2. Set up file paths ---
    sweep_root = os.path.abspath(
        os.path.join("logs", "rsl_rl", "optuna_sweep", args.study_name)
    )
    trial_dir = os.path.join(sweep_root, f"trial_{trial.number:03d}")
    os.makedirs(trial_dir, exist_ok=True)

    params_file = os.path.join(trial_dir, "trial_params.json")
    result_file = os.path.join(trial_dir, "result.json")

    # Write trial params to JSON
    with open(params_file, "w") as f:
        json.dump(trial_params, f, indent=2)

    # --- 3. Launch training subprocess ---
    script_dir = os.path.dirname(os.path.abspath(__file__))
    train_script = os.path.join(script_dir, "train_trial.py")

    cmd = [
        sys.executable,
        train_script,
        f"--task={args.task}",
        f"--num_envs={args.num_envs}",
        f"--seed={args.seed + trial.number}",
        f"--max_iterations={args.max_iterations}",
        f"--params_file={params_file}",
        f"--result_file={result_file}",
        f"--log_dir={trial_dir}",
        f"--metric_window={args.metric_window}",
        "--headless",
    ]

    print(f"  Launching: {' '.join(cmd[:4])}...")
    print(f"  Log dir:   {trial_dir}")

    try:
        process = subprocess.run(
            cmd,
            cwd=os.path.dirname(script_dir),  # Run from project root
            timeout=7200,  # 2 hour timeout per trial
            capture_output=False,  # Let output flow to console
        )

        if process.returncode != 0:
            print(f"  [ERROR] Trial {trial.number} failed with return code {process.returncode}")
            return float("-inf")

    except subprocess.TimeoutExpired:
        print(f"  [ERROR] Trial {trial.number} timed out after 2 hours")
        return float("-inf")
    except Exception as e:
        print(f"  [ERROR] Trial {trial.number} failed: {e}")
        return float("-inf")

    # --- 4. Read result ---
    if not os.path.exists(result_file):
        print(f"  [ERROR] Result file not found: {result_file}")
        return float("-inf")

    with open(result_file, "r") as f:
        result = json.load(f)

    mean_reward = result.get("mean_reward", float("-inf"))
    print(f"\n  TRIAL {trial.number} RESULT: mean_reward = {mean_reward:.4f}\n")

    return mean_reward


def main():
    """Run the Optuna sweep."""
    # Create Optuna study with SQLite storage for resumability
    sweep_root = os.path.abspath(
        os.path.join("logs", "rsl_rl", "optuna_sweep", args.study_name)
    )
    os.makedirs(sweep_root, exist_ok=True)
    db_path = os.path.join(sweep_root, "study.db")
    storage_path = f"sqlite:///{db_path}"

    study = optuna.create_study(
        study_name=args.study_name,
        storage=storage_path,
        direction="maximize",
        load_if_exists=True,
        sampler=optuna.samplers.TPESampler(seed=args.seed),
    )

    # Check if resuming
    n_existing = len(study.trials)
    if n_existing > 0:
        print(f"\n  Resuming study with {n_existing} existing trials.")

    print("\n" + "=" * 70)
    print("  OPTUNA REWARD SWEEP")
    print("=" * 70)
    print(f"  Study:      {args.study_name}")
    print(f"  Trials:     {args.n_trials} (new) + {n_existing} (existing)")
    print(f"  Iters/trial: {args.max_iterations}")
    print(f"  Num envs:   {args.num_envs}")
    print(f"  Task:       {args.task}")
    print(f"  Device:     {args.device}")
    print(f"  Storage:    {db_path}")
    print("  Parameters:")
    for name, cfg in SWEEP_PARAMS.items():
        print(f"    {name}: [{cfg['low']}, {cfg['high']}] (log={cfg['log']})")
    print("=" * 70 + "\n")

    # Run optimization
    study.optimize(run_trial, n_trials=args.n_trials, show_progress_bar=True)

    # --- Print results ---
    best_actual = convert_optuna_params(study.best_params)

    print("\n" + "=" * 70)
    print("  SWEEP COMPLETE")
    print("=" * 70)
    print(f"\n  Best trial: #{study.best_trial.number}")
    print(f"  Best mean reward: {study.best_value:.4f}")
    print(f"\n  Best parameters (ready to copy into flat_env_cfg.py):")
    print(f"  ─────────────────────────────────────────────────────")
    for k, v in best_actual.items():
        desc = SWEEP_PARAMS.get(k, {}).get("description", "")
        print(f"    {k}: {v:.6f}  # {desc}")

    # Save best params
    best_params_path = os.path.join(sweep_root, "best_params.json")
    best_result = {
        "best_trial": study.best_trial.number,
        "best_mean_reward": study.best_value,
        "best_params": best_actual,
        "all_trials": [
            {
                "number": t.number,
                "value": t.value,
                "params": convert_optuna_params(t.params),
                "state": str(t.state),
            }
            for t in study.trials
        ],
    }
    with open(best_params_path, "w") as f:
        json.dump(best_result, f, indent=2)

    print(f"\n  Results saved to: {best_params_path}")
    print(f"  To visualize: optuna-dashboard {storage_path}")
    print("=" * 70 + "\n")


if __name__ == "__main__":
    main()
