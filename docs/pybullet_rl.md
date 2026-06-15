# PyBullet RL quick start

This is a small first reinforcement-learning loop for the robot arm. It trains a tabular Q-learning policy that nudges `j1,j2,j3` so `tool0` reaches a fixed target in PyBullet.

## Setup

```bash
make setup-pybullet-rl
```

The existing `.venv-ubuntu` is reused if present. The scripts only need `pybullet`, `numpy`, and the standard library.

## Train

```bash
make rl-train
```

Outputs:

- `outputs/rl_reacher_q_policy.json`
- `outputs/rl_reacher_training.csv`

Useful overrides:

```bash
.venv-ubuntu/bin/python tools/train_pybullet_reacher_q.py \
  --episodes 1500 \
  --target-x 30 --target-y -130 --target-z 250 \
  --active-joints j1,j2,j3 \
  --joint-step-deg 3
```

## Evaluate

```bash
make rl-eval
make rl-view
```

`rl-eval` runs headless and writes `outputs/rl_reacher_eval_trajectory.csv`. `rl-view` opens the PyBullet GUI and replays the learned policy.

## Notes

This is intentionally simple, so it is a learning scaffold rather than a final controller. Good next upgrades are random target sampling, continuous actions with Stable-Baselines3 SAC/PPO, collision penalties, and exporting successful joint trajectories to the existing OpenCR dry-run pipeline before any real execution.

## Stable-Baselines3 PPO

Install the heavier RL stack:

```bash
make setup-pybullet-sb3
```

Train PPO with continuous joint-delta actions:

```bash
make rl-sb3-train
```

Outputs are written under `outputs/sb3_reacher/`:

- `ppo_reacher.zip`
- `monitor.csv`
- `training_curves.png`

Evaluate headless or replay in the PyBullet GUI:

```bash
make rl-sb3-eval
make rl-sb3-view
```

Regenerate the learning-curve PNG from the Monitor log:

```bash
make rl-sb3-plot
```

For a quick smoke test, use fewer timesteps:

```bash
.venv-ubuntu/bin/python tools/train_sb3_pybullet_reacher.py --total-timesteps 2048
```
