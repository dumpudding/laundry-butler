"""Authoritative Laundry Butler dual-Piper pi0.5 config used for inference.

Copied from the DGX training implementation so workstation inference reconstructs
exactly the same data transforms as training without modifying upstream OpenPI.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import flax.nnx as nnx

import openpi.models.pi0_config as pi0_config
import openpi.training.config as training_config
import openpi.training.optimizer as optimizer
import openpi.training.weight_loaders as weight_loaders
import openpi.transforms as transforms


CONFIG_NAME = "pi05_piper_joint_full"


def _repack_transforms() -> transforms.Group:
    return transforms.Group(
        inputs=[
            transforms.RepackTransform(
                {
                    "images": {
                        "cam_high": "observation.images.front",
                        "cam_left_wrist": "observation.images.left",
                        "cam_right_wrist": "observation.images.right",
                    },
                    "state": "observation.state",
                    "actions": "action",
                }
            )
        ]
    )


def make_train_config(
    *,
    repo_id: str,
    base_checkpoint: str | Path,
    assets_base_dir: str | Path,
    checkpoint_base_dir: str | Path,
    exp_name: str,
    default_prompt: str,
    fsdp_devices: int = 1,
    wandb_enabled: bool = False,
    resume: bool = False,
    overwrite: bool = False,
) -> training_config.TrainConfig:
    base_checkpoint = Path(base_checkpoint).expanduser().resolve()
    params_path = base_checkpoint / "params"
    if not params_path.exists():
        raise FileNotFoundError(f"base checkpoint params not found: {params_path}")

    model = pi0_config.Pi0Config(pi05=True)
    config = training_config.TrainConfig(
        name=CONFIG_NAME,
        project_name="pi05-piper",
        exp_name=exp_name,
        model=model,
        weight_loader=weight_loaders.CheckpointWeightLoader(str(params_path)),
        pytorch_training_precision="bfloat16",
        lr_schedule=optimizer.CosineDecaySchedule(
            warmup_steps=1_000,
            peak_lr=2.5e-5,
            decay_steps=30_000,
            decay_lr=2.5e-6,
        ),
        optimizer=optimizer.AdamW(
            b1=0.9,
            b2=0.95,
            eps=1e-8,
            weight_decay=1e-10,
            clip_gradient_norm=1.0,
        ),
        ema_decay=0.99,
        freeze_filter=nnx.Nothing(),
        data=training_config.LeRobotAlohaDataConfig(
            repo_id=repo_id,
            adapt_to_pi=False,
            use_delta_joint_actions=True,
            default_prompt=default_prompt,
            repack_transforms=_repack_transforms(),
        ),
        assets_base_dir=str(Path(assets_base_dir).expanduser().resolve()),
        checkpoint_base_dir=str(Path(checkpoint_base_dir).expanduser().resolve()),
        seed=42,
        batch_size=32,
        num_workers=2,
        num_train_steps=30_000,
        log_interval=100,
        save_interval=1_000,
        keep_period=5_000,
        overwrite=overwrite,
        resume=resume,
        wandb_enabled=wandb_enabled,
        fsdp_devices=fsdp_devices,
        policy_metadata={
            "robot": "dual_piper",
            "raw_action_representation": "absolute_joint_position",
            "action_layout": "left_6j_gripper_right_6j_gripper",
            "eef_saved_in_dataset": False,
        },
    )
    assert config.model.action_dim == 32
    assert config.model.action_horizon == 50
    assert config.model.max_token_len == 200
    assert config.model.discrete_state_input is True
    return config
