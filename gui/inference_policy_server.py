#!/usr/bin/env python3
"""Serve a Laundry Butler pi0.5 checkpoint over OpenPI websocket inference."""

from __future__ import annotations

import argparse
from pathlib import Path

from openpi.policies import policy_config
from openpi.serving import websocket_policy_server

from piper_openpi_config import make_train_config


DEFAULT_CHECKPOINT = Path(
    "/home/laundrybutler/laundry-butler/checkpoints/piper_pi05_full_v1/15000"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--prompt", default="Fold the shirt.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    checkpoint = args.checkpoint.expanduser().resolve()
    if not (checkpoint / "params").is_dir():
        raise FileNotFoundError(f"Checkpoint params missing: {checkpoint / 'params'}")
    norm_stats = checkpoint / "assets/local/lerobot_168_absjoint_v21/norm_stats.json"
    if not norm_stats.is_file():
        raise FileNotFoundError(f"Checkpoint norm stats missing: {norm_stats}")

    config = make_train_config(
        repo_id="local/lerobot_168_absjoint_v21",
        base_checkpoint=checkpoint,
        assets_base_dir="/tmp/openpi_unused_assets",
        checkpoint_base_dir=checkpoint.parent.parent,
        exp_name=checkpoint.parent.name,
        default_prompt=args.prompt,
        fsdp_devices=1,
        wandb_enabled=False,
    )

    print(f"Loading checkpoint: {checkpoint}", flush=True)
    policy = policy_config.create_trained_policy(
        config,
        checkpoint,
        default_prompt=args.prompt,
    )
    print("POLICY_LOADED", flush=True)
    print(f"metadata: {policy.metadata}", flush=True)
    print(f"serving on {args.host}:{args.port}", flush=True)

    server = websocket_policy_server.WebsocketPolicyServer(
        policy=policy,
        host=args.host,
        port=args.port,
        metadata=policy.metadata,
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
