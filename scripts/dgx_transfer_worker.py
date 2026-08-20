#!/usr/bin/env python3

import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import time
from pathlib import Path


DATA_ROOT = Path(
    os.environ.get(
        "LAUNDRY_BUTLER_DATA_ROOT",
        "/home/laundrybutler/laundry-butler/data_collection/output",
    )
).expanduser()

DGX_HOST = os.environ.get("LB_DGX_HOST", "dgx")

DGX_WEEK = os.environ.get(
    "LB_DGX_WEEK",
    "/raid/dgxtest/laundry-butler/dxx_data/week_2026-08-17",
).rstrip("/")

SCAN_INTERVAL = float(os.environ.get("LB_TRANSFER_SCAN_SECONDS", "15"))

# KiB/s. 25,000 ~= 25 MB/s.
# Recording always takes priority regardless of this limit.
BW_LIMIT = int(os.environ.get("LB_TRANSFER_BWLIMIT", "25000"))


def log(message: str) -> None:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{stamp}] {message}", flush=True)


def read_json(path: Path):
    try:
        with path.open("r", encoding="utf-8") as f:
            value = json.load(f)
        return value if isinstance(value, dict) else None
    except (OSError, json.JSONDecodeError):
        return None


def episode_dirs():
    if not DATA_ROOT.is_dir():
        return []

    return sorted(
        (
            p
            for p in DATA_ROOT.iterdir()
            if p.is_dir()
            and p.name.startswith("episode_")
            and (p / "episode.json").is_file()
        ),
        key=lambda p: p.name,
    )


def recording_active() -> bool:
    """Recording gets absolute priority over DGX transfer."""
    for episode in episode_dirs():
        meta = read_json(episode / "episode.json")
        if meta and meta.get("status") == "recording":
            return True
    return False


def validation_status(episode: Path):
    validation = read_json(episode / "validation.json")
    if not validation:
        return None
    return str(validation.get("status", "")).lower()


def eligible_for_transfer(episode: Path) -> bool:
    meta = read_json(episode / "episode.json")
    if not meta:
        return False

    # Never touch an episode until ROS recording is fully closed.
    if meta.get("status") != "recorded":
        return False

    if meta.get("operator_disposition") != "usable":
        return False

    # Wait for the GUI validator to finish before transfer.
    if validation_status(episode) != "pass":
        return False

    if not (episode / "bag").is_dir():
        return False

    return True


def remote(command: str, check=True):
    return subprocess.run(
        ["ssh", DGX_HOST, command],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=check,
    )


def ensure_remote_layout() -> None:
    dirs = [
        f"{DGX_WEEK}/raw",
        f"{DGX_WEEK}/.incoming",
        f"{DGX_WEEK}/logs",
        f"{DGX_WEEK}/manifests",
    ]
    command = "mkdir -p " + " ".join(shlex.quote(x) for x in dirs)
    remote(command)


def remote_completed() -> set[str]:
    raw = shlex.quote(f"{DGX_WEEK}/raw")
    result = remote(
        f"find {raw} -mindepth 1 -maxdepth 1 -type d -printf '%f\\n' 2>/dev/null || true"
    )
    return {x.strip() for x in result.stdout.splitlines() if x.strip()}


def rsync_prefix():
    command = []

    # Lowest practical local disk priority.
    if shutil.which("ionice"):
        command += ["ionice", "-c", "3"]

    if shutil.which("nice"):
        command += ["nice", "-n", "15"]

    command += ["rsync"]
    return command


def sync_sidecars(episode: Path) -> None:
    """
    Keep disposition/validation metadata current even after the large
    MCAP transfer completed.

    This matters if an episode is later changed from usable -> unusable.
    """
    files = []

    for name in ("episode.json", "validation.json"):
        path = episode / name
        if path.is_file():
            files.append(str(path))

    if not files:
        return

    destination = f"{DGX_HOST}:{DGX_WEEK}/raw/{episode.name}/"

    command = rsync_prefix() + [
        "-a",
        "--quiet",
        *files,
        destination,
    ]

    subprocess.run(command, check=False)


def transfer_episode(episode: Path) -> bool:
    name = episode.name

    remote_tmp = f"{DGX_WEEK}/.incoming/{name}.partial"
    remote_final = f"{DGX_WEEK}/raw/{name}"

    log(
        f"TRANSFER START: {name} "
        f"(validation={validation_status(episode)})"
    )

    # Create/resume in .incoming. Nothing here is eligible for conversion.
    remote(f"mkdir -p {shlex.quote(remote_tmp)}")

    command = rsync_prefix() + [
        "-a",
        "--partial",
        "--append-verify",
        f"--bwlimit={BW_LIMIT}",
        "--info=progress2",
        f"{episode}/",
        f"{DGX_HOST}:{remote_tmp}/",
    ]

    process = subprocess.Popen(
        command,
        start_new_session=True,
    )

    while process.poll() is None:
        # If collection begins during rsync, immediately stop transfer.
        # Partial files remain in .incoming and will resume later.
        if recording_active():
            log(f"RECORDING STARTED: aborting transfer of {name}")

            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait()

            log(f"TRANSFER PAUSED: {name} will resume after recording")
            return False

        time.sleep(0.5)

    if process.returncode != 0:
        log(f"TRANSFER FAILED: {name}, rsync rc={process.returncode}")
        return False

    # Sanity check before exposing the episode under raw/.
    qtmp = shlex.quote(remote_tmp)
    qfinal = shlex.quote(remote_final)

    finalize = f"""
set -e
test -f {qtmp}/episode.json
test -f {qtmp}/validation.json
test -d {qtmp}/bag

if [ -d {qfinal} ]; then
    rm -rf {qtmp}
else
    mv {qtmp} {qfinal}
fi
"""

    result = remote(finalize, check=False)

    if result.returncode != 0:
        log(f"FINALIZE FAILED: {name}: {result.stdout.strip()}")
        return False

    log(f"TRANSFER COMPLETE: {name}")
    return True


def run_once() -> None:
    ensure_remote_layout()

    completed = remote_completed()
    episodes = episode_dirs()

    # First propagate tiny metadata changes for already transferred episodes.
    for episode in episodes:
        if episode.name in completed:
            sync_sidecars(episode)

    if recording_active():
        log("Recording active. DGX transfer idle.")
        return

    transferred_any = False

    for episode in episodes:
        if episode.name in completed:
            continue

        if not eligible_for_transfer(episode):
            continue

        if recording_active():
            log("Recording started. Stopping transfer scan.")
            break

        if transfer_episode(episode):
            completed.add(episode.name)
            transferred_any = True
        else:
            break

    if not transferred_any:
        log("No new usable validated episodes waiting for transfer.")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--once",
        action="store_true",
        help="Run one scan and exit.",
    )
    args = parser.parse_args()

    log(f"Local root: {DATA_ROOT}")
    log(f"DGX root: {DGX_HOST}:{DGX_WEEK}")

    if args.once:
        run_once()
        return

    while True:
        try:
            run_once()
        except KeyboardInterrupt:
            return
        except Exception as exc:
            log(f"ERROR: {type(exc).__name__}: {exc}")

        time.sleep(SCAN_INTERVAL)


if __name__ == "__main__":
    main()
