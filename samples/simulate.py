"""Small CLI to build ISO broadcaster/receiver samples with west."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TARGETS = {
    "broadcaster": ROOT / "iso_broadcast",
    "receiver": ROOT / "iso_receive",
}
WEST_BASE_CMD = ["west", "build", "-p", "always", "-b", "nrf54l15bsim/nrf54l15/cpuapp"]
RECEIVER_MAIN = TARGETS["receiver"] / "src" / "main.c"
BIN_DIR = ROOT / "bin"
SIM_NAME = os.environ.get("BSIM_SIM_NAME", "sim")
RESULTS_DIR = Path("/config/ncs/try1/tools/bsim/results/sim")


def run_west_build(target_dir: Path, extra_args: list[str]) -> int:
    """Run the west build command in the given target directory."""
    cmd = WEST_BASE_CMD + extra_args
    try:
        subprocess.run(cmd, cwd=target_dir, check=True)
    except FileNotFoundError:
        print("west not found on PATH; install west before running builds.", file=sys.stderr)
        raise RuntimeError("west not found on PATH")
    except subprocess.CalledProcessError as exc:
        print(f"Build failed in {target_dir} (exit code {exc.returncode}).", file=sys.stderr)
        raise RuntimeError(f"Build failed in {target_dir}") from exc
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    argv = list(sys.argv[1:] if argv is None else argv)

    # Split off any west args that follow a bare '--'.
    west_args: list[str] = []
    if "--" in argv:
        idx = argv.index("--")
        west_args = argv[idx + 1 :]
        argv = argv[:idx]

    parser = argparse.ArgumentParser(
        description=(
            "Build ISO broadcaster and one or more receivers for specific BIS values.\n"
            "Anything after '--' is passed to 'west build' (default pristine)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python simulate.py BIS2 BIS3 BIS4 BIS5 -- -p always\n"
            "  python simulate.py --sim-length 6   # only broadcaster, 6s sim\n"
        ),
    )
    parser.add_argument(
        "bis_targets",
        nargs="*",
        help="BIS selections like 'BIS2' or '2'. Comma-separated values are also accepted.",
    )
    parser.add_argument(
        "--sim-length",
        type=float,
        default=4.0,
        help="Simulation length in seconds (default: 4.0).",
    )

    args = parser.parse_args(argv)
    args.west_args = west_args
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        plan = build_plan(args.bis_targets)
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1

    if args.sim_length <= 0:
        print("Simulation length must be positive.", file=sys.stderr)
        return 1
    sim_length_us = int(args.sim_length * 1_000_000)

    reset_bin_dir()

    extra_args = list(args.west_args or [])

    for target, bis in plan:
        target_dir = TARGETS[target]
        if not target_dir.is_dir():
            print(f"Expected directory missing: {target_dir}", file=sys.stderr)
            return 1

        if target == "receiver" and bis is not None:
            if bis < 1:
                print("BIS number must be >= 1", file=sys.stderr)
                return 1
            update_receiver_source(bis)

        try:
            run_west_build(target_dir, extra_args)
            copy_artifact(target, bis)
        except (RuntimeError, FileNotFoundError) as exc:
            print(exc, file=sys.stderr)
            return 1

    dest_files: list[Path] = []
    try:
        dest_files = ship_outputs()
        clear_sim_results()
        run_simulation(plan, sim_length_us)
    except RuntimeError as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        if dest_files:
            cleanup_outputs(dest_files)

    return 0


def update_receiver_source(bis_number: int) -> None:
    """Patch iso_receive/src/main.c to use the requested BIS."""
    if not RECEIVER_MAIN.is_file():
        print(f"Receiver source not found: {RECEIVER_MAIN}", file=sys.stderr)
        sys.exit(1)

    payload_seed = bis_number * 0x10
    payload_limit = bis_number
    bis_index = bis_number - 1

    text = RECEIVER_MAIN.read_text()

    def replace_once(pattern: str, replacement: str, label: str) -> str:
        new_text, count = re.subn(pattern, replacement, text, count=1, flags=re.MULTILINE)
        if count != 1:
            print(f"Expected pattern not found for {label} in {RECEIVER_MAIN}", file=sys.stderr)
            sys.exit(1)
        return new_text

    text = replace_once(
        r"(?m)^(\s*)memset\(iso_data,\s*0x[0-9a-fA-F]+\s+\+\s+payload_ctr,\s*sizeof\(iso_data\)\);",
        rf"\1memset(iso_data, 0x{payload_seed:x} + payload_ctr, sizeof(iso_data));",
        "payload seed",
    )
    text = replace_once(
        r"(?m)^(\s*)if\s*\(\s*\+\+payload_ctr\s*>\s*0x[0-9a-fA-F]+\s*\)",
        rf"\1if (++payload_ctr > 0x{payload_limit:x})",
        "payload limit",
    )
    text = replace_once(
        r"(?m)^(\s*)iso_sent\(bis\[\d+\]\);",
        rf"\1iso_sent(bis[{bis_index}]);",
        "BIS index",
    )

    RECEIVER_MAIN.write_text(text)


def build_plan(bis_tokens: list[str]) -> list[tuple[str, int | None]]:
    """Build plan always starts with broadcaster, then one receiver per BIS token."""
    plan: list[tuple[str, int | None]] = [("broadcaster", None)]

    expanded: list[str] = []
    for token in bis_tokens:
        expanded.extend([part for part in token.split(",") if part.strip()])

    for raw in expanded:
        match = re.match(r"(?i)(?:bis)?(\d+)$", raw.strip())
        if not match:
            raise ValueError(f"Unrecognized BIS token '{raw}'. Use forms like 'BIS3' or '3'.")
        bis_num = int(match.group(1))
        if bis_num < 1:
            raise ValueError("BIS number must be >= 1.")
        plan.append(("receiver", bis_num))

    return plan


def copy_artifact(target: str, bis_number: int | None) -> None:
    """Copy the built zephyr.exe into the sibling bin/ folder with a friendly name."""
    BIN_DIR.mkdir(exist_ok=True)

    src_dir = TARGETS[target] / "build" / TARGETS[target].name / "zephyr"
    src = src_dir / "zephyr.exe"
    if not src.exists():
        raise FileNotFoundError(f"Build artifact not found at {src}")

    if target == "broadcaster":
        dest = BIN_DIR / "broadcaster.exe"
    else:
        dest = BIN_DIR / f"receiver_{bis_number}.exe"

    shutil.copy2(src, dest)


def reset_bin_dir() -> None:
    """Remove any existing bin directory and recreate it."""
    if BIN_DIR.exists():
        shutil.rmtree(BIN_DIR)
    BIN_DIR.mkdir()


def ship_outputs() -> list[Path]:
    """Copy all .exe files from bin/ to ${BSIM_OUT_PATH}/bin/ and remove bin/."""
    bsim_out = os.environ.get("BSIM_OUT_PATH")
    if not bsim_out:
        raise RuntimeError("BSIM_OUT_PATH is not set; cannot copy build artifacts.")

    if not BIN_DIR.is_dir():
        raise RuntimeError(f"Local bin directory missing: {BIN_DIR}")

    src_files = list(BIN_DIR.glob("*.exe"))
    if not src_files:
        raise RuntimeError(f"No .exe files found in {BIN_DIR}")

    dest_dir = Path(bsim_out) / "bin"
    dest_dir.mkdir(parents=True, exist_ok=True)

    copied: list[Path] = []
    for exe in src_files:
        dest = dest_dir / exe.name
        shutil.copy2(exe, dest)
        copied.append(dest)

    shutil.rmtree(BIN_DIR)

    return copied


def run_simulation(plan: list[tuple[str, int | None]], sim_length_us: int) -> None:
    """Launch babblesim with the built broadcaster and receivers."""
    bsim_out = os.environ.get("BSIM_OUT_PATH")
    if not bsim_out:
        raise RuntimeError("BSIM_OUT_PATH is not set; cannot launch simulation.")

    bin_dir = Path(bsim_out) / "bin"
    devices: list[tuple[Path, int]] = []

    # Broadcaster is always device 0.
    devices.append((bin_dir / "broadcaster.exe", 0))

    # Receivers follow sequentially.
    next_dev_id = 1
    for target, bis in plan:
        if target != "receiver" or bis is None:
            continue
        devices.append((bin_dir / f"receiver_{bis}.exe", next_dev_id))
        next_dev_id += 1

    for exe_path, _ in devices:
        if not exe_path.exists():
            raise RuntimeError(f"Simulation binary missing: {exe_path}")

    phy = bin_dir / "bs_2G4_phy_v1"
    if not phy.exists():
        raise RuntimeError(f"Expected PHY binary missing: {phy}")

    procs: list[subprocess.Popen] = []
    phy_proc: subprocess.Popen | None = None
    try:
        for exe_path, dev_id in devices:
            cmd = [str(exe_path), f"-s={SIM_NAME}", f"-d={dev_id}"]
            procs.append(subprocess.Popen(cmd, cwd=bin_dir))

        phy_cmd = [str(phy), f"-s={SIM_NAME}", f"-D={len(devices)}", f"-sim_length={sim_length_us}"]
        phy_proc = subprocess.Popen(phy_cmd, cwd=bin_dir)

        for proc in procs:
            proc.wait()
        phy_proc.wait()

        bad = [proc for proc in procs + [phy_proc] if proc and proc.returncode != 0]
        if bad:
            raise RuntimeError(f"Simulation failed; non-zero return codes: {[p.returncode for p in bad]}")
    finally:
        for proc in procs:
            if proc.poll() is None:
                proc.terminate()
        if phy_proc and phy_proc.poll() is None:
            phy_proc.terminate()


def cleanup_outputs(paths: list[Path]) -> None:
    """Delete copied binaries from ${BSIM_OUT_PATH}/bin/."""
    for path in paths:
        try:
            path.unlink()
        except FileNotFoundError:
            continue


def clear_sim_results() -> None:
    """Delete previous simulation results before running a new sim."""
    if RESULTS_DIR.exists():
        shutil.rmtree(RESULTS_DIR)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)


if __name__ == "__main__":
    sys.exit(main())
