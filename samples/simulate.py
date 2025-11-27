"""Small CLI to build ISO broadcaster/receiver samples with west."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
TARGETS = {
    "broadcaster": ROOT / "iso_broadcast",
    "receiver": ROOT / "iso_receive",
}
WEST_BASE_CMD = ["west", "build", "-b", "nrf54l15bsim/nrf54l15/cpuapp"]
RECEIVER_MAIN = TARGETS["receiver"] / "src" / "main.c"


def run_west_build(target_dir: Path, extra_args: list[str]) -> int:
    """Run the west build command in the given target directory."""
    cmd = WEST_BASE_CMD + extra_args
    try:
        subprocess.run(cmd, cwd=target_dir, check=True)
    except FileNotFoundError:
        print("west not found on PATH; install west before running builds.", file=sys.stderr)
        return 1
    except subprocess.CalledProcessError as exc:
        print(f"Build failed in {target_dir} (exit code {exc.returncode}).", file=sys.stderr)
        return exc.returncode
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run west builds for the ISO broadcaster or receiver samples.\n"
            "Extra arguments after '--' are passed directly to 'west build'."
        )
    )
    subparsers = parser.add_subparsers(dest="target", required=True)

    for name, folder in TARGETS.items():
        help_text = f"Build the ISO {name} sample in {folder.name}/"
        sub = subparsers.add_parser(name, help=help_text)
        if name == "receiver":
            sub.add_argument(
                "bis",
                nargs="?",
                type=int,
                help="Optional BIS number (1-indexed) the receiver uses for sending back.",
            )
        sub.add_argument("west_args", nargs=argparse.REMAINDER,
                         help="Additional arguments forwarded to 'west build' (prefix with '--').")

    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    target_dir = TARGETS[args.target]
    if not target_dir.is_dir():
        print(f"Expected directory missing: {target_dir}", file=sys.stderr)
        return 1

    bis_arg = getattr(args, "bis", None)
    if args.target == "receiver" and bis_arg is not None:
        if bis_arg < 1:
            print("BIS number must be >= 1", file=sys.stderr)
            return 1
        update_receiver_source(bis_arg)

    extra_args = list(args.west_args or [])
    if extra_args and extra_args[0] == "--":
        extra_args = extra_args[1:]

    return run_west_build(target_dir, extra_args)


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


if __name__ == "__main__":
    sys.exit(main())
