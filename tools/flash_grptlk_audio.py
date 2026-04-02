#!/usr/bin/env python3
"""
Interactive flashing script for grptlk firmware on nRF5340 Audio DK boards.
Flashes pre-built merged hex files to multiple boards in parallel.
"""

import sys
import os
import re
import subprocess
import signal
from pathlib import Path
from threading import Thread

try:
    from colorama import Fore, Style, init
    init(autoreset=True)
except ImportError:
    # Fallback if colorama is not installed
    class Fore:
        GREEN = RED = YELLOW = CYAN = MAGENTA = BLUE = ""
    class Style:
        RESET_ALL = BRIGHT = ""
    def init(autoreset=True):
        pass

# Check dependencies
MISSING_DEPS = []

try:
    import questionary
except ImportError:
    MISSING_DEPS.append("questionary")

def check_dependencies():
    """Check if all required dependencies are installed."""
    if MISSING_DEPS:
        print("❌ Missing required dependencies:")
        for dep in MISSING_DEPS:
            print(f"   • {dep}")
        print("\nInstall them with:")
        print(f"   pip install {' '.join(MISSING_DEPS)}")
        sys.exit(1)

# Run dependency check
check_dependencies()

# Binary directory path
BINARY_DIR = Path(__file__).parent.parent / "samples" / "binaries"

# Global flag for Ctrl+C handling
exit_requested = False

# Device status tracking
class DeviceStatus:
    PENDING = "PENDING"
    FLASHING = "FLASHING"
    DONE = "DONE"
    FAIL = "FAIL"

def signal_handler(sig, frame):
    """Handle Ctrl+C (SIGINT) gracefully."""
    global exit_requested
    exit_requested = True
    print_info("\n\nInterrupted by user (Ctrl+C). Exiting...")
    sys.exit(0)

class FlashDevice:
    def __init__(self, serial_number, index):
        self.serial_number = serial_number
        self.index = index
        self.status = DeviceStatus.PENDING

def print_header(text):
    print(f"\n{Fore.CYAN}{Style.BRIGHT}{'='*60}{Style.RESET_ALL}")
    print(f"{Fore.CYAN}{Style.BRIGHT}  {text}{Style.RESET_ALL}")
    print(f"{Fore.CYAN}{Style.BRIGHT}{'='*60}{Style.RESET_ALL}\n")

def print_success(text):
    print(f"{Fore.GREEN}✓ {text}{Style.RESET_ALL}")

def print_error(text):
    print(f"{Fore.RED}✗ {text}{Style.RESET_ALL}")

def print_info(text):
    print(f"{Fore.BLUE}ℹ {text}{Style.RESET_ALL}")

def get_connected_devices():
    """Get list of connected nRF5340 devices using nrfjprog."""
    try:
        output = subprocess.check_output(
            ["nrfjprog", "--ids"],
            stderr=subprocess.STDOUT,
            text=True
        )
        # nrfjprog --ids returns one ID per line, but may include error messages
        # Filter out error lines and only parse valid hex/decimal serial numbers
        serial_numbers = []
        for line in output.strip().split('\n'):
            line = line.strip()
            if not line:
                continue
            # Skip error/warning lines
            if line.startswith('[error]') or line.startswith('[warn]') or line.startswith('[info]'):
                continue
            # Try to parse as integer (handles both hex and decimal)
            try:
                serial_numbers.append(int(line))
            except ValueError:
                # Skip lines that aren't valid serial numbers
                continue

        return serial_numbers
    except subprocess.CalledProcessError as e:
        print_error(f"Failed to list devices: {e.output}")
        return []
    except FileNotFoundError:
        print_error("nrfjprog not found. Install from:")
        print("  https://www.nordicsemi.com/products/development-tools/nrf-command-line-tools")
        return []

def display_devices(devices):
    """Display connected devices in a numbered list."""
    if not devices:
        print_error("No nRF5340 devices found!")
        return False

    print_info(f"Found {len(devices)} connected device(s):")
    print()
    for i, dev in enumerate(devices, 1):
        print(f"  [{i}] Serial: {Fore.MAGENTA}{dev}{Style.RESET_ALL}")
    print()
    return True

def parse_device_selection(input_str, max_index):
    """Parse device selection string (e.g., '1,3,4', '1-3', 'all')."""
    input_str = input_str.strip().lower()

    if input_str == "all":
        return list(range(1, max_index + 1))

    selected = set()
    parts = input_str.split(",")

    for part in parts:
        part = part.strip()
        if "-" in part:
            # Range: e.g., "1-3"
            try:
                start, end = part.split("-")
                start, end = int(start.strip()), int(end.strip())
                selected.update(range(start, end + 1))
            except ValueError:
                return None
        else:
            # Single number
            try:
                selected.add(int(part))
            except ValueError:
                return None

    # Validate range
    result = sorted(selected)
    if any(i < 1 or i > max_index for i in result):
        return None

    return result

def select_option(prompt, options, show_back=False):
    """Display options and get user selection using arrow keys."""
    # Create questionary choices with separate display and return values
    choices = [
        questionary.Choice(title=label, value=key)
        for key, label in options.items()
    ]
    # Add back button if requested
    if show_back:
        choices.insert(0, questionary.Choice(title="← Back", value="__back__"))
    try:
        result = questionary.select(
            prompt,
            choices=choices,
            style=questionary.Style([
                ('qmark', 'fg:#67b7a1 bold'),
                ('question', 'fg:#ffffff bold'),
                ('selected', 'fg:#67b7a1 bold'),
                ('pointer', 'fg:#67b7a1 bold'),
                ('answer', 'fg:#f0932b bold'),
            ])
        ).ask()
        # questionary returns None on Ctrl+C
        if result is None:
            return None
        return result
    except KeyboardInterrupt:
        return None

def get_binary_filename(target_type, interval, uplink_strategy):
    """
    Generate binary filename based on selection.

    Args:
        target_type: 'broadcaster' or 'receiver'
        interval: '5ms' or '10ms'
        uplink_strategy: 'fully_random', 'partly_random', or 'occupation_aware'

    Returns:
        Filename string
    """
    # Map interval to codec
    if interval == "5ms":
        codec = "liblc3"
    else:  # 10ms
        codec = "T2"

    # Map target type to prefix
    prefix = "bcst" if target_type == "broadcaster" else "recv"

    # Map interval to directory name
    interval_suffix = "5ms_liblc3" if interval == "5ms" else "10ms_T2"

    # Special mapping: receiver + occupation_aware uses partly_random binary
    if target_type == "receiver" and uplink_strategy == "occupation_aware":
        uplink_suffix = "partly_random"
    else:
        uplink_suffix = uplink_strategy

    return f"grptlk_{prefix}_nrf5340_audio_dk_{interval_suffix}_{uplink_suffix}.hex"

def flash_device(device, binary_path):
    """
    Flash a single device using nrfjprog.

    Args:
        device: FlashDevice instance
        binary_path: Path to hex file to flash
    """
    device.status = DeviceStatus.FLASHING
    print(f"\n{Fore.CYAN}[{device.index}]{Style.RESET_ALL} Flashing {Fore.MAGENTA}{device.serial_number}{Style.RESET_ALL}...", flush=True)

    try:
        # Flash merged hex using nrfjprog with recover and verify
        cmd = [
            "nrfjprog",
            "-f", "NRF53",
            "--snr", str(device.serial_number),
            "--program", str(binary_path),
            "--recover",
            "--verify",
            "--reset"  # Reset device after flashing
        ]

        result = subprocess.run(
            cmd,
            timeout=120
        )

        if result.returncode == 0:
            device.status = DeviceStatus.DONE
            print(f"{Fore.GREEN}✓ DONE{Style.RESET_ALL}")
        else:
            device.status = DeviceStatus.FAIL
            print(f"{Fore.RED}✗ FAILED{Style.RESET_ALL}")

    except subprocess.TimeoutExpired:
        device.status = DeviceStatus.FAIL
        print(f"{Fore.RED}✗ TIMEOUT{Style.RESET_ALL}")
    except FileNotFoundError:
        device.status = DeviceStatus.FAIL
        print(f"{Fore.RED}✗ ERROR{Style.RESET_ALL}")
        print(f"       nrfjprog not found. Install from: https://www.nordicsemi.com/products/development-tools/nrf-command-line-tools")
    except Exception as e:
        device.status = DeviceStatus.FAIL
        print(f"{Fore.RED}✗ ERROR{Style.RESET_ALL}")
        print(f"       Exception: {str(e)[:100]}...")

def flash_devices_parallel(devices, binary_path):
    """Flash multiple devices in parallel using threads."""
    print(f"\n{Fore.CYAN}Flashing {len(devices)} device(s) with:{Style.RESET_ALL}")
    print(f"  {Fore.YELLOW}{binary_path.name}{Style.RESET_ALL}\n")

    threads = []
    for device in devices:
        thread = Thread(target=flash_device, args=(device, binary_path))
        threads.append(thread)
        thread.start()

    # Wait for all threads to complete
    for thread in threads:
        thread.join()

    # Print summary
    print()
    print_header("Flashing Summary")

    done_count = sum(1 for d in devices if d.status == DeviceStatus.DONE)
    fail_count = sum(1 for d in devices if d.status == DeviceStatus.FAIL)

    for device in devices:
        status_color = Fore.GREEN if device.status == DeviceStatus.DONE else Fore.RED
        status_symbol = "✓" if device.status == DeviceStatus.DONE else "✗"
        print(f"  {status_symbol} [{device.index}] Serial {device.serial_number}: {status_color}{device.status}{Style.RESET_ALL}")

    print()
    print(f"  {Fore.CYAN}Total: {done_count} succeeded, {fail_count} failed{Style.RESET_ALL}")
    print()

def interactive_flow():
    """Run the interactive flashing flow. Returns False if user cancelled, True if completed."""
    print_header("grptlk Interactive Flashing Tool")

    # Initialize variables
    interval = None
    uplink_strategy = None
    target_type = None
    selected_serials = []

    while True:
        # Step 1: Select ISO Interval (no back button)
        if interval is None:
            interval = select_option(
                "Step 1: Select ISO Interval",
                {
                    "5ms": "5ms (LC3Plus using liblc3)",
                    "10ms": "10ms (LC3 using T2)"
                }
            )
            if not interval:
                return False
            continue

        # Step 2: Select Uplink Strategy (back to step 1)
        if uplink_strategy is None:
            uplink_strategy = select_option(
                "Step 2: Select Uplink Strategy",
                {
                    "fully_random": "Fully random",
                    "partly_random": "Partly random",
                    "occupation_aware": "Occupation aware"
                },
                show_back=True
            )
            if uplink_strategy == "__back__":
                interval = None
                uplink_strategy = None
                continue
            if not uplink_strategy:
                return False
            continue

        # Step 3: Select Target (back to step 2)
        if target_type is None:
            target_type = select_option(
                "Step 3: Select Target",
                {
                    "broadcaster": "Broadcaster",
                    "receiver": "Receiver"
                },
                show_back=True
            )
            if target_type == "__back__":
                target_type = None
                uplink_strategy = None
                continue
            if not target_type:
                return False
            continue

        # Determine binary filename
        binary_filename = get_binary_filename(target_type, interval, uplink_strategy)
        binary_path = BINARY_DIR / binary_filename

        # Check if binary exists
        if not binary_path.exists():
            print_error(f"Binary not found: {binary_filename}")
            print_info(f"Expected location: {binary_path}")
            print()
            print("Available binaries:")
            try:
                for hex_file in sorted(BINARY_DIR.glob("*.hex")):
                    print(f"  • {hex_file.name}")
            except Exception:
                print("  (unable to list binaries)")
            return False

        print_success(f"Binary selected: {binary_filename}")

        # Step 4: List connected devices
        print_header("Step 4: Select Devices to Flash")

        connected_serials = get_connected_devices()
        if not display_devices(connected_serials):
            return False

        # Step 5: Get device selection (back to step 3)
        device_choices = [f"Device {i} - Serial: {snr}" for i, snr in enumerate(connected_serials, 1)]
        while True:
            # Add back button to device choices
            choices_with_back = ["← Back to target selection"] + device_choices

            try:
                selected_devices = questionary.checkbox(
                    "Step 5: Select devices to flash (use arrow keys + space to select, enter to confirm)",
                    choices=choices_with_back,
                    style=questionary.Style([
                        ('qmark', 'fg:#67b7a1 bold'),
                        ('question', 'fg:#ffffff bold'),
                        ('selected', 'fg:#67b7a1 bold'),
                        ('pointer', 'fg:#67b7a1 bold'),
                        ('answer', 'fg:#f0932b bold'),
                    ])
                ).ask()

                # questionary returns None on Ctrl+C
                if selected_devices is None:
                    print_info("\nOperation cancelled.")
                    return False

                # Check if back was selected
                if "← Back to target selection" in selected_devices:
                    target_type = None
                    break

                if not selected_devices:
                    # Stay on this step until user selects something
                    continue

                # Filter out the back button if it was selected along with devices
                selected_devices = [d for d in selected_devices if d != "← Back to target selection"]

                selected_indices = [device_choices.index(d) + 1 for d in selected_devices]
                selected_serials = [connected_serials[i-1] for i in selected_indices]
                break

            except KeyboardInterrupt:
                print_info("\nOperation cancelled.")
                return False

        # If back was pressed, restart the loop to go back to step 3
        if target_type is None:
            continue

        # Confirm selection (back to step 5)
        print(f"\n{Fore.CYAN}Configuration Summary:{Style.RESET_ALL}")
        print(f"  Target:      {target_type}")
        print(f"  Interval:    {interval}")
        print(f"  Uplink:      {uplink_strategy}")
        print(f"  Binary:      {binary_filename}")
        print(f"  Devices:     {len(selected_serials)} device(s)")
        for i, snr in enumerate(selected_serials, 1):
            print(f"    [{i}] Serial: {snr}")
        print()

        # Use a select menu for confirm with back option
        confirm_choice = select_option(
            "Proceed with flashing?",
            {
                "yes": "Yes, flash now",
                "__back__": "← Back to device selection"
            }
        )
        if confirm_choice == "__back__":
            selected_serials = []
            continue
        if not confirm_choice or confirm_choice != "yes":
            print_info("Flashing cancelled.")
            return False

        # Step 6: Flash devices
        print_header("Flashing Devices")

        # Create FlashDevice objects
        devices = [FlashDevice(snr, i) for i, snr in enumerate(selected_serials, 1)]

        # Flash in parallel
        flash_devices_parallel(devices, binary_path)

        # Ask if user wants to flash more devices
        try:
            continue_choice = select_option(
                "Flash more devices?",
                {
                    "no": "No, exit",
                    "yes": "Yes, flash more"
                }
            )
            if continue_choice == "no" or not continue_choice:
                print_info("Exiting.")
                return False
            # If yes, restart the entire flow by resetting all variables
            interval = None
            uplink_strategy = None
            target_type = None
            selected_serials = []
        except KeyboardInterrupt:
            print_info("\nExiting.")
            return False

def main():
    """Main entry point."""
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    try:
        while True:
            result = interactive_flow()
            if result is False:
                # User cancelled, exit instead of looping
                print_info("Exiting.")
                sys.exit(0)
            # If result is True, user flashed successfully and chose to flash more
            # The flow will restart automatically
    except KeyboardInterrupt:
        print()
        print_info("\nInterrupted by user.")
        sys.exit(0)
    except Exception as e:
        print_error(f"Unexpected error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
