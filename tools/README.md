# flash_grptlk_audio

Interactive CLI tool for flashing grptlk firmware to nRF5340 Audio DK boards.

## Installation

```bash
# Install Python dependencies
pip install questionary

# Install nrfjprog (Nordic Command Line Tools)
# Download from: https://www.nordicsemi.com/products/development-tools/nrf-command-line-tools
# Or via package manager:
#   macOS: brew install nordicsemi/nrf-command-line-tools/nrf-command-line-tools
#   Linux: Download .deb from Nordic website
```

## Usage

```bash
cd grptlk/tools
python flash_grptlk_audio.py
```

The tool will:
1. Detect connected nRF5340 devices
2. Prompt for firmware configuration (interval, uplink strategy, target)
3. Present connected devices for selection (arrow keys + space to select multiple)
4. Flash selected devices in parallel and reset on completion

## Configuration Options

| Step | Options |
|------|---------|
| ISO Interval | 5ms (LC3Plus), 10ms (LC3) |
| Uplink Strategy | Fully random, Partly random, Occupation aware |
| Target | Broadcaster, Receiver |

## Binary Mapping

Binaries are automatically selected based on your configuration:

| Target | Interval | Uplink Strategy | Binary |
|--------|----------|-----------------|--------|
| Broadcaster | 5ms | Fully random | `grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_fully_random.hex` |
| Broadcaster | 5ms | Partly random | `grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_partly_random.hex` |
| Broadcaster | 5ms | Occupation aware | `grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_occupation_aware.hex` |
| Broadcaster | 10ms | Fully random | `grptlk_bcst_nrf5340_audio_dk_10ms_T2_fully_random.hex` |
| Broadcaster | 10ms | Partly random | `grptlk_bcst_nrf5340_audio_dk_10ms_T2_partly_random.hex` |
| Broadcaster | 10ms | Occupation aware | `grptlk_bcst_nrf5340_audio_dk_10ms_T2_occupation_aware.hex` |
| Receiver | 5ms | Fully random | `grptlk_recv_nrf5340_audio_dk_5ms_liblc3_fully_random.hex` |
| Receiver | 5ms | Partly random | `grptlk_recv_nrf5340_audio_dk_5ms_liblc3_partly_random.hex` |
| Receiver | 5ms | Occupation aware | `grptlk_recv_nrf5340_audio_dk_5ms_liblc3_partly_random.hex` |
| Receiver | 10ms | Fully random | `grptlk_recv_nrf5340_audio_dk_10ms_T2_fully_random.hex` |
| Receiver | 10ms | Partly random | `grptlk_recv_nrf5340_audio_dk_10ms_T2_partly_random.hex` |
| Receiver | 10ms | Occupation aware | `grptlk_recv_nrf5340_audio_dk_10ms_T2_partly_random.hex` |

**Note:** Receivers with "Occupation aware" use the `partly_random` binary (no dedicated occupation_aware binary for receivers).

## Requirements

- **nrfjprog** — Nordic Command Line Tools (nRF Command Line Tools)
- **questionary** — Interactive CLI prompts
- Connected nRF5340 Audio DK boards
- Pre-built binaries in `grptlk/samples/binaries/`

## Troubleshooting

**No devices detected**
```bash
nrfjprog --ids
```
Verify USB connections and board power (LED indicators).

**Binary not found**
```bash
west build -b nrf5340_audio_dk/nrf5340/cpuapp --sysbuild
```

**Flashing failed**
- Isolate by flashing one board at a time
- Check power supply stability
- Press board RESET and retry
- Use nrfjprog directly for verbose errors:
```bash
nrfjprog -f NRF53 --snr <SNR> --program <hex> --recover --verify
```
