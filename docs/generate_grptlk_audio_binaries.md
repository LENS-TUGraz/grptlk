# Generate GRPTLK Audio Binaries

The script `samples/generate_grptlk_audio_binaries.sh` builds all GRPTLK audio firmware variants, merges the application and HCI hex files, and organizes the output into an easy-to-navigate folder structure.

## What it does

1. Builds every broadcaster + receiver combination using `west build`
2. Merges each app hex with its `hci_ipc` hex via `mergehex`
3. Stores all raw build artifacts in `binaries/all_bins/`
4. Copies and renames binaries into per-device folder structure

## Output folder structure

```
binaries/
├── all_bins/                              # All 20 raw hex files (original names)
│
├── nrf5340_audio_dk/
│   ├── 5ms/
│   │   ├── fully_random/
│   │   │   ├── bcst_nrf5340_audio_dk_5ms_fully_random.hex
│   │   │   └── recv_nrf5340_audio_dk_5ms_fully_random.hex
│   │   ├── partly_random/
│   │   │   ├── bcst_nrf5340_audio_dk_5ms_partly_random.hex
│   │   │   └── recv_nrf5340_audio_dk_5ms_partly_random.hex
│   │   └── occupation_aware/
│   │       ├── bcst_nrf5340_audio_dk_5ms_occupation_aware.hex
│   │       └── recv_nrf5340_audio_dk_5ms_occupation_aware.hex
│   └── 10ms/
│       ├── fully_random/
│       │   ├── bcst_nrf5340_audio_dk_10ms_fully_random.hex
│       │   └── recv_nrf5340_audio_dk_10ms_fully_random.hex
│       ├── partly_random/
│       │   ├── bcst_nrf5340_audio_dk_10ms_partly_random.hex
│       │   └── recv_nrf5340_audio_dk_10ms_partly_random.hex
│       └── occupation_aware/
│           ├── bcst_nrf5340_audio_dk_10ms_occupation_aware.hex
│           └── recv_nrf5340_audio_dk_10ms_occupation_aware.hex
│
└── le_audio_playground/
    ├── 5ms/
    │   ├── fully_random/ ...
    │   ├── partly_random/ ...
    │   └── occupation_aware/ ...
    └── 10ms/
        ├── fully_random/ ...
        ├── partly_random/ ...
        └── occupation_aware/ ...
```

## How to use

1. Navigate to `binaries/<your_device>/<timing>/<strategy>/`
2. Flash the `bcst_*.hex` file onto the broadcaster device
3. Flash the `recv_*.hex` file onto the receiver device(s)

## Configurations

### Devices

| Device | Description |
|--------|-------------|
| `nrf5340_audio_dk` | Nordic nRF5340 Audio Development Kit |
| `le_audio_playground` | LE Audio Playground board (same SoC, different board overlay + conf) |

### Frame durations

| Timing | Description |
|--------|-------------|
| `5ms` | 5 ms audio frame interval |
| `10ms` | 10 ms audio frame interval |

### Uplink strategies

| Strategy | Broadcaster Kconfig | Receiver Kconfig | Description |
|----------|-------------------|------------------|-------------|
| `fully_random` | `GRPTLK_INIT_LC3_CONSTANTLY=y` | `GRPTLK_UPLINK_RANDOM=y` | Receiver picks a random uplink BIS every transmit interval. Broadcaster must init LC3 constantly because the same talker may arrive on a different BIS each frame. |
| `partly_random` | _(default)_ | `GRPTLK_UPLINK_RANDOM_PER_PTT=y` _(default)_ | Receiver picks a random uplink BIS when PTT starts and keeps it for the entire PTT session. Next PTT press picks again. |
| `occupation_aware` | `GRPTLK_DOWNLINK_APPENDIX=y` | _(same as partly_random)_ | Broadcaster adds 4 extra bytes to each downlink packet on BIS1, telling receivers which uplink BISes are currently occupied. Receivers avoid choosing an occupied BIS. |

### Note on occupation_aware receiver

The `occupation_aware` strategy only changes the broadcaster firmware (adding the downlink appendix). The receiver binary is identical to the `partly_random` receiver since the occupation-awareness logic lives entirely on the broadcast side. The script copies the `partly_random` receiver into the `occupation_aware` folder for convenience.

## Prerequisites

- `west` (nRF Connect SDK toolchain)
- `mergehex` (nRF Command Line Tools)
