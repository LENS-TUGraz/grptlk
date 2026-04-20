# Generate GRPTLK Audio Binaries

The script `samples/generate_grptlk_audio_binaries.sh` builds all GRPTLK audio firmware variants, merges the application and HCI hex files, and organizes the output into an easy-to-navigate folder structure.

## What it does

1. Builds every broadcaster + receiver combination using `west build`
2. Merges each app hex with its `hci_ipc` hex via `mergehex`
3. Builds broadcaster variants for each channel count (5ch and 2ch)
4. Stores all raw build artifacts in `binaries/all_bins/`
5. Copies and renames binaries into per-device folder structure (3 binaries per folder)

## Output folder structure

```
binaries/
├── all_bins/                              # All 32 raw hex files (original names)
│
├── nrf5340_audio_dk/
│   ├── 5ms/
│   │   ├── fully_random/
│   │   │   ├── bcst_nrf5340_audio_dk_5ms_fully_random_5ch.hex
│   │   │   ├── bcst_nrf5340_audio_dk_5ms_fully_random_2ch.hex
│   │   │   └── recv_nrf5340_audio_dk_5ms_fully_random.hex
│   │   ├── partly_random/
│   │   │   ├── bcst_nrf5340_audio_dk_5ms_partly_random_5ch.hex
│   │   │   ├── bcst_nrf5340_audio_dk_5ms_partly_random_2ch.hex
│   │   │   └── recv_nrf5340_audio_dk_5ms_partly_random.hex
│   │   └── occupation_aware/
│   │       ├── bcst_nrf5340_audio_dk_5ms_occupation_aware_5ch.hex
│   │       ├── bcst_nrf5340_audio_dk_5ms_occupation_aware_2ch.hex
│   │       └── recv_nrf5340_audio_dk_5ms_occupation_aware.hex
│   └── 10ms/
│       ├── fully_random/
│       │   ├── bcst_nrf5340_audio_dk_10ms_fully_random_5ch.hex
│       │   ├── bcst_nrf5340_audio_dk_10ms_fully_random_2ch.hex
│       │   └── recv_nrf5340_audio_dk_10ms_fully_random.hex
│       ├── partly_random/
│       │   ├── bcst_nrf5340_audio_dk_10ms_partly_random_5ch.hex
│       │   ├── bcst_nrf5340_audio_dk_10ms_partly_random_2ch.hex
│       │   └── recv_nrf5340_audio_dk_10ms_partly_random.hex
│       └── occupation_aware/
│           ├── bcst_nrf5340_audio_dk_10ms_occupation_aware_5ch.hex
│           ├── bcst_nrf5340_audio_dk_10ms_occupation_aware_2ch.hex
│           └── recv_nrf5340_audio_dk_10ms_occupation_aware.hex
│
└── le_audio_playground/
    ├── 5ms/
    │   ├── fully_random/ ...       (3 hex files: 2 bcst + 1 recv)
    │   ├── partly_random/ ...      (3 hex files)
    │   └── occupation_aware/ ...   (3 hex files)
    └── 10ms/
        ├── fully_random/ ...       (3 hex files)
        ├── partly_random/ ...      (3 hex files)
        └── occupation_aware/ ...   (3 hex files)
```

## How to use

1. Navigate to `binaries/<your_device>/<timing>/<strategy>/`
2. Flash a `bcst_*_Xch.hex` file onto the broadcaster — pick `_5ch` or `_2ch`
3. Flash the `recv_*.hex` file onto the receiver device(s)

> **Note:** The broadcaster determines how many BISes exist in the BIG. The receiver adapts to whatever BIG is advertised.

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

### Channel counts (broadcaster only)

The broadcaster determines how many BISes the BIG contains. The receiver adapts automatically.

| Channels | Suffix | Description |
|----------|--------|-------------|
| `5ch` | `_5ch` | Broadcaster creates 5 BISes: 1 downlink + 4 uplink (default) |
| `2ch` | `_2ch` | Broadcaster creates 2 BISes: 1 downlink + 1 uplink |

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
