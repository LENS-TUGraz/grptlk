# GRPTLK Audio Workspace

This repository is the west manifest and sample workspace for the GRPTLK audio setup. It contains the GRPTLK audio broadcaster and receiver samples, prebuilt nRF5340 Audio DK binaries, the flashing helper script, and short mode descriptions in `docs/`.

## Workspace Setup

Use this repository as the local west manifest for the workspace. The commands below keep the original setup variants and fetch the matching GRPTLK-enabled `sdk-nrf` with `west update`.

``` sh
git clone git@github.com:LENS-TUGraz/grptlk.git grptlk
west update
```

or

``` sh
git clone git@github.com:LENS-TUGraz/grptlk.git grptlk
west init -l grptlk 
west update
```

## Flashing Prebuilt Audio Binaries

Prebuilt grptlk audio binaries for the nRF5340 Audio DK are available in `samples/binaries/`.

- `fully_random`: the receiver picks a random uplink BIS on every transmit interval.
- `partly_random`: the receiver picks one random uplink BIS per PTT session and keeps it until PTT is released.
- `occupation_aware`: the receiver uses the downlink occupancy information to avoid uplink BISes that are already in use.

Each option is available with a `5 ms` or `10 ms` ISO interval.

Use `tools/flash_grptlk_audio.py` to flash a board; it lets you choose the target, ISO interval, and uplink option, then automatically selects the matching binary from `samples/binaries/`.

More detailed information about the different modes is available in `docs/`.
