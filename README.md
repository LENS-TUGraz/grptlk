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

## Relay Mode

Flash an nRF5340 DK as broadcaster using one of the prebuilt binaries in `samples/binaries/nrf5340dk/` (e.g. `5ms/bcst_nrf5340dk_5ms_2ch.hex`), then pair it with the matching receiver binary (e.g. `samples/binaries/nrf5340_audio_dk/5ms/fully_random/recv_nrf5340_audio_dk_5ms_fully_random.hex`).

In this configuration the broadcaster has no audio io. It forwards everything it receives on the uplink BIS directly (no LC3 decode, no mixing, no LC3 encode), giving the lowest possible end-to-end latency. The uplink selection variant of the receiver binary (`fully_random`, `partly_random`, `occupation_aware`) does not matter in this mode. A `10ms/` variant is available under the same folder structure.
