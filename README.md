# grptlk manifest

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

- `fully_random`: selects uplink resources fully at random.
- `partly_random`: keeps part of the uplink selection constrained and randomizes the rest.
- `occupation_aware`: selects uplink resources with current channel occupation in mind.

Each option is available with a `5 ms` or `10 ms` ISO interval.

Use `tools/flash_grptlk_audio.py` to flash a board; it lets you choose the target, ISO interval, and uplink option, then automatically selects the matching binary from `samples/binaries/`.

More detailed information about the different modes is available in `docs/`.
