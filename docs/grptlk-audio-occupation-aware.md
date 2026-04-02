# GRPTLK Audio Occupation Aware

This note explains the GRPTLK audio "occupation aware" behavior.

Internally, it is enabled by `CONFIG_GRPTLK_DOWNLINK_APPENDIX`, but the actual purpose is simple:

- the broadcaster adds 4 extra bytes to each downlink packet on `BIS1`
- those 4 bytes tell receivers which uplink BISes are currently occupied
- a receiver can then avoid choosing an uplink BIS that is already in use

## What the 4 bytes contain

The appendix is one little-endian `uint32_t`.

- bit 0 = uplink 1 = `BIS2`
- bit 1 = uplink 2 = `BIS3`
- bit 2 = uplink 3 = `BIS4`
- and so on

`BIS1` is not included because `BIS1` is the downlink stream.

This sample supports at most 30 uplink BISes:

- a BIG can contain at most 31 BIS total
- `BIS1` is used for downlink
- so at most 30 remain for uplink

Example:

- `000000000000000000000000000001` means uplink 1 / `BIS2` is occupied
- `010000000000000000000000000000` means uplink 2 / `BIS3` is occupied

## When the appendix is present

The broadcaster controls it with `CONFIG_GRPTLK_DOWNLINK_APPENDIX`.

- 5 ms frame profile: `20 + 4 = 24` bytes
- 10 ms frame profile: `40 + 4 = 44` bytes

If the option is off, the downlink packet contains only LC3 audio.

## When bits are set and cleared

On the broadcaster side, a bit is set when that uplink BIS delivered a valid packet in the latest finalized interval.

Bits are not cleared immediately. They are held for 6 finalized intervals after the last valid packet so the occupancy view does not flicker too much.

Approximate hold time:

- about 30 ms with 5 ms frames
- about 60 ms with 10 ms frames

The whole bitmap is reset to zero when a new downlink session starts on `BIS1`.

So the meaning is "recently occupied" rather than "occupied in this exact instant".

## What the receiver does

The receive sample detects the appendix at runtime.

- if the received downlink packet is exactly `LC3 bytes + 4`, it treats the last 4 bytes as the appendix
- otherwise it treats the packet as plain LC3 audio

The receiver always decodes only the LC3 part. The 4 extra bytes are metadata only.

It also prints the bitmap as a string where the first character is uplink 1 / `BIS2`, the second is uplink 2 / `BIS3`, and so on.

## How occupation-aware uplink selection works

If `CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT` is enabled and the receiver currently sees the appendix:

- on PTT start, it randomly chooses only from uplink BISes whose bit is `0`
- if all uplink BISes are occupied, it waits and retries until one becomes free
- while waiting, it does not queue uplink audio frames, so no uplink backlog is built up

That means the PTT session simply does not start transmitting until a free uplink BIS exists.

If there is no appendix, the receiver falls back to the normal random uplink BIS selection.

## What the receiver sends

To keep the transport format aligned, the receiver mirrors the current downlink format on uplink:

- if downlink currently has the 4-byte appendix, uplink packets also carry 4 extra bytes
- those uplink appendix bytes are currently just zero-filled padding

Only the downlink appendix carries meaning today.
