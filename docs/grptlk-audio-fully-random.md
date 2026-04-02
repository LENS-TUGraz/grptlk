# GRPTLK Audio Fully Random

This note describes the fully-random uplink mode.

The matching setup is:

- broadcaster: `CONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y`
- receiver: `CONFIG_GRPTLK_UPLINK_RANDOM=y`

When `CONFIG_GRPTLK_UPLINK_RANDOM=y`, the receiver picks a random uplink BIS from `BIS2..BISn` on every transmit interval. So the talker can hop to a different uplink BIS from one audio frame to the next.

## Why the broadcaster needs `GRPTLK_INIT_LC3_CONSTANTLY`

In fully-random mode, the same talker may arrive on a different BIS every frame.

Without `CONFIG_GRPTLK_INIT_LC3_CONSTANTLY`, the broadcaster keeps normal LC3 decoder state per BIS. That means decoder history follows the BIS, not the talker, which is a poor match for full BIS hopping.

With `CONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y`, the broadcaster resets the uplink decoder state before each valid uplink packet and treats missed packets as silence instead of PLC. That makes fully-random BIS hopping workable, at the cost of losing normal PLC history across missed frames.

## What happens with the downlink appendix

If the broadcaster has `CONFIG_GRPTLK_DOWNLINK_APPENDIX=n`, the downlink contains only LC3 audio and no occupancy bitmap.

If the broadcaster has `CONFIG_GRPTLK_DOWNLINK_APPENDIX=y`, the receiver can still see the occupancy bitmap.

But in fully-random mode, the receiver currently ignores that bitmap either way. It still picks a random uplink BIS every interval.

So:

- the appendix does not currently make fully-random mode occupation-aware
- collisions are still possible
- occupation-aware waiting is only implemented for the partly-random per-PTT mode
