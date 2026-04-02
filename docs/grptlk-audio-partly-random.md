# GRPTLK Audio Partly Random

This note describes the "partly random" mode, which in the code is `CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT`.

## What it means

The receiver does not stay on one fixed uplink BIS forever, but it also does not change BIS every audio interval.

Instead:

- when PTT starts, the receiver picks one uplink BIS from `BIS2..BISn`
- it keeps using that same BIS for the whole PTT session
- when PTT is released, that choice is cleared
- the next PTT press picks again

So it is "partly random": random per PTT session, not random per packet.

## What the broadcaster does

The broadcaster does not have a matching "partly random" setting.

It simply listens on all uplink BISes, decodes whichever BISes delivered valid uplink packets in that interval, and mixes the valid audio it received.

Because the receiver stays on one BIS for one whole PTT session, this is much easier for the broadcaster than the fully random per-interval mode.

## What happens when there is no downlink appendix

If the broadcaster has `CONFIG_GRPTLK_DOWNLINK_APPENDIX=n`, then the downlink packet contains only LC3 audio and no uplink-occupancy bitmap.

In that case the receiver is still allowed to use `CONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT`, but it has no occupancy information.

So the receiver behavior becomes:

- pick a random uplink BIS on PTT start
- keep it for that PTT session
- do not check whether another talker is already using that BIS

That means collisions are possible.

## What happens when the appendix is present

If the downlink appendix is present, the receiver can see which uplink BISes are occupied.

Then on PTT start it:

- randomly chooses only from free uplink BISes
- waits if all uplink BISes are occupied
- starts sending only after a free BIS is found

So the appendix turns partly-random mode into occupation-aware partly-random mode.
