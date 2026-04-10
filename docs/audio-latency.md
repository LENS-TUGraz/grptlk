# GRPTLK End-to-End Audio Latency

## Introduction

GRPTLK is a Bluetooth LE Audio group-talk platform built on Broadcast Isochronous Streams (BIS). One broadcaster node sits at the center: it receives microphone audio from N receivers over shared uplink BIS channels, mixes the incoming streams together, and rebroadcasts the result as downlink to every receiver simultaneously — all within a single Broadcast Isochronous Group (BIG). Audio is compressed with the LC3 codec. The system supports two frame durations (5ms and 10ms) and three operating modes: mixing (full decode/mix/re-encode at the broadcaster), relay (raw LC3 bytes forwarded without decoding), and BCST-only (broadcaster transmits its own microphone without any uplink).

---

## The Building Blocks

### BIG / BIS

A BIG is a single Bluetooth broadcast group that contains N BIS channels. The BT controller fires a BIG event every SDU_Interval — either 5ms or 10ms. Within each BIG event, BIS subevents are scheduled sequentially: BIS[0] (downlink, broadcaster → receivers) fires first, followed by BIS[1] through BIS[N] (uplink, receivers → broadcaster). All subevents occur within the same BIG event window.

### LC3 Codec

LC3 is a low-complexity codec optimized for low-latency audio. At 16 kHz, 16-bit:

- **5ms frame**: 80 PCM samples → 20 bytes encoded
- **10ms frame**: 160 PCM samples → 40 bytes encoded

Encode and decode times are well under 1ms each on the target hardware.

### I2S DMA

The I2S peripheral captures audio in 1ms hardware blocks (16 samples at 16 kHz). An LC3 frame is assembled from 5 blocks (5ms mode) or 10 blocks (10ms mode) before encoding begins.

### RTN (Retransmission Count)

RTN adds extra subevents for a given BIS within the same BIG event window. This improves packet reception rate under RF interference. Because the extra subevents are packed into the current BIG event — not scheduled in a future interval — RTN does not add interval-to-interval latency. It trades air time for reliability, not speed for reliability.

---

## The Uplink Path: Receiver Mic → Broadcaster

1. **I2S DMA** accumulates 1ms hardware blocks. After 5 blocks (5ms mode) or 10 blocks (10ms mode), `audio_rx_mono_frame()` copies samples into `mic_pcm_shared` and signals `mic_frame_sem`.
2. **Encoder thread** wakes, LC3-encodes the frame into 20 or 40 bytes, and pushes the result into `uplink_q` (a depth-2 message queue).
3. **TX thread** dequeues immediately — there is no `iso_sent()` pacing on the uplink — and calls `bt_iso_chan_send()` on the selected uplink BIS.
4. **BT controller** schedules the SDU in the next available BIG event.
5. **Broadcaster's `iso_recv()`** fires after the uplink BIS subevents complete within that BIG event.

Each receiver selects its uplink BIS on a per-frame basis. Three selection strategies are available: fully random, partly random, and occupation-aware (which monitors which BIS channels are already in use before choosing).

---

## The Broadcaster: Decode, Mix, Re-encode (Mixing Mode)

1. **`iso_recv()`** stores the received LC3 frame in `rx_frames[bis_idx]`. Once all expected uplink BISes have been received, `decoder_sem` is signalled.
2. **Decoder thread** wakes and LC3-decodes each uplink BIS with its dedicated `lc3_decoders[i]` instance. Packet loss concealment (PLC) is applied automatically to any missing packets.
3. **Mix**: decoded PCM samples from all uplink channels are summed sample-by-sample and clamped to int16, producing `mixed_uplink_pcm[]`.
4. `decoder_proceed_sem` is signalled and the **encoder thread** wakes.
5. If uplink data is present, the encoder blends local mic audio with the uplink mix at 50/50: `(local[i]/2) + (mixed_uplink[i]/2)`, clamped. If no uplink is present, only the local mic is used.
6. The blended PCM is LC3-encoded into `encoded_shared[]` and `encoded_data_ready` is set.
7. **TX thread** waits on `tx_sem`, which fires via `iso_sent()` after BIS[0] completes its subevents. The TX thread reads `encoded_shared` and calls `bt_iso_chan_send(bis[0], ...)`, queuing the SDU for the next BIG event.

---

## The Downlink Path: Broadcaster → Receiver Playback

1. **Receiver `iso_recv()`** fires when BIS[0] delivers a packet; the raw LC3 frame is queued into `encoded_frame_q`.
2. **Decoder thread** dequeues the frame and LC3-decodes it into `mono_pcm[]`.
3. Decoded PCM is written directly into `playback_ring`, the I2S DMA ring buffer composed of 1ms blocks.
4. **I2S DMA** reads the ring continuously and drives the DAC → speaker.

There is no software presentation delay. Decoded PCM goes straight to the ring; the ring absorbs only the small jitter that arises within a single BIG event window.

---

## The BIG Event Pipeline and Why Latency Accumulates

The dominant latency source — beyond frame accumulation — is the BT controller pipeline across two hops.

Within each BIG event, the sequential BIS ordering creates a fundamental ordering constraint:

- **BIS[0] (downlink) fires first** → broadcaster's `iso_sent()` fires → TX thread reads `encoded_shared` and submits a new SDU to the controller
- **BIS[1..N] (uplink) fire after** → broadcaster's `iso_recv()` updates `encoded_shared` with the newly received uplink data

Because the TX thread runs **before** the uplink data arrives within the same BIG event, uplink data received in event N cannot be forwarded as downlink until event N+2 at the earliest — and the controller itself needs the SDU available some interval ahead of the target transmission event, pushing this further. Each hop therefore contributes multiple BIG intervals of latency, and that contribution scales linearly with the interval length.

In 5ms mode, the two-hop pipeline accounts for roughly 29ms of the total latency. In 10ms mode, it accounts for roughly 48ms.

---

## Relay Mode

When `CONFIG_GRPTLK_RELAY=y` is set:

- **`iso_recv()`** copies raw LC3 bytes from uplink BIS[1] directly into `encoded_shared[]` — no decode, no mix, no re-encode.
- **TX thread** forwards those bytes verbatim on BIS[0].
- Pre-encoded LC3 silence fills `encoded_shared[]` when no uplink packet is present.

Relay mode eliminates the broadcaster's codec round-trip (decode + mix + encode), saving approximately 3ms. The trade-off is that only one uplink BIS is supported (`NUM_CHAN=2`); mixing across multiple receivers is not possible in relay mode.

---

## Latency Breakdown

### 5ms Frame Mode

| Stage | 5ms mixing | 5ms relay |
|---|---|---|
| Receiver mic accumulation (5 × 1ms) | 5ms | 5ms |
| LC3 encode at receiver | ~0.3ms | ~0.3ms |
| BT controller pipeline — uplink hop | ~14ms | ~14ms |
| Broadcaster decode + mix + re-encode | ~3ms | 0ms |
| BT controller pipeline — downlink hop | ~15ms | ~15ms |
| Receiver LC3 decode + ring write | ~0.7ms | ~0.7ms |
| **Total (measured)** | **~38ms** | **~35ms** |

### 10ms Frame Mode

| Stage | 10ms mixing |
|---|---|
| Receiver mic accumulation (10 × 1ms) | 10ms |
| LC3 encode at receiver | ~0.5ms |
| BT controller pipeline — uplink hop | ~24ms |
| Broadcaster decode + mix + re-encode | ~3ms |
| BT controller pipeline — downlink hop | ~24ms |
| Receiver LC3 decode + ring write | ~0.5ms |
| **Total (measured)** | **~62ms** |

---

## Why These Numbers?

**Frame accumulation is the hard floor.** Audio that hasn't been captured cannot be encoded. In 5ms mode the floor is 5ms; in 10ms mode it's 10ms. Nothing in the software stack can shrink this.

**The BIS pipeline is the dominant term.** The sequential BIS ordering — downlink before uplink within each event — combined with the controller's own scheduling pipeline means each hop consumes several full BIG intervals. This effect scales linearly with the interval: 5ms intervals produce ~14–15ms per hop; 10ms intervals produce ~24ms per hop.

**RTN adds robustness, not latency.** Extra retransmission subevents are packed into the same BIG event window. They don't stretch the interval-to-interval timing, so enabling higher RTN values improves reliability under interference without affecting the latency budget.

**Relay saves exactly the codec round-trip.** The 3ms difference between relay (35ms) and mixing (38ms) in 5ms mode corresponds precisely to the broadcaster's decode + mix + encode time. The BT pipeline cost is identical in both modes.

**10ms mode is ~1.63× slower than 5ms mode.** The measured ratio of 62ms to 38ms is close to the 2× interval ratio — confirming that pipeline latency scales approximately linearly with the BIG interval, as expected from the BIS ordering analysis above.
