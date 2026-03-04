# GRPTLK iso_broadcast Log Format

This document describes every line the `iso_broadcast` sample emits on its serial port.
It is intended as a reference for building a dashboard or parser.

---

## Overview

The broadcaster emits two categories of output:

| Category | Prefix | Frequency |
|---|---|---|
| Structured data lines | `GRPTLK_DATA ` | Every 200 ms |
| Human-readable info logs | `[00:00:00.000] <inf> grptlk_iso_broadcast: ` | Every 200 ms and every 2 s |

**All counters are cumulative since boot unless marked `(+N)`, which is the delta since the previous tick.**

---

## Timing

- Stats snapshot period: **200 ms**
- Channel-info snapshot period: **2000 ms** (every 10th stats tick)
- Device stale/eviction timeout: **5000 ms** (device entry removed if silent for 5 s)

---

## Line Types

### 1. Stats banner

```
[timestamp] <inf> grptlk_iso_broadcast: ----- ISO_BROADCAST STATS [200ms] -----
```

Marks the start of one stats snapshot. Every structured `GRPTLK_DATA` block between two banners belongs to the same tick.

---

### 2. Downlink TX counters

```
[timestamp] <inf> grptlk_iso_broadcast: DL TX (BIS1): ok=N(+N) alloc_fail=N(+N) send_fail=N(+N)
```

Broadcaster's own transmit counters for BIS1 (the downlink channel it broadcasts on).

| Field | Meaning |
|---|---|
| `ok` | Total ISO SDUs successfully submitted to the controller |
| `alloc_fail` | Times `net_buf_alloc` failed — TX pool exhausted, host is too slow |
| `send_fail` | Times `bt_iso_chan_send` returned an error |

**What to watch:** `alloc_fail` or `send_fail` incrementing means the broadcaster is dropping downlink packets at the host layer before they even reach the radio.

---

### 3. Uplink RX counters per BIS

One line per uplink BIS (BIS2, BIS3, … BISn). These are the broadcaster's view of packets arriving on each uplink channel.

```
[timestamp] <inf> grptlk_iso_broadcast: UL RX (BIS2): tot=N(+N) valid=N(+N) lost=N(+N) err=N(+N) bad_len=N(+N) gap_pkt=N(+N) gap_evt=N(+N) dup=N(+N) raw_rx=N
```

| Field | Meaning |
|---|---|
| `tot` | Total ISO events received on this BIS (valid + lost + error combined) |
| `valid` | Packets with `BT_ISO_FLAGS_VALID` — good CRC, payload delivered |
| `lost` | Packets with `BT_ISO_FLAGS_LOST` — controller flagged the packet as lost on-air |
| `err` | Packets with `BT_ISO_FLAGS_ERROR` — CRC error or framing error |
| `bad_len` | Packets with unexpected payload length (0 bytes or exceeds RX MTU) |
| `gap_pkt` | Total missing sequence numbers detected (sum of all gap sizes) |
| `gap_evt` | Number of separate gap events (each gap between consecutive received packets counts as one event) |
| `dup` | Out-of-order or duplicate packets (sequence number in the past) |
| `raw_rx` | Raw ISO callback invocation count — incremented on every callback including lost/error events; useful to confirm the controller is delivering events at all |

**What to watch for link quality:**
- `lost` and `err` incrementing rapidly → poor radio conditions on this uplink BIS
- `gap_pkt` growing while `valid` is low → collisions or interference causing bursts of missed packets
- `raw_rx` staying flat while other BISes increment → a receiver may have stopped transmitting entirely
- `gap_evt` vs `gap_pkt`: if `gap_evt` is low but `gap_pkt` is high, the gaps are large and sustained (e.g. a device reset); if both are high, there are many small intermittent drops

---

### 4. Uplink anomaly counters

```
[timestamp] <inf> grptlk_iso_broadcast: UL RX anomalies: unknown_chan=N unexpected_bis1=N
```

| Field | Meaning |
|---|---|
| `unknown_chan` | ISO RX callbacks on a channel index that doesn't map to any known BIS — should always be 0 |
| `unexpected_bis1` | Packets received on BIS1 (the downlink) that were not expected — should always be 0 |

**What to watch:** Any non-zero value here indicates a configuration or sync problem.

---

### 5. Uplink report health counters

```
[timestamp] <inf> grptlk_iso_broadcast: UL reports: frames=N(+N) short=N(+N) parse_fail=N(+N) table_full=N(+N)
```

| Field | Meaning |
|---|---|
| `frames` | Total uplink stats payloads successfully parsed from any receiver |
| `short` | Packets too small to contain a valid stats header — likely non-stats traffic or corrupted frames |
| `parse_fail` | Packets with wrong magic bytes or unknown version/type |
| `table_full` | New device seen but reporter table is full (`BIS_ISO_CHAN_COUNT × 4` slots); increase `MAX_UPLINK_REPORTERS` if this increments |

---

### 6. Tick summary line (structured)

```
GRPTLK_DATA ts=N active=N bis_count=N
```

Emitted once per tick, before any per-device lines.

| Field | Meaning |
|---|---|
| `ts` | Broadcaster uptime in milliseconds (`k_uptime_get_32`) at the moment of snapshot |
| `active` | Number of devices currently in the reporter table (not yet stale) |
| `bis_count` | Total number of BIS channels in the BIG (including BIS1 downlink) |

---

### 7. Per-device stats line (structured)

One line per known active receiver, emitted after the tick summary.

```
GRPTLK_DATA dev=AABBCCDDEEFF bis_rx=2 uplink_bis=2 bis_claim=match age_ms=45 missed=0 seq=1234 dl_rx=500(+10) dl_valid=490(+10) dl_lost=10(+0) dl_gap=2(+0) dl_gap_evt=1(+0) dl_dup=0(+0) ul_ok=50(+1) ul_alloc=0(+0) ul_send=0(+0) ul_lost=0 phy=2 max_pdu=40 bn=1 irc=2
```

#### Identity

| Field | Meaning |
|---|---|
| `dev` | Device ID as hex string — derived from the BT BD_ADDR of the receiver (6 bytes = 12 hex chars) |
| `bis_rx` | BIS number on which this device's uplink packets arrived at the broadcaster |
| `uplink_bis` | BIS number the device claims to be transmitting on (from inside its payload) |
| `bis_claim` | `match` if `bis_rx == uplink_bis`; `mismatch` if they differ (indicates BIS assignment confusion or a mis-configured receiver) |

#### Staleness

| Field | Meaning |
|---|---|
| `age_ms` | Milliseconds since the broadcaster last received a valid stats payload from this device |
| `missed` | `age_ms / 200` — how many stats ticks have passed with no report from this device. `0` = reported this tick; `1` = missed one tick; `≥3` = investigate; `≥25` = device will be evicted at 5000 ms |
| `seq` | Last uplink payload sequence number received from this device |

#### Downlink quality (receiver's perspective)

These counters are what the **receiver** measured about how well it is receiving the downlink from the broadcaster. All values are cumulative (wrapping at 65535); `(+N)` is the delta since the previous tick.

| Field | Meaning |
|---|---|
| `dl_rx` | Total ISO downlink events the receiver processed |
| `dl_valid` | Downlink packets with valid CRC received by the receiver |
| `dl_lost` | Downlink packets the receiver's controller flagged as lost on-air |
| `dl_gap` | Total missed downlink sequence numbers detected by the receiver |
| `dl_gap_evt` | Number of separate gap events the receiver detected |
| `dl_dup` | Out-of-order or duplicate downlink packets seen by the receiver |

**Derived link quality metric:** `dl_valid / dl_rx` gives the receiver's downlink reception rate. A healthy link should be above 95%. `dl_lost / dl_rx` is the on-air loss rate from the receiver's perspective.

**Collision indicator:** `dl_gap_evt` rising rapidly while `dl_gap` is small (gaps of 1–2 packets) suggests repeated short interference bursts. `dl_gap` rising steeply with few `dl_gap_evt` suggests a device temporarily lost sync entirely.

#### Uplink TX (receiver's perspective)

What the receiver measured about its own uplink transmit attempts.

| Field | Meaning |
|---|---|
| `ul_ok` | Uplink packets the receiver successfully submitted to its controller |
| `ul_alloc` | Times the receiver's TX buffer pool was exhausted |
| `ul_send` | Times `bt_iso_chan_send` failed on the receiver |
| `ul_lost` | Uplink payload sequence number gaps detected by the **broadcaster** — packets the receiver sent but the broadcaster never received. This is the broadcaster-side view of uplink radio loss |

**`ul_lost` is the key uplink link quality indicator.** It is computed on the broadcaster by tracking the receiver's `seq` field; if packets jump by more than 1, the broadcaster counts those as lost uplink payloads.

#### PHY and channel parameters (from receiver's `bt_iso_chan_get_info`)

| Field | Meaning |
|---|---|
| `phy` | BLE PHY in use on the downlink, as reported by the broadcaster's BIG advertisement. `1` = 1M, `2` = 2M, `3` = Coded |
| `max_pdu` | Maximum PDU size in bytes negotiated for this BIS |
| `bn` | Burst Number — number of new payloads transmitted per BIS event. Higher = more redundancy |
| `irc` | Immediate Repetition Count — number of times each payload is retransmitted within a BIS event. Higher = more retransmissions, lower latency penalty than increasing `bn` |

**Link quality context:** `bn` and `irc` together define how aggressively the BIG retransmits. If `bn=1` and `irc=1`, there is no redundancy; every missed packet is a loss. If these values are higher, a single radio interference event may not cause a loss at all.

---

### 8. Device eviction line (structured)

Emitted when a device has not sent a report for more than 5000 ms and is removed from the table.

```
GRPTLK_DATA dev=AABBCCDDEEFF status=stale age_ms=5123
```

| Field | Meaning |
|---|---|
| `dev` | Device ID of the evicted device |
| `status` | Always `stale` |
| `age_ms` | How long ago the last report was received before eviction |

**This is the "device disconnected" event.** After this line, the device will no longer appear in the per-device section until it reconnects and the broadcaster receives a new payload from it.

---

### 9. Channel info snapshot (every 2 seconds)

Emitted once per BIS, from the broadcaster's own `bt_iso_chan_get_info()` call. Not structured — use as supplementary diagnostic information.

**BIS1 (downlink, broadcaster side):**
```
[timestamp] <inf> grptlk_iso_broadcast: BIS1 (DL): iso_interval=8 max_subevent=1 can_send=1 can_recv=0 sync_delay=7500 latency=7500 pto=0 max_pdu=40 phy=2 bn=1 irc=2
```

**BIS2..BISn (uplink, broadcaster as sync receiver):**
```
[timestamp] <inf> grptlk_iso_broadcast: BIS2 (UL): iso_interval=8 max_subevent=1 can_send=0 can_recv=1 latency=7500 pto=0 max_pdu=40 bn=1 irc=2
```

| Field | Meaning |
|---|---|
| `iso_interval` | ISO interval in units of 1.25 ms (e.g. `8` = 10 ms) |
| `max_subevent` | Maximum number of subevents per ISO interval |
| `can_send` | Whether this channel can transmit (`1` for BIS1/DL, `0` for UL sync receivers) |
| `can_recv` | Whether this channel can receive (`0` for BIS1/DL broadcaster, `1` for UL sync receivers) |
| `sync_delay` | BIG sync delay in microseconds (BIS1 only) |
| `latency` | Transport latency in microseconds |
| `pto` | Pre-Transmission Offset — number of ISO intervals before the anchor point at which the first PDU is transmitted |
| `max_pdu` | Maximum PDU payload size in bytes |
| `phy` | PHY (`1`=1M, `2`=2M, `3`=Coded) — BIS1 only |
| `bn` | Burst Number |
| `irc` | Immediate Repetition Count |

These values are static for the lifetime of the BIG and will not change while the broadcaster is running. They are logged periodically as a convenience for offline log analysis.

---

## Parsing Guide

### Line identification

```
line starts with "GRPTLK_DATA "  →  structured data line, parse key=value pairs
line contains "-----"            →  stats tick boundary
line contains "BIS" and "(DL):"  →  downlink chan-info snapshot
line contains "BIS" and "(UL):"  →  uplink chan-info snapshot
line contains "status=stale"     →  device eviction event
```

### Key=value parsing

All structured `GRPTLK_DATA` lines use space-separated `key=value` tokens. Values with `(+N)` contain a cumulative total and a delta: `dl_rx=500(+10)` means total=500, delta=10.

Suggested regex: `(\w+)=(\d+)(?:\(\+(\d+)\))?`

### Device tracking

- A device is **online** when it appears in a `GRPTLK_DATA dev=... age_ms=...` line with `missed < 5`.
- A device is **degraded** when `missed >= 3` or when `dl_valid / dl_rx < 0.90` (delta values).
- A device is **disconnected** when `status=stale` appears, or when it stops appearing in the per-device section entirely.
- A device **reconnects** when it reappears in the per-device section after being absent.

### On-air error events

| Condition | Fields to monitor |
|---|---|
| Downlink packet loss | `dl_lost(+N) > 0` in per-device line |
| Downlink CRC errors | `err(+N) > 0` in `UL RX (BIS...)` line |
| Downlink gap/collision | `dl_gap(+N) > 0` and `dl_gap_evt(+N) > 0` in per-device line |
| Uplink packet loss | `ul_lost > 0` in per-device line |
| Uplink radio problems | `lost(+N)` or `err(+N)` in `UL RX (BIS...)` line |
| Broadcaster TX failure | `alloc_fail(+N)` or `send_fail(+N)` in `DL TX (BIS1)` line |

### Timestamp

`ts` in the `GRPTLK_DATA ts=N` line is the broadcaster's `k_uptime_get_32()` in milliseconds. It is monotonic from boot and wraps at ~49.7 days. Use it to correlate events across the same tick. All per-device lines within one tick share the same `ts`.
