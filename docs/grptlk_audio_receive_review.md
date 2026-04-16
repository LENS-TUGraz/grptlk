# grptlk_audio_receive — review notes

Two findings: (1) a pre-existing PTT-lock override bug, now fixed; (2) an
assessment of the uncommitted audio-backend refactor.

## 1. PTT-lock override bug (fixed)

### Symptom

With PTT-lock engaged (via `sw3`), pressing `sw2` (PTT) would unlatch the
uplink TX. Expected: while the lock is on, `sw2` should do nothing; only
`sw3` can clear the lock.

### Cause

In `src/io/buttons.c`, `ptt_isr` unconditionally cleared `ptt_active` on
the release edge:

```c
} else {
    atomic_set(&ptt_active, 0);   /* cleared even when lock was holding it */
    ...
}
```

Because `ptt_lock_work_handler` drives `ptt_active` to 1 when the lock
engages, any physical sw2 press → release cycle would overwrite the lock's
output state. The uplink chain gates on `ptt_active` (see `src/main.c`),
so TX stalled until the lock was toggled again.

### Not caused by the uncommitted refactor

`git log -S 'ptt_lock_active' -- samples/grptlk_audio_receive/src/io/buttons.c`
shows the handler was introduced in commit `71e9b28` with the same bug.
`git diff HEAD` on `buttons.c` before this fix had only 5 lines of change
(gating `src_toggle_init` behind `CONFIG_GRPTLK_AUDIO_CODEC_CIRRUS`), none
touching the PTT paths. The bug has been latent since the lock feature
landed.

### Fix (semantics: sw2 is a no-op while lock engaged)

`ptt_isr` now early-returns when `ptt_lock_active` is set. Both press and
release from sw2 are ignored while the lock holds. The PTT-LOCK block was
moved above the PTT block so `PTT_LOCK_AVAILABLE` and `ptt_lock_active`
are visible to `ptt_isr`.

```c
if (atomic_get(&src_line_in_active)) {
    return; /* PTT is MIC-only */
}

#if PTT_LOCK_AVAILABLE
if (atomic_get(&ptt_lock_active)) {
    return; /* lock owns ptt_active; sw2 is a no-op while engaged */
}
#endif
```

Only `sw3` (via `ptt_lock_work_handler`) and `src_toggle_work_handler`
(LINE-IN switch) can now alter `ptt_active` while the lock is on.

## 2. Uncommitted audio-backend refactor — assessment

The refactor splits the monolithic CS47L63 driver into a dispatcher
(`src/audio/audio.c`) plus pluggable backends via a small interface
(`src/audio/backend.h`). MAX9867 is added as a second backend.

### Changes

| File | Type | Notes |
|---|---|---|
| `src/audio/audio.c` | NEW | Dispatcher; owns I2S DMA ring, underrun counters, volume/source routing. |
| `src/audio/backend.h` | NEW | Backend interface: init, prepare-stream-start, volume-adjust, input-source-switch. |
| `src/audio/backends/max9867.c` | NEW | MAX9867 backend over I2C (TWIM1). |
| `src/audio/backends/max9867_regs.h` | NEW | MAX9867 register map. |
| `src/audio/backends/cs47l63.c` | MODIFIED | Dispatcher code moved out; now a pure backend. |
| `src/audio/audio.h` | MODIFIED | Docs + `-ENOTSUP` return for fixed-source backends. |
| `src/io/buttons.c` | MODIFIED | `src_toggle_init` gated behind `CONFIG_GRPTLK_AUDIO_CODEC_CIRRUS`. |
| `src/main.c` | MODIFIED | Whitespace/format + mic peak log. No semantic change. |
| `Kconfig` | MODIFIED | `choice GRPTLK_AUDIO_CODEC` {CIRRUS, MAX9867}; I2C/TWIM defaults for MAX9867. |
| `CMakeLists.txt` | MODIFIED | Conditional backend sources. |
| `boards/..._le_audio_playground.{conf,overlay}` | NEW | MAX9867 build overlay. |
| `boards/..._cpuapp.conf` | MODIFIED | Drops codec/source defines now in Kconfig. |

### Assessment (Zephyr idioms)

- `choice GRPTLK_AUDIO_CODEC` is canonical Kconfig for exclusive options.
- Dispatcher/backend split mirrors `nrf5340_audio`'s `sw_codec_select`
  + HW codec split — good precedent.
- Conditional I2C/TWIM enablement only for MAX9867 keeps the CS47L63
  (SPI) build lean.
- `-ENOTSUP` from `audio_input_source_switch` on MAX9867 is correct
  errno usage.
- Idempotency (`is_initialized` / `is_started`) in `audio_init` /
  `audio_start` is correct.
- `FILE_SUFFIX`-based board overlay (`*_le_audio_playground.{conf,overlay}`)
  follows upstream Zephyr convention.

### Minor flags (not blockers)

1. `src_line_in_active` does not initialise from
   `CONFIG_GRPTLK_AUDIO_SOURCE_LINE_IN` at boot. Latent with the CS47L63
   default (LINE-IN HW routing + flag=0), so `ptt_isr` does not
   early-return even though HW is in LINE-IN mode. Worth addressing now
   that the backend layer has formalised the source-switch API.
2. `audio.c` uses `printk` directly; upstream style prefers
   `LOG_MODULE_REGISTER`. Project-wide convention issue, not a
   regression.
3. `src_toggle_init` alt-path message is generic ("unavailable on
   selected codec"). Nice-to-have: state that MAX9867 is fixed-MIC.
4. MAX9867 backend clock/I2S framing (24- vs 32-bit, LRCLK polarity)
   not reviewed here — flag for a dedicated pass if desired.

## References

- `samples/grptlk_audio_receive/src/io/buttons.c` — `ptt_isr`,
  `ptt_lock_work_handler`
- `samples/grptlk_audio_receive/src/main.c` — `ptt_active` gates
- `samples/grptlk_audio_receive/src/audio/audio.c` — dispatcher
- `samples/grptlk_audio_receive/src/audio/backend.h` — interface
- `samples/grptlk_audio_receive/src/audio/backends/{cs47l63,max9867}.c`
- Commit `71e9b28` — PTT-lock feature landing (bug first appeared here)
