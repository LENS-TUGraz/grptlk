#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SAMPLES_DIR="${WORKSPACE}/grptlk/samples"
BINARIES_DIR="${SAMPLES_DIR}/binaries"
BUILD_DIR_NAME="build_gen"
BOARD_ROOT_ARG="-DBOARD_ROOT=${WORKSPACE}"
# Per-timing GRPTLK_NUM_CHAN matrix (NUM_CHAN includes 1 downlink BIS).
# 5 ms cannot fit 5 BISes — uses 3ch as its "many-BIS" variant.
# 5 ms  → 3ch + 2ch        (5ch infeasible at 5 ms)
# 10 ms → 5ch + 3ch + 2ch
channels_for_timing() {
  case "$1" in
    5ms)  echo "3 2" ;;
    10ms) echo "5 3 2" ;;
    *)    fail "channels_for_timing: unknown timing '$1'" ;;
  esac
}
MISSING_ONLY=false
for arg in "$@"; do
  case "${arg}" in
    --missing-only) MISSING_ONLY=true ;;
    *) echo "[ERROR] Unknown argument: ${arg}" >&2; exit 1 ;;
  esac
done

BUILD_JOBS=(
  # Nordic nRF5340 Audio DK variants — TEMPORARILY DISABLED (le_audio_playground only).
  # # Fully Random
  # "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_fully_random.hex"
  # "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_nrf5340_audio_dk_5ms_liblc3_fully_random.hex"
  # "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_fully_random.hex"
  # "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_nrf5340_audio_dk_10ms_T2_fully_random.hex"
  # # Partly Random
  # "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_partly_random.hex"
  # "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_partly_random.hex"
  # "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y|grptlk_recv_nrf5340_audio_dk_5ms_liblc3_partly_random.hex"
  # "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y|grptlk_recv_nrf5340_audio_dk_10ms_T2_partly_random.hex"
  # # Occupation Aware
  # "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_DOWNLINK_APPENDIX=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_occupation_aware.hex"
  # "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_DOWNLINK_APPENDIX=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_occupation_aware.hex"

  # LE Audio Playground variants
  # Fully Random
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_bcst_le_audio_playground_5ms_liblc3_fully_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_recv_le_audio_playground_5ms_liblc3_fully_random.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_bcst_le_audio_playground_10ms_T2_fully_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_recv_le_audio_playground_10ms_T2_fully_random.hex"
  # Partly Random
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_bcst_le_audio_playground_5ms_liblc3_partly_random.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_bcst_le_audio_playground_10ms_T2_partly_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_recv_le_audio_playground_5ms_liblc3_partly_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_recv_le_audio_playground_10ms_T2_partly_random.hex"
  # Occupation Aware
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_DOWNLINK_APPENDIX=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_bcst_le_audio_playground_5ms_liblc3_occupation_aware.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_DOWNLINK_APPENDIX=y -DEXTRA_CONF_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.conf -DEXTRA_DTC_OVERLAY_FILE=boards/nrf5340_audio_dk_nrf5340_cpuapp_le_audio_playground.overlay|grptlk_bcst_le_audio_playground_10ms_T2_occupation_aware.hex"
)

# nRF5340 DK (non-audio) — TEMPORARILY DISABLED (le_audio_playground only).
# Originally: broadcaster only, 2ch (1 uplink BIS), GRPTLK_RELAY=y default.
# NRF5340DK_BUILD_JOBS=(
#   "grptlk_audio_broadcast|nrf5340dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340dk_5ms_liblc3.hex"
#   "grptlk_audio_broadcast|nrf5340dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340dk_10ms_T2.hex"
# )
NRF5340DK_BUILD_JOBS=()

# rbv2h — TEMPORARILY DISABLED (le_audio_playground only).
# Originally: recv only (no broadcaster ported).
# RBV2H_BUILD_JOBS=(
#   # Fully Random
#   "grptlk_audio_receive|rbv2h/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_rbv2h_5ms_liblc3_fully_random.hex"
#   "grptlk_audio_receive|rbv2h/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_rbv2h_10ms_T2_fully_random.hex"
#   # Partly Random
#   "grptlk_audio_receive|rbv2h/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y|grptlk_recv_rbv2h_5ms_liblc3_partly_random.hex"
#   "grptlk_audio_receive|rbv2h/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y|grptlk_recv_rbv2h_10ms_T2_partly_random.hex"
# )
RBV2H_BUILD_JOBS=()

log()  { echo "[grptlk] $*"; }
info() { echo ""; echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; echo "$*"; echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; }
fail() { echo "[ERROR] $*" >&2; exit 1; }

timing_of_job() {
  case "$1" in
    *AUDIO_FRAME_5_MS=y*)  echo "5ms"  ;;
    *AUDIO_FRAME_10_MS=y*) echo "10ms" ;;
    *) fail "timing_of_job: cannot determine timing from '$1'" ;;
  esac
}

check_tool() {
  command -v "$1" &>/dev/null || fail "'$1' not found in PATH. Ensure the nRF Connect toolchain is activated."
}

check_tool west
check_tool mergehex

log "west    : $(west --version 2>&1 | head -1)"
log "mergehex: $(mergehex --version 2>&1 | head -1)"
log "workspace: ${WORKSPACE}"

ALL_BINS_DIR="${BINARIES_DIR}/all_bins"
if [[ "${MISSING_ONLY}" == false ]]; then
  rm -rf "${BINARIES_DIR}"
fi
mkdir -p "${ALL_BINS_DIR}"

# Total build count: broadcasters expand per channels_for_timing(timing); receivers built once.
TOTAL=0
for job in "${BUILD_JOBS[@]}"; do
  IFS='|' read -r sn _ extra _ <<< "${job}"
  if [[ "${sn}" == "grptlk_audio_broadcast" ]]; then
    read -ra chs <<< "$(channels_for_timing "$(timing_of_job "${extra}")")"
    TOTAL=$(( TOTAL + ${#chs[@]} ))
  else
    TOTAL=$(( TOTAL + 1 ))
  fi
done
TOTAL=$(( TOTAL + ${#NRF5340DK_BUILD_JOBS[@]} + ${#RBV2H_BUILD_JOBS[@]} ))
BUILT=()
IDX=0

for JOB in "${BUILD_JOBS[@]}"; do
  IFS='|' read -r SAMPLE_NAME BOARD EXTRA_CMAKE BASE_HEX <<< "${JOB}"
  TIMING="$(timing_of_job "${EXTRA_CMAKE}")"

  # Broadcaster fans out per timing's channel matrix; receiver builds once (NUM_CHAN-agnostic).
  if [[ "${SAMPLE_NAME}" == "grptlk_audio_broadcast" ]]; then
    read -ra JOB_CHANNELS <<< "$(channels_for_timing "${TIMING}")"
  else
    JOB_CHANNELS=("none")
  fi

  for ch in "${JOB_CHANNELS[@]}"; do
    IDX=$((IDX + 1))

    if [[ "${ch}" == "none" ]]; then
      EFFECTIVE_CMAKE="${EXTRA_CMAKE}"
      OUTPUT_HEX="${BASE_HEX}"
    else
      EFFECTIVE_CMAKE="${EXTRA_CMAKE} -DCONFIG_GRPTLK_NUM_CHAN=${ch}"
      OUTPUT_HEX="${BASE_HEX%.hex}_${ch}ch.hex"
    fi

    SAMPLE_DIR="${SAMPLES_DIR}/${SAMPLE_NAME}"
    BUILD_DIR="${SAMPLE_DIR}/${BUILD_DIR_NAME}"

    if [[ "${MISSING_ONLY}" == true && -f "${ALL_BINS_DIR}/${OUTPUT_HEX}" ]]; then
      log "Skipping (exists): ${OUTPUT_HEX}"
      BUILT+=("${OUTPUT_HEX}")
      continue
    fi

    info "[BUILD ${IDX}/${TOTAL}] ${SAMPLE_NAME}  board=${BOARD}  ch=${ch}  ${EFFECTIVE_CMAKE}  →  ${OUTPUT_HEX}"

    [[ -d "${SAMPLE_DIR}" ]] || fail "Sample directory not found: ${SAMPLE_DIR}"

    (
      cd "${SAMPLE_DIR}"
      west build \
        --pristine always \
        --board "${BOARD}" \
        --build-dir "${BUILD_DIR_NAME}" \
        -- ${BOARD_ROOT_ARG} ${EFFECTIVE_CMAKE}
    )

    HCI_HEX="${BUILD_DIR}/hci_ipc/zephyr/zephyr.hex"
    APP_HEX="${BUILD_DIR}/${SAMPLE_NAME}/zephyr/zephyr.hex"
    OUT_HEX="${ALL_BINS_DIR}/${OUTPUT_HEX}"

    [[ -f "${HCI_HEX}" ]] || fail "hci_ipc hex not found: ${HCI_HEX}"
    [[ -f "${APP_HEX}" ]] || fail "App hex not found: ${APP_HEX}"

    mergehex -m "${HCI_HEX}" "${APP_HEX}" -o "${OUT_HEX}"

    log "Merged → ${OUT_HEX}"
    BUILT+=("${OUTPUT_HEX}")
  done
done

# nRF5340 DK — 2ch (1 uplink BIS) broadcaster only — TEMPORARILY DISABLED
: <<'DISABLED'
for job in "${NRF5340DK_BUILD_JOBS[@]}"; do
  IFS='|' read -r SAMPLE_NAME BOARD EXTRA_CMAKE BASE_HEX <<< "${job}"

  IDX=$((IDX + 1))
  EXTRA_CMAKE="${EXTRA_CMAKE} -DCONFIG_GRPTLK_NUM_CHAN=2"
  OUTPUT_HEX="${BASE_HEX%.hex}_2ch.hex"

  SAMPLE_DIR="${SAMPLES_DIR}/${SAMPLE_NAME}"
  BUILD_DIR="${SAMPLE_DIR}/${BUILD_DIR_NAME}"

  if [[ "${MISSING_ONLY}" == true && -f "${ALL_BINS_DIR}/${OUTPUT_HEX}" ]]; then
    log "Skipping (exists): ${OUTPUT_HEX}"
    BUILT+=("${OUTPUT_HEX}")
    continue
  fi

  info "[BUILD ${IDX}/${TOTAL}] ${SAMPLE_NAME}  board=${BOARD}  ch=2  ${EXTRA_CMAKE}  →  ${OUTPUT_HEX}"

  [[ -d "${SAMPLE_DIR}" ]] || fail "Sample directory not found: ${SAMPLE_DIR}"

  (
    cd "${SAMPLE_DIR}"
    west build \
      --pristine always \
      --board "${BOARD}" \
      --build-dir "${BUILD_DIR_NAME}" \
      -- ${BOARD_ROOT_ARG} ${EXTRA_CMAKE}
  )

  HCI_HEX="${BUILD_DIR}/hci_ipc/zephyr/zephyr.hex"
  APP_HEX="${BUILD_DIR}/${SAMPLE_NAME}/zephyr/zephyr.hex"
  OUT_HEX="${ALL_BINS_DIR}/${OUTPUT_HEX}"

  [[ -f "${HCI_HEX}" ]] || fail "hci_ipc hex not found: ${HCI_HEX}"
  [[ -f "${APP_HEX}" ]] || fail "App hex not found: ${APP_HEX}"

  mergehex -m "${HCI_HEX}" "${APP_HEX}" -o "${OUT_HEX}"

  log "Merged → ${OUT_HEX}"
  BUILT+=("${OUTPUT_HEX}")
done
DISABLED

# rbv2h — recv only — TEMPORARILY DISABLED
: <<'DISABLED'
for job in "${RBV2H_BUILD_JOBS[@]}"; do
  IFS='|' read -r SAMPLE_NAME BOARD EXTRA_CMAKE BASE_HEX <<< "${job}"

  IDX=$((IDX + 1))
  OUTPUT_HEX="${BASE_HEX}"

  SAMPLE_DIR="${SAMPLES_DIR}/${SAMPLE_NAME}"
  BUILD_DIR="${SAMPLE_DIR}/${BUILD_DIR_NAME}"

  if [[ "${MISSING_ONLY}" == true && -f "${ALL_BINS_DIR}/${OUTPUT_HEX}" ]]; then
    log "Skipping (exists): ${OUTPUT_HEX}"
    BUILT+=("${OUTPUT_HEX}")
    continue
  fi

  info "[BUILD ${IDX}/${TOTAL}] ${SAMPLE_NAME}  board=${BOARD}  ${EXTRA_CMAKE}  →  ${OUTPUT_HEX}"

  [[ -d "${SAMPLE_DIR}" ]] || fail "Sample directory not found: ${SAMPLE_DIR}"

  (
    cd "${SAMPLE_DIR}"
    west build \
      --pristine always \
      --board "${BOARD}" \
      --build-dir "${BUILD_DIR_NAME}" \
      -- ${BOARD_ROOT_ARG} ${EXTRA_CMAKE}
  )

  HCI_HEX="${BUILD_DIR}/hci_ipc/zephyr/zephyr.hex"
  APP_HEX="${BUILD_DIR}/${SAMPLE_NAME}/zephyr/zephyr.hex"
  OUT_HEX="${ALL_BINS_DIR}/${OUTPUT_HEX}"

  [[ -f "${HCI_HEX}" ]] || fail "hci_ipc hex not found: ${HCI_HEX}"
  [[ -f "${APP_HEX}" ]] || fail "App hex not found: ${APP_HEX}"

  mergehex -m "${HCI_HEX}" "${APP_HEX}" -o "${OUT_HEX}"

  log "Merged → ${OUT_HEX}"
  BUILT+=("${OUTPUT_HEX}")
done
DISABLED

info "Build complete — ${#BUILT[@]}/${TOTAL} binaries in all_bins/"

# ---------------------------------------------------------------------------
# Organize binaries into device / timing / strategy folders
# ---------------------------------------------------------------------------
info "Organizing binaries into per-device folders …"

# TEMPORARILY scoped down — stock nrf5340_audio_dk disabled, playground only.
DEVICES=("le_audio_playground")
# DEVICES=("nrf5340_audio_dk" "le_audio_playground")
TIMINGS=("5ms" "10ms")
STRATEGIES=("fully_random" "partly_random" "occupation_aware")

# Helper: find exactly one file matching a glob in all_bins
find_bin() {
  local pattern="$1"
  local matches=("${ALL_BINS_DIR}"/${pattern})
  if [[ ${#matches[@]} -eq 0 || ! -f "${matches[0]}" ]]; then
    fail "No binary matching '${pattern}' in all_bins/"
  fi
  echo "${matches[0]}"
}

# Map timing token as it appears in filenames (5ms → 5ms_liblc3, 10ms → 10ms_T2)
timing_tag() {
  case "$1" in
    5ms)  echo "5ms_liblc3" ;;
    10ms) echo "10ms_T2"    ;;
  esac
}

for device in "${DEVICES[@]}"; do
  for timing in "${TIMINGS[@]}"; do
    tag="$(timing_tag "${timing}")"
    read -ra timing_chs <<< "$(channels_for_timing "${timing}")"
    for strategy in "${STRATEGIES[@]}"; do
      dest="${BINARIES_DIR}/${device}/${timing}/${strategy}"
      mkdir -p "${dest}"

      # Broadcaster — one binary per channel count
      for ch in "${timing_chs[@]}"; do
        ch_tag="${ch}ch"
        bcst_src="$(find_bin "grptlk_bcst_${device}_${tag}_${strategy}_${ch_tag}.hex")"
        cp "${bcst_src}" "${dest}/bcst_${device}_${timing}_${strategy}_${ch_tag}.hex"
      done

      # Receiver — single binary (syncs to whatever BIG exists)
      if [[ "${strategy}" == "occupation_aware" ]]; then
        recv_src="$(find_bin "grptlk_recv_${device}_${tag}_partly_random.hex")"
      else
        recv_src="$(find_bin "grptlk_recv_${device}_${tag}_${strategy}.hex")"
      fi
      cp "${recv_src}" "${dest}/recv_${device}_${timing}_${strategy}.hex"

      log "  ${device}/${timing}/${strategy}  ✓"
    done
  done
done

# nRF5340 DK + rbv2h organize — TEMPORARILY DISABLED
: <<'DISABLED'
# nRF5340 DK — bcst only, 2ch only, no strategy subfolder (strategy irrelevant with 1 uplink BIS)
for timing in "${TIMINGS[@]}"; do
  tag="$(timing_tag "${timing}")"
  dest="${BINARIES_DIR}/nrf5340dk/${timing}"
  mkdir -p "${dest}"

  bcst_src="$(find_bin "grptlk_bcst_nrf5340dk_${tag}_2ch.hex")"
  cp "${bcst_src}" "${dest}/bcst_nrf5340dk_${timing}_2ch.hex"

  log "  nrf5340dk/${timing}  ✓"
done

# rbv2h — recv only, per timing, per strategy
for timing in "${TIMINGS[@]}"; do
  tag="$(timing_tag "${timing}")"
  for strategy in "${STRATEGIES[@]}"; do
    dest="${BINARIES_DIR}/rbv2h/${timing}/${strategy}"
    mkdir -p "${dest}"

    if [[ "${strategy}" == "occupation_aware" ]]; then
      recv_src="$(find_bin "grptlk_recv_rbv2h_${tag}_partly_random.hex")"
    else
      recv_src="$(find_bin "grptlk_recv_rbv2h_${tag}_${strategy}.hex")"
    fi
    cp "${recv_src}" "${dest}/recv_rbv2h_${timing}_${strategy}.hex"

    log "  rbv2h/${timing}/${strategy}  ✓"
  done
done
DISABLED

# ---------------------------------------------------------------------------
# Final summary
# ---------------------------------------------------------------------------
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Build & organize complete"
echo "  ${BINARIES_DIR}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "  all_bins/  (${#BUILT[@]} raw build artifacts)"
for f in "${BUILT[@]}"; do
  SIZE=$(wc -c < "${ALL_BINS_DIR}/${f}" 2>/dev/null || echo "?")
  printf "    %-55s  %s bytes\n" "${f}" "${SIZE}"
done
echo ""
echo "  Per-device folders:"
for device in "${DEVICES[@]}"; do
  for timing in "${TIMINGS[@]}"; do
    for strategy in "${STRATEGIES[@]}"; do
      printf "    %s/%s/%s/\n" "${device}" "${timing}" "${strategy}"
      for hex in "${BINARIES_DIR}/${device}/${timing}/${strategy}"/*.hex; do
        printf "      └─ %s\n" "$(basename "${hex}")"
      done
    done
  done
done
# nrf5340dk + rbv2h summary — TEMPORARILY DISABLED
: <<'DISABLED'
for timing in "${TIMINGS[@]}"; do
  printf "    nrf5340dk/%s/\n" "${timing}"
  for hex in "${BINARIES_DIR}/nrf5340dk/${timing}"/*.hex; do
    printf "      └─ %s\n" "$(basename "${hex}")"
  done
done
for timing in "${TIMINGS[@]}"; do
  for strategy in "${STRATEGIES[@]}"; do
    printf "    rbv2h/%s/%s/\n" "${timing}" "${strategy}"
    for hex in "${BINARIES_DIR}/rbv2h/${timing}/${strategy}"/*.hex; do
      printf "      └─ %s\n" "$(basename "${hex}")"
    done
  done
done
DISABLED
echo ""
