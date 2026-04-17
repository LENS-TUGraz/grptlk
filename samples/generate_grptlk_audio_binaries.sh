#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SAMPLES_DIR="${WORKSPACE}/grptlk/samples"
BINARIES_DIR="${SAMPLES_DIR}/binaries"
BUILD_DIR_NAME="build_gen"

BUILD_JOBS=(
  # Nordic nRF5340 Audio DK variants
  # Fully Random
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_fully_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_nrf5340_audio_dk_5ms_liblc3_fully_random.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_fully_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_nrf5340_audio_dk_10ms_T2_fully_random.hex"
  # Partly Random
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_partly_random.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_partly_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y|grptlk_recv_nrf5340_audio_dk_5ms_liblc3_partly_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y|grptlk_recv_nrf5340_audio_dk_10ms_T2_partly_random.hex"
  # Occupation Aware
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_DOWNLINK_APPENDIX=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_occupation_aware.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_DOWNLINK_APPENDIX=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_occupation_aware.hex"

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

log()  { echo "[grptlk] $*"; }
info() { echo ""; echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; echo "$*"; echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"; }
fail() { echo "[ERROR] $*" >&2; exit 1; }

check_tool() {
  command -v "$1" &>/dev/null || fail "'$1' not found in PATH. Ensure the nRF Connect toolchain is activated."
}

check_tool west
check_tool mergehex

log "west    : $(west --version 2>&1 | head -1)"
log "mergehex: $(mergehex --version 2>&1 | head -1)"
log "workspace: ${WORKSPACE}"

ALL_BINS_DIR="${BINARIES_DIR}/all_bins"
rm -rf "${BINARIES_DIR}"
mkdir -p "${ALL_BINS_DIR}"

TOTAL=${#BUILD_JOBS[@]}
BUILT=()

for i in "${!BUILD_JOBS[@]}"; do
  JOB="${BUILD_JOBS[$i]}"
  IDX=$((i + 1))

  IFS='|' read -r SAMPLE_NAME BOARD EXTRA_CMAKE OUTPUT_HEX <<< "${JOB}"

  SAMPLE_DIR="${SAMPLES_DIR}/${SAMPLE_NAME}"
  BUILD_DIR="${SAMPLE_DIR}/${BUILD_DIR_NAME}"

  info "[BUILD ${IDX}/${TOTAL}] ${SAMPLE_NAME}  board=${BOARD}  ${EXTRA_CMAKE}  →  ${OUTPUT_HEX}"

  [[ -d "${SAMPLE_DIR}" ]] || fail "Sample directory not found: ${SAMPLE_DIR}"

  (
    cd "${SAMPLE_DIR}"
    west build \
      --pristine always \
      --board "${BOARD}" \
      --build-dir "${BUILD_DIR_NAME}" \
      -- ${EXTRA_CMAKE}
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

info "Build complete — ${#BUILT[@]}/${TOTAL} binaries in all_bins/"

# ---------------------------------------------------------------------------
# Organize binaries into device / timing / strategy folders
# ---------------------------------------------------------------------------
info "Organizing binaries into per-device folders …"

DEVICES=("nrf5340_audio_dk" "le_audio_playground")
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
    for strategy in "${STRATEGIES[@]}"; do
      dest="${BINARIES_DIR}/${device}/${timing}/${strategy}"
      mkdir -p "${dest}"

      # Broadcaster — always exists for every strategy
      bcst_src="$(find_bin "grptlk_bcst_${device}_${tag}_${strategy}.hex")"
      cp "${bcst_src}" "${dest}/bcst_${device}_${timing}_${strategy}.hex"

      # Receiver — occupation_aware has no dedicated recv, reuse partly_random
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
echo ""
