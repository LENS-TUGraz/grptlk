#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SAMPLES_DIR="${WORKSPACE}/grptlk/samples"
BINARIES_DIR="${SAMPLES_DIR}/binaries"
BUILD_DIR_NAME="build_gen"

BUILD_JOBS=(
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_fully_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_nrf5340_audio_dk_5ms_liblc3_fully_random.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_INIT_LC3_CONSTANTLY=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_fully_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y -DCONFIG_GRPTLK_UPLINK_RANDOM_PER_PTT=n -DCONFIG_GRPTLK_UPLINK_RANDOM=y|grptlk_recv_nrf5340_audio_dk_10ms_T2_fully_random.hex"

  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y|grptlk_bcst_nrf5340_audio_dk_5ms_liblc3_partly_random.hex"
  "grptlk_audio_broadcast|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y|grptlk_bcst_nrf5340_audio_dk_10ms_T2_partly_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_5_MS=y|grptlk_recv_nrf5340_audio_dk_5ms_liblc3_partly_random.hex"
  "grptlk_audio_receive|nrf5340_audio_dk/nrf5340/cpuapp|-DCONFIG_GRPTLK_AUDIO_FRAME_10_MS=y|grptlk_recv_nrf5340_audio_dk_10ms_T2_partly_random.hex"
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

rm -rf "${BINARIES_DIR}"
mkdir -p "${BINARIES_DIR}"

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
  OUT_HEX="${BINARIES_DIR}/${OUTPUT_HEX}"

  [[ -f "${HCI_HEX}" ]] || fail "hci_ipc hex not found: ${HCI_HEX}"
  [[ -f "${APP_HEX}" ]] || fail "App hex not found: ${APP_HEX}"

  mergehex -m "${HCI_HEX}" "${APP_HEX}" -o "${OUT_HEX}"

  log "Merged → ${OUT_HEX}"
  BUILT+=("${OUTPUT_HEX}")
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Build complete — ${#BUILT[@]}/${TOTAL} binaries in:"
echo "  ${BINARIES_DIR}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
for f in "${BUILT[@]}"; do
  SIZE=$(wc -c < "${BINARIES_DIR}/${f}" 2>/dev/null || echo "?")
  printf "  %-55s  %s bytes\n" "${f}" "${SIZE}"
done
echo ""
