# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

# The nRF Connect VS Code extension caches -DCONF_FILE=<build>/_sysbuild/empty.conf
# from sysbuild runs and reuses it in --no-sysbuild / --pristine builds where that
# file does not yet exist.  Create it here, before find_package(Zephyr) checks for it.
if(DEFINED CONF_FILE AND "${CONF_FILE}" MATCHES "_sysbuild/empty\\.conf$")
  get_filename_component(_sysbuild_dir "${CONF_FILE}" DIRECTORY)
  if(NOT EXISTS "${CONF_FILE}")
    file(MAKE_DIRECTORY "${_sysbuild_dir}")
    file(WRITE "${CONF_FILE}" "# Auto-generated placeholder\n")
  endif()
  # In sysbuild mode FORCED_CONF_FILE is already set to the merged .config.sysbuild,
  # which includes prj.conf.  Only redirect in standalone (--no-sysbuild) mode.
  if(NOT DEFINED FORCED_CONF_FILE)
    set(CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/prj.conf" CACHE STRING "" FORCE)
    # When CONF_FILE is overridden Zephyr no longer auto-loads the board app conf
    # (boards/<board_target>.conf).  Parse the BOARD argument ourselves (before
    # find_package(Zephyr) splits it) and pass it as EXTRA_CONF_FILE.
    if("${BOARD}" MATCHES "^([^/]+)/(.+)$")
      set(_b_name "${CMAKE_MATCH_1}")
      string(REPLACE "/" "_" _b_qual "${CMAKE_MATCH_2}")
      set(_board_conf_stem "${_b_name}_${_b_qual}")
    else()
      set(_board_conf_stem "${BOARD}")
    endif()
    set(_board_app_conf "${CMAKE_CURRENT_SOURCE_DIR}/boards/${_board_conf_stem}.conf")
    if(EXISTS "${_board_app_conf}")
      # Use a regular (non-cached) variable - Zephyr reads EXTRA_CONF_FILE as
      # a regular CMake variable, and we don't want this to persist to sysbuild runs.
      set(EXTRA_CONF_FILE "${_board_app_conf}")
    endif()
  else()
    # sysbuild mode: clear any stale EXTRA_CONF_FILE cached from a previous
    # standalone build so sysbuild gets a clean Kconfig environment.
    unset(EXTRA_CONF_FILE CACHE)
  endif()
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(grptlk_audio_broadcast)

# Supported targets:
#   nrf5340_audio_dk/nrf5340/cpuapp  — local audio I/O + CS47L63 backend
#   nrf5340dk/nrf5340/cpuapp         — relay mode (no local audio I/O)
#
# Source tree layout:
#   src/
#     main.c                         — BLE BIG broadcaster + LC3 threads
#     audio/
#       audio.h                      — public audio API (init/start/volume)
#       backends/cs47l63.c           — CS47L63 codec backend (I2S DMA, mic, DAC)
#       backends/cs47l63_reg_conf.h  — CS47L63 register configuration tables
#       drivers/audio_i2s.c          — nrfx I2S DMA low-level driver
#       drivers/audio_i2s.h
#       drivers/cs47l63_comm.c       — CS47L63 SPI communication layer
#       drivers/cs47l63_comm.h
#     io/
#       led.c / led.h                — broadcast status LED

target_include_directories(app PRIVATE src)

# Common sources (all boards)
target_sources(app PRIVATE
  src/main.c
  src/io/led.c
  src/io/debug_gpio.c
)

# Local audio path uses the real codec backend; relay-only path uses stubs.
if(CONFIG_GRPTLK_LOCAL_AUDIO_IO)
  if(CONFIG_HW_CODEC_CIRRUS_LOGIC)
    target_sources(app PRIVATE
      src/audio/backends/cs47l63.c
      src/audio/drivers/audio_i2s.c
      src/audio/drivers/cs47l63_comm.c
      src/audio/sync/clk_sync.c
    )

    if(NOT ZEPHYR_CIRRUS_LOGIC_MODULE_DIR)
      message(FATAL_ERROR
        "cirrus-logic module not found (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR missing)")
    endif()

    add_subdirectory(${ZEPHYR_CIRRUS_LOGIC_MODULE_DIR} cirrus_logic_bin_dir)
  else()
    message(FATAL_ERROR
      "CONFIG_GRPTLK_LOCAL_AUDIO_IO requires a local audio backend")
  endif()
else()
  target_sources(app PRIVATE
    src/audio/backends/null_audio.c
    src/audio/sync/clk_sync_stub.c
  )
endif()
