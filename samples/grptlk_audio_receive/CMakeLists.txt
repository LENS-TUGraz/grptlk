# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(grptlk_audio_receive)

# Supported target: nrf5340_audio_dk/nrf5340/cpuapp with CS47L63 codec.
if(NOT CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP)
  message(FATAL_ERROR
    "grptlk_audio_receive only supports nrf5340_audio_dk/nrf5340/cpuapp")
endif()

if(NOT ZEPHYR_CIRRUS_LOGIC_MODULE_DIR)
  message(FATAL_ERROR
    "cirrus-logic module not found (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR missing)")
endif()

# Source tree layout:
#   src/
#     main.c                        — BLE BIG sync state machine + LC3 threads
#     audio/
#       audio.h                     — public audio API (init/start/volume)
#       backends/cs47l63.c          — CS47L63 codec backend (I2S DMA, mic, DAC)
#       backends/cs47l63_reg_conf.h — CS47L63 register configuration tables
#       drivers/audio_i2s.c         — nrfx I2S DMA low-level driver
#       drivers/audio_i2s.h
#       drivers/cs47l63_comm.c      — CS47L63 SPI communication layer
#       drivers/cs47l63_comm.h
#       sync/clk_sync.c             — BLE anchor → HFCLKAUDIO PI controller
#       sync/clk_sync.h
#     io/
#       led.c / led.h               — sync LED indicator

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
  src/main.c
  src/io/buttons.c
  src/io/led.c
  src/io/debug_gpio.c
  src/audio/backends/cs47l63.c
  src/audio/drivers/audio_i2s.c
  src/audio/drivers/cs47l63_comm.c
  src/audio/sync/clk_sync.c
)

add_subdirectory(${ZEPHYR_CIRRUS_LOGIC_MODULE_DIR} cirrus_logic_bin_dir)
