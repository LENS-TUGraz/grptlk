# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(grptlk_audio_receive)

if(NOT CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP)
  message(FATAL_ERROR
    "grptlk_audio_receive only supports nrf5340_audio_dk/nrf5340/cpuapp")
endif()

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE
  src/main.c
  src/io/buttons.c
  src/io/led.c
  src/io/debug_gpio.c
  src/audio/audio.c
  src/audio/drivers/audio_i2s.c
  src/audio/sync/clk_sync.c
)

if(CONFIG_GRPTLK_AUDIO_CODEC_CIRRUS)
  if(NOT ZEPHYR_CIRRUS_LOGIC_MODULE_DIR)
    message(FATAL_ERROR
      "cirrus-logic module not found (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR missing)")
  endif()

  target_sources(app PRIVATE
    src/audio/backends/cs47l63.c
    src/audio/drivers/cs47l63_comm.c
  )

  add_subdirectory(${ZEPHYR_CIRRUS_LOGIC_MODULE_DIR} cirrus_logic_bin_dir)
elseif(CONFIG_GRPTLK_AUDIO_CODEC_MAX9867)
  target_sources(app PRIVATE
    src/audio/backends/max9867.c
  )
else()
  message(FATAL_ERROR "No GRPTLK audio codec backend selected")
endif()
