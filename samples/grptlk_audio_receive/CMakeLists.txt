# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(grptlk_audio_receive)

target_include_directories(app PRIVATE src src/io src/drivers)
target_sources(app PRIVATE src/io/led.c)

if(CONFIG_GRPTLK_AUDIO_BACKEND_CS47L63)
  if(NOT CONFIG_BOARD_NRF5340_AUDIO_DK_NRF5340_CPUAPP)
    message(FATAL_ERROR
      "CS47L63 backend is only supported on nrf5340_audio_dk/nrf5340/cpuapp")
  endif()

  target_sources(app PRIVATE
    src/main.c
    src/io/audio_cs47l63.c
    src/drivers/audio_i2s.c
    src/drivers/cs47l63_comm.c
  )

  if (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR)
    add_subdirectory(${ZEPHYR_CIRRUS_LOGIC_MODULE_DIR} cirrus_logic_bin_dir)
  else()
    message(FATAL_ERROR
      "cirrus-logic module not found (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR missing)")
  endif()
elseif(CONFIG_GRPTLK_AUDIO_BACKEND_MAX9867)
  if(NOT CONFIG_BOARD_HWN001_NRF54L15_CPUAPP)
    message(FATAL_ERROR
      "MAX9867 backend is only supported on hwn001/nrf54l15/cpuapp")
  endif()

  target_sources(app PRIVATE
    src/main.c
    src/io/audio_max9867.c
    src/drivers/max9867.c
  )
else()
  message(FATAL_ERROR "No GRPTLK audio backend selected")
endif()
