# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(grptlk_audio_broadcast_audio_dk)

target_sources(app PRIVATE
  src/main.c
  src/drivers/audio_i2s.c
  src/drivers/cs47l63_comm.c
)

target_include_directories(app PRIVATE
  src
  src/drivers
)

if (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR)
  add_subdirectory(${ZEPHYR_CIRRUS_LOGIC_MODULE_DIR} cirrus_logic_bin_dir)
else()
  message(FATAL_ERROR "cirrus-logic module not found (ZEPHYR_CIRRUS_LOGIC_MODULE_DIR missing)")
endif()
