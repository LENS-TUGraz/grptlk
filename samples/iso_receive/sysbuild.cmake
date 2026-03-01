# Copyright (c) 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Single source of truth for max ISO channels in sysbuild/nRF53 builds.
# Use SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT.
set_config_int(${DEFAULT_IMAGE} CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT
  ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})
set_config_int(${DEFAULT_IMAGE} CONFIG_BT_ISO_MAX_CHAN
  ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})

if(SB_CONFIG_NET_CORE_IMAGE_HCI_IPC)
  # For builds in the nrf5340, we build the netcore image with the controller

  set(NET_APP hci_ipc)
  set(NET_APP_SRC_DIR ${ZEPHYR_BASE}/samples/bluetooth/${NET_APP})
  get_filename_component(NETCORE_CONF "${CMAKE_CURRENT_LIST_DIR}/netcore.conf" ABSOLUTE)
  math(EXPR NETCORE_UL_BIS_COUNT "${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT} - 1")

  ExternalZephyrProject_Add(
    APPLICATION ${NET_APP}
    SOURCE_DIR  ${NET_APP_SRC_DIR}
    BOARD       ${SB_CONFIG_NET_CORE_BOARD}
  )

  set(${NET_APP}_CONF_FILE
    ${NET_APP_SRC_DIR}/nrf5340_cpunet_iso_receive-bt_ll_sw_split.conf
    CACHE INTERNAL ""
  )

  set(${NET_APP}_EXTRA_CONF_FILE
    ${NETCORE_CONF}
    CACHE INTERNAL ""
  )

  list(APPEND ${NET_APP}_SNIPPET ${SNIPPET})
  list(APPEND ${NET_APP}_SNIPPET bt-ll-sw-split)
  set(${NET_APP}_SNIPPET ${${NET_APP}_SNIPPET} CACHE STRING "" FORCE)

  # Keep netcore stream/ISOAL channel limits aligned with app-core.
  set_config_int(${NET_APP} CONFIG_BT_CTLR_ADV_ISO_STREAM_MAX
    ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})
  set_config_int(${NET_APP} CONFIG_BT_CTLR_ADV_ISO_STREAM_COUNT
    ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})
  set_config_int(${NET_APP} CONFIG_BT_CTLR_SYNC_ISO_STREAM_MAX
    ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})
  set_config_int(${NET_APP} CONFIG_BT_CTLR_SYNC_ISO_STREAM_COUNT
    ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})
  set_config_int(${NET_APP} CONFIG_BT_CTLR_ISOAL_SOURCES
    ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})
  set_config_int(${NET_APP} CONFIG_BT_CTLR_ISOAL_SINKS
    ${NETCORE_UL_BIS_COUNT})
  set_config_int(${NET_APP} CONFIG_BT_ISO_MAX_CHAN
    ${SB_CONFIG_GRPTLK_MAX_ISO_CHAN_COUNT})

  native_simulator_set_child_images(${DEFAULT_IMAGE} ${NET_APP})
endif()

native_simulator_set_final_executable(${DEFAULT_IMAGE})
