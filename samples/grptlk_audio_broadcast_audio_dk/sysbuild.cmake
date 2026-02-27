# Copyright (c) 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

if(DEFINED WEST_PYTHON)
  # Keep all sysbuild child images on the same Python interpreter as the
  # top-level invocation (avoids host/Homebrew Python mismatches).
  set(Python3_EXECUTABLE ${WEST_PYTHON} CACHE FILEPATH "Sysbuild Python" FORCE)
  set(PYTHON_EXECUTABLE ${WEST_PYTHON} CACHE FILEPATH "Sysbuild Python" FORCE)
endif()

if(SB_CONFIG_NET_CORE_IMAGE_HCI_IPC)
  # For builds in the nrf5340, we build the netcore image with the controller

  set(NET_APP hci_ipc)
  set(NET_APP_SRC_DIR ${ZEPHYR_BASE}/samples/bluetooth/${NET_APP})
  get_filename_component(NETCORE_CONF "${CMAKE_CURRENT_LIST_DIR}/netcore.conf" ABSOLUTE)

  ExternalZephyrProject_Add(
    APPLICATION ${NET_APP}
    SOURCE_DIR  ${NET_APP_SRC_DIR}
    BOARD       ${SB_CONFIG_NET_CORE_BOARD}
  )

  if(DEFINED WEST_PYTHON)
    set_property(TARGET ${NET_APP} APPEND PROPERTY _EP_CMAKE_ARGS
      -DPython3_EXECUTABLE:FILEPATH=${WEST_PYTHON}
      -DPYTHON_EXECUTABLE:FILEPATH=${WEST_PYTHON}
    )
  endif()

  set(${NET_APP}_CONF_FILE
    ${NET_APP_SRC_DIR}/nrf5340_cpunet_iso-bt_ll_sw_split.conf
    CACHE INTERNAL ""
  )

  set(${NET_APP}_EXTRA_CONF_FILE
    ${NETCORE_CONF}
    CACHE INTERNAL ""
  )

  list(APPEND ${NET_APP}_SNIPPET ${SNIPPET})
  list(APPEND ${NET_APP}_SNIPPET bt-ll-sw-split)
  set(${NET_APP}_SNIPPET ${${NET_APP}_SNIPPET} CACHE STRING "" FORCE)

  native_simulator_set_child_images(${DEFAULT_IMAGE} ${NET_APP})
endif()

native_simulator_set_final_executable(${DEFAULT_IMAGE})
