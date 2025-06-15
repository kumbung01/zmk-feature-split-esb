# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_sources( common.c )
zephyr_library_sources_ifdef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL  central.c)
zephyr_library_sources_ifndef(CONFIG_ZMK_SPLIT_ROLE_CENTRAL peripheral.c)

# zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/include)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

if(CONFIG_SOC_NRF5340_CPUAPP)
    zephyr_library_sources(
        53_app/app_esb_53_app.c
    )
else ()
    # assume a 52 series board is selected
    zephyr_library_sources(app_esb.c)
    zephyr_library_sources(timeslot.c)
endif()
zephyr_library_include_directories(.)
