# Copyright (c) 2025 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_subdirectory(src/split)
