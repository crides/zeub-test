# Copyright (c) 2020 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_named(zmk__drivers)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(ec11)
add_subdirectory(battery)
zephyr_library_sources(kscan_gpio_direct.c)
