# Copyright (c) 2020-2021 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources(battery_common.c battery_voltage_divider.c)
