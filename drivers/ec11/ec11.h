/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/kscan.h>
#include <sys/util.h>

struct ec11_config {
    const struct gpio_dt_spec a_gpio, b_gpio;
    const uint8_t resolution;
};

struct ec11_data {
    uint8_t ab_state;
    int8_t pulses;
    int8_t ticks;

    struct gpio_callback a_gpio_cb;
    struct gpio_callback b_gpio_cb;
    const struct device *dev;

    kscan_callback_t callback;

#if defined(CONFIG_EC11_TRIGGER_OWN_THREAD)
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_EC11_THREAD_STACK_SIZE);
    struct k_sem gpio_sem;
    struct k_thread thread;
#elif defined(CONFIG_EC11_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif
};
