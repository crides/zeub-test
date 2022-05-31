/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT alps_ec11

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/kscan.h>
#include <sys/util.h>
#include <kernel.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "ec11.h"

LOG_MODULE_REGISTER(EC11, CONFIG_SENSOR_LOG_LEVEL);

int ec11_config_int(const struct device *dev, bool enable) {
    int ret;
    const struct ec11_config *cfg = dev->config;

    LOG_DBG("enabled %s", (enable ? "true" : "false"));

    ret = gpio_pin_interrupt_configure_dt(&cfg->a_gpio, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Unable to set A pin GPIO interrupt");
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&cfg->b_gpio, enable ? GPIO_INT_EDGE_BOTH : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("Unable to set B pin GPIO interrupt");
        return ret;
    }
    return 0;
}

static void ec11_a_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                 uint32_t pins) {
    struct ec11_data *drv_data = CONTAINER_OF(cb, struct ec11_data, a_gpio_cb);

    LOG_DBG("");

    ec11_config_int(drv_data->dev, false);

#if defined(CONFIG_EC11_TRIGGER_OWN_THREAD)
    k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_EC11_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&drv_data->work);
#endif
}

static void ec11_b_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                 uint32_t pins) {
    struct ec11_data *drv_data = CONTAINER_OF(cb, struct ec11_data, b_gpio_cb);

    LOG_DBG("");

    ec11_config_int(drv_data->dev, false);

#if defined(CONFIG_EC11_TRIGGER_OWN_THREAD)
    k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_EC11_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&drv_data->work);
#endif
}

static int ec11_get_ab_state(const struct device *dev) {
    const struct ec11_config *drv_cfg = dev->config;

    return (gpio_pin_get_dt(&drv_cfg->a_gpio) << 1) | gpio_pin_get_dt(&drv_cfg->b_gpio);
}

static int8_t ec11_get(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;
    uint8_t val;
    int8_t delta;

    val = ec11_get_ab_state(dev);

    LOG_DBG("prev: %d, new: %d", drv_data->ab_state, val);

    switch (val | (drv_data->ab_state << 2)) {
    case 0b0010:
    case 0b0100:
    case 0b1101:
    case 0b1011:
        delta = -1;
        break;
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
        delta = 1;
        break;
    default:
        delta = 0;
        break;
    }

    LOG_DBG("Delta: %d", delta);

    drv_data->pulses += delta;
    drv_data->ab_state = val;

    const int8_t ticks = drv_data->pulses / drv_cfg->resolution;
    drv_data->pulses %= drv_cfg->resolution;

    return ticks;
}

static void ec11_thread_cb(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const int8_t res = ec11_get(dev);
    if (res != 0) {
        const int8_t key = res == -1;
        drv_data->callback(dev, 0, key, true);
        k_msleep(5);
        drv_data->callback(dev, 0, key, false);
    }

    ec11_config_int(dev, true);
}

#ifdef CONFIG_EC11_TRIGGER_OWN_THREAD
static void ec11_thread(int dev_ptr, int unused) {
    const struct device *dev = INT_TO_POINTER(dev_ptr);
    struct ec11_data *drv_data = dev->data;

    ARG_UNUSED(unused);

    while (1) {
        k_sem_take(&drv_data->gpio_sem, K_FOREVER);
        ec11_thread_cb(dev);
    }
}
#endif

#ifdef CONFIG_EC11_TRIGGER_GLOBAL_THREAD
static void ec11_work_cb(struct k_work *work) {
    struct ec11_data *drv_data = CONTAINER_OF(work, struct ec11_data, work);

    LOG_DBG("");

    ec11_thread_cb(drv_data->dev);
}
#endif

int ec11_kscan_config(const struct device *dev, kscan_callback_t callback) {
    struct ec11_data *drv_data = dev->data;
    drv_data->callback = callback;
    return 0;
}

int ec11_init_interrupt(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    drv_data->dev = dev;
    /* setup gpio interrupt */

    gpio_init_callback(&drv_data->a_gpio_cb, ec11_a_gpio_callback, BIT(drv_cfg->a_gpio.pin));

    if (gpio_add_callback(drv_cfg->a_gpio.port, &drv_data->a_gpio_cb) < 0) {
        LOG_DBG("Failed to set A callback!");
        return -EIO;
    }

    gpio_init_callback(&drv_data->b_gpio_cb, ec11_b_gpio_callback, BIT(drv_cfg->b_gpio.pin));

    if (gpio_add_callback(drv_cfg->b_gpio.port, &drv_data->b_gpio_cb) < 0) {
        LOG_DBG("Failed to set B callback!");
        return -EIO;
    }

#if defined(CONFIG_EC11_TRIGGER_OWN_THREAD)
    k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

    k_thread_create(&drv_data->thread, drv_data->thread_stack, CONFIG_EC11_THREAD_STACK_SIZE,
                    (k_thread_entry_t)ec11_thread, (void *) dev, 0, NULL,
                    K_PRIO_COOP(CONFIG_EC11_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_EC11_TRIGGER_GLOBAL_THREAD)
    k_work_init(&drv_data->work, ec11_work_cb);
#endif

    return 0;
}
static int ec11_kscan_enable(const struct device *dev) {
    return ec11_config_int(dev, true);
}

static int ec11_kscan_disable(const struct device *dev) {
    return ec11_config_int(dev, false);
}

static const struct kscan_driver_api ec11_driver_api = {
    .config = ec11_kscan_config,
    .enable_callback = ec11_kscan_enable,
    .disable_callback = ec11_kscan_disable,
};

int ec11_init(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    LOG_DBG("A: %d B: %d resolution %d", drv_cfg->a_gpio.pin, drv_cfg->b_gpio.pin, drv_cfg->resolution);

    if (gpio_pin_configure_dt(&drv_cfg->a_gpio, GPIO_INPUT)) {
        LOG_DBG("Failed to configure A pin");
        return -EIO;
    }

    if (gpio_pin_configure_dt(&drv_cfg->b_gpio, GPIO_INPUT)) {
        LOG_DBG("Failed to configure B pin");
        return -EIO;
    }

    if (ec11_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }

    drv_data->ab_state = ec11_get_ab_state(dev);

    return 0;
}

#define EC11_INST(n)                                                                               \
    struct ec11_data ec11_data_##n = {                                                             \
    };                                                                                             \
    const struct ec11_config ec11_cfg_##n = {                                                      \
        .a_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(n), a_gpios),                                       \
        .b_gpio = GPIO_DT_SPEC_GET(DT_DRV_INST(n), b_gpios),                                       \
        COND_CODE_0(DT_INST_NODE_HAS_PROP(n, resolution), (1), (DT_INST_PROP(n, resolution))),     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, ec11_init, NULL, &ec11_data_##n, &ec11_cfg_##n, POST_KERNEL,          \
                          CONFIG_SENSOR_INIT_PRIORITY, &ec11_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EC11_INST)
