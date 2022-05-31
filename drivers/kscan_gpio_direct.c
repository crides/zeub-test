/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zeub_kscan_gpio_direct

#include <device.h>
#include <drivers/kscan.h>
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(zeub, CONFIG_ZEUB_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

union work_reference {
    struct k_work_delayable delayed;
    struct k_work direct;
};

struct kscan_gpio_config {
    uint8_t num_of_inputs;
    uint8_t debounce_period;
    struct gpio_dt_spec input_specs[];
};

struct kscan_gpio_data {
    kscan_callback_t callback;
    union work_reference work;
    const struct device *dev;
    uint32_t pin_state;
};

static void kscan_gpio_direct_queue_read(union work_reference *work, uint8_t debounce_period) {
    if (debounce_period > 0) {
        k_work_reschedule(&work->delayed, K_MSEC(debounce_period));
    } else {
        k_work_submit(&work->direct);
    }
}

struct kscan_gpio_irq_callback {
    const struct device *dev;
    union work_reference *work;
    uint8_t debounce_period;
    struct gpio_callback callback;
};

static int kscan_gpio_config_interrupts(const struct device *dev, gpio_flags_t flags) {
    const struct kscan_gpio_config *cfg = dev->config;
    const struct gpio_dt_spec *specs = cfg->input_specs;

    for (int i = 0; i < cfg->num_of_inputs; i++) {
        int err = gpio_pin_interrupt_configure_dt(&specs[i], flags);

        if (err) {
            LOG_ERR("Unable to enable direct GPIO interrupt");
            return err;
        }
    }

    return 0;
}

static int kscan_gpio_direct_enable(const struct device *dev) {
    return kscan_gpio_config_interrupts(dev, GPIO_INT_LEVEL_ACTIVE);
}
static int kscan_gpio_direct_disable(const struct device *dev) {
    return kscan_gpio_config_interrupts(dev, GPIO_INT_DISABLE);
}

static void kscan_gpio_irq_callback_handler(const struct device *dev, struct gpio_callback *cb,
                                            gpio_port_pins_t pin) {
    struct kscan_gpio_irq_callback *data =
        CONTAINER_OF(cb, struct kscan_gpio_irq_callback, callback);

    kscan_gpio_direct_disable(data->dev);
    kscan_gpio_direct_queue_read(data->work, data->debounce_period);
}

static int kscan_gpio_direct_configure(const struct device *dev, kscan_callback_t callback) {
    struct kscan_gpio_data *data = dev->data;
    if (!callback) {
        return -EINVAL;
    }
    data->callback = callback;
    return 0;
}

static int kscan_gpio_read(const struct device *dev) {
    struct kscan_gpio_data *data = dev->data;
    const struct kscan_gpio_config *cfg = dev->config;
    uint32_t read_state = data->pin_state;
    bool submit_follow_up_read = false;
    for (int i = 0; i < cfg->num_of_inputs; i++) {
        const struct gpio_dt_spec *pin_spec = &cfg->input_specs[i];
        WRITE_BIT(read_state, i, gpio_pin_get_dt(pin_spec) > 0);
    }
    for (int i = 0; i < cfg->num_of_inputs; i++) {
        bool prev_pressed = BIT(i) & data->pin_state;
        bool pressed = (BIT(i) & read_state) != 0;
        submit_follow_up_read = (submit_follow_up_read || pressed);
        if (pressed != prev_pressed) {
            LOG_DBG("Sending event at %d,%d state %s", 0, i, (pressed ? "on" : "off"));
            WRITE_BIT(data->pin_state, i, pressed);
            data->callback(dev, 0, i, pressed);
        }
    }

    if (submit_follow_up_read) {
        kscan_gpio_direct_queue_read(&data->work, cfg->debounce_period);
    } else {
        kscan_gpio_direct_enable(dev);
    }

    return 0;
}

static void kscan_gpio_work_handler(struct k_work *work) {
    struct kscan_gpio_data *data = CONTAINER_OF(work, struct kscan_gpio_data, work);
    kscan_gpio_read(data->dev);
}

static const struct kscan_driver_api gpio_driver_api = {
    .config = kscan_gpio_direct_configure,
    .enable_callback = kscan_gpio_direct_enable,
    .disable_callback = kscan_gpio_direct_disable,
};

#define KSCAN_DIRECT_INPUT_ITEM(i, n) GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), input_gpios, i),

#define INST_INPUT_LEN(n) DT_INST_PROP_LEN(n, input_gpios)

#define GPIO_INST_INIT(n)                                                                          \
    static struct kscan_gpio_irq_callback irq_callbacks_##n[INST_INPUT_LEN(n)];                    \
    static struct kscan_gpio_data kscan_gpio_data_##n = {};                                        \
    static int kscan_gpio_init_##n(const struct device *dev) {                                     \
        struct kscan_gpio_data *data = dev->data;                                                  \
        const struct kscan_gpio_config *cfg = dev->config;                                         \
        int err;                                                                                   \
        const struct gpio_dt_spec *specs = cfg->input_specs;                                       \
        for (int i = 0; i < cfg->num_of_inputs; i++) {                                             \
            err = gpio_pin_configure_dt(&specs[i], GPIO_INPUT);                                    \
            if (err) {                                                                             \
                LOG_ERR("Unable to configure pin %d on for input", specs[i].pin);                  \
                return err;                                                                        \
            }                                                                                      \
            irq_callbacks_##n[i].work = &data->work; irq_callbacks_##n[i].dev = dev;               \
            irq_callbacks_##n[i].debounce_period = cfg->debounce_period;                           \
            gpio_init_callback(&irq_callbacks_##n[i].callback,                                     \
                               kscan_gpio_irq_callback_handler, BIT(specs[i].pin));                \
            err = gpio_add_callback(specs[i].port, &irq_callbacks_##n[i].callback);                \
            if (err) {                                                                             \
                LOG_ERR("Error adding the callback to the column device");                         \
                return err;                                                                        \
            }                                                                                      \
        }                                                                                          \
        data->dev = dev;                                                                           \
        if (cfg->debounce_period > 0) {                                                            \
            k_work_init_delayable(&data->work.delayed, kscan_gpio_work_handler);                   \
        } else {                                                                                   \
            k_work_init(&data->work.direct, kscan_gpio_work_handler);                              \
        }                                                                                          \
        return 0;                                                                                  \
    }                                                                                              \
    static const struct kscan_gpio_config kscan_gpio_config_##n = {                                \
        .input_specs = {UTIL_LISTIFY(INST_INPUT_LEN(n), KSCAN_DIRECT_INPUT_ITEM, n)},              \
        .num_of_inputs = INST_INPUT_LEN(n),                                                        \
        .debounce_period = DT_INST_PROP(n, debounce_period)};                                      \
    DEVICE_DT_INST_DEFINE(n, kscan_gpio_init_##n, NULL, &kscan_gpio_data_##n,                      \
                          &kscan_gpio_config_##n, POST_KERNEL, CONFIG_ZEUB_KSCAN_INIT_PRIORITY,    \
                          &gpio_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_INST_INIT)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
