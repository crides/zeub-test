#include <device.h>
#include <drivers/gpio.h>
#include <drivers/led.h>
#include <drivers/kscan.h>
#include <drivers/sensor.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(zeub, CONFIG_ZEUB_LOG_LEVEL);

#define WHITE_BRIGHT 0
#define YELLOW_BRIGHT 20

static const struct device *led_strip = DEVICE_DT_GET(DT_NODELABEL(led_strip)),
             *buttons = DEVICE_DT_GET(DT_NODELABEL(buttons)),
             *roller = DEVICE_DT_GET(DT_NODELABEL(roller));
static uint8_t channel = 0;
static uint8_t brights[4] = {WHITE_BRIGHT, YELLOW_BRIGHT, 0, 0};

static int set_bright() {
    int ret;
    for (uint8_t i = 0; i < sizeof(brights); i ++) {
        ret = led_set_brightness(led_strip, i, brights[i]);
        if (ret < 0) {
            LOG_ERR("set strip %d: %d", i, ret);
            return ret;
        }
    }
    return 0;
}

enum button {
    ENC_PUSH = 0,
    SEL_UP,
    SEL_DOWN,
    SEL_LEFT,
    SEL_RIGHT,
    SEL_PUSH,
    ROLL_UP,
    ROLL_DOWN,
};

static void handle_button(const struct device *dev, enum button button, bool pressed) {
    int ret;
    if (pressed) {
        switch (button) {
        case SEL_PUSH:
            break;
        case ENC_PUSH:
            channel = !channel;
            break;
        case ROLL_UP:
            if (brights[channel * 2 + 1] < 100) {
                brights[channel * 2 + 1] += 5;
            }
            set_bright();
            break;
        case ROLL_DOWN:
            if (brights[channel * 2 + 1] > 0) {
                brights[channel * 2 + 1] -= 5;
            }
            set_bright();
            break;
        default:
            break;
        }
    }
}

static inline void buttons_cb(const struct device *dev, uint32_t row, uint32_t col, bool pressed) {
    LOG_DBG("buttons %u %d", col, pressed);
    handle_button(dev, col, pressed);
}

static inline void roller_cb(const struct device *dev, uint32_t row, uint32_t col, bool pressed) {
    LOG_DBG("roller %u %d", col, pressed);
    handle_button(dev, col + ROLL_UP, pressed);
}

int main() {
    int ret;
    ret = led_set_brightness(led_strip, 0, WHITE_BRIGHT);
    if (ret < 0) {
        LOG_ERR("white 0 %d", ret);
        return ret;
    }
    ret = led_set_brightness(led_strip, 1, YELLOW_BRIGHT);
    if (ret < 0) {
        LOG_ERR("yellow 0 %d", ret);
        return ret;
    }

    kscan_config(roller, roller_cb);
    kscan_enable_callback(roller);
    kscan_config(buttons, buttons_cb);
    kscan_enable_callback(buttons);

    return 0;
}
