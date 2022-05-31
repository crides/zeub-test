#include <device.h>
#include <drivers/display.h>
#include <drivers/sensor.h>
#include <lvgl.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(zeub_disp, CONFIG_ZEUB_LOG_LEVEL);

#define TICK_MS 10

static const struct device *oled = DEVICE_DT_GET(DT_NODELABEL(oled)),
             *temp_sensor = DEVICE_DT_GET(DT_NODELABEL(tmp117));
struct disp_screen {
    lv_obj_t *screen,
             *time, *temp;
};

static struct disp_screen screen;

static void display_tick_cb(struct k_work *work);
static void display_timer_cb();
static K_WORK_DEFINE(display_tick_work, display_tick_cb);
static K_THREAD_STACK_DEFINE(display_work_stack_area, CONFIG_ZEUB_DISPLAY_THREAD_STACK_SIZE);
static K_TIMER_DEFINE(display_timer, display_timer_cb, NULL);

static void temp_update_work_cb(struct k_work *work);
static void temp_update_timer_cb();
static K_WORK_DEFINE(temp_update_work, temp_update_work_cb);
static K_TIMER_DEFINE(temp_update_timer, temp_update_timer_cb, NULL);

static struct k_work_q display_work_q;

static void display_timer_cb() {
    lv_tick_inc(TICK_MS);
    k_work_submit_to_queue(&display_work_q, &display_tick_work);
}

static void display_tick_cb(struct k_work *work) { lv_task_handler(); }

static void temp_update_timer_cb() { k_work_submit_to_queue(&display_work_q, &temp_update_work); }

static void temp_update_work_cb(struct k_work *work) {
    int ret;
    ret = sensor_sample_fetch(temp_sensor);
    if (ret < 0) {
        LOG_ERR("temp fetch %d", ret);
        return;
    }
    struct sensor_value temp;
    ret = sensor_channel_get(temp_sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp);
    if (ret < 0) {
        LOG_ERR("temp get %d", ret);
        return;
    }
    lv_label_set_text_fmt(screen.temp, "temp: %d.%d C", temp.val1, temp.val2);
    LOG_INF("temp: %d.%d C", temp.val1, temp.val2);
}

static void init_ui() {
    screen.screen = lv_obj_create(NULL, NULL);
    screen.time = lv_obj_create(screen.screen, NULL);
    screen.temp = lv_obj_create(screen.screen, NULL);
    /* lv_label_set_text(screen.temp, "------------------------------"); */
    lv_obj_align(screen.time, screen.screen, LV_ALIGN_IN_TOP_LEFT, 0, 0);
    lv_obj_set_width(screen.time, CONFIG_LVGL_HOR_RES_MAX);
    lv_obj_align(screen.temp, screen.screen, LV_ALIGN_IN_BOTTOM_LEFT, 0, 0);
    lv_obj_set_width(screen.temp, CONFIG_LVGL_HOR_RES_MAX);
}

static int disp_init(const struct device *_arg) {
    display_blanking_off(oled);

    init_ui();
    k_work_submit_to_queue(&display_work_q, &temp_update_work);
    lv_scr_load(screen.screen);

    k_timer_start(&temp_update_timer, K_MSEC(CONFIG_TEMP_UPDATE_INTERVAL_MS), K_MSEC(CONFIG_TEMP_UPDATE_INTERVAL_MS));
    k_timer_start(&display_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
    k_work_queue_start(&display_work_q, display_work_stack_area,
                       K_THREAD_STACK_SIZEOF(display_work_stack_area),
                       CONFIG_ZEUB_DISPLAY_THREAD_PRIORITY, NULL);
}

SYS_INIT(disp_init, APPLICATION, CONFIG_DISPLAY_INIT_PRIORITY);
