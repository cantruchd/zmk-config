// File: config/boards/shields/lily58/widgets/temperature.c

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include <zmk/display.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include "temperature.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static int16_t current_temp = 0;
static bool temp_available = false;
static lv_obj_t *temp_label;

static void read_temperature(void) {
    const struct device *dev = DEVICE_DT_GET_ONE(nordic_nrf_temp);
    
    if (!device_is_ready(dev)) {
        LOG_WRN("Temperature sensor not ready");
        temp_available = false;
        return;
    }

    struct sensor_value temp_val;
    int rc = sensor_sample_fetch(dev);
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp_val);
        if (rc == 0) {
            current_temp = temp_val.val1;
            temp_available = true;
            LOG_DBG("Temperature: %d°C", current_temp);
        }
    } else {
        LOG_WRN("Failed to fetch temperature: %d", rc);
        temp_available = false;
    }
}

static void update_display(lv_obj_t *label) {
    if (label == NULL) return;
    
    if (temp_available) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d°C", current_temp);
        lv_label_set_text(label, buf);
    } else {
        lv_label_set_text(label, "--°C");
    }
}

static void temp_work_handler(struct k_work *work) {
    read_temperature();
    update_display(temp_label);
}

K_WORK_DEFINE(temp_work, temp_work_handler);

static void temp_timer_handler(struct k_timer *timer) {
    k_work_submit(&temp_work);
}

K_TIMER_DEFINE(temp_timer, temp_timer_handler, NULL);

int zmk_widget_temperature_init(struct zmk_widget_temperature *widget, lv_obj_t *parent) {
    widget->obj = lv_obj_create(parent);
    lv_obj_set_size(widget->obj, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    
    temp_label = lv_label_create(widget->obj);
    lv_obj_align(temp_label, LV_ALIGN_CENTER, 0, 0);
    
    read_temperature();
    update_display(temp_label);
    
    k_timer_start(&temp_timer, K_SECONDS(5), K_SECONDS(30));
    
    LOG_INF("Temperature widget initialized");
    
    return 0;
}

lv_obj_t *zmk_widget_temperature_obj(struct zmk_widget_temperature *widget) {
    return widget->obj;
}

static int temperature_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->state == ZMK_ACTIVITY_IDLE || ev->state == ZMK_ACTIVITY_SLEEP) {
        k_timer_stop(&temp_timer);
        LOG_DBG("Temperature timer stopped");
    } else if (ev->state == ZMK_ACTIVITY_ACTIVE) {
        k_timer_start(&temp_timer, K_SECONDS(1), K_SECONDS(30));
        LOG_DBG("Temperature timer started");
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(temperature, temperature_listener);
ZMK_SUBSCRIPTION(temperature, zmk_activity_state_changed);
