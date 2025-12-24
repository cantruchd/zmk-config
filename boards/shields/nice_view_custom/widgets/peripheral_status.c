/*
 * Copyright (c) 2023 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/battery.h>
#include <zmk/display.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/split/bluetooth/peripheral.h>
#include <zmk/events/split_peripheral_status_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/usb.h>
#include <zmk/ble.h>

#include "peripheral_status.h"

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

struct peripheral_status_state {
    bool connected;
};

// ========================================
// TEMPERATURE SENSOR
// ========================================

static void read_temperature(struct zmk_widget_status *widget) {
#if IS_ENABLED(CONFIG_ZMK_WIDGET_TEMPERATURE)
    const struct device *dev = DEVICE_DT_GET_ONE(nordic_nrf_temp);
    
    if (!device_is_ready(dev)) {
        LOG_WRN("Temperature sensor not ready");
        widget->state.temperature = 0;
        return;
    }

    struct sensor_value temp_val;
    int rc = sensor_sample_fetch(dev);
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp_val);
        if (rc == 0) {
            widget->state.temperature = temp_val.val1;
            LOG_DBG("Temperature: %d°C", widget->state.temperature);
        }
    } else {
        LOG_WRN("Failed to fetch temperature: %d", rc);
    }
#endif
}

static void temp_work_handler(struct k_work *work) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) {
        read_temperature(widget);
        // Redraw
        lv_obj_t *canvas = lv_obj_get_child(widget->obj, 0);
        if (canvas) {
            lv_draw_rect_dsc_t rect_black_dsc;
            init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);
            
            lv_draw_label_dsc_t label_dsc;
            init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lv_font_montserrat_16, LV_TEXT_ALIGN_RIGHT);
            
            lv_draw_label_dsc_t label_dsc_battery;
            init_label_dsc(&label_dsc_battery, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_CENTER);
            
            // Fill background
            lv_canvas_draw_rect(canvas, 0, 0, CANVAS_SIZE, CANVAS_SIZE, &rect_black_dsc);
            
            // Draw battery
            draw_battery(canvas, &widget->state);
            
            // Draw WiFi
            lv_canvas_draw_text(canvas, 0, 0, CANVAS_SIZE, &label_dsc,
                              widget->state.connected ? LV_SYMBOL_WIFI : LV_SYMBOL_CLOSE);
            
            // Draw battery percentage
            char battery_text[5] = {};
            snprintf(battery_text, sizeof(battery_text), "%d%%", widget->state.battery);
            lv_canvas_draw_text(canvas, 0, 20, 68, &label_dsc_battery, battery_text);
            
            rotate_canvas(canvas, widget->cbuf);
        }
        
        // Update temperature display
        lv_obj_t *temp_label = lv_obj_get_child(widget->obj, 1);
        if (temp_label) {
            char temp_text[16];
            if (widget->state.temperature != 0) {
                snprintf(temp_text, sizeof(temp_text), "%d°C", widget->state.temperature);
            } else {
                snprintf(temp_text, sizeof(temp_text), "--°C");
            }
            lv_label_set_text(temp_label, temp_text);
        }
    }
}

K_WORK_DEFINE(temp_work, temp_work_handler);

static void temp_timer_handler(struct k_timer *timer) {
    k_work_submit(&temp_work);
}

K_TIMER_DEFINE(temp_timer, temp_timer_handler, NULL);

// ========================================
// DRAWING FUNCTIONS
// ========================================

static void draw_top(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state) {
    lv_obj_t *canvas = lv_obj_get_child(widget, 0);

    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lv_font_montserrat_16, LV_TEXT_ALIGN_RIGHT);
    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);

    // Fill background
    lv_canvas_draw_rect(canvas, 0, 0, CANVAS_SIZE, CANVAS_SIZE, &rect_black_dsc);

    // Draw battery
    draw_battery(canvas, state);

    // Draw output status
    lv_canvas_draw_text(canvas, 0, 0, CANVAS_SIZE, &label_dsc,
                        state->connected ? LV_SYMBOL_WIFI : LV_SYMBOL_CLOSE);

    lv_draw_label_dsc_t label_dsc_battery;
    init_label_dsc(&label_dsc_battery, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_CENTER);
    // Draw battery percentage
    char battery_text[5] = {};
    snprintf(battery_text, sizeof(battery_text), "%d%%", state->battery);
    lv_canvas_draw_text(canvas, 0, 20, 68, &label_dsc_battery, battery_text);

    // Rotate canvas
    rotate_canvas(canvas, cbuf);
}

// ========================================
// BATTERY STATUS
// ========================================

static void set_battery_status(struct zmk_widget_status *widget,
                               struct battery_status_state state) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    widget->state.charging = state.usb_present;
#endif

    widget->state.battery = state.level;

    draw_top(widget->obj, widget->cbuf, &widget->state);
}

static void battery_status_update_cb(struct battery_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { 
        set_battery_status(widget, state); 
    }
}

static struct battery_status_state battery_status_get_state(const zmk_event_t *eh) {
    return (struct battery_status_state){
        .level = zmk_battery_state_of_charge(),
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
        .usb_present = zmk_usb_is_powered(),
#endif
    };
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_battery_status, struct battery_status_state,
                            battery_status_update_cb, battery_status_get_state)

ZMK_SUBSCRIPTION(widget_battery_status, zmk_battery_state_changed);
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_SUBSCRIPTION(widget_battery_status, zmk_usb_conn_state_changed);
#endif

// ========================================
// PERIPHERAL CONNECTION STATUS
// ========================================

static struct peripheral_status_state get_state(const zmk_event_t *_eh) {
    return (struct peripheral_status_state){.connected = zmk_split_bt_peripheral_is_connected()};
}

static void set_connection_status(struct zmk_widget_status *widget,
                                  struct peripheral_status_state state) {
    widget->state.connected = state.connected;

    draw_top(widget->obj, widget->cbuf, &widget->state);
}

static void output_status_update_cb(struct peripheral_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { 
        set_connection_status(widget, state); 
    }
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_peripheral_status, struct peripheral_status_state,
                            output_status_update_cb, get_state)
ZMK_SUBSCRIPTION(widget_peripheral_status, zmk_split_peripheral_status_changed);

// ========================================
// ACTIVITY STATE (để tắt timer khi idle)
// ========================================

#if IS_ENABLED(CONFIG_ZMK_WIDGET_TEMPERATURE)
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

ZMK_LISTENER(widget_temperature, temperature_listener);
ZMK_SUBSCRIPTION(widget_temperature, zmk_activity_state_changed);
#endif

// ========================================
// WIDGET INITIALIZATION
// ========================================

int zmk_widget_status_init(struct zmk_widget_status *widget, lv_obj_t *parent) {
    widget->obj = lv_obj_create(parent);
    lv_obj_set_size(widget->obj, 160, 68);
    
    // Canvas bên phải (Battery + WiFi)
    lv_obj_t *top = lv_canvas_create(widget->obj);
    lv_obj_align(top, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_canvas_set_buffer(top, widget->cbuf, CANVAS_SIZE, CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR);

#if IS_ENABLED(CONFIG_ZMK_WIDGET_TEMPERATURE)
    // Temperature label bên trái (thay random art)
    lv_obj_t *temp_label = lv_label_create(widget->obj);
    lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_26, LV_PART_MAIN);
    lv_obj_set_style_text_color(temp_label, lv_color_white(), LV_PART_MAIN);
    lv_label_set_text(temp_label, "--°C");
    lv_obj_align(temp_label, LV_ALIGN_TOP_LEFT, -48, 0);
    
    // Init state
    widget->state.temperature = 0;
    
    // Đọc nhiệt độ ngay lập tức
    read_temperature(widget);
    
    // Start timer (đọc mỗi 30s)
    k_timer_start(&temp_timer, K_SECONDS(5), K_SECONDS(30));
    
    LOG_INF("Temperature widget enabled");
#else
    // Fallback: Hiển thị random art như cũ nếu không enable temperature
    LV_IMG_DECLARE(balloon);
    LV_IMG_DECLARE(mountain);
    
    lv_obj_t *art = lv_img_create(widget->obj);
    bool random = sys_rand32_get() & 1;
    lv_img_set_src(art, random ? &balloon : &mountain);
    lv_obj_align(art, LV_ALIGN_TOP_LEFT, -48, 0);
#endif

    sys_slist_append(&widgets, &widget->node);
    widget_battery_status_init();
    widget_peripheral_status_init();

    return 0;
}

lv_obj_t *zmk_widget_status_obj(struct zmk_widget_status *widget) { 
    return widget->obj; 
}
