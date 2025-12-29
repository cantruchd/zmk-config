/*
 *
 * Copyright (c) 2025 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/battery.h>
#include <zmk/display.h>
#include "status.h"
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/wpm_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/usb.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/keymap.h>
#include <zmk/wpm.h>

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

struct output_status_state {
    struct zmk_endpoint_instance selected_endpoint;
    int active_profile_index;
    bool active_profile_connected;
    bool active_profile_bonded;
    bool profiles_connected[NICEVIEW_PROFILE_COUNT];
    bool profiles_bonded[NICEVIEW_PROFILE_COUNT];
};

struct layer_status_state {
    zmk_keymap_layer_index_t index;
    const char *label;
};

struct wpm_status_state {
    uint8_t wpm;
};

int max = 0;
int min = 256;
char logtext[10];

static uint16_t current_voltage_mv = 0;

// Biến static để ghi nhớ channel đúng sau lần dò đầu tiên
static enum sensor_channel discovered_channel = SENSOR_CHAN_PRIV_START; // Giá trị mặc định chưa xác định

static void read_battery_voltage(void) {
    const struct device *battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
    
    if (!device_is_ready(battery)) {
        LOG_WRN("Battery sensor not ready");
        return;
    }

    // 1. Lấy mẫu dữ liệu từ cảm biến (Fetch)
    int rc = sensor_sample_fetch(battery);
    if (rc != 0) {
        LOG_WRN("Failed to fetch battery: %d", rc);
        return;
    }

    struct sensor_value voltage;

    // 2. Nếu đã biết channel đúng, lấy trực tiếp luôn
    if (discovered_channel != SENSOR_CHAN_PRIV_START) {
        rc = sensor_channel_get(battery, discovered_channel, &voltage);
    } 
    // 3. Nếu chưa biết (lần đầu chạy), tiến hành dò tìm
    else {
        static const enum sensor_channel candidates[] = {
            SENSOR_CHAN_VOLTAGE,
            SENSOR_CHAN_ALL,
            SENSOR_CHAN_GAUGE_VOLTAGE            
        };

        for (int i = 0; i < ARRAY_SIZE(candidates); i++) {
            rc = sensor_channel_get(battery, candidates[i], &voltage);
            if (rc == 0) {
                discovered_channel = candidates[i]; // Ghi nhớ channel này
                sprintf(logtext, "Ch %d", i + 1); // Ghi log channel tìm được
                break;
            }
        }
    }

    // 4. Xử lý kết quả cuối cùng
    if (rc == 0) {
        // Tính toán mV: val1 (Volts), val2 (Microvolts)
        current_voltage_mv = (voltage.val1 * 1000) + (voltage.val2 / 1000);
        
        // Chỉ log khi cần thiết để tránh tràn log buffer
        LOG_INF("Voltage: %d mV", current_voltage_mv);
    } else {
        LOG_ERR("No valid voltage channel found");
        current_voltage_mv = 0;
    }
}


// ========================================
// TEMPERATURE SENSOR
// ========================================

static struct sensor_value current_temp_val = {0};
static bool temp_data_valid = false;

static void draw_top(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state);
static void draw_middle(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state);

static void read_temperature(void) {
    const struct device *dev = DEVICE_DT_GET_ONE(nordic_nrf_temp);
    
    if (!device_is_ready(dev)) {
        LOG_WRN("Temperature sensor not ready");
        temp_data_valid = false;
        return;
    }

    struct sensor_value temp_val;
    int rc = sensor_sample_fetch(dev);
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp_val);
        if (rc == 0) {
            current_temp_val = temp_val;
            temp_data_valid = true;
            LOG_DBG("Temperature: %d.%d°C", temp_val.val1, abs(temp_val.val2 / 100000));
        }
    } else {
        LOG_WRN("Failed to fetch temperature: %d", rc);
        temp_data_valid = false;
    }
}

static void temp_work_handler(struct k_work *work) {
    read_temperature();
    read_battery_voltage();
    
    // Redraw all widgets
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) {
        draw_top(widget->obj, widget->cbuf, &widget->state);
        draw_middle(widget->obj, widget->cbuf2, &widget->state);
    }
}

K_WORK_DEFINE(temp_work, temp_work_handler);

static void temp_timer_handler(struct k_timer *timer) {
    k_work_submit(&temp_work);
}

K_TIMER_DEFINE(temp_timer, temp_timer_handler, NULL);

// Activity listener để tắt timer khi idle
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

// ========================================
// DRAWING FUNCTIONS
// ========================================

static void draw_top(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state) {
    lv_obj_t *canvas = lv_obj_get_child(widget, 0);

    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lv_font_montserrat_16, LV_TEXT_ALIGN_RIGHT);
    lv_draw_label_dsc_t label_dsc_wpm;
    init_label_dsc(&label_dsc_wpm, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_RIGHT);
    lv_draw_label_dsc_t label_dsc_temp;
    init_label_dsc(&label_dsc_temp, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_LEFT);
    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);
    lv_draw_rect_dsc_t rect_white_dsc;
    init_rect_dsc(&rect_white_dsc, LVGL_FOREGROUND);
    lv_draw_line_dsc_t line_dsc;
    init_line_dsc(&line_dsc, LVGL_FOREGROUND, 1);

    lv_draw_label_dsc_t label_dsc_temp_unit;
    init_label_dsc(&label_dsc_temp_unit, LVGL_FOREGROUND, &lv_font_montserrat_14, LV_TEXT_ALIGN_RIGHT);


    // Fill background
    lv_canvas_draw_rect(canvas, 0, 0, CANVAS_SIZE, CANVAS_SIZE, &rect_black_dsc);

    // Draw battery
    draw_battery(canvas, state);

    // Draw output status
    char output_text[10] = {};

    switch (state->selected_endpoint.transport) {
    case ZMK_TRANSPORT_USB:
        strcat(output_text, LV_SYMBOL_USB);
        break;
    case ZMK_TRANSPORT_BLE:
        if (state->active_profile_bonded) {
            if (state->active_profile_connected) {
                strcat(output_text, LV_SYMBOL_WIFI);
            } else {
                strcat(output_text, LV_SYMBOL_CLOSE);
            }
        } else {
            strcat(output_text, LV_SYMBOL_SETTINGS);
        }
        break;
    }

    lv_canvas_draw_text(canvas, 0, 0, CANVAS_SIZE, &label_dsc, output_text);

    // Draw WPM box
    lv_canvas_draw_rect(canvas, 0, 21, 68, 42, &rect_white_dsc);
    lv_canvas_draw_rect(canvas, 0, 22, 68, 40, &rect_black_dsc);

    // Calculate WPM stats
    for (int i = 0; i < 10; i++) {
        if (state->wpm[i] > max) {
            max = state->wpm[i];
        }
        if (state->wpm[i] < min) {
            min = state->wpm[i];
        }
    }

    // Draw WPM number (top right)
    char wpm_text[6] = {};
    snprintf(wpm_text, sizeof(wpm_text), "%d", max);
    lv_canvas_draw_text(canvas, 18, 17, 48, &label_dsc_wpm, wpm_text);
    
    // Draw WPM graph
    int range = max - min;
    if (range == 0) {
        range = 1;
    }

    lv_point_t points[10];
    for (int i = 0; i < 10; i++) {
        points[i].x = 2 + i * 7;
        points[i].y = 60 - (state->wpm[i] - min) * 36 / range;
    }
    lv_canvas_draw_line(canvas, points, 10, &line_dsc);

    // ========================================
    // VẼ NHIỆT ĐỘ - GÓC DƯỚI TRÁI KHUNG WPM
    // ========================================
    char temp_text[16];
    if (temp_data_valid) {
        // Lấy 1 chữ số thập phân
        int decimal = abs(current_temp_val.val2) / 100000;
        
        // Format: "27.2°C"
        if (current_temp_val.val1 == 0 && current_temp_val.val2 < 0) {
            snprintf(temp_text, sizeof(temp_text), "-0.%d", decimal);
        } else {
            snprintf(temp_text, sizeof(temp_text), "%d.%d", 
                     current_temp_val.val1, decimal);
        }
    } else {
        snprintf(temp_text, sizeof(temp_text), "--.-");
    }
    
    // Vẽ nhiệt độ ở góc dưới trái khung WPM
    // x=2: sát lề trái khung (khung bắt đầu từ x=1)
    // y=48: góc dưới (khung kết thúc ở y=62, text height ~14px, 62-14=48)
    // max_width=64: chiều rộng khung - 2px padding
    lv_canvas_draw_text(canvas, 0, 37, 68, &label_dsc_temp, temp_text);

    lv_canvas_draw_text(canvas, 0, 41, 68, &label_dsc_temp_unit, "°C");

    // Rotate canvas
    rotate_canvas(canvas, cbuf);
}

static void draw_middle(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state) {
    lv_obj_t *canvas = lv_obj_get_child(widget, 1);

    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);
    lv_draw_rect_dsc_t rect_white_dsc;
    init_rect_dsc(&rect_white_dsc, LVGL_FOREGROUND);
    lv_draw_arc_dsc_t arc_dsc;
    init_arc_dsc(&arc_dsc, LVGL_FOREGROUND, 2);
    lv_draw_arc_dsc_t arc_dsc_filled;
    init_arc_dsc(&arc_dsc_filled, LVGL_FOREGROUND, 9);
    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lv_font_montserrat_18, LV_TEXT_ALIGN_CENTER);
    lv_draw_label_dsc_t label_dsc_black;
    init_label_dsc(&label_dsc_black, LVGL_BACKGROUND, &lv_font_montserrat_18, LV_TEXT_ALIGN_CENTER);

    // Fill background
    lv_canvas_draw_rect(canvas, 0, 0, CANVAS_SIZE, CANVAS_SIZE, &rect_black_dsc);

    // Draw circles
    int circle_offsets[NICEVIEW_PROFILE_COUNT][2] = {
        {13, 13}, {55, 13}, {34, 28}, {13, 43}, {55, 43},
    };

    for (int i = 0; i < NICEVIEW_PROFILE_COUNT; i++) {
        bool selected = i == state->active_profile_index;

        if (state->profiles_connected[i]) {
            lv_canvas_draw_arc(canvas, circle_offsets[i][0], circle_offsets[i][1], 13, 0, 360,
                               &arc_dsc);
        } else if (state->profiles_bonded[i]) {
            const int segments = 8;
            const int gap = 20;
            for (int j = 0; j < segments; ++j)
                lv_canvas_draw_arc(canvas, circle_offsets[i][0], circle_offsets[i][1], 13,
                                   360. / segments * j + gap / 2.0,
                                   360. / segments * (j + 1) - gap / 2.0, &arc_dsc);
        }

        if (selected) {
            lv_canvas_draw_arc(canvas, circle_offsets[i][0], circle_offsets[i][1], 9, 0, 359,
                               &arc_dsc_filled);
        }

        char label[2];
        snprintf(label, sizeof(label), "%d", i + 1);
        lv_canvas_draw_text(canvas, circle_offsets[i][0] - 8, circle_offsets[i][1] - 10, 16,
                            (selected ? &label_dsc_black : &label_dsc), label);
    }

    lv_draw_label_dsc_t label_dsc_volt;
    init_label_dsc(&label_dsc_volt, LVGL_FOREGROUND, &lv_font_montserrat_14, LV_TEXT_ALIGN_CENTER);

    //vẽ voltage bên dưới
    char v_text[10];
    
    snprintf(v_label, sizeof(v_label), "%d", (current_voltage_mv % 10) );
    if (current_voltage_mv > 0) {
     // 2 chữ số thập phân
        // Ví dụ: 4200 mV -> 4.20Vi
        // 3750 mV -> 3.75V
        snprintf(v_text, sizeof(v_text), "%d.%03d", 
                 current_voltage_mv / 1000,           // Phần nguyên: 4200/1000 = 4
                 (current_voltage_mv % 1000));   
    } else {
        snprintf(v_text, sizeof(v_text), "---");
    }

    lv_canvas_draw_text(canvas, 0, 0, 68, &label_dsc_volt, v_text);

    // Rotate canvas
    rotate_canvas(canvas, cbuf);
}

static void draw_bottom(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state) {
    lv_obj_t *canvas = lv_obj_get_child(widget, 2);

    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);
    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lv_font_montserrat_16, LV_TEXT_ALIGN_CENTER);

    // Fill background
    lv_canvas_draw_rect(canvas, 0, 0, CANVAS_SIZE, CANVAS_SIZE, &rect_black_dsc);

    // Draw layer
    if (state->layer_label == NULL || strlen(state->layer_label) == 0) {
        char text[10] = {};
        sprintf(text, "LAYER %i", state->layer_index);
        lv_canvas_draw_text(canvas, 0, 24, 68, &label_dsc, text);
    } else {
        lv_canvas_draw_text(canvas, 0, 24, 68, &label_dsc, state->layer_label);
    }

    // Draw battery
    lv_draw_label_dsc_t label_dsc_battery;
    init_label_dsc(&label_dsc_battery, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_CENTER);
    char battery_text[5] = {};
    snprintf(battery_text, sizeof(battery_text), "%d%%", state->battery);
    lv_canvas_draw_text(canvas, 0, -1, 68, &label_dsc_battery, battery_text);
    
    // Rotate canvas
    rotate_canvas(canvas, cbuf);
}

// ========================================
// EVENT HANDLERS
// ========================================

static void set_battery_status(struct zmk_widget_status *widget,
                               struct battery_status_state state) {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    widget->state.charging = state.usb_present;
#endif

    widget->state.battery = state.level;

    draw_top(widget->obj, widget->cbuf, &widget->state);
    draw_bottom(widget->obj, widget->cbuf3, &widget->state);
}

static void battery_status_update_cb(struct battery_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_battery_status(widget, state); }
}

static struct battery_status_state battery_status_get_state(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);

    return (struct battery_status_state){
        .level = (ev != NULL) ? ev->state_of_charge : zmk_battery_state_of_charge(),
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

static void set_output_status(struct zmk_widget_status *widget,
                              const struct output_status_state *state) {
    widget->state.selected_endpoint = state->selected_endpoint;
    widget->state.active_profile_index = state->active_profile_index;
    widget->state.active_profile_connected = state->active_profile_connected;
    widget->state.active_profile_bonded = state->active_profile_bonded;
    for (int i = 0; i < NICEVIEW_PROFILE_COUNT; ++i) {
        widget->state.profiles_connected[i] = state->profiles_connected[i];
        widget->state.profiles_bonded[i] = state->profiles_bonded[i];
    }

    draw_top(widget->obj, widget->cbuf, &widget->state);
    draw_middle(widget->obj, widget->cbuf2, &widget->state);
}

static void output_status_update_cb(struct output_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_output_status(widget, &state); }
}

static struct output_status_state output_status_get_state(const zmk_event_t *_eh) {
    struct output_status_state state = {
        .selected_endpoint = zmk_endpoints_selected(),
        .active_profile_index = zmk_ble_active_profile_index(),
        .active_profile_connected = zmk_ble_active_profile_is_connected(),
        .active_profile_bonded = !zmk_ble_active_profile_is_open(),
    };
    for (int i = 0; i < MIN(NICEVIEW_PROFILE_COUNT, ZMK_BLE_PROFILE_COUNT); ++i) {
        state.profiles_connected[i] = zmk_ble_profile_is_connected(i);
        state.profiles_bonded[i] = !zmk_ble_profile_is_open(i);
    }
    return state;
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_output_status, struct output_status_state,
                            output_status_update_cb, output_status_get_state)
ZMK_SUBSCRIPTION(widget_output_status, zmk_endpoint_changed);

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_SUBSCRIPTION(widget_output_status, zmk_usb_conn_state_changed);
#endif
#if defined(CONFIG_ZMK_BLE)
ZMK_SUBSCRIPTION(widget_output_status, zmk_ble_active_profile_changed);
#endif

static void set_layer_status(struct zmk_widget_status *widget, struct layer_status_state state) {
    widget->state.layer_index = state.index;
    widget->state.layer_label = state.label;

    draw_bottom(widget->obj, widget->cbuf3, &widget->state);
}

static void layer_status_update_cb(struct layer_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_layer_status(widget, state); }
}

static struct layer_status_state layer_status_get_state(const zmk_event_t *eh) {
    zmk_keymap_layer_index_t index = zmk_keymap_highest_layer_active();
    return (struct layer_status_state){
        .index = index, .label = zmk_keymap_layer_name(zmk_keymap_layer_index_to_id(index))};
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_layer_status, struct layer_status_state, layer_status_update_cb,
                            layer_status_get_state)

ZMK_SUBSCRIPTION(widget_layer_status, zmk_layer_state_changed);

static void set_wpm_status(struct zmk_widget_status *widget, struct wpm_status_state state) {
    for (int i = 0; i < 9; i++) {
        widget->state.wpm[i] = widget->state.wpm[i + 1];
    }
    widget->state.wpm[9] = state.wpm;

    draw_top(widget->obj, widget->cbuf, &widget->state);
}

static void wpm_status_update_cb(struct wpm_status_state state) {
    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_wpm_status(widget, state); }
}

struct wpm_status_state wpm_status_get_state(const zmk_event_t *eh) {
    return (struct wpm_status_state){.wpm = zmk_wpm_get_state()};
};

ZMK_DISPLAY_WIDGET_LISTENER(widget_wpm_status, struct wpm_status_state, wpm_status_update_cb,
                            wpm_status_get_state)
ZMK_SUBSCRIPTION(widget_wpm_status, zmk_wpm_state_changed);

int zmk_widget_status_init(struct zmk_widget_status *widget, lv_obj_t *parent) {
    widget->obj = lv_obj_create(parent);
    lv_obj_set_size(widget->obj, 160, 68);
    lv_obj_t *top = lv_canvas_create(widget->obj);
    lv_obj_align(top, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_canvas_set_buffer(top, widget->cbuf, CANVAS_SIZE, CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR);
    lv_obj_t *middle = lv_canvas_create(widget->obj);
    lv_obj_align(middle, LV_ALIGN_TOP_RIGHT, -64, 0);
    lv_canvas_set_buffer(middle, widget->cbuf2, CANVAS_SIZE, CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR);
    lv_obj_t *bottom = lv_canvas_create(widget->obj);
    lv_obj_align(bottom, LV_ALIGN_TOP_RIGHT, -120, 0);
    lv_canvas_set_buffer(bottom, widget->cbuf3, CANVAS_SIZE, CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR);

    // Đọc nhiệt độ ban đầu
    read_temperature();
    read_battery_voltage();
    
    // Start temperature timer
    k_timer_start(&temp_timer, K_SECONDS(2), K_SECONDS(30));
    LOG_INF("Temperature monitoring started");

    sys_slist_append(&widgets, &widget->node);
    widget_battery_status_init();
    widget_output_status_init();
    widget_layer_status_init();
    widget_wpm_status_init();

    return 0;
}

lv_obj_t *zmk_widget_status_obj(struct zmk_widget_status *widget) { return widget->obj; }