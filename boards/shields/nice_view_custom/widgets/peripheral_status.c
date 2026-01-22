/*
 * Copyright (c) 2023 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <stdlib.h> // Để dùng hàm abs()
#include <zmk/battery.h> // Thêm để dùng hàm lấy voltage

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

LV_IMG_DECLARE(balloon);
LV_IMG_DECLARE(mountain);
static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

struct peripheral_status_state {
    bool connected;
};


char logtext[10];


int uptime_save(void);
int64_t uptime_get_total_ms(void);
static void check_auto_reset_uptime(uint8_t battery_level, bool is_charging);
// Lưu trữ cấu trúc sensor đầy đủ
static struct sensor_value current_temp_val = {0};
static bool temp_data_valid = false;

// Thêm buffer cho canvas middle (92x68)
// Lưu ý: Nếu RAM bị thiếu, bạn có thể giảm kích thước buffer này xuống
#define MIDDLE_WIDTH 68
#define MIDDLE_HEIGHT 68
static lv_color_t middle_cbuf[MIDDLE_WIDTH * MIDDLE_HEIGHT];
// Thêm buffer cho canvas bottom (68x68)
#define BOTTOM_WIDTH 68
#define BOTTOM_HEIGHT 68
static lv_color_t bottom_cbuf[BOTTOM_WIDTH * BOTTOM_HEIGHT];
// FORWARD DECLARATION
static void draw_top(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state);
static void draw_middle(lv_obj_t *widget, lv_color_t cbuf[]); // Hàm vẽ voltage mới
static void draw_bottom(lv_obj_t *widget, lv_color_t cbuf[]); 

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
        LOG_WRN("Voltage: %d mV", current_voltage_mv);
    } else {
        LOG_ERR("No valid voltage channel found");
        current_voltage_mv = 0;
    }
}

// ========================================
// TEMPERATURE SENSOR
// ========================================

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
            // Log với 2 chữ số thập phân để kiểm tra
            LOG_WRN("Temperature: %d.%02d°C", temp_val.val1, abs(temp_val.val2 / 10000));
        }
    } else {
        LOG_WRN("Failed to fetch temperature: %d", rc);
        temp_data_valid = false;
    }
}

static void temp_work_handler(struct k_work *work) {
    read_temperature();
    read_battery_voltage();
    uptime_save(); // Lưu uptime định kỳ

    // THÊM: Kiểm tra auto-reset (lấy thông tin pin hiện tại)
    uint8_t battery_level = zmk_battery_state_of_charge();
    bool is_charging = false;
    #if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
        is_charging = zmk_usb_is_powered();
    #endif
    check_auto_reset_uptime(battery_level, is_charging);



    struct zmk_widget_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) {
        draw_top(widget->obj, widget->cbuf, &widget->state);
        // Vẽ phần bên trái (92x68) - Lấy child index 1
        lv_obj_t *middle_canvas = lv_obj_get_child(widget->obj, 1);
        draw_middle(middle_canvas, middle_cbuf);
        draw_bottom(lv_obj_get_child(widget->obj, 2), bottom_cbuf);
    }
}

K_WORK_DEFINE(temp_work, temp_work_handler);

static void temp_timer_handler(struct k_timer *timer) {
    k_work_submit(&temp_work);
}

K_TIMER_DEFINE(temp_timer, temp_timer_handler, NULL);

// ========================================
// DRAWING FUNCTION
// ========================================

// ========================================
// NEW: DRAWING VOLTAGE (MIDDLE)
// ========================================

// ========================================
// ĐỌC VOLTAGE TỪ ZMK BATTERY SENSOR
// ========================================

// Biến theo dõi trạng thái pin đầy để auto-reset uptime
static int64_t full_charge_start_time = 0;  // Thời điểm bắt đầu đạt 100%
static bool is_at_100_percent = false;       // Đang ở 100%?
static bool uptime_reset_triggered = false;  // Đã reset trong chu kỳ này chưa?

#define FULL_CHARGE_DURATION_MS (2 * 60 * 1000)  // 2 phút = 120,000 ms


// ========================================
// THÊM HÀM MỚI - Kiểm tra và reset uptime tự động
// ========================================

static void check_auto_reset_uptime(uint8_t battery_level, bool is_charging) {
    int64_t current_time = k_uptime_get();
    
    // Kiểm tra điều kiện: Pin = 100% VÀ đang sạc
    if (battery_level == 100 && is_charging) {
        
        // Lần đầu tiên đạt 100%
        if (!is_at_100_percent) {
            is_at_100_percent = true;
            full_charge_start_time = current_time;
            uptime_reset_triggered = false;  // Reset flag
            LOG_INF("Battery at 100%% and charging - starting 2min timer");
        }
        // Đã ở 100% được 2 phút và chưa reset
        else if (!uptime_reset_triggered) {
            int64_t elapsed = current_time - full_charge_start_time;
            
            if (elapsed >= FULL_CHARGE_DURATION_MS) {
                LOG_INF("Auto-resetting uptime after 2min at 100%%");
                uptime_reset();
                //uptime_reset_triggered = true;  // Đánh dấu đã reset
                
                // Cập nhật UI
                struct zmk_widget_status *widget;
                SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) {
                    lv_obj_t *middle_canvas = lv_obj_get_child(widget->obj, 1);
                    draw_middle(middle_canvas, middle_cbuf);
                }
            }
        }
    }
    // Nếu pin không còn 100% hoặc ngừng sạc -> reset trạng thái
    else {
        if (is_at_100_percent) {
            LOG_INF("Battery dropped below 100%% or charging stopped - reset timer");
        }
        is_at_100_percent = false;
        uptime_reset_triggered = false;
        full_charge_start_time = 0;
    }
}


#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>




// Lưu tổng uptime vào settings (persistent storage)
static int64_t total_uptime_ms = 0;
static int64_t session_start_ms = 0;
static bool uptime_initialized = false;

// Struct thông tin uptime
typedef struct {
    uint32_t days;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} uptime_info_t;

// Settings handler - load uptime từ flash
static int uptime_settings_set(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg) {
    if (!strcmp(name, "total")) {
        if (len == sizeof(total_uptime_ms)) {
            read_cb(cb_arg, &total_uptime_ms, len);
            LOG_INF("Loaded uptime from storage: %lld ms", total_uptime_ms);
        }
    }
    return 0;
}

static struct settings_handler uptime_settings = {
    .name = "uptime",
    .h_set = uptime_settings_set,
};

// Khởi tạo uptime system
int uptime_init(void) {
    int rc;
    
    // Init settings subsystem
    rc = settings_subsys_init();
    if (rc != 0) {
        LOG_ERR("Settings init failed: %d", rc);
        return rc;
    }
    
    // Register handler
    rc = settings_register(&uptime_settings);
    if (rc != 0) {
        LOG_ERR("Settings register failed: %d", rc);
        return rc;
    }
    
    // Load saved uptime
    rc = settings_load_subtree("uptime");
    if (rc != 0) {
        LOG_WRN("Failed to load uptime, starting from 0");
        total_uptime_ms = 0;
    }
    
    // Lưu thời điểm bắt đầu session này
    session_start_ms = k_uptime_get();
    uptime_initialized = true;
    
    LOG_INF("Uptime system initialized");
    return 0;
}

// Lưu uptime vào flash
int uptime_save(void) {
    if (!uptime_initialized) {
        return -EINVAL;
    }
    
    int64_t current = uptime_get_total_ms();
    int rc = settings_save_one("uptime/total", &current, sizeof(current));
    
    if (rc == 0) {
        LOG_DBG("Uptime saved: %lld ms", current);
    } else {
        LOG_ERR("Failed to save uptime: %d", rc);
    }
    
    return rc;
}

// RESET uptime về 0
int uptime_reset(void) {
    if (!uptime_initialized) {
        return -EINVAL;
    }
    
    total_uptime_ms = 0;
    session_start_ms = k_uptime_get();
    
    // Xóa khỏi flash
    int rc = settings_delete("uptime/total");
    if (rc == 0) {
        LOG_INF("Uptime reset successfully");
    } else {
        LOG_ERR("Failed to reset uptime: %d", rc);
    }
    
    return rc;
}

// Lấy tổng uptime (ms) - bao gồm cả previous sessions
int64_t uptime_get_total_ms(void) {
    if (!uptime_initialized) {
        return 0;
    }
    
    int64_t current_session = k_uptime_get() - session_start_ms;
    return total_uptime_ms + current_session;
}

// Lấy uptime của session hiện tại (ms)
int64_t uptime_get_session_ms(void) {
    if (!uptime_initialized) {
        return 0;
    }
    
    return k_uptime_get() - session_start_ms;
}

// Chuyển đổi sang ngày/giờ/phút/giây
void uptime_to_dhms(uptime_info_t *info, int64_t uptime_ms) {
    int64_t total_seconds = uptime_ms / 1000;
    info->seconds = total_seconds % 60;
    
    int64_t total_minutes = total_seconds / 60;
    info->minutes = total_minutes % 60;
    
    int64_t total_hours = total_minutes / 60;
    info->hours = total_hours % 24;
    
    info->days = total_hours / 24;
}

// In total uptime
void uptime_print_total(void) {
    uptime_info_t uptime;
    int64_t total_ms = uptime_get_total_ms();
    uptime_to_dhms(&uptime, total_ms);
    
    printk("Total Uptime: %ud %02uh %02um %02us\n",
           uptime.days, uptime.hours, uptime.minutes, uptime.seconds);
}

// In session uptime
void uptime_print_session(void) {
    uptime_info_t uptime;
    int64_t session_ms = uptime_get_session_ms();
    uptime_to_dhms(&uptime, session_ms);
    
    printk("Session Uptime: %ud %02uh %02um %02us\n",
           uptime.days, uptime.hours, uptime.minutes, uptime.seconds);
}

// Format uptime thành string
int uptime_format(char *buf, size_t buf_size, int64_t uptime_ms) {
    uptime_info_t uptime;
    uptime_to_dhms(&uptime, uptime_ms);
    
    return snprintf(buf, buf_size, "%ud %02uh %02um %02us",
                    uptime.days, uptime.hours, uptime.minutes, uptime.seconds);
}

// Auto-save worker
void uptime_autosave_work_handler(struct k_work *work) {
    uptime_save();
    // Schedule lại sau 1 giờ
    k_work_schedule((struct k_work_delayable *)work, K_HOURS(1));
}

K_WORK_DELAYABLE_DEFINE(uptime_autosave_work, uptime_autosave_work_handler);

// Bắt đầu auto-save
void uptime_start_autosave(void) {
    k_work_schedule(&uptime_autosave_work, K_HOURS(1));
    LOG_INF("Uptime auto-save enabled (every 1 hour)");
}

// Dừng auto-save
void uptime_stop_autosave(void) {
    k_work_cancel_delayable(&uptime_autosave_work);
    LOG_INF("Uptime auto-save disabled");
}

// Hook trước khi sleep - lưu uptime
void uptime_before_sleep(void) {
    uptime_save();
}

// Init system
SYS_INIT(uptime_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);


static void draw_middle(lv_obj_t *canvas, lv_color_t cbuf[]) {
    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);
    
    lv_draw_label_dsc_t label_dsc_v, label_dsc_v_label, label_dsc_v_label_uptime_day, label_dsc_v_label_uptime_hourmin;
    init_label_dsc(&label_dsc_v, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_LEFT);
    init_label_dsc(&label_dsc_v_label, LVGL_FOREGROUND, &lv_font_montserrat_24, LV_TEXT_ALIGN_RIGHT);

    init_label_dsc(&label_dsc_v_label_uptime_day, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_CENTER);
    init_label_dsc(&label_dsc_v_label_uptime_hourmin, LVGL_FOREGROUND, &lv_font_montserrat_22, LV_TEXT_ALIGN_CENTER);

    lv_canvas_draw_rect(canvas, 0, 0, MIDDLE_WIDTH, MIDDLE_HEIGHT, &rect_black_dsc);

    char v_text[10];
    char v_label[10];
    snprintf(v_label, sizeof(v_label), "%d", (current_voltage_mv % 10) );
    if (current_voltage_mv > 0) {
     // 2 chữ số thập phân
        // Ví dụ: 4200 mV -> 4.20Vi
        // 3750 mV -> 3.75V
        snprintf(v_text, sizeof(v_text), "%d.%02d", 
                 current_voltage_mv / 1000,           // Phần nguyên: 4200/1000 = 4
                 (current_voltage_mv % 1000) / 10);   // Phần thập phân: (4200%1000)/10 = 20
    } else {
        snprintf(v_text, sizeof(v_text), logtext);
    }

    
    
    lv_canvas_draw_text(canvas, 0, 0, MIDDLE_WIDTH, &label_dsc_v, v_text);
    lv_canvas_draw_text(canvas, 0, 0, MIDDLE_WIDTH, &label_dsc_v_label, v_label);


    LOG_WRN("Drawing voltage: %s V", v_text);
    //k_sleep(K_MSEC(10000)); // Small delay to ensure proper rendering

    // draw uptime
    char uptime_day[10];
    char uptime_hourmin[10];
    LOG_WRN("Drawing uptime info");
    uptime_info_t uptime;
    int64_t total_ms = uptime_get_total_ms();
    uptime_to_dhms(&uptime, total_ms);
    snprintf(uptime_day, sizeof(uptime_day), "%dd", uptime.days);
    snprintf(uptime_hourmin, sizeof(uptime_hourmin), "%02d:%02d", uptime.hours, uptime.minutes);

    LOG_WRN("Drawing uptime: %s %s", uptime_day, uptime_hourmin);
    lv_canvas_draw_text(canvas, 0, 22, MIDDLE_WIDTH, &label_dsc_v_label_uptime_day, uptime_day);
    lv_canvas_draw_text(canvas, 0, 44, MIDDLE_WIDTH, &label_dsc_v_label_uptime_hourmin, uptime_hourmin);

    rotate_canvas(canvas, cbuf);

}

static void draw_bottom(lv_obj_t *canvas, lv_color_t cbuf[]) {
    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);
    
    lv_draw_label_dsc_t label_dsc_v;
    init_label_dsc(&label_dsc_v, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_CENTER);

    lv_canvas_draw_rect(canvas, 0, 0, BOTTOM_WIDTH, BOTTOM_HEIGHT, &rect_black_dsc);

    lv_canvas_draw_text(canvas, 0, 0, BOTTOM_WIDTH, &label_dsc_v, logtext);
    rotate_canvas(canvas, cbuf);
}


static void draw_top(lv_obj_t *widget, lv_color_t cbuf[], const struct status_state *state) {
    lv_obj_t *canvas = lv_obj_get_child(widget, 0);

    lv_draw_label_dsc_t label_dsc;
    init_label_dsc(&label_dsc, LVGL_FOREGROUND, &lv_font_montserrat_16, LV_TEXT_ALIGN_RIGHT);
    
    lv_draw_label_dsc_t label_dsc_battery;
    init_label_dsc(&label_dsc_battery, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_CENTER);
    
    lv_draw_label_dsc_t label_dsc_temp;
	lv_draw_label_dsc_t label_dsc_temp_label;
    // Có thể cần hạ size font xuống một chút nếu text quá dài (ví dụ Montserrat 20)
    init_label_dsc(&label_dsc_temp, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_LEFT);
	init_label_dsc(&label_dsc_temp_label, LVGL_FOREGROUND, &lv_font_montserrat_16, LV_TEXT_ALIGN_RIGHT);
    
    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);

    lv_canvas_draw_rect(canvas, 0, 0, CANVAS_SIZE, CANVAS_SIZE, &rect_black_dsc);

    draw_battery(canvas, state);

    lv_canvas_draw_text(canvas, 0, 0, CANVAS_SIZE, &label_dsc,
                        state->connected ? LV_SYMBOL_WIFI : LV_SYMBOL_CLOSE);

    char battery_text[5] = {};
    snprintf(battery_text, sizeof(battery_text), "%d%%", state->battery);
    lv_canvas_draw_text(canvas, 0, 17, 68, &label_dsc_battery, battery_text);

    // HIỂN THỊ TEMPERATURE 1 SỐ THẬP PHÂN (KHÔNG LÀM TRÒN)
    char temp_text[16];
    if (temp_data_valid) {
        // Lấy đúng chữ số đầu tiên sau dấu phẩy bằng cách chia cho 100,000
        // Ví dụ: 750,000 / 100,000 = 7
        int decimal = abs(current_temp_val.val2) / 100000;

        // Xử lý dấu âm cho trường hợp đặc biệt -0.X độ
        if (current_temp_val.val1 == 0 && current_temp_val.val2 < 0) {
            snprintf(temp_text, sizeof(temp_text), "-0.%d", decimal);
        } else {
            snprintf(temp_text, sizeof(temp_text), "%d.%d", current_temp_val.val1, decimal);
        }
    } else {
        snprintf(temp_text, sizeof(temp_text), "--.-°");
    }
	

	
    
    lv_canvas_draw_text(canvas, 0, 42, 68, &label_dsc_temp, temp_text);
	lv_canvas_draw_text(canvas, 0, 42, 68, &label_dsc_temp_label, "°C");
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


    // THÊM DÒNG NÀY: Kiểm tra auto-reset uptime
    check_auto_reset_uptime(state.level, state.usb_present);
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
// PERIPHERAL CONNECTION
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
// ACTIVITY STATE
// ========================================

static int temperature_listener(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->state == ZMK_ACTIVITY_IDLE || ev->state == ZMK_ACTIVITY_SLEEP) {
        uptime_save(); // Lưu uptime trước khi sleep
        k_timer_stop(&temp_timer);
        LOG_WRN("Temperature timer stopped");
    } else if (ev->state == ZMK_ACTIVITY_ACTIVE) {
        k_timer_start(&temp_timer, K_SECONDS(1), K_SECONDS(30));
        LOG_WRN("Temperature timer started");
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(widget_temperature, temperature_listener);
ZMK_SUBSCRIPTION(widget_temperature, zmk_activity_state_changed);

// ========================================
// WIDGET INIT
// ========================================

int zmk_widget_status_init(struct zmk_widget_status *widget, lv_obj_t *parent) {


    // uptime_rtc_init();

    LOG_WRN("=== Init nice_view with temperature ===");
    
    widget->obj = lv_obj_create(parent);
    lv_obj_set_size(widget->obj, 160, 68);
    
    // Canvas (68x68)
    lv_obj_t *top = lv_canvas_create(widget->obj);
    lv_obj_align(top, LV_ALIGN_TOP_RIGHT, 0, 0);
    lv_canvas_set_buffer(top, widget->cbuf, CANVAS_SIZE, CANVAS_SIZE, LV_IMG_CF_TRUE_COLOR);
	
	// 2. Canvas Middle (Bên Trái - 92x68) - Thay thế vị trí của ART
    lv_obj_t *middle = lv_canvas_create(widget->obj);
        
    lv_obj_align(middle, LV_ALIGN_TOP_LEFT, 24, 0); // Dịch trái 24px để căn giữa

    lv_canvas_set_buffer(middle, middle_cbuf, MIDDLE_WIDTH, MIDDLE_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    // 3. Canvas Bottom (Bên Trái Dưới - 68x68) - Thay thế vị trí của ART
    lv_obj_t *bottom = lv_canvas_create(widget->obj);
        
    lv_obj_align(bottom, LV_ALIGN_TOP_LEFT, -44, 0);

    lv_canvas_set_buffer(bottom, bottom_cbuf, BOTTOM_WIDTH, BOTTOM_HEIGHT, LV_IMG_CF_TRUE_COLOR);

	// Vẽ dữ liệu ban đầu
    read_temperature();
    read_battery_voltage();

    draw_top(widget->obj, widget->cbuf, &widget->state);
    draw_middle(middle, middle_cbuf);
    draw_bottom(bottom, bottom_cbuf);
    
    // Start timer
    k_timer_start(&temp_timer, K_SECONDS(2), K_SECONDS(30));
	LOG_WRN("Temperature timer started");
	
	/* lv_obj_t *art = lv_img_create(widget->obj);
    bool random = sys_rand32_get() & 1;
    lv_img_set_src(art, random ? &balloon : &mountain);
    lv_obj_align(art, LV_ALIGN_TOP_LEFT, -48, 0); */
	
    sys_slist_append(&widgets, &widget->node);
    widget_battery_status_init();
    widget_peripheral_status_init();

    LOG_WRN("=== Widget init complete ===");
    return 0;
}

lv_obj_t *zmk_widget_status_obj(struct zmk_widget_status *widget) { 
    return widget->obj; 
}