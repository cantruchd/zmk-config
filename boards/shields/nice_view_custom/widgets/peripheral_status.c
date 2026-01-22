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
        LOG_DBG("Voltage: %d mV", current_voltage_mv);
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
            LOG_DBG("Temperature: %d.%02d°C", temp_val.val1, abs(temp_val.val2 / 10000));
        }
    } else {
        LOG_WRN("Failed to fetch temperature: %d", rc);
        temp_data_valid = false;
    }
}

static void temp_work_handler(struct k_work *work) {
    read_temperature();
    read_battery_voltage();
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

#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/logging/log.h>


static const struct device *rtc_dev;
static uint32_t rtc_start_ticks = 0;
static bool rtc_initialized = false;

// RTC0 trên nRF52 chạy ở 32.768 kHz
#define RTC_FREQ 32768

// // Khởi tạo RTC
// int uptime_rtc_init(void) {
//     rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc0));
    
//     if (!device_is_ready(rtc_dev)) {
     
//         return -ENODEV;
//     }
    
//     // Đọc giá trị hiện tại làm mốc bắt đầu
//     int ret = counter_get_value(rtc_dev, &rtc_start_ticks);
//     if (ret != 0) {
 
//         return ret;
//     }
    
//     rtc_initialized = true;
   
    
//     return 0;
// }

// Lấy uptime tính bằng milliseconds
int64_t uptime_rtc_get_ms(void) {
    // if (!rtc_initialized) {
    //     return -EINVAL;
    // }
    
    uint32_t current_ticks;
    int ret = counter_get_value(rtc_dev, &current_ticks);
    if (ret != 0) {
     
        return -EIO;
    }
    LOG_DBG("RTC ticks: %u", current_ticks);


    // Tính elapsed ticks (xử lý overflow)
    uint32_t elapsed_ticks = current_ticks - rtc_start_ticks;
    // if (current_ticks >= rtc_start_ticks) {
    //     elapsed_ticks = current_ticks - rtc_start_ticks;
    // } else {
    //     // RTC0 trên nRF52 là 24-bit counter
    //     uint32_t top_value = counter_get_top_value(rtc_dev);
    //     elapsed_ticks = (top_value - rtc_start_ticks) + current_ticks + 1;
    // }
    
    // Chuyển đổi: ticks -> milliseconds
    // elapsed_ms = (elapsed_ticks * 1000) / 32768
    // Tối ưu: (elapsed_ticks * 125) / 4096 để tránh overflow
    int64_t uptime_ms = ((int64_t)elapsed_ticks * 1000) / RTC_FREQ;
    LOG_DBG("Uptime ms: %lld", uptime_ms);
    return uptime_ms;
}

// Lấy uptime tính bằng giây
int64_t uptime_rtc_get_sec(void) {
    int64_t ms = uptime_rtc_get_ms();
    return (ms > 0) ? (ms / 1000) : ms;
}

// Struct thông tin uptime
typedef struct {
    uint32_t days;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
} uptime_info_t;

// Chuyển đổi sang ngày/giờ/phút/giây
void uptime_rtc_to_dhms(uptime_info_t *info) {
    int64_t uptime_ms = uptime_rtc_get_ms();
    
    if (uptime_ms < 0) {
        memset(info, 0, sizeof(uptime_info_t));
        return;
    }
    
    int64_t total_seconds = uptime_ms / 1000;
    info->seconds = total_seconds % 60;
    
    int64_t total_minutes = total_seconds / 60;
    info->minutes = total_minutes % 60;
    
    int64_t total_hours = total_minutes / 60;
    info->hours = total_hours % 24;
    
    info->days = total_hours / 24;
}




static void draw_middle(lv_obj_t *canvas, lv_color_t cbuf[]) {
    lv_draw_rect_dsc_t rect_black_dsc;
    init_rect_dsc(&rect_black_dsc, LVGL_BACKGROUND);
    
    lv_draw_label_dsc_t label_dsc_v, label_dsc_v_label;
    init_label_dsc(&label_dsc_v, LVGL_FOREGROUND, &lv_font_montserrat_26, LV_TEXT_ALIGN_LEFT);
    init_label_dsc(&label_dsc_v_label, LVGL_FOREGROUND, &lv_font_montserrat_24, LV_TEXT_ALIGN_RIGHT);
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

    LOG_DBG("Drawing voltage: %s V", v_text);
    k_sleep(K_MSEC(10000)); // Small delay to ensure proper rendering
    
    lv_canvas_draw_text(canvas, 0, 0, MIDDLE_WIDTH, &label_dsc_v, v_text);
    lv_canvas_draw_text(canvas, 0, 0, MIDDLE_WIDTH, &label_dsc_v_label, v_label);

    // draw uptime
    char uptime_day[10];
    char uptime_hourmin[10];

    uptime_info_t uptime;
    uptime_rtc_to_dhms(&uptime);
    snprintf(uptime_day, sizeof(uptime_day), "%ud", uptime.days);
    snprintf(uptime_hourmin, sizeof(uptime_hourmin), "%02d:%02d", uptime.hours, uptime.minutes);

    LOG_DBG("Drawing uptime: %s %s", uptime_day, uptime_hourmin);
    // lv_canvas_draw_text(canvas, 0, 20, MIDDLE_WIDTH, &label_dsc_v_label, uptime_day);
    // lv_canvas_draw_text(canvas, 0, 40, MIDDLE_WIDTH, &label_dsc_v_label, uptime_hourmin);

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
// WIDGET INIT
// ========================================

int zmk_widget_status_init(struct zmk_widget_status *widget, lv_obj_t *parent) {


    // uptime_rtc_init();

    LOG_DBG("=== Init nice_view with temperature ===");
    
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
	LOG_DBG("Temperature timer started");
	
	/* lv_obj_t *art = lv_img_create(widget->obj);
    bool random = sys_rand32_get() & 1;
    lv_img_set_src(art, random ? &balloon : &mountain);
    lv_obj_align(art, LV_ALIGN_TOP_LEFT, -48, 0); */
	
    sys_slist_append(&widgets, &widget->node);
    widget_battery_status_init();
    widget_peripheral_status_init();

    LOG_DBG("=== Widget init complete ===");
    return 0;
}

lv_obj_t *zmk_widget_status_obj(struct zmk_widget_status *widget) { 
    return widget->obj; 
}