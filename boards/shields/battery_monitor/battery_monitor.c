/*
 * battery_monitor.c
 * Custom battery monitoring with Auto MOSFET control and ZMK Settings storage
 * 
 * Features:
 * - Auto MOSFET ON when battery < threshold (e.g. 30%)
 * - Auto MOSFET OFF when battery > threshold (e.g. 80%)
 * - All settings saved via ZMK Settings (like ZMK Studio)
 * - BLE control for all settings
 * - Dual temperature monitoring (internal + NTC)
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/reboot.h>
#include <stdlib.h>
#include <math.h>

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/settings.h>
#include <hal/nrf_power.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ============================================================================
// Configuration Constants
// ============================================================================

#define MOSFET_PIN  24

// NTC Configuration
#define NTC_ADC_PIN           4
#define NTC_REFERENCE_MV      3300
#define NTC_SERIES_RESISTOR   10000
#define NTC_NOMINAL_RESISTANCE 10000
#define NTC_NOMINAL_TEMP      25.0
#define NTC_B_COEFFICIENT     3950

// Default thresholds
#define DEFAULT_AUTO_ON_ENABLE    true
#define DEFAULT_AUTO_OFF_ENABLE   true
#define DEFAULT_AUTO_ON_PERCENT   30  // Bật MOSFET khi pin < 30%
#define DEFAULT_AUTO_OFF_PERCENT  80  // Tắt MOSFET khi pin > 80%
#define DEFAULT_STORAGE_PERCENT   40

#define UPDATE_INTERVAL_MS      10000
#define AUTO_UPDATE_DURATION_MS 1800000

// NVS Settings keys
#define SETTINGS_SUBTREE "battery_monitor"
#define SETTING_AUTO_ON_EN "aon_en"
#define SETTING_AUTO_OFF_EN "aoff_en"
#define SETTING_AUTO_ON "aon"
#define SETTING_AUTO_OFF "aoff"
#define SETTING_STORAGE "stor"

// ============================================================================
// BLE UUIDs
// ============================================================================

#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)
#define BT_UUID_CUSTOM_SERVICE \
    BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)

#define BT_UUID_POWER_CONTROL_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)
#define BT_UUID_POWER_CONTROL \
    BT_UUID_DECLARE_128(BT_UUID_POWER_CONTROL_VAL)

#define BT_UUID_TEMP_INTERNAL_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)
#define BT_UUID_TEMP_INTERNAL \
    BT_UUID_DECLARE_128(BT_UUID_TEMP_INTERNAL_VAL)

#define BT_UUID_VOLTAGE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)
#define BT_UUID_VOLTAGE \
    BT_UUID_DECLARE_128(BT_UUID_VOLTAGE_VAL)

#define BT_UUID_BOOTLOADER_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4)
#define BT_UUID_BOOTLOADER \
    BT_UUID_DECLARE_128(BT_UUID_BOOTLOADER_VAL)

#define BT_UUID_TEMP_EXTERNAL_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5)
#define BT_UUID_TEMP_EXTERNAL \
    BT_UUID_DECLARE_128(BT_UUID_TEMP_EXTERNAL_VAL)

// ⭐ NEW: Auto MOSFET settings UUID
#define BT_UUID_AUTO_SETTINGS_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef6)
#define BT_UUID_AUTO_SETTINGS \
    BT_UUID_DECLARE_128(BT_UUID_AUTO_SETTINGS_VAL)

// Commands
#define CMD_POWER_OFF    0x00
#define CMD_POWER_ON     0x01
#define CMD_POWER_TOGGLE 0x02
#define CMD_ENTER_BOOTLOADER 0x42
#define CMD_RESET_DEVICE     0x52

// ============================================================================
// ADC Configuration
// ============================================================================

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
    !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
    ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

// ============================================================================
// Auto MOSFET Settings Structure
// ============================================================================

struct auto_mosfet_settings {
    bool auto_on_enabled;      // Enable auto ON when battery low
    bool auto_off_enabled;     // Enable auto OFF when battery high
    uint8_t auto_on_percent;   // Bật MOSFET khi pin < giá trị này
    uint8_t auto_off_percent;  // Tắt MOSFET khi pin > giá trị này
    uint8_t storage_percent;   // Storage mode threshold
};

// ============================================================================
// Global Variables
// ============================================================================

static const struct device *gpio_dev;
static const struct device *temp_dev;
static const struct device *battery_dev;
static bool power_state = false;
static uint8_t last_battery_percent = 100;

static int16_t temp_internal = 0;
static int16_t temp_external = 0;
static uint16_t current_voltage_mv = 0;

static bool auto_update_active = false;
static int64_t auto_update_start_time = 0;
static struct k_work_delayable update_work;
static struct k_work_delayable bootloader_work;

static enum sensor_channel discovered_channel = SENSOR_CHAN_PRIV_START;

// ⭐ Auto MOSFET settings (loaded from NVS)
static struct auto_mosfet_settings auto_settings = {
    .auto_on_enabled = DEFAULT_AUTO_ON_ENABLE,
    .auto_off_enabled = DEFAULT_AUTO_OFF_ENABLE,
    .auto_on_percent = DEFAULT_AUTO_ON_PERCENT,
    .auto_off_percent = DEFAULT_AUTO_OFF_PERCENT,
    .storage_percent = DEFAULT_STORAGE_PERCENT,
};

static bool settings_loaded = false;

extern const struct bt_gatt_service_static battery_monitor_svc;

// Forward declarations
static ssize_t read_power_control(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t write_power_control(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags);
static ssize_t read_temp_internal(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t read_temp_external(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t read_voltage(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset);
static ssize_t write_bootloader(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags);
static ssize_t read_auto_settings(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t write_auto_settings(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags);

// ============================================================================
// ZMK Settings Management (like ZMK Studio)
// ============================================================================

static int save_settings(void) {
    int rc;

    rc = zmk_settings_save_uint8(SETTINGS_SUBTREE, SETTING_AUTO_ON_EN, 
                                  auto_settings.auto_on_enabled ? 1 : 0);
    if (rc < 0) {
        LOG_ERR("Failed to save auto_on_en: %d", rc);
        return rc;
    }

    rc = zmk_settings_save_uint8(SETTINGS_SUBTREE, SETTING_AUTO_OFF_EN, 
                                  auto_settings.auto_off_enabled ? 1 : 0);
    if (rc < 0) {
        LOG_ERR("Failed to save auto_off_en: %d", rc);
        return rc;
    }

    rc = zmk_settings_save_uint8(SETTINGS_SUBTREE, SETTING_AUTO_ON, 
                                  auto_settings.auto_on_percent);
    if (rc < 0) {
        LOG_ERR("Failed to save auto_on: %d", rc);
        return rc;
    }

    rc = zmk_settings_save_uint8(SETTINGS_SUBTREE, SETTING_AUTO_OFF, 
                                  auto_settings.auto_off_percent);
    if (rc < 0) {
        LOG_ERR("Failed to save auto_off: %d", rc);
        return rc;
    }

    rc = zmk_settings_save_uint8(SETTINGS_SUBTREE, SETTING_STORAGE, 
                                  auto_settings.storage_percent);
    if (rc < 0) {
        LOG_ERR("Failed to save storage: %d", rc);
        return rc;
    }

    LOG_INF("Settings saved via ZMK Settings");
    return 0;
}

static int load_settings(void) {
    uint8_t val;
    int rc;

    // Load auto_on_enabled
    rc = zmk_settings_load_uint8(SETTINGS_SUBTREE, SETTING_AUTO_ON_EN, &val);
    if (rc == 0) {
        auto_settings.auto_on_enabled = (val != 0);
        LOG_INF("Loaded auto_on_enabled: %d", auto_settings.auto_on_enabled);
    }

    // Load auto_off_enabled
    rc = zmk_settings_load_uint8(SETTINGS_SUBTREE, SETTING_AUTO_OFF_EN, &val);
    if (rc == 0) {
        auto_settings.auto_off_enabled = (val != 0);
        LOG_INF("Loaded auto_off_enabled: %d", auto_settings.auto_off_enabled);
    }

    // Load auto_on_percent
    rc = zmk_settings_load_uint8(SETTINGS_SUBTREE, SETTING_AUTO_ON, &val);
    if (rc == 0) {
        auto_settings.auto_on_percent = val;
        LOG_INF("Loaded auto_on: %d%%", auto_settings.auto_on_percent);
    }

    // Load auto_off_percent
    rc = zmk_settings_load_uint8(SETTINGS_SUBTREE, SETTING_AUTO_OFF, &val);
    if (rc == 0) {
        auto_settings.auto_off_percent = val;
        LOG_INF("Loaded auto_off: %d%%", auto_settings.auto_off_percent);
    }

    // Load storage_percent
    rc = zmk_settings_load_uint8(SETTINGS_SUBTREE, SETTING_STORAGE, &val);
    if (rc == 0) {
        auto_settings.storage_percent = val;
        LOG_INF("Loaded storage: %d%%", auto_settings.storage_percent);
    }

    LOG_INF("Settings loaded successfully");
    LOG_INF("  Auto ON: %s at <%d%%", 
            auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_on_percent);
    LOG_INF("  Auto OFF: %s at >%d%%", 
            auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_off_percent);
    LOG_INF("  Storage at: %d%%", auto_settings.storage_percent);

    return 0;
}

// ============================================================================
// Bootloader Control
// ============================================================================

static bool bootloader_intentional = false;

static void enter_bootloader_mode(void) {
    if (!bootloader_intentional) {
        LOG_ERR("Bootloader trigger blocked - not intentional");
        return;
    }
    LOG_WRN("========================================");
    LOG_WRN("ENTERING BOOTLOADER VIA KEYMAP");
    LOG_WRN("========================================");
    bootloader_intentional = false;
    
    k_sleep(K_MSEC(100));
    
    raise_zmk_position_state_changed(
        (struct zmk_position_state_changed){
            .position = 99,
            .state = true,
            .timestamp = k_uptime_get()
        }
    );
    
    k_sleep(K_MSEC(10));
    
    raise_zmk_position_state_changed(
        (struct zmk_position_state_changed){
            .position = 99,
            .state = false,
            .timestamp = k_uptime_get()
        }
    );
}

static void reset_device(void) {
    LOG_WRN("========================================");
    LOG_WRN("RESETTING DEVICE");
    LOG_WRN("========================================");
    k_sleep(K_MSEC(100));
    sys_reboot(SYS_REBOOT_COLD);
}

static void bootloader_work_handler(struct k_work *work) {
    enter_bootloader_mode();
}

// ============================================================================
// Auto-Update Management
// ============================================================================

static void start_auto_updates(void) {
    if (!auto_update_active) {
        auto_update_active = true;
        auto_update_start_time = k_uptime_get();
        LOG_INF("Auto-updates STARTED - 30 min duration");
    } else {
        auto_update_start_time = k_uptime_get();
        LOG_DBG("Auto-update timer RESET");
    }
}

static bool should_auto_update(void) {
    if (!auto_update_active) return false;
    
    int64_t elapsed_ms = k_uptime_get() - auto_update_start_time;
    
    if (elapsed_ms >= AUTO_UPDATE_DURATION_MS) {
        auto_update_active = false;
        LOG_INF("Auto-updates STOPPED - 30 minutes elapsed");
        return false;
    }
    return true;
}

// ============================================================================
// NTC Thermistor Reading
// ============================================================================

static int16_t read_ntc_temperature(void) {
    int err;
    uint16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };
    
    if (ARRAY_SIZE(adc_channels) == 0) {
        LOG_ERR("No ADC channels configured");
        return 0;
    }
    
    const struct adc_dt_spec *channel = &adc_channels[0];
    
    err = adc_sequence_init_dt(channel, &sequence);
    if (err < 0) {
        LOG_ERR("Failed to init ADC sequence: %d", err);
        return 0;
    }
    
    err = adc_read(channel->dev, &sequence);
    if (err < 0) {
        LOG_ERR("Failed to read ADC: %d", err);
        return 0;
    }
    
    int32_t val_mv = buf;
    err = adc_raw_to_millivolts_dt(channel, &val_mv);
    if (err < 0) {
        LOG_ERR("Failed to convert to mV: %d", err);
        return 0;
    }
    
    if (val_mv >= NTC_REFERENCE_MV) {
        LOG_WRN("NTC voltage at max - thermistor may be disconnected");
        return 0;
    }
    
    float voltage = (float)val_mv / 1000.0f;
    float v_ref = (float)NTC_REFERENCE_MV / 1000.0f;
    
    float ntc_resistance = NTC_SERIES_RESISTOR * voltage / (v_ref - voltage);
    
    float ln_ratio = logf(ntc_resistance / NTC_NOMINAL_RESISTANCE);
    float temp_kelvin = 1.0f / ((1.0f / (NTC_NOMINAL_TEMP + 273.15f)) + 
                                (ln_ratio / NTC_B_COEFFICIENT));
    float temp_celsius = temp_kelvin - 273.15f;
    
    int16_t temp_hundredths = (int16_t)(temp_celsius * 100.0f);
    
    LOG_DBG("NTC: %d mV, %.1f kΩ, %.2f°C", 
            val_mv, ntc_resistance/1000.0f, temp_celsius);
    
    return temp_hundredths;
}

// ============================================================================
// Battery Voltage Reading
// ============================================================================

static void read_battery_voltage(void) {
    const struct device *battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
    
    if (!device_is_ready(battery)) {
        LOG_WRN("Battery sensor not ready");
        return;
    }

    int rc = sensor_sample_fetch(battery);
    if (rc != 0) {
        LOG_WRN("Failed to fetch battery: %d", rc);
        return;
    }

    struct sensor_value voltage;

    if (discovered_channel != SENSOR_CHAN_PRIV_START) {
        rc = sensor_channel_get(battery, discovered_channel, &voltage);
    } else {
        static const enum sensor_channel candidates[] = {
            SENSOR_CHAN_VOLTAGE,
            SENSOR_CHAN_ALL,
            SENSOR_CHAN_GAUGE_VOLTAGE            
        };

        for (int i = 0; i < ARRAY_SIZE(candidates); i++) {
            rc = sensor_channel_get(battery, candidates[i], &voltage);
            if (rc == 0) {
                discovered_channel = candidates[i];
                LOG_INF("Voltage channel discovered: %d", i);
                break;
            }
        }
    }

    if (rc == 0) {
        current_voltage_mv = (voltage.val1 * 1000) + (voltage.val2 / 1000);
        LOG_INF("Battery: %d mV", current_voltage_mv);
    } else {
        LOG_ERR("No valid voltage channel");
        current_voltage_mv = 0;
    }
}

// ============================================================================
// Internal Temperature Sensor
// ============================================================================

static int16_t read_internal_temp(void) {
    struct sensor_value temp_value;
    int ret;
    
    if (temp_dev == NULL) {
        LOG_ERR("Internal temp device not ready");
        return 0;
    }
    
    ret = sensor_sample_fetch(temp_dev);
    if (ret < 0) {
        LOG_ERR("Failed to fetch internal temp: %d", ret);
        return 0;
    }
    
    ret = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &temp_value);
    if (ret < 0) {
        LOG_ERR("Failed to get internal temp: %d", ret);
        return 0;
    }
    
    int16_t temp_celsius = temp_value.val1 * 100;
    temp_celsius += temp_value.val2 / 10000;
    
    return temp_celsius;
}

// ============================================================================
// Update All Sensors
// ============================================================================

static void update_all_sensors(void) {
    temp_internal = read_internal_temp();
    LOG_INF("Internal temp: %.2f°C", temp_internal / 100.0f);
    
    temp_external = read_ntc_temperature();
    LOG_INF("External temp: %.2f°C", temp_external / 100.0f);
    
    read_battery_voltage();
    
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[5], 
                   &temp_internal, sizeof(temp_internal));
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[8], 
                   &current_voltage_mv, sizeof(current_voltage_mv));
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[11], 
                   &temp_external, sizeof(temp_external));
}

static void update_work_handler(struct k_work *work) {
    if (!should_auto_update()) return;
    
    update_all_sensors();
    
    int64_t elapsed = k_uptime_get() - auto_update_start_time;
    int remaining_min = (AUTO_UPDATE_DURATION_MS - elapsed) / 60000;
    LOG_DBG("Auto-update (%d min left)", remaining_min);
    
    k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
}

// ============================================================================
// MOSFET Control
// ============================================================================

static void set_power_state(bool on) {
    if (gpio_dev == NULL) {
        LOG_ERR("GPIO not ready");
        return;
    }
    
    int ret = gpio_pin_set(gpio_dev, MOSFET_PIN, on ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to set MOSFET: %d", ret);
        return;
    }
    
    power_state = on;
    LOG_INF("Battery power %s", on ? "ON" : "OFF");
    
    uint8_t status = power_state ? 0x01 : 0x00;
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[2], &status, sizeof(status));
}

void battery_monitor_power_on(void) {
    set_power_state(true);
}

void battery_monitor_power_off(void) {
    set_power_state(false);
}

void battery_monitor_power_toggle(void) {
    set_power_state(!power_state);
}

bool battery_monitor_get_power_state(void) {
    return power_state;
}

// ============================================================================
// ⭐ Auto MOSFET Control Logic
// ============================================================================

static void check_auto_mosfet(uint8_t percent) {
    // Check AUTO ON (independent control)
    if (auto_settings.auto_on_enabled) {
        if (percent < auto_settings.auto_on_percent && !power_state) {
            LOG_WRN("🔋 AUTO ON: Battery %d%% < %d%% - Enabling MOSFET", 
                    percent, auto_settings.auto_on_percent);
            set_power_state(true);
        }
    }
    
    // Check AUTO OFF (independent control)
    if (auto_settings.auto_off_enabled) {
        if (percent > auto_settings.auto_off_percent && power_state) {
            LOG_INF("🔌 AUTO OFF: Battery %d%% > %d%% - Disabling MOSFET", 
                    percent, auto_settings.auto_off_percent);
            set_power_state(false);
        }
    }
}

// ============================================================================
// Battery Level Monitoring
// ============================================================================

static int battery_level_listener(const zmk_event_t *eh) {
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) return 0;
    
    uint8_t percent = ev->state_of_charge;
    LOG_INF("Battery: %d%%", percent);
    
    // ⭐ Check auto MOSFET first
    check_auto_mosfet(percent);
    
    // Storage mode (always active)
    if (percent <= auto_settings.storage_percent && power_state) {
        LOG_WRN("Storage mode at %d%%", percent);
        set_power_state(false);
    }
    
    // Warnings
    if (percent <= 35 && percent > 20) {
        if (last_battery_percent > 35) {
            LOG_WRN("Battery low: %d%%", percent);
        }
    }
    
    if (percent <= 20) {
        if (last_battery_percent > 20) {
            LOG_ERR("Battery critical: %d%%", percent);
        }
        if (power_state) {
            set_power_state(false);
        }
    }
    
    last_battery_percent = percent;
    return 0;
}

ZMK_LISTENER(battery_monitor, battery_level_listener);
ZMK_SUBSCRIPTION(battery_monitor, zmk_battery_state_changed);

// ============================================================================
// BLE GATT Handlers
// ============================================================================

static ssize_t read_power_control(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    uint8_t status = power_state ? 0x01 : 0x00;
    start_auto_updates();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &status, sizeof(status));
}

static ssize_t write_power_control(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags) {
    if (offset + len > sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t cmd = *((uint8_t *)buf);
    LOG_INF("Power command: 0x%02X", cmd);
    
    switch (cmd) {
        case CMD_POWER_OFF: battery_monitor_power_off(); break;
        case CMD_POWER_ON: battery_monitor_power_on(); break;
        case CMD_POWER_TOGGLE: battery_monitor_power_toggle(); break;
        default:
            LOG_WRN("Unknown command: 0x%02X", cmd);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    return len;
}

static ssize_t read_temp_internal(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("Internal temp read - starting auto-updates");
    temp_internal = read_internal_temp();
    
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &temp_internal, sizeof(temp_internal));
}

static ssize_t read_temp_external(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("External temp read - starting auto-updates");
    temp_external = read_ntc_temperature();
    
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &temp_external, sizeof(temp_external));
}

static ssize_t read_voltage(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("Voltage read - starting auto-updates");
    read_battery_voltage();
    
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &current_voltage_mv, sizeof(current_voltage_mv));
}

static ssize_t write_bootloader(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags) {
    if (offset + len > sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t cmd = *((uint8_t *)buf);
    LOG_WRN("Bootloader command: 0x%02X", cmd);
    
    switch (cmd) {
        case CMD_ENTER_BOOTLOADER:
            LOG_WRN("⚠️  BOOTLOADER in 2s");
            bootloader_intentional = true;
            k_work_reschedule(&bootloader_work, K_MSEC(2000));
            break;
        case CMD_RESET_DEVICE:
            LOG_WRN("⚠️  RESET in 2s");
            k_sleep(K_MSEC(2000));
            reset_device();
            break;
        default:
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    return len;
}

// ⭐ NEW: Read Auto Settings (6 bytes)
// Format: [auto_on_en:1][auto_off_en:1][auto_on:1][auto_off:1][storage:1][reserved:1]
static ssize_t read_auto_settings(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    uint8_t data[6];
    data[0] = auto_settings.auto_on_enabled ? 0x01 : 0x00;
    data[1] = auto_settings.auto_off_enabled ? 0x01 : 0x00;
    data[2] = auto_settings.auto_on_percent;
    data[3] = auto_settings.auto_off_percent;
    data[4] = auto_settings.storage_percent;
    data[5] = 0x00;  // Reserved
    
    LOG_INF("Read auto settings: ON_EN=%d, OFF_EN=%d, ON<%d%%, OFF>%d%%, STOR=%d%%",
            data[0], data[1], data[2], data[3], data[4]);
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

// ⭐ NEW: Write Auto Settings
static ssize_t write_auto_settings(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags) {
    if (offset + len > 6) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *data = (const uint8_t *)buf;
    
    // Validate percentages
    if (data[2] > 100 || data[3] > 100 || data[4] > 100) {
        LOG_ERR("Invalid percentage values");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    if (data[2] >= data[3]) {
        LOG_ERR("auto_on must be < auto_off");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    // Update settings
    auto_settings.auto_on_enabled = (data[0] != 0);
    auto_settings.auto_off_enabled = (data[1] != 0);
    auto_settings.auto_on_percent = data[2];
    auto_settings.auto_off_percent = data[3];
    auto_settings.storage_percent = data[4];
    
    LOG_WRN("⚙️  Auto settings updated:");
    LOG_WRN("   Auto ON: %s at <%d%%", 
            auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_on_percent);
    LOG_WRN("   Auto OFF: %s at >%d%%", 
            auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_off_percent);
    LOG_WRN("   Storage: %d%%", auto_settings.storage_percent);
    
    // Save to NVS
    int rc = save_settings();
    if (rc) {
        LOG_ERR("Failed to save settings: %d", rc);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    // Notify clients
    uint8_t notify_data[6];
    notify_data[0] = auto_settings.auto_on_enabled ? 0x01 : 0x00;
    notify_data[1] = auto_settings.auto_off_enabled ? 0x01 : 0x00;
    notify_data[2] = auto_settings.auto_on_percent;
    notify_data[3] = auto_settings.auto_off_percent;
    notify_data[4] = auto_settings.storage_percent;
    notify_data[5] = 0x00;
    bt_gatt_notify(NULL, attr, notify_data, sizeof(notify_data));
    
    // Immediately check if we need to change power state
    check_auto_mosfet(last_battery_percent);
    
    return len;
}

// ============================================================================
// BLE GATT Service
// ============================================================================

BT_GATT_SERVICE_DEFINE(battery_monitor_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
    
    // Power control
    BT_GATT_CHARACTERISTIC(BT_UUID_POWER_CONTROL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_power_control, write_power_control, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Internal temperature
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMP_INTERNAL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_temp_internal, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Voltage
    BT_GATT_CHARACTERISTIC(BT_UUID_VOLTAGE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_voltage, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // External NTC temperature
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMP_EXTERNAL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_temp_external, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // ⭐ NEW: Auto MOSFET Settings
    BT_GATT_CHARACTERISTIC(BT_UUID_AUTO_SETTINGS,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_auto_settings, write_auto_settings, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Bootloader control
    BT_GATT_CHARACTERISTIC(BT_UUID_BOOTLOADER,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, write_bootloader, NULL),
);

// ============================================================================
// Initialization
// ============================================================================

static int battery_monitor_init(const struct device *dev) {
    ARG_UNUSED(dev);
    int ret;
    
    LOG_INF("Initializing Battery Monitor with Auto MOSFET...");
    
    // Load settings from ZMK Settings
    ret = load_settings();
    if (ret) {
        LOG_WRN("Failed to load settings: %d (using defaults)", ret);
    }
    
    // GPIO
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }
    
    // Internal temp sensor
    temp_dev = DEVICE_DT_GET(DT_NODELABEL(temp));
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("Internal temp not ready");
        return -ENODEV;
    }
    
    // Battery sensor
    battery_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
    if (!device_is_ready(battery_dev)) {
        LOG_WRN("Battery sensor not ready");
        battery_dev = NULL;
    }
    
    // Initialize ADC for NTC
    for (int i = 0; i < ARRAY_SIZE(adc_channels); i++) {
        if (!adc_is_ready_dt(&adc_channels[i])) {
            LOG_ERR("ADC channel %d not ready", i);
            return -ENODEV;
        }
        
        ret = adc_channel_setup_dt(&adc_channels[i]);
        if (ret < 0) {
            LOG_ERR("Failed to setup ADC channel %d: %d", i, ret);
            return ret;
        }
        LOG_INF("ADC channel %d ready (NTC thermistor)", i);
    }
    
    // Configure MOSFET
    ret = gpio_pin_configure(gpio_dev, MOSFET_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure MOSFET: %d", ret);
        return ret;
    }
    
    set_power_state(false);
    
    k_work_init_delayable(&update_work, update_work_handler);
    k_work_init_delayable(&bootloader_work, bootloader_work_handler);
    
    LOG_INF("✅ Battery Monitor initialized");
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("  MOSFET: P0.%d", MOSFET_PIN);
    LOG_INF("  Internal temp: enabled");
    LOG_INF("  External NTC: enabled (P0.28/A4)");
    LOG_INF("  NTC config: 10K@25C, B=%d", NTC_B_COEFFICIENT);
    LOG_INF("  Voltage sensor: %s", battery_dev ? "enabled" : "disabled");
    LOG_INF("  Settings: ZMK Settings API (like ZMK Studio)");
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("⚙️  AUTO MOSFET SETTINGS:");
    LOG_INF("  Auto ON: %s at Battery < %d%%", 
            auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_on_percent);
    LOG_INF("  Auto OFF: %s at Battery > %d%%", 
            auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_off_percent);
    LOG_INF("  Storage: %d%%", auto_settings.storage_percent);
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    return 0;
}

SYS_INIT(battery_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * ============================================================================
 * AUTO MOSFET FEATURE DOCUMENTATION
 * ============================================================================
 * 
 * STORAGE METHOD:
 * ---------------
 * Uses ZMK Settings API (same as ZMK Studio) instead of raw NVS
 * - More reliable and compatible with ZMK ecosystem
 * - Settings persist across reboots and firmware updates
 * - No need to configure NVS partitions manually
 * 
 * FEATURE OVERVIEW:
 * ----------------
 * Automatically controls MOSFET based on battery percentage with INDEPENDENT
 * enable/disable for AUTO ON and AUTO OFF:
 * 
 * 1. AUTO ON (can be enabled/disabled independently)
 *    - When battery < auto_on_percent (default 30%)
 *    - MOSFET turns ON to charge battery
 * 
 * 2. AUTO OFF (can be enabled/disabled independently)
 *    - When battery > auto_off_percent (default 80%)
 *    - MOSFET turns OFF to stop charging
 * 
 * 3. All settings saved via ZMK Settings API (survive reboot)
 * 
 * INDEPENDENT CONTROL EXAMPLES:
 * ----------------------------
 * - Enable ONLY Auto ON: Manually turn off when desired
 * - Enable ONLY Auto OFF: Manually turn on when needed
 * - Enable BOTH: Full automatic control
 * - Disable BOTH: Manual control only
 * 
 * 
 * BLE CHARACTERISTIC:
 * ------------------
 * UUID: 12345678-1234-5678-1234-56789abcdef6
 * 
 * Format (6 bytes):
 *   [0] auto_on_enabled  (0x00=disabled, 0x01=enabled)
 *   [1] auto_off_enabled (0x00=disabled, 0x01=enabled)
 *   [2] auto_on_percent  (percentage 0-100)
 *   [3] auto_off_percent (percentage 0-100)
 *   [4] storage_percent  (percentage 0-100)
 *   [5] reserved         (0x00)
 * 
 * Constraints:
 *   - auto_on_percent must be < auto_off_percent
 *   - All percentages: 0-100
 * 
 * 
 * PYTHON BLE CLIENT EXAMPLE:
 * --------------------------
 * ```python
 * import asyncio
 * from bleak import BleakClient
 * 
 * AUTO_SETTINGS_UUID = "12345678-1234-5678-1234-56789abcdef6"
 * 
 * async def read_auto_settings(address):
 *     async with BleakClient(address) as client:
 *         data = await client.read_gatt_char(AUTO_SETTINGS_UUID)
 *         print(f"Auto ON: {'ENABLED' if data[0] else 'DISABLED'}")
 *         print(f"Auto OFF: {'ENABLED' if data[1] else 'DISABLED'}")
 *         print(f"Auto ON threshold: <{data[2]}%")
 *         print(f"Auto OFF threshold: >{data[3]}%")
 *         print(f"Storage: {data[4]}%")
 * 
 * async def write_auto_settings(address, auto_on_en, auto_off_en, 
 *                               auto_on, auto_off, storage):
 *     async with BleakClient(address) as client:
 *         data = bytes([
 *             1 if auto_on_en else 0,
 *             1 if auto_off_en else 0,
 *             auto_on,
 *             auto_off,
 *             storage,
 *             0  # reserved
 *         ])
 *         await client.write_gatt_char(AUTO_SETTINGS_UUID, data)
 *         print("Settings updated!")
 * 
 * # Usage examples:
 * 
 * # Example 1: Enable BOTH auto ON and auto OFF
 * asyncio.run(write_auto_settings("XX:XX:XX:XX:XX:XX", 
 *     auto_on_en=True,   # Enable auto ON
 *     auto_off_en=True,  # Enable auto OFF
 *     auto_on=25,        # Turn on at <25%
 *     auto_off=85,       # Turn off at >85%
 *     storage=40))
 * 
 * # Example 2: Enable ONLY auto ON (auto OFF disabled)
 * asyncio.run(write_auto_settings("XX:XX:XX:XX:XX:XX", 
 *     auto_on_en=True,   # Enable auto ON
 *     auto_off_en=False, # Disable auto OFF - manual control
 *     auto_on=30,        # Turn on at <30%
 *     auto_off=80,       # Ignored (disabled)
 *     storage=40))
 * 
 * # Example 3: Enable ONLY auto OFF (auto ON disabled)
 * asyncio.run(write_auto_settings("XX:XX:XX:XX:XX:XX", 
 *     auto_on_en=False,  # Disable auto ON - manual control
 *     auto_off_en=True,  # Enable auto OFF
 *     auto_on=30,        # Ignored (disabled)
 *     auto_off=80,       # Turn off at >80%
 *     storage=40))
 * 
 * # Example 4: Disable BOTH (manual control only)
 * asyncio.run(write_auto_settings("XX:XX:XX:XX:XX:XX", 
 *     auto_on_en=False,  # Disable auto ON
 *     auto_off_en=False, # Disable auto OFF
 *     auto_on=30,        # Ignored
 *     auto_off=80,       # Ignored
 *     storage=40))       # Storage still active
 * ```
 * 
 * 
 * USE CASES:
 * ----------
 * 1. Full Auto (both enabled):
 *    - Solar charging system
 *    - Maintain battery 30-80% automatically
 * 
 * 2. Auto ON only:
 *    - Emergency backup power
 *    - Charges when battery critical
 *    - User decides when to stop
 * 
 * 3. Auto OFF only:
 *    - Prevent overcharging
 *    - User starts charging manually
 *    - Auto stops at safe level
 * 
 * 4. Manual only (both disabled):
 *    - Full user control
 *    - Testing or special scenarios
 * 
 * 
 * LOGIC FLOW EXAMPLES:
 * -------------------
 * 
 * BOTH ENABLED:
 * Battery = 25% → Auto ON → MOSFET ON → Charging
 * Battery = 50% → No change → Continue charging
 * Battery = 85% → Auto OFF → MOSFET OFF → Stop charging
 * Battery = 20% → Auto ON → MOSFET ON → Start charging again
 * 
 * ONLY AUTO ON ENABLED:
 * Battery = 25% → Auto ON → MOSFET ON → Charging
 * Battery = 85% → No auto action (stays ON)
 * User manually turns OFF when desired
 * 
 * ONLY AUTO OFF ENABLED:
 * User manually turns ON MOSFET
 * Battery = 50% → No change → Continue charging
 * Battery = 85% → Auto OFF → MOSFET OFF → Stop charging
 * 
 * 
 * ZMK SETTINGS STORAGE:
 * ---------------------
 * Settings stored using ZMK Settings API:
 * - battery_monitor/aon_en    (auto_on_enabled)
 * - battery_monitor/aoff_en   (auto_off_enabled)
 * - battery_monitor/aon       (auto_on_percent)
 * - battery_monitor/aoff      (auto_off_percent)
 * - battery_monitor/stor      (storage_percent)
 * 
 * Advantages over raw NVS:
 * - Compatible with ZMK Studio
 * - Automatic persistence management
 * - No manual partition configuration
 * - Better error handling
 * - Consistent with ZMK ecosystem
 * 
 * Settings persist across:
 * - Device reboots
 * - Power cycles
 * - Firmware updates
 * 
 * REQUIRED CONFIG:
 * ----------------
 * Add to prj.conf:
 * ```
 * # ZMK Settings (automatically includes necessary NVS/Flash configs)
 * CONFIG_ZMK_SETTINGS=y
 * ```
 * 
 * No additional NVS configuration needed!
 * 
 * ============================================================================
 */