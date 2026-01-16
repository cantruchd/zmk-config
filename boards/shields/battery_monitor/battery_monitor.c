
/*
 * battery_monitor.c
 * Custom battery monitoring with Auto MOSFET control + Temperature Protection + Reverse Control
 * 
 * Settings storage: Uses Zephyr Settings API directly (ZMK core already enables NVS)
 * No need to add CONFIG_SETTINGS or CONFIG_NVS - already in ZMK core!
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
#include <zephyr/settings/settings.h>
#include <stdlib.h>
#include <math.h>

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/event_manager.h>
#include <hal/nrf_power.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ============================================================================
// Configuration
// ============================================================================

#define MOSFET_PIN  24

// NTC
#define NTC_REFERENCE_MV      3300
#define NTC_SERIES_RESISTOR   10000
#define NTC_NOMINAL_RESISTANCE 10000
#define NTC_NOMINAL_TEMP      25.0
#define NTC_B_COEFFICIENT     3950

// Defaults - Battery
#define DEFAULT_AUTO_ON_ENABLE    true
#define DEFAULT_AUTO_OFF_ENABLE   true
#define DEFAULT_AUTO_ON_PERCENT   30
#define DEFAULT_AUTO_OFF_PERCENT  80
#define DEFAULT_STORAGE_PERCENT   40
#define DEFAULT_REVERSE_OFF_ENABLE false
#define DEFAULT_REVERSE_OFF_PERCENT 25
#define DEFAULT_REVERSE_ON_ENABLE  false
#define DEFAULT_REVERSE_ON_PERCENT 60

// Defaults - Temperature (in hundredths of degree Celsius)
#define DEFAULT_TEMP_INT_HIGH_ENABLE   false
#define DEFAULT_TEMP_INT_HIGH_THRESHOLD 5000   // 50.00°C
#define DEFAULT_TEMP_INT_LOW_ENABLE    false
#define DEFAULT_TEMP_INT_LOW_THRESHOLD 1000    // 10.00°C
#define DEFAULT_TEMP_EXT_HIGH_ENABLE   false
#define DEFAULT_TEMP_EXT_HIGH_THRESHOLD 6000   // 60.00°C
#define DEFAULT_TEMP_EXT_LOW_ENABLE    false
#define DEFAULT_TEMP_EXT_LOW_THRESHOLD 500     // 5.00°C

#define UPDATE_INTERVAL_MS      10000
#define AUTO_UPDATE_DURATION_MS 1800000

// Settings key (stored as "btmon/cfg" in NVS)
#define SETTINGS_NAME "btmon"


// Connection management
#define MAX_CONNECTIONS 4
#define BOND_ALIAS_MAX_LEN 32

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

#define BT_UUID_AUTO_SETTINGS_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef6)
#define BT_UUID_AUTO_SETTINGS \
    BT_UUID_DECLARE_128(BT_UUID_AUTO_SETTINGS_VAL)

#define BT_UUID_TEMP_SETTINGS_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef7)
#define BT_UUID_TEMP_SETTINGS \
    BT_UUID_DECLARE_128(BT_UUID_TEMP_SETTINGS_VAL)


#define BT_UUID_BOND_MANAGEMENT_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef8)
#define BT_UUID_BOND_MANAGEMENT \
    BT_UUID_DECLARE_128(BT_UUID_BOND_MANAGEMENT_VAL)

// Commands
#define CMD_POWER_OFF    0x00
#define CMD_POWER_ON     0x01
#define CMD_POWER_TOGGLE 0x02
#define CMD_ENTER_BOOTLOADER 0x42
#define CMD_RESET_DEVICE     0x52





// ============================================================================
// Multi bonds
// ============================================================================
struct bond_info {
    bt_addr_le_t addr;
    char alias[BOND_ALIAS_MAX_LEN];
    bool is_connected;
};

struct bond_management_data {
    uint8_t cmd;           // 0x01=list, 0x02=set_alias, 0x03=delete
    uint8_t bond_index;    // Index của bond
    char alias[BOND_ALIAS_MAX_LEN];
} __packed;

static struct bt_conn *active_conns[MAX_CONNECTIONS];
static struct bond_info bond_list[CONFIG_BT_MAX_PAIRED];
static uint8_t bond_count = 0;
static K_MUTEX_DEFINE(conn_mutex);

static void security_changed(struct bt_conn *conn, bt_security_t level,
                             enum bt_security_err err);


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
// Settings Structure
// ============================================================================

struct auto_mosfet_settings {
    bool auto_on_enabled;
    bool auto_off_enabled;
    uint8_t auto_on_percent;
    uint8_t auto_off_percent;
    uint8_t storage_percent;
    // Reverse settings
    bool reverse_off_enabled;
    uint8_t reverse_off_percent;
    bool reverse_on_enabled;
    uint8_t reverse_on_percent;
};

struct temp_protection_settings {
    bool int_high_enabled;
    int16_t int_high_threshold;  // in hundredths °C
    bool int_low_enabled;
    int16_t int_low_threshold;   // in hundredths °C
    bool ext_high_enabled;
    int16_t ext_high_threshold;  // in hundredths °C
    bool ext_low_enabled;
    int16_t ext_low_threshold;   // in hundredths °C
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

static struct auto_mosfet_settings auto_settings = {
    .auto_on_enabled = DEFAULT_AUTO_ON_ENABLE,
    .auto_off_enabled = DEFAULT_AUTO_OFF_ENABLE,
    .auto_on_percent = DEFAULT_AUTO_ON_PERCENT,
    .auto_off_percent = DEFAULT_AUTO_OFF_PERCENT,
    .storage_percent = DEFAULT_STORAGE_PERCENT,
    .reverse_off_enabled = DEFAULT_REVERSE_OFF_ENABLE,
    .reverse_off_percent = DEFAULT_REVERSE_OFF_PERCENT,
    .reverse_on_enabled = DEFAULT_REVERSE_ON_ENABLE,
    .reverse_on_percent = DEFAULT_REVERSE_ON_PERCENT,
};

static struct temp_protection_settings temp_settings = {
    .int_high_enabled = DEFAULT_TEMP_INT_HIGH_ENABLE,
    .int_high_threshold = DEFAULT_TEMP_INT_HIGH_THRESHOLD,
    .int_low_enabled = DEFAULT_TEMP_INT_LOW_ENABLE,
    .int_low_threshold = DEFAULT_TEMP_INT_LOW_THRESHOLD,
    .ext_high_enabled = DEFAULT_TEMP_EXT_HIGH_ENABLE,
    .ext_high_threshold = DEFAULT_TEMP_EXT_HIGH_THRESHOLD,
    .ext_low_enabled = DEFAULT_TEMP_EXT_LOW_ENABLE,
    .ext_low_threshold = DEFAULT_TEMP_EXT_LOW_THRESHOLD,
};

extern const struct bt_gatt_service_static battery_monitor_svc;

// Forward declarations
static ssize_t read_power_control(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t write_power_control(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t read_temp_internal(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t read_temp_external(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t read_voltage(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset);
static ssize_t write_bootloader(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t read_auto_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t write_auto_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t read_temp_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t write_temp_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

// THÊM 2 DÒNG NÀY:
static ssize_t read_bond_management(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset);
static ssize_t write_bond_management(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset, uint8_t flags);     
                                      
static void switch_to_next_available_profile(void);

// ============================================================================
// Settings Management (using Zephyr Settings API like ZMK Studio)
// ============================================================================

static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    
    // Battery settings
    if (settings_name_steq(name, "cfg", &next) && !next) {
        if (len != 9) {
            return -EINVAL;
        }
        
        uint8_t data[9];
        if (read_cb(cb_arg, data, sizeof(data)) != sizeof(data)) {
            return -EINVAL;
        }
        
        auto_settings.auto_on_enabled = (data[0] != 0);
        auto_settings.auto_off_enabled = (data[1] != 0);
        auto_settings.auto_on_percent = data[2];
        auto_settings.auto_off_percent = data[3];
        auto_settings.storage_percent = data[4];
        auto_settings.reverse_off_enabled = (data[5] != 0);
        auto_settings.reverse_off_percent = data[6];
        auto_settings.reverse_on_enabled = (data[7] != 0);
        auto_settings.reverse_on_percent = data[8];
        
        LOG_INF("Battery settings loaded from NVS:");
        LOG_INF("  Auto ON: %s at <%d%%", 
                auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
                auto_settings.auto_on_percent);
        LOG_INF("  Auto OFF: %s at >%d%%", 
                auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
                auto_settings.auto_off_percent);
        LOG_INF("  Storage: %d%%", auto_settings.storage_percent);
        LOG_INF("  Reverse OFF: %s at <%d%%", 
                auto_settings.reverse_off_enabled ? "ENABLED" : "DISABLED",
                auto_settings.reverse_off_percent);
        LOG_INF("  Reverse ON: %s at >%d%%", 
                auto_settings.reverse_on_enabled ? "ENABLED" : "DISABLED",
                auto_settings.reverse_on_percent);
        
        return 0;
    }
    
    // Temperature settings
    if (settings_name_steq(name, "temp", &next) && !next) {
        if (len != 12) {
            return -EINVAL;
        }
        
        uint8_t data[12];
        if (read_cb(cb_arg, data, sizeof(data)) != sizeof(data)) {
            return -EINVAL;
        }
        
        temp_settings.int_high_enabled = (data[0] != 0);
        temp_settings.int_high_threshold = (int16_t)((data[1] << 8) | data[2]);
        temp_settings.int_low_enabled = (data[3] != 0);
        temp_settings.int_low_threshold = (int16_t)((data[4] << 8) | data[5]);
        temp_settings.ext_high_enabled = (data[6] != 0);
        temp_settings.ext_high_threshold = (int16_t)((data[7] << 8) | data[8]);
        temp_settings.ext_low_enabled = (data[9] != 0);
        temp_settings.ext_low_threshold = (int16_t)((data[10] << 8) | data[11]);
        
        LOG_INF("Temperature settings loaded from NVS:");
        LOG_INF("  Internal High: %s at >%d.%02d°C", 
                temp_settings.int_high_enabled ? "ENABLED" : "DISABLED",
                temp_settings.int_high_threshold / 100,
                abs(temp_settings.int_high_threshold % 100));
        LOG_INF("  Internal Low: %s at <%d.%02d°C", 
                temp_settings.int_low_enabled ? "ENABLED" : "DISABLED",
                temp_settings.int_low_threshold / 100,
                abs(temp_settings.int_low_threshold % 100));
        LOG_INF("  External High: %s at >%d.%02d°C", 
                temp_settings.ext_high_enabled ? "ENABLED" : "DISABLED",
                temp_settings.ext_high_threshold / 100,
                abs(temp_settings.ext_high_threshold % 100));
        LOG_INF("  External Low: %s at <%d.%02d°C", 
                temp_settings.ext_low_enabled ? "ENABLED" : "DISABLED",
                temp_settings.ext_low_threshold / 100,
                abs(temp_settings.ext_low_threshold % 100));
        
        return 0;
    }
    
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(battery_monitor, SETTINGS_NAME, NULL, settings_set, NULL, NULL);

static int save_battery_settings(void) {
    uint8_t data[9];
    data[0] = auto_settings.auto_on_enabled ? 1 : 0;
    data[1] = auto_settings.auto_off_enabled ? 1 : 0;
    data[2] = auto_settings.auto_on_percent;
    data[3] = auto_settings.auto_off_percent;
    data[4] = auto_settings.storage_percent;
    data[5] = auto_settings.reverse_off_enabled ? 1 : 0;
    data[6] = auto_settings.reverse_off_percent;
    data[7] = auto_settings.reverse_on_enabled ? 1 : 0;
    data[8] = auto_settings.reverse_on_percent;
    
    int rc = settings_save_one(SETTINGS_NAME "/cfg", data, sizeof(data));
    if (rc) {
        LOG_ERR("Failed to save battery settings: %d", rc);
        return rc;
    }
    
    LOG_INF("Battery settings saved to NVS");
    return 0;
}

static int save_temp_settings(void) {
    uint8_t data[12];
    data[0] = temp_settings.int_high_enabled ? 1 : 0;
    data[1] = (temp_settings.int_high_threshold >> 8) & 0xFF;
    data[2] = temp_settings.int_high_threshold & 0xFF;
    data[3] = temp_settings.int_low_enabled ? 1 : 0;
    data[4] = (temp_settings.int_low_threshold >> 8) & 0xFF;
    data[5] = temp_settings.int_low_threshold & 0xFF;
    data[6] = temp_settings.ext_high_enabled ? 1 : 0;
    data[7] = (temp_settings.ext_high_threshold >> 8) & 0xFF;
    data[8] = temp_settings.ext_high_threshold & 0xFF;
    data[9] = temp_settings.ext_low_enabled ? 1 : 0;
    data[10] = (temp_settings.ext_low_threshold >> 8) & 0xFF;
    data[11] = temp_settings.ext_low_threshold & 0xFF;
    
    int rc = settings_save_one(SETTINGS_NAME "/temp", data, sizeof(data));
    if (rc) {
        LOG_ERR("Failed to save temp settings: %d", rc);
        return rc;
    }
    
    LOG_INF("Temperature settings saved to NVS");
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
    
    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .position = 99, .state = true, .timestamp = k_uptime_get()
    });
    
    k_sleep(K_MSEC(10));
    
    raise_zmk_position_state_changed((struct zmk_position_state_changed){
        .position = 99, .state = false, .timestamp = k_uptime_get()
    });
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
// NTC Thermistor
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
    
    int resistance_int = (int)(ntc_resistance / 1000.0f);
    int resistance_frac = (int)((ntc_resistance / 1000.0f - resistance_int) * 10);
    int temp_int = (int)temp_celsius;
    int temp_frac = (int)((temp_celsius - temp_int) * 100);
    
    LOG_DBG("NTC: %d mV, %d.%d kΩ, %d.%02d°C", 
            val_mv, resistance_int, resistance_frac, temp_int, temp_frac);
    
    return temp_hundredths;
}

// ============================================================================
// Battery Voltage
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
    int rc2;

    if (discovered_channel != SENSOR_CHAN_PRIV_START) {
        rc2 = sensor_channel_get(battery, discovered_channel, &voltage);
    } else {
        static const enum sensor_channel candidates[] = {
            SENSOR_CHAN_VOLTAGE, SENSOR_CHAN_ALL, SENSOR_CHAN_GAUGE_VOLTAGE
        };
        for (int i = 0; i < ARRAY_SIZE(candidates); i++) {
            rc2 = sensor_channel_get(battery, candidates[i], &voltage);
            if (rc2 == 0) {
                discovered_channel = candidates[i];
                LOG_INF("Voltage channel discovered: %d", i);
                break;
            }
        }
    }

    if (rc2 == 0) {
        current_voltage_mv = (voltage.val1 * 1000) + (voltage.val2 / 1000);
        LOG_INF("Battery: %d mV", current_voltage_mv);
    } else {
        LOG_ERR("No valid voltage channel");
        current_voltage_mv = 0;
    }
}

// ============================================================================
// Internal Temperature
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
    //bt_gatt_notify(NULL, &battery_monitor_svc.attrs[2], &status, sizeof(status));


        // Notify all active connections
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[2], &status, sizeof(status));
        }
    }
    k_mutex_unlock(&conn_mutex);
}

void battery_monitor_power_on(void) { set_power_state(true); }
void battery_monitor_power_off(void) { set_power_state(false); }
void battery_monitor_power_toggle(void) { set_power_state(!power_state); }
bool battery_monitor_get_power_state(void) { return power_state; }

// ============================================================================
// Temperature Protection Logic
// ============================================================================

static void check_temp_protection(void) {
    // Internal High - Turn OFF if too hot
    if (temp_settings.int_high_enabled) {
        if (temp_internal > temp_settings.int_high_threshold && power_state) {
            LOG_WRN("🌡️  TEMP PROTECTION: Internal %d.%02d°C > %d.%02d°C - Disabling MOSFET", 
                    temp_internal / 100, abs(temp_internal % 100),
                    temp_settings.int_high_threshold / 100, 
                    abs(temp_settings.int_high_threshold % 100));
            set_power_state(false);
        }
    }
    
    // Internal Low - Turn ON if too cold
    if (temp_settings.int_low_enabled) {
        if (temp_internal < temp_settings.int_low_threshold && !power_state) {
            LOG_INF("🌡️  TEMP RECOVERY: Internal %d.%02d°C < %d.%02d°C - Enabling MOSFET", 
                    temp_internal / 100, abs(temp_internal % 100),
                    temp_settings.int_low_threshold / 100, 
                    abs(temp_settings.int_low_threshold % 100));
            set_power_state(true);
        }
    }
    
    // External High - Turn OFF if too hot
    if (temp_settings.ext_high_enabled) {
        if (temp_external > temp_settings.ext_high_threshold && power_state) {
            LOG_WRN("🌡️  TEMP PROTECTION: External %d.%02d°C > %d.%02d°C - Disabling MOSFET", 
                    temp_external / 100, abs(temp_external % 100),
                    temp_settings.ext_high_threshold / 100, 
                    abs(temp_settings.ext_high_threshold % 100));
            set_power_state(false);
        }
    }
    
    // External Low - Turn ON if too cold
    if (temp_settings.ext_low_enabled) {
        if (temp_external < temp_settings.ext_low_threshold && !power_state) {
            LOG_INF("🌡️  TEMP RECOVERY: External %d.%02d°C < %d.%02d°C - Enabling MOSFET", 
                    temp_external / 100, abs(temp_external % 100),
                    temp_settings.ext_low_threshold / 100, 
                    abs(temp_settings.ext_low_threshold % 100));
            set_power_state(true);
        }
    }
}

// ============================================================================
// Auto MOSFET Logic
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
    
    // Check REVERSE OFF (turn off when battery drops below threshold)
    if (auto_settings.reverse_off_enabled) {
        if (percent < auto_settings.reverse_off_percent && power_state) {
            LOG_WRN("🔋 REVERSE OFF: Battery %d%% < %d%% - Disabling MOSFET", 
                    percent, auto_settings.reverse_off_percent);
            set_power_state(false);
        }
    }
    
    // Check REVERSE ON (turn on when battery rises above threshold)
    if (auto_settings.reverse_on_enabled) {
        if (percent > auto_settings.reverse_on_percent && !power_state) {
            LOG_INF("🔌 REVERSE ON: Battery %d%% > %d%% - Enabling MOSFET", 
                    percent, auto_settings.reverse_on_percent);
            set_power_state(true);
        }
    }
}

// ============================================================================
// Sensors Update
// ============================================================================

static void update_all_sensors(void) {
    temp_internal = read_internal_temp();
    int temp_int_int = temp_internal / 100;
    int temp_int_frac = abs(temp_internal % 100);
    LOG_INF("Internal temp: %d.%02d°C", temp_int_int, temp_int_frac);
    
    temp_external = read_ntc_temperature();
    int temp_ext_int = temp_external / 100;
    int temp_ext_frac = abs(temp_external % 100);
    LOG_INF("External temp: %d.%02d°C", temp_ext_int, temp_ext_frac);
    
    read_battery_voltage();
    
    // Check temperature protection
    check_temp_protection();
    
    // bt_gatt_notify(NULL, &battery_monitor_svc.attrs[5], &temp_internal, sizeof(temp_internal));
    // bt_gatt_notify(NULL, &battery_monitor_svc.attrs[8], &current_voltage_mv, sizeof(current_voltage_mv));
    // bt_gatt_notify(NULL, &battery_monitor_svc.attrs[11], &temp_external, sizeof(temp_external));

    // Notify all active connections
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            // bt_gatt_notify(active_conns[i], attr, data, len);
            bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[5], &temp_internal, sizeof(temp_internal));
            bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[8], &current_voltage_mv, sizeof(current_voltage_mv));
            bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[11], &temp_external, sizeof(temp_external));
        }
    }
    k_mutex_unlock(&conn_mutex);


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
// Battery Monitor
// ============================================================================

static int battery_level_listener(const zmk_event_t *eh) {
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) return 0;
    
    uint8_t percent = ev->state_of_charge;
    LOG_INF("Battery: %d%%", percent);
    
    // Auto MOSFET control
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


// ===========================================================================
// multibond
// ==========================================================================
// ============================================================================
// Connection Management
// ============================================================================

// ============================================================================
// Connection Management
// ============================================================================

// Callback for bt_foreach_bond
static void bond_list_cb(const struct bt_bond_info *info, void *user_data) {
    if (bond_count >= CONFIG_BT_MAX_PAIRED) {
        return;
    }
    
    memcpy(&bond_list[bond_count].addr, &info->addr, sizeof(bt_addr_le_t));
    
    // Check if connected
    bond_list[bond_count].is_connected = false;
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            struct bt_conn_info conn_info;
            bt_conn_get_info(active_conns[i], &conn_info);
            if (bt_addr_le_eq(conn_info.le.dst, &info->addr)) {
                bond_list[bond_count].is_connected = true;
                break;
            }
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    bond_count++;
}


static void refresh_bond_list(void) {
    // Save current aliases before refresh
    char saved_aliases[CONFIG_BT_MAX_PAIRED][BOND_ALIAS_MAX_LEN];
    bt_addr_le_t saved_addrs[CONFIG_BT_MAX_PAIRED];
    
    // Backup aliases
    for (int i = 0; i < CONFIG_BT_MAX_PAIRED; i++) {
        memcpy(saved_aliases[i], bond_list[i].alias, BOND_ALIAS_MAX_LEN);
        memcpy(&saved_addrs[i], &bond_list[i].addr, sizeof(bt_addr_le_t));
    }
    
    bond_count = 0;
    memset(bond_list, 0, sizeof(bond_list));
    
    bt_foreach_bond(BT_ID_DEFAULT, bond_list_cb, NULL);
    
    // Restore aliases by matching addresses
    for (int i = 0; i < bond_count; i++) {
        for (int j = 0; j < CONFIG_BT_MAX_PAIRED; j++) {
            if (bt_addr_le_eq(&bond_list[i].addr, &saved_addrs[j])) {
                if (saved_aliases[j][0] != '\0') {
                    memcpy(bond_list[i].alias, saved_aliases[j], BOND_ALIAS_MAX_LEN);
                    LOG_DBG("Restored alias for bond %d: '%s'", i, bond_list[i].alias);
                }
                break;
            }
        }
    }
    
    LOG_INF("Bond list refreshed: %d bonds", bond_count);
}


static void add_connection(struct bt_conn *conn) {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i] == NULL) {
            active_conns[i] = bt_conn_ref(conn);
            LOG_INF("Connection added at slot %d", i);
            break;
        }
    }
    k_mutex_unlock(&conn_mutex);
}

static void remove_connection(struct bt_conn *conn) {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i] == conn) {
            bt_conn_unref(active_conns[i]);
            active_conns[i] = NULL;
            LOG_INF("Connection removed from slot %d", i);
            break;
        }
    }
    k_mutex_unlock(&conn_mutex);
}

static void connected_cb(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Connection failed: %u", err);
        return;
    }
    
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("✅ Connected: %s", addr);
    
    add_connection(conn);
    refresh_bond_list();
    
    k_mutex_lock(&conn_mutex, K_FOREVER);
    int active_count = 0;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i] != NULL) {
            active_count++;
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    LOG_INF("📊 Active connections: %d/%d (Profile: %d)", 
            active_count, MAX_CONNECTIONS, zmk_ble_active_profile_index());
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason) {
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    
    const char *reason_str;
    switch (reason) {
        case 0x13: reason_str = "Remote user terminated"; break;
        case 0x16: reason_str = "Local host terminated"; break;
        case 0x08: reason_str = "Connection timeout"; break;
        default: reason_str = "Unknown"; break;
    }
    
    LOG_INF("🔌 Disconnected: %s (0x%02X: %s)", addr, reason, reason_str);
    
    remove_connection(conn);
    refresh_bond_list();
    
    k_mutex_lock(&conn_mutex, K_FOREVER);
    int active_count = 0;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i] != NULL) {
            active_count++;
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    LOG_INF("📊 Connections after disconnect: %d/%d", active_count, MAX_CONNECTIONS);
    
    // Auto-switch to next available profile if no connections
    if (active_count == 0) {
        LOG_INF("🔄 No active connections - switching to next profile");
        switch_to_next_available_profile();
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected_cb,
    .disconnected = disconnected_cb,
};



// ============================================================================
// BLE GATT Handlers
// ============================================================================

static ssize_t read_power_control(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    uint8_t status = power_state ? 0x01 : 0x00;
    start_auto_updates();
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &status, sizeof(status));
}

static ssize_t write_power_control(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset + len > sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t cmd = *((uint8_t *)buf);
    LOG_INF("Power command: 0x%02X", cmd);
    
    switch (cmd) {
        case CMD_POWER_OFF: 
            battery_monitor_power_off(); 
            break;
        case CMD_POWER_ON: 
            battery_monitor_power_on(); 
            break;
        case CMD_POWER_TOGGLE: 
            battery_monitor_power_toggle(); 
            break;
        default:
            LOG_WRN("Unknown command: 0x%02X", cmd);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    return len;
}

static ssize_t read_temp_internal(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("Internal temp read - starting auto-updates");
    temp_internal = read_internal_temp();
    
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_internal, sizeof(temp_internal));
}

static ssize_t read_temp_external(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("External temp read - starting auto-updates");
    temp_external = read_ntc_temperature();
    
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &temp_external, sizeof(temp_external));
}

static ssize_t read_voltage(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("Voltage read - starting auto-updates");
    read_battery_voltage();
    
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_voltage_mv, sizeof(current_voltage_mv));
}

static ssize_t write_bootloader(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
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

static ssize_t read_auto_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    uint8_t data[10] = {
        auto_settings.auto_on_enabled ? 0x01 : 0x00,
        auto_settings.auto_off_enabled ? 0x01 : 0x00,
        auto_settings.auto_on_percent,
        auto_settings.auto_off_percent,
        auto_settings.storage_percent,
        auto_settings.reverse_off_enabled ? 0x01 : 0x00,
        auto_settings.reverse_off_percent,
        auto_settings.reverse_on_enabled ? 0x01 : 0x00,
        auto_settings.reverse_on_percent,
        0x00
    };
    
    LOG_INF("Read auto settings: ON_EN=%d, OFF_EN=%d, ON<%d%%, OFF>%d%%, STOR=%d%%, REV_OFF_EN=%d, REV_OFF<%d%%, REV_ON_EN=%d, REV_ON>%d%%",
            data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

static ssize_t write_auto_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset + len > 10) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *data = (const uint8_t *)buf;
    
    // Validate
    if (data[2] > 100 || data[3] > 100 || data[4] > 100 || 
        data[6] > 100 || data[8] > 100) {
        LOG_ERR("Invalid percentage values");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    if (data[2] >= data[3]) {
        LOG_ERR("auto_on must be < auto_off");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    if (data[6] >= data[8]) {
        LOG_ERR("reverse_off must be < reverse_on");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    // Update
    auto_settings.auto_on_enabled = (data[0] != 0);
    auto_settings.auto_off_enabled = (data[1] != 0);
    auto_settings.auto_on_percent = data[2];
    auto_settings.auto_off_percent = data[3];
    auto_settings.storage_percent = data[4];
    auto_settings.reverse_off_enabled = (data[5] != 0);
    auto_settings.reverse_off_percent = data[6];
    auto_settings.reverse_on_enabled = (data[7] != 0);
    auto_settings.reverse_on_percent = data[8];
    
    LOG_WRN("⚙️  Auto settings updated:");
    LOG_WRN("   Auto ON: %s at <%d%%", 
            auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_on_percent);
    LOG_WRN("   Auto OFF: %s at >%d%%", 
            auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_off_percent);
    LOG_WRN("   Storage: %d%%", auto_settings.storage_percent);
    LOG_WRN("   Reverse OFF: %s at <%d%%", 
            auto_settings.reverse_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.reverse_off_percent);
    LOG_WRN("   Reverse ON: %s at >%d%%", 
            auto_settings.reverse_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.reverse_on_percent);
    
    // Save to NVS
    int rc = save_battery_settings();
    if (rc) {
        LOG_ERR("Failed to save settings: %d", rc);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    // Notify
    uint8_t notify_data[10];
    notify_data[0] = auto_settings.auto_on_enabled ? 0x01 : 0x00;
    notify_data[1] = auto_settings.auto_off_enabled ? 0x01 : 0x00;
    notify_data[2] = auto_settings.auto_on_percent;
    notify_data[3] = auto_settings.auto_off_percent;
    notify_data[4] = auto_settings.storage_percent;
    notify_data[5] = auto_settings.reverse_off_enabled ? 0x01 : 0x00;
    notify_data[6] = auto_settings.reverse_off_percent;
    notify_data[7] = auto_settings.reverse_on_enabled ? 0x01 : 0x00;
    notify_data[8] = auto_settings.reverse_on_percent;
    notify_data[9] = 0x00;
    // bt_gatt_notify(NULL, attr, notify_data, sizeof(notify_data));

    // Notify all active connections
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {            
            bt_gatt_notify(active_conns[i], attr, notify_data, sizeof(notify_data));
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    // Apply immediately
    check_auto_mosfet(last_battery_percent);
    
    return len;
}

static ssize_t read_temp_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    uint8_t data[12];
    data[0] = temp_settings.int_high_enabled ? 0x01 : 0x00;
    data[1] = (temp_settings.int_high_threshold >> 8) & 0xFF;
    data[2] = temp_settings.int_high_threshold & 0xFF;
    data[3] = temp_settings.int_low_enabled ? 0x01 : 0x00;
    data[4] = (temp_settings.int_low_threshold >> 8) & 0xFF;
    data[5] = temp_settings.int_low_threshold & 0xFF;
    data[6] = temp_settings.ext_high_enabled ? 0x01 : 0x00;
    data[7] = (temp_settings.ext_high_threshold >> 8) & 0xFF;
    data[8] = temp_settings.ext_high_threshold & 0xFF;
    data[9] = temp_settings.ext_low_enabled ? 0x01 : 0x00;
    data[10] = (temp_settings.ext_low_threshold >> 8) & 0xFF;
    data[11] = temp_settings.ext_low_threshold & 0xFF;
    
    LOG_INF("Read temp settings");
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

static ssize_t write_temp_settings(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (offset + len > 12) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    const uint8_t *data = (const uint8_t *)buf;
    
    // Parse
    temp_settings.int_high_enabled = (data[0] != 0);
    temp_settings.int_high_threshold = (int16_t)((data[1] << 8) | data[2]);
    temp_settings.int_low_enabled = (data[3] != 0);
    temp_settings.int_low_threshold = (int16_t)((data[4] << 8) | data[5]);
    temp_settings.ext_high_enabled = (data[6] != 0);
    temp_settings.ext_high_threshold = (int16_t)((data[7] << 8) | data[8]);
    temp_settings.ext_low_enabled = (data[9] != 0);
    temp_settings.ext_low_threshold = (int16_t)((data[10] << 8) | data[11]);
    
    // Validate
    if (temp_settings.int_low_threshold >= temp_settings.int_high_threshold) {
        LOG_ERR("Internal low must be < high");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    if (temp_settings.ext_low_threshold >= temp_settings.ext_high_threshold) {
        LOG_ERR("External low must be < high");
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    LOG_WRN("🌡️  Temperature settings updated:");
    LOG_WRN("   Internal High: %s at >%d.%02d°C", 
            temp_settings.int_high_enabled ? "ENABLED" : "DISABLED",
            temp_settings.int_high_threshold / 100,
            abs(temp_settings.int_high_threshold % 100));
    LOG_WRN("   Internal Low: %s at <%d.%02d°C", 
            temp_settings.int_low_enabled ? "ENABLED" : "DISABLED",
            temp_settings.int_low_threshold / 100,
            abs(temp_settings.int_low_threshold % 100));
    LOG_WRN("   External High: %s at >%d.%02d°C", 
            temp_settings.ext_high_enabled ? "ENABLED" : "DISABLED",
            temp_settings.ext_high_threshold / 100,
            abs(temp_settings.ext_high_threshold % 100));
    LOG_WRN("   External Low: %s at <%d.%02d°C", 
            temp_settings.ext_low_enabled ? "ENABLED" : "DISABLED",
            temp_settings.ext_low_threshold / 100,
            abs(temp_settings.ext_low_threshold % 100));
    
    // Save to NVS
    int rc = save_temp_settings();
    if (rc) {
        LOG_ERR("Failed to save temp settings: %d", rc);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    // Notify
    // bt_gatt_notify(NULL, attr, data, 12);

    // Notify all active connections
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            bt_gatt_notify(active_conns[i], attr, data, 12);
        }
    }
    k_mutex_unlock(&conn_mutex);

    // Apply immediately
    check_temp_protection();
    
    return len;
}

// ============================================================================
// GATT Service
// ============================================================================

BT_GATT_SERVICE_DEFINE(battery_monitor_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_POWER_CONTROL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_power_control, write_power_control, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMP_INTERNAL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_temp_internal, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_VOLTAGE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_voltage, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMP_EXTERNAL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_temp_external, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_AUTO_SETTINGS,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_auto_settings, write_auto_settings, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_TEMP_SETTINGS,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_temp_settings, write_temp_settings, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    BT_GATT_CHARACTERISTIC(BT_UUID_BOOTLOADER,
                          BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_WRITE,
                          NULL, write_bootloader, NULL),

    BT_GATT_CHARACTERISTIC(BT_UUID_BOND_MANAGEMENT,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_bond_management, write_bond_management, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);





// ============================================================================
// MULTI BOND STORAGE
// ============================================================================


// ============================================================================
// Pairing Management for Multi-Connection
// ============================================================================


// THÊM DÒNG NÀY:
static int save_bond_aliases(void);



// ============================================================================
// Pairing Management for Multi-Connection
// ============================================================================

// ============================================================================
// Pairing Management for Multi-Connection
// ============================================================================






static int settings_set_bonds(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    
    if (settings_name_steq(name, "bonds", &next) && !next) {
        if (len > sizeof(bond_list)) {
            return -EINVAL;
        }
        
        if (read_cb(cb_arg, bond_list, len) != len) {
            return -EINVAL;
        }
        
        LOG_INF("Bond aliases loaded from NVS");
        return 0;
    }
    
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(bond_aliases, SETTINGS_NAME, NULL, settings_set_bonds, NULL, NULL);

static int save_bond_aliases(void) {
    int rc = settings_save_one(SETTINGS_NAME "/bonds", bond_list, sizeof(bond_list));
    if (rc) {
        LOG_ERR("Failed to save bond aliases: %d", rc);
        return rc;
    }
    LOG_INF("Bond aliases saved to NVS with aliases %s", bond_list[0].alias);
    return 0;
}


// ============================================================================
// Simple Profile Switching
// ============================================================================

static void switch_to_next_available_profile(void) {
    uint8_t current_profile = zmk_ble_active_profile_index();
    
    LOG_INF("🔄 Current profile: %d - Searching for next available...", current_profile);
    
    // Try next profiles in sequence
    for (int i = 1; i <= CONFIG_BT_MAX_PAIRED; i++) {
        uint8_t next_profile = (current_profile + i) % CONFIG_BT_MAX_PAIRED;
        
        LOG_DBG("  Trying profile %d...", next_profile);
        
        // Switch to this profile
        int ret = zmk_ble_prof_select(next_profile);
        if (ret == 0) {
            LOG_INF("✅ Switched to profile %d", next_profile);
            
            // Check if this profile is already connected
            bool is_open = zmk_ble_active_profile_is_open();
            if (is_open) {
                LOG_INF("  Profile %d is OPEN - ready for new connections", next_profile);
                return;
            } else {
                LOG_DBG("  Profile %d is already connected - trying next", next_profile);
            }
        } else {
            LOG_DBG("  Profile %d: switch failed (%d)", next_profile, ret);
        }
    }
    
    LOG_WRN("⚠️  All profiles are full or in use - staying on profile %d", current_profile);
}

static ssize_t read_bond_management(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     void *buf, uint16_t len, uint16_t offset) {
    refresh_bond_list();
    
    // Format: [count][bond0_data][bond1_data]...
    // Each bond: [addr:7][alias:32][is_connected:1] = 40 bytes
    uint8_t data[1 + (40 * CONFIG_BT_MAX_PAIRED)];
    data[0] = bond_count;
    
    for (int i = 0; i < bond_count; i++) {
        uint8_t *bond_data = &data[1 + (i * 40)];
        memcpy(bond_data, &bond_list[i].addr, 7);
        memcpy(bond_data + 7, bond_list[i].alias, BOND_ALIAS_MAX_LEN);
        bond_data[39] = bond_list[i].is_connected ? 0x01 : 0x00;
        LOG_INF("Bond list read: %d bonds WITH ALIAS %s", bond_count, bond_list[i].alias);
    }
    
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, 1 + (bond_count * 40));
}

static ssize_t write_bond_management(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                      const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (len < 2) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    const struct bond_management_data *cmd = (const struct bond_management_data *)buf;
    
    refresh_bond_list();
    
    switch (cmd->cmd) {
        case 0x01: // List bonds (already done in read)
            LOG_INF("List bonds command");
            break;
            
        case 0x02: // Set alias
            if (cmd->bond_index >= bond_count) {
                LOG_ERR("Invalid bond index: %d", cmd->bond_index);
                return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
            }
            
            memcpy(bond_list[cmd->bond_index].alias, cmd->alias, BOND_ALIAS_MAX_LEN);
            bond_list[cmd->bond_index].alias[BOND_ALIAS_MAX_LEN - 1] = '\0';
            
            LOG_INF("Set alias for bond %d: %s", cmd->bond_index, bond_list[cmd->bond_index].alias);
            save_bond_aliases();
            break;
            
        case 0x03: // Delete bond
            if (cmd->bond_index >= bond_count) {
                LOG_ERR("Invalid bond index: %d", cmd->bond_index);
                return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
            }
            
            if (bond_list[cmd->bond_index].is_connected) {
                LOG_ERR("Cannot delete active bond");
                return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
            }
            
            LOG_WRN("Deleting bond %d", cmd->bond_index);
            bt_unpair(BT_ID_DEFAULT, &bond_list[cmd->bond_index].addr);
            refresh_bond_list();
            save_bond_aliases();
            break;
            
        default:
            LOG_ERR("Unknown command: 0x%02X", cmd->cmd);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    return len;
}





// ============================================================================
// Initialization
// ============================================================================

static int battery_monitor_init(void) {
    int ret;
    
    LOG_INF("Initializing Battery Monitor with Auto MOSFET + Temperature Protection...");
    
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO not ready");
        return -ENODEV;
    }
    
    temp_dev = DEVICE_DT_GET(DT_NODELABEL(temp));
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("Internal temp not ready");
        return -ENODEV;
    }
    
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


    // Initialize connection array
    memset(active_conns, 0, sizeof(active_conns));
    refresh_bond_list();
    

 


    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("📡 CONNECTION MANAGEMENT:");
    LOG_INF("  Max connections: %d", MAX_CONNECTIONS);
    LOG_INF("  Bonded devices: %d", bond_count);
    LOG_INF("  Multi-connection: ENABLED");
    
    LOG_INF("✅ Battery Monitor initialized");
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("  MOSFET: P0.%d", MOSFET_PIN);
    LOG_INF("  Internal temp: enabled");
    LOG_INF("  External NTC: enabled (P0.28/A4)");
    LOG_INF("  NTC config: 10K@25C, B=%d", NTC_B_COEFFICIENT);
    LOG_INF("  Voltage sensor: %s", battery_dev ? "enabled" : "disabled");
    LOG_INF("  Settings: Zephyr Settings API (NVS)");
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("⚙️  BATTERY AUTO SETTINGS:");
    LOG_INF("  Auto ON: %s at Battery < %d%%", 
            auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_on_percent);
    LOG_INF("  Auto OFF: %s at Battery > %d%%", 
            auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_off_percent);
    LOG_INF("  Storage: %d%%", auto_settings.storage_percent);
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("🌡️  TEMPERATURE PROTECTION:");
    LOG_INF("  Internal High: %s at >%d.%02d°C", 
            temp_settings.int_high_enabled ? "ENABLED" : "DISABLED",
            temp_settings.int_high_threshold / 100,
            abs(temp_settings.int_high_threshold % 100));
    LOG_INF("  Internal Low: %s at <%d.%02d°C", 
            temp_settings.int_low_enabled ? "ENABLED" : "DISABLED",
            temp_settings.int_low_threshold / 100,
            abs(temp_settings.int_low_threshold % 100));
    LOG_INF("  External High: %s at >%d.%02d°C", 
            temp_settings.ext_high_enabled ? "ENABLED" : "DISABLED",
            temp_settings.ext_high_threshold / 100,
            abs(temp_settings.ext_high_threshold % 100));
    LOG_INF("  External Low: %s at <%d.%02d°C", 
            temp_settings.ext_low_enabled ? "ENABLED" : "DISABLED",
            temp_settings.ext_low_threshold / 100,
            abs(temp_settings.ext_low_threshold % 100));
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    return 0;
}

SYS_INIT(battery_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * TEMPERATURE PROTECTION - HOW IT WORKS
 * =====================================
 * 
 * New BLE Characteristic: BT_UUID_TEMP_SETTINGS (def7)
 * Data format: 12 bytes
 * 
 * [0]     int_high_enabled (0/1)
 * [1-2]   int_high_threshold (int16_t, hundredths °C)
 * [3]     int_low_enabled (0/1)
 * [4-5]   int_low_threshold (int16_t, hundredths °C)
 * [6]     ext_high_enabled (0/1)
 * [7-8]   ext_high_threshold (int16_t, hundredths °C)
 * [9]     ext_low_enabled (0/1)
 * [10-11] ext_low_threshold (int16_t, hundredths °C)
 * 
 * Temperature stored in hundredths: 5000 = 50.00°C
 * 
 * Python BLE Example:
 * ```python
 * import asyncio
 * from bleak import BleakClient
 * import struct
 * 
 * TEMP_UUID = "12345678-1234-5678-1234-56789abcdef7"
 * 
 * async def configure_temp(addr):
 *     async with BleakClient(addr) as c:
 *         # Enable all protections:
 *         # Internal: OFF if >50°C, ON if <10°C
 *         # External: OFF if >60°C, ON if <5°C
 *         data = struct.pack('>B h B h B h B h',
 *             1, 5000,  # int high enabled, 50.00°C
 *             1, 1000,  # int low enabled, 10.00°C
 *             1, 6000,  # ext high enabled, 60.00°C
 *             1, 500    # ext low enabled, 5.00°C
 *         )
 *         await c.write_gatt_char(TEMP_UUID, data)
 * 
 * asyncio.run(configure_temp("XX:XX:XX:XX:XX:XX"))
 * ```
 * 
 * Storage: "btmon/temp" in NVS (12 bytes)
 */