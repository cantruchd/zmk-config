/*
 * battery_monitor.c
 * Custom battery monitoring, MOSFET control, and dual temperature reporting
 * 
 * Location: config/boards/shields/battery_monitor/battery_monitor.c
 * 
 * Features:
 * - MOSFET control for battery power (on/off/toggle)
 * - Automatic storage mode at 40%
 * - Low battery warnings
 * - Dual temperature monitoring:
 *   * Internal: nRF52840 die temperature
 *   * External: NTC thermistor (10K) via ADC
 * - Voltage monitoring and BLE reporting
 * - Smart power saving: Auto-updates 30min after first read, then stops
 * - Bootloader control via BLE (enter flash/DFU mode remotely)
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
#include <hal/nrf_power.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ============================================================================
// Configuration Constants
// ============================================================================

// GPIO pins
#define MOSFET_PIN  24  // P0.24 = D5 on nice!nano

// NTC Thermistor Configuration
#define NTC_ADC_PIN           4    // AIN4 = P0.28 = A4 on nice!nano
#define NTC_REFERENCE_MV      3300 // 3.3V reference
#define NTC_SERIES_RESISTOR   10000 // 10K ohm series resistor
#define NTC_NOMINAL_RESISTANCE 10000 // 10K thermistor at 25°C
#define NTC_NOMINAL_TEMP      25.0  // 25°C
#define NTC_B_COEFFICIENT     3950  // B value for 10K NTC

// Battery thresholds (percentage)
#define STORAGE_THRESHOLD   40
#define LOW_WARNING         35
#define CRITICAL_LOW        20

// Update intervals
#define UPDATE_INTERVAL_MS      10000
#define AUTO_UPDATE_DURATION_MS 1800000

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

// ⭐ NEW: External NTC temperature UUID
#define BT_UUID_TEMP_EXTERNAL_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5)
#define BT_UUID_TEMP_EXTERNAL \
    BT_UUID_DECLARE_128(BT_UUID_TEMP_EXTERNAL_VAL)

// Commands
#define CMD_POWER_OFF    0x00
#define CMD_POWER_ON     0x01
#define CMD_POWER_TOGGLE 0x02
#define CMD_ENTER_BOOTLOADER 0x42
#define CMD_RESET_DEVICE     0x52

// ============================================================================
// ADC Configuration for NTC
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
// Global Variables
// ============================================================================

static const struct device *gpio_dev;
static const struct device *temp_dev;
static const struct device *battery_dev;
static bool power_state = false;
static uint8_t last_battery_percent = 100;

static int16_t temp_internal = 0;      // Internal temp in 0.01°C
static int16_t temp_external = 0;      // External NTC temp in 0.01°C
static uint16_t current_voltage_mv = 0;

static bool auto_update_active = false;
static int64_t auto_update_start_time = 0;
static struct k_work_delayable update_work;
static struct k_work_delayable bootloader_work;

static enum sensor_channel discovered_channel = SENSOR_CHAN_PRIV_START;

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

// ============================================================================
// Bootloader Control
// ============================================================================
static bool bootloader_intentional = false;  // Flag để phân biệt intentional vs accidental

static void enter_bootloader_mode(void) {

    if (!bootloader_intentional) {
        LOG_ERR("Bootloader trigger blocked - not intentional");
        return;
    }
    LOG_WRN("========================================");
    LOG_WRN("ENTERING BOOTLOADER VIA KEYMAP");
    LOG_WRN("========================================");
    bootloader_intentional = false;  // Reset flag
    
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
// NTC Thermistor Reading (External Temperature)
// ============================================================================

/**
 * Read NTC thermistor via ADC and calculate temperature
 * Uses Steinhart-Hart equation
 * @return temperature in 0.01°C
 */
static int16_t read_ntc_temperature(void) {
    int err;
    uint16_t buf;
    struct adc_sequence sequence = {
        .buffer = &buf,
        .buffer_size = sizeof(buf),
    };
    
    // Use first ADC channel (NTC thermistor)
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
    
    // Convert ADC reading to millivolts
    int32_t val_mv = buf;
    err = adc_raw_to_millivolts_dt(channel, &val_mv);
    if (err < 0) {
        LOG_ERR("Failed to convert to mV: %d", err);
        return 0;
    }
    
    // Calculate NTC resistance using voltage divider
    // V_out = V_in * (R_ntc / (R_series + R_ntc))
    // R_ntc = R_series * V_out / (V_in - V_out)
    
    if (val_mv >= NTC_REFERENCE_MV) {
        LOG_WRN("NTC voltage at max - thermistor may be disconnected");
        return 0;
    }
    
    float voltage = (float)val_mv / 1000.0f;
    float v_ref = (float)NTC_REFERENCE_MV / 1000.0f;
    
    float ntc_resistance = NTC_SERIES_RESISTOR * voltage / (v_ref - voltage);
    
    // Steinhart-Hart simplified (B parameter equation)
    // 1/T = 1/T0 + (1/B) * ln(R/R0)
    // T in Kelvin
    
    float ln_ratio = logf(ntc_resistance / NTC_NOMINAL_RESISTANCE);
    float temp_kelvin = 1.0f / ((1.0f / (NTC_NOMINAL_TEMP + 273.15f)) + 
                                (ln_ratio / NTC_B_COEFFICIENT));
    float temp_celsius = temp_kelvin - 273.15f;
    
    // Convert to 0.01°C
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
    // Read internal temperature
    temp_internal = read_internal_temp();
    LOG_INF("Internal temp: %.2f°C", temp_internal / 100.0f);
    
    // Read external NTC temperature
    temp_external = read_ntc_temperature();
    LOG_INF("External temp: %.2f°C", temp_external / 100.0f);
    
    // Read voltage
    read_battery_voltage();
    
    // Notify BLE clients
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
// Battery Level Monitoring
// ============================================================================

static int battery_level_listener(const zmk_event_t *eh) {
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) return 0;
    
    uint8_t percent = ev->state_of_charge;
    LOG_INF("Battery: %d%%", percent);
    
    if (percent <= STORAGE_THRESHOLD && power_state) {
        LOG_WRN("Storage mode at %d%%", percent);
        set_power_state(false);
    }
    
    if (percent <= LOW_WARNING && percent > CRITICAL_LOW) {
        if (last_battery_percent > LOW_WARNING) {
            LOG_WRN("Battery low: %d%%", percent);
        }
    }
    
    if (percent <= CRITICAL_LOW) {
        if (last_battery_percent > CRITICAL_LOW) {
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
// BLE GATT Service
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
            bootloader_intentional = true;// Set flag
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
    
    LOG_INF("Initializing Battery Monitor with NTC...");
    
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
    
    LOG_INF("Battery Monitor initialized");
    LOG_INF("  - MOSFET: P0.%d", MOSFET_PIN);
    LOG_INF("  - Internal temp: enabled");
    LOG_INF("  - External NTC: enabled (P0.28/A4)");
    LOG_INF("  - NTC config: 10K@25C, B=%d", NTC_B_COEFFICIENT);
    LOG_INF("  - Voltage sensor: %s", battery_dev ? "enabled" : "disabled");
    LOG_INF("  - Storage threshold: %d%%", STORAGE_THRESHOLD);
    LOG_INF("  - Bootloader control: enabled");
    
    return 0;
}

SYS_INIT(battery_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * ============================================================================
 * BLE Service Summary
 * ============================================================================
 * 
 * Service UUID: 12345678-1234-5678-1234-56789abcdef0
 * 
 * Characteristics:
 * 1. Power Control (12345678-1234-5678-1234-56789abcdef1)
 *    - Read/Write/Notify
 *    - Commands: 0x00=OFF, 0x01=ON, 0x02=TOGGLE
 * 
 * 2. Internal Temperature (12345678-1234-5678-1234-56789abcdef2)
 *    - Read/Notify - nRF52840 die temperature
 *    - Format: int16_t in 0.01°C (2550 = 25.50°C)
 * 
 * 3. Voltage (12345678-1234-5678-1234-56789abcdef3)
 *    - Read/Notify - Battery voltage
 *    - Format: uint16_t in mV (3700 = 3.7V)
 * 
 * 4. Bootloader Control (12345678-1234-5678-1234-56789abcdef4)
 *    - Write only
 *    - Commands: 0x42=Bootloader, 0x52=Reset
 * 
 * 5. External Temperature (12345678-1234-5678-1234-56789abcdef5) ⭐ NEW
 *    - Read/Notify - NTC thermistor temperature
 *    - Format: int16_t in 0.01°C (2550 = 25.50°C)
 * 
 * ============================================================================
 * Hardware Connections
 * ============================================================================
 * 
 * NTC Thermistor Connection:
 * 
 *     VCC (3.3V)
 *         │
 *         ├─── 10kΩ resistor
 *         │
 *         ├─────────── P0.28 (A4/AIN4) ← ADC reads here
 *         │
 *         ├─── NTC thermistor (10K@25°C)
 *         │
 *        GND
 * 
 * This is a voltage divider circuit:
 * - Series resistor: 10K ohm (connect VCC to P0.28)
 * - NTC thermistor: 10K @ 25°C (connect P0.28 to GND)
 * - ADC measures voltage at the midpoint
 * 
 * Temperature Calculation:
 * 1. ADC reads voltage (0-3.3V)
 * 2. Calculate NTC resistance from voltage divider
 * 3. Use Steinhart-Hart equation with B coefficient
 * 4. Convert to Celsius
 * 
 * Supported NTC Types:
 * - 10K NTC thermistor (most common)
 * - B coefficient: 3950 (typical for 10K NTC)
 * - Can adjust B_COEFFICIENT constant for different thermistors
 * 
 * Example Python BLE client:
 * ```python
 * import asyncio
 * from bleak import BleakClient
 * 
 * TEMP_EXT_UUID = "12345678-1234-5678-1234-56789abcdef5"
 * 
 * async def read_external_temp(address):
 *     async with BleakClient(address) as client:
 *         data = await client.read_gatt_char(TEMP_EXT_UUID)
 *         temp_raw = int.from_bytes(data, 'little', signed=True)
 *         temp_c = temp_raw / 100.0
 *         print(f"External temperature: {temp_c:.2f}°C")
 * ```
 */