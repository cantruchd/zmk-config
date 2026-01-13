/*
 * battery_monitor.c
 * Custom battery monitoring, MOSFET control, and temperature reporting
 * 
 * Location: config/boards/shields/battery_monitor/battery_monitor.c
 * 
 * Features:
 * - MOSFET control for battery power (on/off/toggle)
 * - Automatic storage mode at 40%
 * - Low battery warnings
 * - Temperature monitoring and BLE reporting
 * - Voltage monitoring and BLE reporting
 * - Smart power saving: Auto-updates 30min after first read, then stops
 * - Bootloader control via BLE (enter flash/DFU mode remotely) ⭐ NEW
 * - Bond clearing via ZMK keymap
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/reboot.h>
#include <stdlib.h>

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/events/battery_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ============================================================================
// Configuration Constants
// ============================================================================

// GPIO pins
#define MOSFET_PIN  24  // P0.24 = D5 on nice!nano (right side)

// Battery thresholds (percentage)
#define STORAGE_THRESHOLD   40  // Auto-off at 40%
#define LOW_WARNING         35  // Warning at 35%
#define CRITICAL_LOW        20  // Critical at 20%

// Update intervals
#define UPDATE_INTERVAL_MS      10000   // 10 seconds between updates
#define AUTO_UPDATE_DURATION_MS 1800000 // 30 minutes (30 * 60 * 1000)

// ============================================================================
// BLE Service and Characteristic UUIDs
// ============================================================================

// Custom service UUID for battery control and temperature
#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_CUSTOM_SERVICE \
    BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)

// Battery power control characteristic UUID
#define BT_UUID_POWER_CONTROL_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_POWER_CONTROL \
    BT_UUID_DECLARE_128(BT_UUID_POWER_CONTROL_VAL)

// Temperature characteristic UUID
#define BT_UUID_CUSTOM_TEMP_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BT_UUID_CUSTOM_TEMP \
    BT_UUID_DECLARE_128(BT_UUID_CUSTOM_TEMP_VAL)

// Voltage characteristic UUID
#define BT_UUID_VOLTAGE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3)

#define BT_UUID_VOLTAGE \
    BT_UUID_DECLARE_128(BT_UUID_VOLTAGE_VAL)

// ⭐ NEW: Bootloader control characteristic UUID
#define BT_UUID_BOOTLOADER_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4)

#define BT_UUID_BOOTLOADER \
    BT_UUID_DECLARE_128(BT_UUID_BOOTLOADER_VAL)

// Commands for power control
#define CMD_POWER_OFF    0x00
#define CMD_POWER_ON     0x01
#define CMD_POWER_TOGGLE 0x02

// ⭐ NEW: Bootloader commands
#define CMD_ENTER_BOOTLOADER 0x42  // Magic value: 'B' for Bootloader
#define CMD_RESET_DEVICE     0x52  // Magic value: 'R' for Reset

// ============================================================================
// Global Variables
// ============================================================================

static const struct device *gpio_dev;
static const struct device *temp_dev;
static const struct device *battery_dev;
static bool power_state = false;
static uint8_t last_battery_percent = 100;
static int16_t current_temperature = 0;  // Temperature in 0.01°C
static uint16_t current_voltage_mv = 0;  // Voltage in millivolts

// Auto-update state management
static bool auto_update_active = false;
static int64_t auto_update_start_time = 0;
static struct k_work_delayable update_work;

// Channel discovery
static enum sensor_channel discovered_channel = SENSOR_CHAN_PRIV_START;

// ⭐ NEW: Bootloader control work
static struct k_work_delayable bootloader_work;

// Forward declaration of GATT service
extern const struct bt_gatt_service_static battery_monitor_svc;

// Forward declarations for GATT service
static ssize_t read_power_control(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset);
static ssize_t write_power_control(struct bt_conn *conn,
                                    const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len,
                                    uint16_t offset, uint8_t flags);
static ssize_t read_temperature(struct bt_conn *conn,
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
// ⭐ NEW: Bootloader Control Functions
// ============================================================================

/**
 * Enter bootloader/DFU mode
 * This allows firmware flashing via USB without physical reset button
 */
static void enter_bootloader_mode(void) {
    LOG_WRN("========================================");
    LOG_WRN("ENTERING BOOTLOADER MODE");
    LOG_WRN("Device will reboot to bootloader");
    LOG_WRN("Ready for firmware flashing");
    LOG_WRN("========================================");
    
    // Give time for log to flush
    k_sleep(K_MSEC(100));
    
    // Sử dụng ZMK bootloader behavior
    const struct device *dev = device_get_binding("BOOTLOADER");
    if (dev) {
        // Trigger bootloader behavior
        zmk_behavior_invoke_binding(&zmk_behavior_bootloader_config.behavior, 0, true);
    } else {
        // Fallback: sử dụng magic number để vào bootloader
        NRF_POWER->GPREGRET = 0xB1;  // Magic number cho nRF52840
        NVIC_SystemReset();
    }
}

/**
 * Reset device (normal reboot, not bootloader)
 */
static void reset_device(void) {
    LOG_WRN("========================================");
    LOG_WRN("RESETTING DEVICE");
    LOG_WRN("Normal reboot in 1 second...");
    LOG_WRN("========================================");
    
    k_sleep(K_MSEC(100));
    sys_reboot(SYS_REBOOT_COLD);
}

/**
 * Delayed bootloader entry (with countdown)
 * This gives BLE time to send response before reboot
 */
static void bootloader_work_handler(struct k_work *work) {
    enter_bootloader_mode();
}

// ============================================================================
// Auto-Update Management
// ============================================================================

/**
 * Start or restart the 30-minute auto-update timer
 */
static void start_auto_updates(void) {
    if (!auto_update_active) {
        auto_update_active = true;
        auto_update_start_time = k_uptime_get();
        LOG_INF("Auto-updates STARTED - will run for 30 minutes");
    } else {
        // Reset timer if already active
        auto_update_start_time = k_uptime_get();
        LOG_DBG("Auto-update timer RESET");
    }
}

/**
 * Check if auto-updates should still be active
 * @return true if within 30-minute window, false otherwise
 */
static bool should_auto_update(void) {
    if (!auto_update_active) {
        return false;
    }
    
    int64_t elapsed_ms = k_uptime_get() - auto_update_start_time;
    
    if (elapsed_ms >= AUTO_UPDATE_DURATION_MS) {
        auto_update_active = false;
        LOG_INF("Auto-updates STOPPED - 30 minutes elapsed");
        LOG_INF("Power saving mode: Updates disabled until next read");
        return false;
    }
    
    return true;
}

// ============================================================================
// Battery Voltage Reading
// ============================================================================

/**
 * Read battery voltage from ZMK battery sensor
 * Uses channel discovery
 */
static void read_battery_voltage(void) {
    const struct device *battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
    
    if (!device_is_ready(battery)) {
        LOG_WRN("Battery sensor not ready");
        return;
    }

    // 1. Fetch sample from sensor
    int rc = sensor_sample_fetch(battery);
    if (rc != 0) {
        LOG_WRN("Failed to fetch battery: %d", rc);
        return;
    }

    struct sensor_value voltage;

    // 2. If we already know the correct channel, use it directly
    if (discovered_channel != SENSOR_CHAN_PRIV_START) {
        rc = sensor_channel_get(battery, discovered_channel, &voltage);
    } 
    // 3. Otherwise, discover the correct channel (first run)
    else {
        static const enum sensor_channel candidates[] = {
            SENSOR_CHAN_VOLTAGE,
            SENSOR_CHAN_ALL,
            SENSOR_CHAN_GAUGE_VOLTAGE            
        };

        for (int i = 0; i < ARRAY_SIZE(candidates); i++) {
            rc = sensor_channel_get(battery, candidates[i], &voltage);
            if (rc == 0) {
                discovered_channel = candidates[i];
                LOG_INF("Voltage channel discovered: index %d", i);
                break;
            }
        }
    }

    // 4. Process the result
    if (rc == 0) {
        // Calculate mV: val1 (Volts), val2 (Microvolts)
        current_voltage_mv = (voltage.val1 * 1000) + (voltage.val2 / 1000);
        LOG_INF("Voltage: %d mV", current_voltage_mv);
    } else {
        LOG_ERR("No valid voltage channel found");
        current_voltage_mv = 0;
    }
}

// ============================================================================
// Temperature Sensor Functions
// ============================================================================

/**
 * Read temperature from nRF52840 internal sensor
 * @return temperature in 0.01°C (e.g., 2550 = 25.50°C)
 */
static int16_t read_temp_sensor(void) {
    struct sensor_value temp_value;
    int ret;
    
    if (temp_dev == NULL) {
        LOG_ERR("Temperature device not ready");
        return 0;
    }
    
    ret = sensor_sample_fetch(temp_dev);
    if (ret < 0) {
        LOG_ERR("Failed to fetch temperature: %d", ret);
        return 0;
    }
    
    ret = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &temp_value);
    if (ret < 0) {
        LOG_ERR("Failed to get temperature: %d", ret);
        return 0;
    }
    
    // Convert to 0.01°C
    int16_t temp_celsius = temp_value.val1 * 100;
    temp_celsius += temp_value.val2 / 10000;
    
    return temp_celsius;
}

/**
 * Update all sensor values and notify BLE clients
 */
static void update_all_sensors(void) {
    // Read temperature
    current_temperature = read_temp_sensor();
    float temp_float = current_temperature / 100.0f;
    LOG_INF("Temperature: %.2f°C", temp_float);
    
    // Read voltage
    read_battery_voltage();
    
    // Notify BLE clients about temperature change
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[5], 
                   &current_temperature, sizeof(current_temperature));
    
    // Notify BLE clients about voltage change
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[8], 
                   &current_voltage_mv, sizeof(current_voltage_mv));
}

/**
 * Periodic update work handler
 * Only runs when auto-updates are active (within 30min window)
 */
static void update_work_handler(struct k_work *work) {
    if (!should_auto_update()) {
        // 30 minutes elapsed, stop updates
        return;
    }
    
    // Update all sensors
    update_all_sensors();
    
    // Calculate remaining time
    int64_t elapsed_ms = k_uptime_get() - auto_update_start_time;
    int64_t remaining_ms = AUTO_UPDATE_DURATION_MS - elapsed_ms;
    int remaining_min = remaining_ms / 60000;
    
    LOG_DBG("Auto-update running (%d min remaining)", remaining_min);
    
    // Schedule next update
    k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
}

// ============================================================================
// MOSFET Control Functions
// ============================================================================

static void set_power_state(bool on) {
    if (gpio_dev == NULL) {
        LOG_ERR("GPIO device not ready");
        return;
    }
    
    int ret = gpio_pin_set(gpio_dev, MOSFET_PIN, on ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to set MOSFET pin: %d", ret);
        return;
    }
    
    power_state = on;
    LOG_INF("Battery power %s", on ? "ON" : "OFF");
    
    uint8_t status = power_state ? 0x01 : 0x00;
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[2], &status, sizeof(status));
}

void battery_monitor_power_on(void) {
    LOG_INF("Manual power ON command");
    set_power_state(true);
}

void battery_monitor_power_off(void) {
    LOG_INF("Manual power OFF command");
    set_power_state(false);
}

void battery_monitor_power_toggle(void) {
    LOG_INF("Toggle power command");
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
    
    if (ev == NULL) {
        return 0;
    }
    
    uint8_t battery_percent = ev->state_of_charge;
    
    LOG_INF("Battery: %d%%", battery_percent);
    
    // Storage mode logic
    if (battery_percent <= STORAGE_THRESHOLD && power_state) {
        LOG_WRN("Battery at storage level (%d%%), entering storage mode", 
                battery_percent);
        set_power_state(false);
    }
    
    // Low battery warning
    if (battery_percent <= LOW_WARNING && battery_percent > CRITICAL_LOW) {
        if (last_battery_percent > LOW_WARNING) {
            LOG_WRN("Battery low: %d%%", battery_percent);
        }
    }
    
    // Critical battery warning
    if (battery_percent <= CRITICAL_LOW) {
        if (last_battery_percent > CRITICAL_LOW) {
            LOG_ERR("Battery critical: %d%% - Please charge soon!", 
                    battery_percent);
        }
        if (power_state) {
            LOG_ERR("Forcing power OFF due to critical battery");
            set_power_state(false);
        }
    }
    
    last_battery_percent = battery_percent;
    
    return 0;
}

ZMK_LISTENER(battery_monitor, battery_level_listener);
ZMK_SUBSCRIPTION(battery_monitor, zmk_battery_state_changed);

// ============================================================================
// BLE GATT Service Implementation
// ============================================================================

static ssize_t read_power_control(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    uint8_t status = power_state ? 0x01 : 0x00;
    
    // Trigger auto-updates on any characteristic read
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

    uint8_t command = *((uint8_t *)buf);
    
    LOG_INF("Received BLE command: 0x%02X", command);
    
    switch (command) {
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
            LOG_WRN("Unknown command: 0x%02X", command);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    return len;
}

static ssize_t read_temperature(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("Temperature read by host - starting auto-updates");
    
    // Update immediately
    current_temperature = read_temp_sensor();
    
    // Start auto-update timer
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &current_temperature, sizeof(current_temperature));
}

static ssize_t read_voltage(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             void *buf, uint16_t len, uint16_t offset) {
    LOG_INF("Voltage read by host - starting auto-updates");
    
    // Update immediately
    read_battery_voltage();
    
    // Start auto-update timer
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &current_voltage_mv, sizeof(current_voltage_mv));
}

/**
 * ⭐ NEW: Bootloader control write handler
 */
static ssize_t write_bootloader(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 const void *buf, uint16_t len,
                                 uint16_t offset, uint8_t flags) {
    if (offset + len > sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t command = *((uint8_t *)buf);
    
    LOG_WRN("Received bootloader command: 0x%02X", command);
    
    switch (command) {
        case CMD_ENTER_BOOTLOADER:
            LOG_WRN("⚠️  BOOTLOADER MODE REQUESTED");
            LOG_WRN("Device will reboot to bootloader in 2 seconds...");
            
            // Schedule delayed bootloader entry
            // This gives BLE stack time to send response
            k_work_reschedule(&bootloader_work, K_MSEC(2000));
            break;
            
        case CMD_RESET_DEVICE:
            LOG_WRN("⚠️  DEVICE RESET REQUESTED");
            LOG_WRN("Device will reboot in 2 seconds...");
            
            // Delay to allow BLE response
            k_sleep(K_MSEC(2000));
            reset_device();
            break;
            
        default:
            LOG_WRN("Unknown bootloader command: 0x%02X", command);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    return len;
}

/**
 * GATT Service Definition (with bootloader control added)
 */
BT_GATT_SERVICE_DEFINE(battery_monitor_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
    
    // Power control characteristic (read/write/notify)
    BT_GATT_CHARACTERISTIC(BT_UUID_POWER_CONTROL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_power_control, write_power_control, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Temperature characteristic (read/notify)
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_TEMP,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_temperature, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // Voltage characteristic (read/notify)
    BT_GATT_CHARACTERISTIC(BT_UUID_VOLTAGE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_voltage, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // ⭐ NEW: Bootloader control characteristic (write only)
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
    
    LOG_INF("Initializing Battery Monitor...");
    
    // Get GPIO device
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }
    
    // Get temperature sensor device
    temp_dev = DEVICE_DT_GET(DT_NODELABEL(temp));
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("Temperature device not ready");
        return -ENODEV;
    }
    
    // Get battery sensor device
    battery_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
    if (!device_is_ready(battery_dev)) {
        LOG_WRN("Battery voltage sensor not ready");
        battery_dev = NULL;
    } else {
        LOG_INF("Battery voltage sensor ready");
    }
    
    // Configure MOSFET pin
    ret = gpio_pin_configure(gpio_dev, MOSFET_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure MOSFET pin: %d", ret);
        return ret;
    }
    
    set_power_state(false);
    
    // Initialize update work (starts inactive)
    k_work_init_delayable(&update_work, update_work_handler);
    
    // ⭐ NEW: Initialize bootloader work
    k_work_init_delayable(&bootloader_work, bootloader_work_handler);
    
    LOG_INF("Battery Monitor initialized successfully");
    LOG_INF("  - MOSFET control: P0.%d", MOSFET_PIN);
    LOG_INF("  - Storage threshold: %d%%", STORAGE_THRESHOLD);
    LOG_INF("  - Low warning: %d%%", LOW_WARNING);
    LOG_INF("  - Critical: %d%%", CRITICAL_LOW);
    LOG_INF("  - Temperature sensor: enabled");
    LOG_INF("  - Voltage sensor: %s", battery_dev ? "enabled" : "disabled");
    LOG_INF("  - Power saving: Auto-updates disabled (activate by reading)");
    LOG_INF("  - Update interval: %d seconds", UPDATE_INTERVAL_MS / 1000);
    LOG_INF("  - Auto-update duration: 30 minutes");
    LOG_INF("  - Bootloader control: enabled via BLE ⭐");
    LOG_INF("  - Bond clear: via keymap (P0.17/D2 button)");
    
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
 *    - Read: Get current power state (0x00=OFF, 0x01=ON)
 *    - Write: Control power (0x00=OFF, 0x01=ON, 0x02=TOGGLE)
 *    - Notify: Notifies when power state changes
 * 
 * 2. Temperature (12345678-1234-5678-1234-56789abcdef2)
 *    - Read: Get current temperature (int16_t in 0.01°C)
 *    - Notify: Updates every 10 seconds (when auto-update active)
 * 
 * 3. Voltage (12345678-1234-5678-1234-56789abcdef3)
 *    - Read: Get current voltage (uint16_t in millivolts)
 *    - Notify: Updates every 10 seconds (when auto-update active)
 * 
 * 4. Bootloader Control (12345678-1234-5678-1234-56789abcdef4) ⭐ NEW
 *    - Write: Control bootloader/reset
 *      * 0x42 = Enter bootloader mode (for firmware flashing)
 *      * 0x52 = Reset device (normal reboot)
 * 
 * Temperature Format:
 * - Value: int16_t (2 bytes, little-endian)
 * - Unit: 0.01°C
 * - Example: 2550 = 25.50°C
 * 
 * Voltage Format:
 * - Value: uint16_t (2 bytes, little-endian)
 * - Unit: millivolts (mV)
 * - Example: 3700 = 3.7V
 * - Range: 3000-4200 mV (typical LiPo)
 * 
 * ============================================================================
 * Bootloader Control Usage
 * ============================================================================
 * 
 * To enter bootloader mode via BLE:
 * 1. Connect to device via Bluetooth
 * 2. Find characteristic: 12345678-1234-5678-1234-56789abcdef4
 * 3. Write value: 0x42 (decimal 66)
 * 4. Device will reboot to bootloader after 2 seconds
 * 5. Ready for firmware flashing via USB
 * 
 * To reset device:
 * 1. Write value: 0x52 (decimal 82)
 * 2. Device will reboot normally after 2 seconds
 * 
 * Example (Python with bleak):
 * ```python
 * import asyncio
 * from bleak import BleakClient
 * 
 * UUID = "12345678-1234-5678-1234-56789abcdef4"
 * 
 * async def enter_bootloader(address):
 *     async with BleakClient(address) as client:
 */

 