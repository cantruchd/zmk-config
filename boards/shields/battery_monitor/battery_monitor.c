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
 * - Optional reed switch support
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

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/events/battery_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ============================================================================
// Configuration Constants
// ============================================================================

// GPIO pins
#define MOSFET_PIN  24  // P0.24 = D5 on nice!nano (right side)
#define REED_PIN    17  // P0.17 = D2 on nice!nano (left side)

// Battery thresholds (percentage)
#define STORAGE_THRESHOLD   40  // Auto-off at 40%
#define LOW_WARNING         35  // Warning at 35%
#define CRITICAL_LOW        20  // Critical at 20%

// Temperature update interval (milliseconds)
#define TEMP_UPDATE_INTERVAL_MS  10000  // 10 seconds

// Reed switch debounce time (milliseconds)
#define REED_DEBOUNCE_MS    50

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

// Temperature characteristic UUID (custom, not standard BLE temp service)
#define BT_UUID_CUSTOM_TEMP_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2)

#define BT_UUID_CUSTOM_TEMP \
    BT_UUID_DECLARE_128(BT_UUID_CUSTOM_TEMP_VAL)

// Commands for power control
#define CMD_POWER_OFF    0x00
#define CMD_POWER_ON     0x01
#define CMD_POWER_TOGGLE 0x02

// ============================================================================
// Global Variables
// ============================================================================

static const struct device *gpio_dev;
static const struct device *temp_dev;
static bool power_state = false;
static uint8_t last_battery_percent = 100;
static int16_t current_temperature = 0;  // Temperature in 0.01°C
static struct k_work_delayable temp_work;

// Forward declaration of GATT service (defined later)
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
    // temp_value.val1 = integer part, val2 = fractional part (in millionths)
    int16_t temp_celsius = temp_value.val1 * 100;
    temp_celsius += temp_value.val2 / 10000;
    
    return temp_celsius;
}

/**
 * Temperature update work handler
 * Periodically reads temperature and updates BLE characteristic
 */
static void temp_work_handler(struct k_work *work) {
    current_temperature = read_temp_sensor();
    
    float temp_float = current_temperature / 100.0f;
    LOG_INF("Temperature: %.2f°C", temp_float);
    
    // Notify connected BLE clients about temperature change
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[5], 
                   &current_temperature, sizeof(current_temperature));
    
    // Schedule next update
    k_work_reschedule(&temp_work, K_MSEC(TEMP_UPDATE_INTERVAL_MS));
}

// ============================================================================
// MOSFET Control Functions
// ============================================================================

/**
 * Set MOSFET state (battery power on/off)
 * @param on: true to turn on, false to turn off
 */
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
    
    // Notify BLE clients about power state change
    uint8_t status = power_state ? 0x01 : 0x00;
    bt_gatt_notify(NULL, &battery_monitor_svc.attrs[2], &status, sizeof(status));
}

/**
 * Turn battery power on
 */
void battery_monitor_power_on(void) {
    LOG_INF("Manual power ON command");
    set_power_state(true);
}

/**
 * Turn battery power off
 */
void battery_monitor_power_off(void) {
    LOG_INF("Manual power OFF command");
    set_power_state(false);
}

/**
 * Toggle battery power state
 */
void battery_monitor_power_toggle(void) {
    LOG_INF("Toggle power command");
    set_power_state(!power_state);
}

/**
 * Get current power state
 */
bool battery_monitor_get_power_state(void) {
    return power_state;
}

// ============================================================================
// Battery Level Monitoring
// ============================================================================

/**
 * Battery state change listener
 */
static int battery_level_listener(const zmk_event_t *eh) {
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    
    if (ev == NULL) {
        return 0;
    }
    
    uint8_t battery_percent = ev->state_of_charge;
    
    LOG_INF("Battery: %d%%", battery_percent);
    
    // Storage mode logic - auto turn off at 40%
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

// Register listener for battery state changes
ZMK_LISTENER(battery_monitor, battery_level_listener);
ZMK_SUBSCRIPTION(battery_monitor, zmk_battery_state_changed);

// ============================================================================
// BLE GATT Service Implementation
// ============================================================================

/**
 * Read handler for power control characteristic
 */
static ssize_t read_power_control(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    uint8_t status = power_state ? 0x01 : 0x00;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &status, sizeof(status));
}

/**
 * Write handler for power control characteristic
 */
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

/**
 * Read handler for temperature characteristic
 */
static ssize_t read_temperature(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset) {
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &current_temperature, sizeof(current_temperature));
}

/**
 * GATT Service Definition
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
);

// ============================================================================
// Reed Switch Handler (Optional)
// ============================================================================

#ifdef CONFIG_BATTERY_MONITOR_REED_SWITCH

static struct gpio_callback reed_cb_data;
static struct k_work_delayable reed_work;

/**
 * Reed switch work handler (debounced)
 * Function: Clear Bluetooth bonds (unpair all devices)
 */
static void reed_switch_work_handler(struct k_work *work) {
    LOG_INF("Reed switch activated - Clearing Bluetooth bonds");
    
    // Clear all Bluetooth bonds
    zmk_ble_clear_bonds();
    
    LOG_WRN("All Bluetooth bonds cleared! Device will restart advertising.");
}

/**
 * Reed switch interrupt handler
 */
static void reed_switch_handler(const struct device *dev, 
                                struct gpio_callback *cb,
                                uint32_t pins) {
    k_work_reschedule(&reed_work, K_MSEC(REED_DEBOUNCE_MS));
}

/**
 * Initialize reed switch
 */
static int init_reed_switch(void) {
    int ret;
    
    ret = gpio_pin_configure(gpio_dev, REED_PIN, 
                            GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret < 0) {
        LOG_ERR("Failed to configure reed switch pin: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_interrupt_configure(gpio_dev, REED_PIN, 
                                       GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure reed interrupt: %d", ret);
        return ret;
    }
    
    gpio_init_callback(&reed_cb_data, reed_switch_handler, BIT(REED_PIN));
    gpio_add_callback(gpio_dev, &reed_cb_data);
    
    k_work_init_delayable(&reed_work, reed_switch_work_handler);
    
    LOG_INF("Reed switch initialized on P0.%d", REED_PIN);
    
    return 0;
}

#endif // CONFIG_BATTERY_MONITOR_REED_SWITCH

// ============================================================================
// Initialization
// ============================================================================

/**
 * Initialize battery monitor
 */
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
    
    // Configure MOSFET pin as output, initially LOW (power off)
    ret = gpio_pin_configure(gpio_dev, MOSFET_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure MOSFET pin: %d", ret);
        return ret;
    }
    
    // Set initial state: OFF (safe default for storage mode)
    set_power_state(false);
    
    // Initialize temperature monitoring
    k_work_init_delayable(&temp_work, temp_work_handler);
    k_work_schedule(&temp_work, K_MSEC(1000));  // Start after 1 second
    
    #ifdef CONFIG_BATTERY_MONITOR_REED_SWITCH
    // Initialize reed switch if enabled
    ret = init_reed_switch();
    if (ret < 0) {
        LOG_WRN("Reed switch initialization failed, continuing without it");
    }
    #endif
    
    LOG_INF("Battery Monitor initialized successfully");
    LOG_INF("  - MOSFET control: P0.%d", MOSFET_PIN);
    LOG_INF("  - Storage threshold: %d%%", STORAGE_THRESHOLD);
    LOG_INF("  - Low warning: %d%%", LOW_WARNING);
    LOG_INF("  - Critical: %d%%", CRITICAL_LOW);
    LOG_INF("  - Temperature sensor: enabled");
    
    return 0;
}

// Initialize at application level
SYS_INIT(battery_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// ============================================================================
// Shell Commands (Optional - for debugging)
// ============================================================================

#ifdef CONFIG_SHELL

#include <zephyr/shell/shell.h>

static int cmd_power_on(const struct shell *shell, size_t argc, char **argv) {
    battery_monitor_power_on();
    shell_print(shell, "Battery power turned ON");
    return 0;
}

static int cmd_power_off(const struct shell *shell, size_t argc, char **argv) {
    battery_monitor_power_off();
    shell_print(shell, "Battery power turned OFF");
    return 0;
}

static int cmd_power_toggle(const struct shell *shell, size_t argc, char **argv) {
    battery_monitor_power_toggle();
    shell_print(shell, "Battery power toggled");
    return 0;
}

static int cmd_status(const struct shell *shell, size_t argc, char **argv) {
    shell_print(shell, "Battery power is currently %s", 
                power_state ? "ON" : "OFF");
    shell_print(shell, "Last battery reading: %d%%", last_battery_percent);
    
    float temp_float = current_temperature / 100.0f;
    shell_print(shell, "Current temperature: %.2f°C", temp_float);
    return 0;
}

static int cmd_temp(const struct shell *shell, size_t argc, char **argv) {
    int16_t temp = read_temp_sensor();
    float temp_float = temp / 100.0f;
    shell_print(shell, "Temperature: %.2f°C", temp_float);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_battery,
    SHELL_CMD(on, NULL, "Turn battery power on", cmd_power_on),
    SHELL_CMD(off, NULL, "Turn battery power off", cmd_power_off),
    SHELL_CMD(toggle, NULL, "Toggle battery power", cmd_power_toggle),
    SHELL_CMD(status, NULL, "Show status", cmd_status),
    SHELL_CMD(temp, NULL, "Read temperature", cmd_temp),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(battery, &sub_battery, "Battery control commands", NULL);

#endif // CONFIG_SHELL

/*
 * BLE Service Summary:
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
 *    - Notify: Updates every 10 seconds
 * 
 * Temperature Format:
 * - Value is int16_t (2 bytes, little-endian)
 * - Unit: 0.01°C
 * - Example: 2550 = 25.50°C
 * - Range: -128°C to +127°C (nRF52840 spec: -40°C to +85°C)
 */