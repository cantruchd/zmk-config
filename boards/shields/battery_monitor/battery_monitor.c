// Thêm vào đầu file
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

// Define custom service UUID
#define BT_UUID_CUSTOM_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

#define BT_UUID_CUSTOM_SERVICE \
    BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)

// Define characteristic UUID for fan control
#define BT_UUID_FAN_CONTROL_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1)

#define BT_UUID_FAN_CONTROL \
    BT_UUID_DECLARE_128(BT_UUID_FAN_CONTROL_VAL)

// Commands
#define CMD_FAN_OFF    0x00
#define CMD_FAN_ON     0x01
#define CMD_FAN_TOGGLE 0x02
#define CMD_GET_STATUS 0x03

// ============================================================================
// GATT Characteristic Handlers
// ============================================================================

/**
 * Write handler for fan control characteristic
 * Called when app writes to the characteristic
 */
static ssize_t write_fan_control(struct bt_conn *conn,
                                  const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len,
                                  uint16_t offset, uint8_t flags) {
    if (offset + len > sizeof(uint8_t)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

    uint8_t command = *((uint8_t *)buf);
    
    LOG_INF("Received BLE command: 0x%02X", command);
    
    switch (command) {
        case CMD_FAN_OFF:
            battery_monitor_fan_off();
            break;
            
        case CMD_FAN_ON:
            battery_monitor_fan_on();
            break;
            
        case CMD_FAN_TOGGLE:
            battery_monitor_fan_toggle();
            break;
            
        case CMD_GET_STATUS:
            // Status will be sent via notification/indication
            LOG_INF("Status request - Fan is %s", fan_state ? "ON" : "OFF");
            break;
            
        default:
            LOG_WRN("Unknown command: 0x%02X", command);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    return len;
}

/**
 * Read handler for fan control characteristic
 * Called when app reads the characteristic
 */
static ssize_t read_fan_control(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 void *buf, uint16_t len, uint16_t offset) {
    uint8_t status = fan_state ? 0x01 : 0x00;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &status, sizeof(status));
}

// ============================================================================
// GATT Service Definition
// ============================================================================

BT_GATT_SERVICE_DEFINE(fan_control_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
    
    // Fan control characteristic (read/write)
    BT_GATT_CHARACTERISTIC(BT_UUID_FAN_CONTROL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_fan_control, write_fan_control, NULL),
);




/*
 * battery_monitor.c
 * Custom battery monitoring and MOSFET control logic
 * 
 * Location: config/boards/shields/battery_monitor/battery_monitor.c
 * 
 * Features:
 * - MOSFET control for fan power (on/off/toggle)
 * - Automatic storage mode at 40%
 * - Low battery warnings
 * - Optional reed switch support
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/events/battery_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// ============================================================================
// Configuration Constants
// ============================================================================

// GPIO pins
#define MOSFET_PIN  15  // P0.15 - MOSFET gate control
#define REED_PIN    17  // P0.17 - Reed switch (optional)

// Battery thresholds (percentage)
#define STORAGE_THRESHOLD   40  // Auto-off at 40%
#define LOW_WARNING         35  // Warning at 35%
#define CRITICAL_LOW        20  // Critical at 20%

// Reed switch debounce time (milliseconds)
#define REED_DEBOUNCE_MS    50

// ============================================================================
// Global Variables
// ============================================================================

static const struct device *gpio_dev;
static bool fan_state = false;
static uint8_t last_battery_percent = 100;

// ============================================================================
// MOSFET Control Functions
// ============================================================================

/**
 * Set MOSFET state (fan on/off)
 * @param on: true to turn on, false to turn off
 */
static void set_fan_state(bool on) {
    if (gpio_dev == NULL) {
        LOG_ERR("GPIO device not ready");
        return;
    }
    
    int ret = gpio_pin_set(gpio_dev, MOSFET_PIN, on ? 1 : 0);
    if (ret < 0) {
        LOG_ERR("Failed to set MOSFET pin: %d", ret);
        return;
    }
    
    fan_state = on;
    LOG_INF("Fan %s", on ? "ON" : "OFF");
}

/**
 * Turn fan on
 * Can be called from BLE command or other triggers
 */
void battery_monitor_fan_on(void) {
    LOG_INF("Manual fan ON command");
    set_fan_state(true);
}

/**
 * Turn fan off
 * Can be called from BLE command or other triggers
 */
void battery_monitor_fan_off(void) {
    LOG_INF("Manual fan OFF command");
    set_fan_state(false);
}

/**
 * Toggle fan state
 * Useful for reed switch or button control
 */
void battery_monitor_fan_toggle(void) {
    LOG_INF("Toggle fan command");
    set_fan_state(!fan_state);
}

/**
 * Get current fan state
 * @return true if fan is on, false if off
 */
bool battery_monitor_get_fan_state(void) {
    return fan_state;
}

// ============================================================================
// Battery Monitoring
// ============================================================================

/**
 * Battery state change listener
 * Automatically monitors battery and takes action based on thresholds
 * 
 * This function is called whenever battery state changes (voltage, percentage)
 */
static int battery_monitor_listener(const zmk_event_t *eh) {
    struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    
    if (ev == NULL) {
        return 0;
    }
    
    uint8_t battery_percent = ev->state_of_charge;
    
    LOG_INF("Battery: %d%%", battery_percent);
    
    // Storage mode logic - auto turn off fan at 40%
    if (battery_percent <= STORAGE_THRESHOLD && fan_state) {
        LOG_WRN("Battery at storage level (%d%%), entering storage mode", 
                battery_percent);
        set_fan_state(false);
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
        // Force off if still on at critical level
        if (fan_state) {
            LOG_ERR("Forcing fan OFF due to critical battery");
            set_fan_state(false);
        }
    }
    
    last_battery_percent = battery_percent;
    
    return 0;
}

// Register listener for battery state changes
ZMK_LISTENER(battery_monitor, battery_monitor_listener);
ZMK_SUBSCRIPTION(battery_monitor, zmk_battery_state_changed);

// ============================================================================
// Reed Switch Handler (Optional)
// ============================================================================

#ifdef CONFIG_BATTERY_MONITOR_REED_SWITCH

static struct gpio_callback reed_cb_data;
static struct k_work_delayable reed_work;

/**
 * Reed switch work handler (debounced)
 * This is called after debounce delay
 */
static void reed_switch_work_handler(struct k_work *work) {
    // Toggle fan state when reed switch is activated
    battery_monitor_fan_toggle();
    LOG_INF("Reed switch activated");
}

/**
 * Reed switch interrupt handler
 * Called immediately when reed switch triggers
 */
static void reed_switch_handler(const struct device *dev, 
                                struct gpio_callback *cb,
                                uint32_t pins) {
    // Schedule debounced work
    k_work_reschedule(&reed_work, K_MSEC(REED_DEBOUNCE_MS));
}

/**
 * Initialize reed switch
 * @return 0 on success, negative error code on failure
 */
static int init_reed_switch(void) {
    int ret;
    
    // Configure reed switch pin as input with pull-down
    ret = gpio_pin_configure(gpio_dev, REED_PIN, 
                            GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret < 0) {
        LOG_ERR("Failed to configure reed switch pin: %d", ret);
        return ret;
    }
    
    // Configure interrupt on rising edge
    ret = gpio_pin_interrupt_configure(gpio_dev, REED_PIN, 
                                       GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure reed interrupt: %d", ret);
        return ret;
    }
    
    // Initialize callback
    gpio_init_callback(&reed_cb_data, reed_switch_handler, BIT(REED_PIN));
    gpio_add_callback(gpio_dev, &reed_cb_data);
    
    // Initialize work queue
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
 * Called during system startup
 * @param dev: device structure (unused)
 * @return 0 on success, negative error code on failure
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
    
    // Configure MOSFET pin as output, initially LOW (fan off)
    ret = gpio_pin_configure(gpio_dev, MOSFET_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure MOSFET pin: %d", ret);
        return ret;
    }
    
    // Set initial state: OFF (safe default for storage mode)
    set_fan_state(false);
    
    #ifdef CONFIG_BATTERY_MONITOR_REED_SWITCH
    // Initialize reed switch if enabled
    ret = init_reed_switch();
    if (ret < 0) {
        LOG_WRN("Reed switch initialization failed, continuing without it");
        // Don't return error, continue without reed switch
    }
    #endif
    
    LOG_INF("Battery Monitor initialized successfully");
    LOG_INF("  - MOSFET control: P0.%d", MOSFET_PIN);
    LOG_INF("  - Storage threshold: %d%%", STORAGE_THRESHOLD);
    LOG_INF("  - Low warning: %d%%", LOW_WARNING);
    LOG_INF("  - Critical: %d%%", CRITICAL_LOW);
    
    return 0;
}

// Initialize at application level, after other subsystems
SYS_INIT(battery_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

// ============================================================================
// Shell Commands (Optional - for debugging via USB)
// ============================================================================

#ifdef CONFIG_SHELL

#include <zephyr/shell/shell.h>

static int cmd_fan_on(const struct shell *shell, size_t argc, char **argv) {
    battery_monitor_fan_on();
    shell_print(shell, "Fan turned ON");
    return 0;
}

static int cmd_fan_off(const struct shell *shell, size_t argc, char **argv) {
    battery_monitor_fan_off();
    shell_print(shell, "Fan turned OFF");
    return 0;
}

static int cmd_fan_toggle(const struct shell *shell, size_t argc, char **argv) {
    battery_monitor_fan_toggle();
    shell_print(shell, "Fan toggled");
    return 0;
}

static int cmd_fan_status(const struct shell *shell, size_t argc, char **argv) {
    shell_print(shell, "Fan is currently %s", 
                fan_state ? "ON" : "OFF");
    shell_print(shell, "Last battery reading: %d%%", last_battery_percent);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_fan,
    SHELL_CMD(on, NULL, "Turn fan on", cmd_fan_on),
    SHELL_CMD(off, NULL, "Turn fan off", cmd_fan_off),
    SHELL_CMD(toggle, NULL, "Toggle fan", cmd_fan_toggle),
    SHELL_CMD(status, NULL, "Show fan status", cmd_fan_status),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(fan, &sub_fan, "Fan control commands", NULL);

#endif // CONFIG_SHELL

/*
 * Usage Notes:
 * 
 * This code runs on nice!nano #1 (Cell 1 board) which has MOSFET connected.
 * nice!nano #2 (Cell 2 board) will also run this code but MOSFET control
 * won't do anything since no MOSFET is connected.
 * 
 * Both boards independently:
 * - Monitor their connected cell voltage
 * - Report battery percentage via Bluetooth
 * - Can be paired to same phone/device
 * 
 * To control fan via BLE:
 * - Implement BLE characteristic write handler
 * - Call battery_monitor_fan_on/off/toggle functions
 * - Or use reed switch for physical control
 * 
 * For debugging:
 * - Enable CONFIG_SHELL in .conf
 * - Connect USB
 * - Use serial terminal (115200 baud)
 * - Commands: fan on, fan off, fan toggle, fan status
 */