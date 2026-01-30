
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
#define NTC_SERIES_RESISTOR   26620
#define NTC_NOMINAL_RESISTANCE 40700
#define NTC_NOMINAL_TEMP      29.5
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

// ============================================================================
// Standard BLE UUIDs (Bluetooth SIG)
// ============================================================================

// // Battery Service (0x180F) - Standard Bluetooth SIG
// #define BT_UUID_BAS_VAL 0x180F
// #define BT_UUID_BAS \
//     BT_UUID_DECLARE_16(BT_UUID_BAS_VAL)

// // Battery Level Characteristic (0x2A19) - Standard Bluetooth SIG
// #define BT_UUID_BAS_BATTERY_LEVEL_VAL 0x2A19
// #define BT_UUID_BAS_BATTERY_LEVEL \
//     BT_UUID_DECLARE_16(BT_UUID_BAS_BATTERY_LEVEL_VAL)



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

static uint8_t last_battery_percent = 100;




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
extern const struct bt_gatt_service_static bas_svc;  // ⭐ THÊM dòng này

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

static uint8_t voltage_to_percent(uint16_t voltage_mv);
static void start_auto_updates(void);

// ============================================================================
// IR TRANSMISSION FUNCTIONS
// ============================================================================// ============================================================================
// IR AC CONTROL - THÊM VÀO ĐẦU FILE (sau LOG_MODULE_DECLARE)
// ============================================================================

#include <zephyr/drivers/pwm.h>
// IR Configuration
#define IR_TX_PIN   20  // P0.20 (D3) - IR LED
#define IR_RX_PIN   11  // P0.11 (D7) - IR Receiver
#define IR_CARRIER_FREQ 38000  // 38kHz carrier

// IR Timing (microseconds) - NEC Protocol
#define IR_MARK_TIME   560   // Mark pulse
#define IR_SPACE_TIME  560   // Space for bit 0
#define IR_ONE_SPACE   1690  // Space for bit 1
#define IR_HEADER_MARK 9000  // Header mark
#define IR_HEADER_SPACE 4500 // Header space

// IR RX Configuration
#define IR_RX_TIMEOUT_US  100000  // 100ms timeout
#define IR_MAX_PULSES     256     // Max pulses to capture
#define IR_TOLERANCE_US   200     // ±200us tolerance


// IR Receiver data structure
struct ir_pulse {
    uint32_t duration_us;
    bool is_mark;  // true = mark (carrier on), false = space
};

// Thay đổi structure để dùng k_cycle_get_32() cho độ chính xác cao
static struct ir_rx_data {
    struct ir_pulse pulses[IR_MAX_PULSES];
    uint16_t pulse_count;
    bool is_receiving;
    uint32_t last_edge_cycles;  // ⭐ Dùng cycles thay vì time
} ir_rx = {
    .pulse_count = 0,
    .is_receiving = false
};

static struct gpio_callback ir_rx_cb_data;
static K_MUTEX_DEFINE(ir_rx_mutex);


// Max IR data length (support up to 1024 bits = 128 bytes)
#define MAX_IR_DATA_LEN 128

// Auto control rules
#define MAX_AUTO_RULES 50  // Tối đa 50 rules

// ============================================================================
// THÊM UUID CHO AC CONTROL (sau các UUID hiện tại)
// ============================================================================

// UUID cho IR Raw Data transmission
#define BT_UUID_IR_RAW_DATA_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef9)
#define BT_UUID_IR_RAW_DATA \
    BT_UUID_DECLARE_128(BT_UUID_IR_RAW_DATA_VAL)

// UUID cho IR Status
#define BT_UUID_IR_STATUS_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdefa)
#define BT_UUID_IR_STATUS \
    BT_UUID_DECLARE_128(BT_UUID_IR_STATUS_VAL)

// UUID cho IR Command Database (lưu các IR codes với ID)
#define BT_UUID_IR_CMD_DB_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdefb)
#define BT_UUID_IR_CMD_DB \
    BT_UUID_DECLARE_128(BT_UUID_IR_CMD_DB_VAL)

// UUID cho Auto Rules Management
#define BT_UUID_IR_AUTO_RULES_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdefc)
#define BT_UUID_IR_AUTO_RULES \
    BT_UUID_DECLARE_128(BT_UUID_IR_AUTO_RULES_VAL)

// UUID cho Auto Control Enable/Disable
#define BT_UUID_IR_AUTO_ENABLE_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdefd)
#define BT_UUID_IR_AUTO_ENABLE \
    BT_UUID_DECLARE_128(BT_UUID_IR_AUTO_ENABLE_VAL)

// ============================================================================
// THÊM STRUCTURE CHO IR DATA (sau temp_protection_settings)
// ============================================================================

struct ir_raw_data {
    uint16_t data_len;           // Số byte dữ liệu
    uint8_t data[MAX_IR_DATA_LEN]; // Raw IR data từ app
    bool is_transmitting;        // Đang truyền IR
};

struct ir_status {
    uint8_t last_result;  // 0=success, 1=error, 2=busy
    uint16_t bits_sent;   // Số bit đã gửi
};

// IR Command Database - Lưu các IR codes với ID
struct ir_command {
    uint16_t cmd_id;             // Command ID (0-65535)
    uint16_t data_len;           // Độ dài IR data
    uint8_t data[MAX_IR_DATA_LEN]; // Raw IR data
    char description[32];        // Mô tả: "Cool 24C", "Power Off", etc.
};

// Auto Rule - Nếu temp trong khoảng [temp_min, temp_max) thì gửi cmd_id
struct ir_auto_rule {
    uint8_t rule_id;             // Rule ID (0-49)
    bool enabled;                // Enable/disable rule này
    int16_t temp_min;            // Nhiệt độ tối thiểu (hundredths °C)
    int16_t temp_max;            // Nhiệt độ tối đa (hundredths °C)
    uint16_t cmd_id;             // Command ID cần gửi
    uint32_t min_interval_ms;    // Khoảng cách tối thiểu giữa 2 lần gửi (mặc định 10 phút)
    int64_t last_sent_time;      // Timestamp lần gửi cuối
};

// Global auto control state
struct ir_auto_state {
    bool global_enabled;         // Bật/tắt toàn bộ auto control
    uint8_t active_rule_count;   // Số rule đang active
    uint8_t last_matched_rule;   // Rule cuối cùng đã match
};

// ============================================================================
// THÊM GLOBAL VARIABLES (sau temp_settings)
// ============================================================================

static struct ir_raw_data ir_data = {
    .data_len = 0,
    .is_transmitting = false
};

static struct ir_status ir_status = {
    .last_result = 0,
    .bits_sent = 0
};

// Command database - lưu tất cả IR commands
// Sử dụng hash map hoặc array, ở đây dùng array đơn giản
#define MAX_IR_COMMANDS 100
static struct ir_command ir_commands[MAX_IR_COMMANDS] = {0};
static uint8_t ir_cmd_count = 0;

// Auto rules - tối đa 50 rules
static struct ir_auto_rule ir_auto_rules[MAX_AUTO_RULES] = {0};

// Global auto state
static struct ir_auto_state ir_auto_state = {
    .global_enabled = false,
    .active_rule_count = 0,
    .last_matched_rule = 0xFF
};

static const struct device *ir_pwm_dev;
static struct k_work_delayable ir_work;
static K_MUTEX_DEFINE(ir_mutex);

// Default minimum interval (10 minutes)
#define IR_DEFAULT_MIN_INTERVAL_MS 600000

// ============================================================================
// Forward declarations
// ============================================================================
static void ir_carrier_on(uint32_t duration_us);
static void ir_carrier_off(uint32_t duration_us);
static int ir_send_raw_data(const uint8_t *data, uint16_t len_bytes);
static int save_ir_commands(void);
static int save_ir_auto_rules(void);
static int save_ir_auto_state(void);
static void ir_convert_pulses_to_bytes(void);
static int ir_start_learning(void);



// ============================================================================
// IR AUTO CONTROL LOGIC (thêm function mới)
// ============================================================================


// Helper: Convert CPU cycles to microseconds
static inline uint32_t cycles_to_us(uint32_t cycles) {
    // nRF52840 @ 64MHz
    return (cycles * 1000000ULL) / sys_clock_hw_cycles_per_sec();
}


// Helper: Tìm IR command theo ID
static struct ir_command* find_ir_command(uint16_t cmd_id) {
    for (int i = 0; i < ir_cmd_count; i++) {
        if (ir_commands[i].cmd_id == cmd_id) {
            return &ir_commands[i];
        }
    }
    return NULL;
}

// Kiểm tra và thực thi auto rules dựa trên nhiệt độ
static void check_ir_auto_control(void) {
    if (!ir_auto_state.global_enabled) return;
    
    int64_t now = k_uptime_get();
    
    // Duyệt qua tất cả rules (theo thứ tự ưu tiên)
    for (int i = 0; i < MAX_AUTO_RULES; i++) {
        struct ir_auto_rule *rule = &ir_auto_rules[i];
        
        // Skip disabled rules
        if (!rule->enabled) continue;
        
        // Kiểm tra nhiệt độ có nằm trong khoảng không
        if (temp_internal >= rule->temp_min && temp_internal < rule->temp_max) {
            // Check minimum interval
            int64_t elapsed = now - rule->last_sent_time;
            uint32_t min_interval = rule->min_interval_ms > 0 ? 
                                   rule->min_interval_ms : IR_DEFAULT_MIN_INTERVAL_MS;
            
            if (elapsed < min_interval && rule->last_sent_time != 0) {
                LOG_DBG("Rule %d: Too soon (elapsed %lld ms < %u ms)", 
                        i, elapsed, min_interval);
                continue;  // Too soon
            }
            
            // Tìm command tương ứng
            struct ir_command *cmd = find_ir_command(rule->cmd_id);
            if (cmd == NULL) {
                LOG_WRN("Rule %d: Command ID %u not found", i, rule->cmd_id);
                continue;
            }
            
            if (cmd->data_len == 0) {
                LOG_WRN("Rule %d: Command ID %u is empty", i, rule->cmd_id);
                continue;
            }
            
            LOG_INF("❄️ Auto Rule %d matched:", i);
            LOG_INF("   Temp %d.%02d°C in range [%d.%02d, %d.%02d)", 
                    temp_internal / 100, abs(temp_internal % 100),
                    rule->temp_min / 100, abs(rule->temp_min % 100),
                    rule->temp_max / 100, abs(rule->temp_max % 100));
            LOG_INF("   Sending Command %u: %s", cmd->cmd_id, cmd->description);
            
            // Gửi IR command
            int ret = ir_send_raw_data(cmd->data, cmd->data_len);
            
            if (ret == 0) {
                rule->last_sent_time = now;
                ir_auto_state.last_matched_rule = i;
                LOG_INF("✅ Auto IR sent successfully");
                
                // Chỉ gửi 1 rule đầu tiên match, break
                break;
            } else {
                LOG_ERR("❌ Auto IR failed: %d", ret);
            }
        }
    }
}

// Generate 38kHz carrier (bit-banging)
static void ir_carrier_on(uint32_t duration_us) {
    if (!device_is_ready(gpio_dev)) return;
    
    // 38kHz = 26.3us period (13us HIGH + 13us LOW)
    uint32_t cycles = (duration_us * 38) / 1000;
    
    for (uint32_t i = 0; i < cycles; i++) {
        gpio_pin_set(gpio_dev, IR_TX_PIN, 1);
        k_busy_wait(13);  // 13us HIGH
        gpio_pin_set(gpio_dev, IR_TX_PIN, 0);
        k_busy_wait(13);  // 13us LOW
    }
}

// No carrier (space)
static void ir_carrier_off(uint32_t duration_us) {
    if (!device_is_ready(gpio_dev)) return;
    gpio_pin_set(gpio_dev, IR_TX_PIN, 0);
    k_busy_wait(duration_us);
}

// Send NEC header
static void ir_send_header(void) {
    ir_carrier_on(IR_HEADER_MARK);
    ir_carrier_off(IR_HEADER_SPACE);
}

// Send one bit
static void ir_send_bit(uint8_t bit) {
    ir_carrier_on(IR_MARK_TIME);
    if (bit) {
        ir_carrier_off(IR_ONE_SPACE);
    } else {
        ir_carrier_off(IR_SPACE_TIME);
    }
}

// Send raw IR data (with header)
static int ir_send_raw_data(const uint8_t *data, uint16_t len_bytes) {
    if (len_bytes == 0 || len_bytes > MAX_IR_DATA_LEN) {
        LOG_ERR("Invalid IR data length: %d", len_bytes);
        return -EINVAL;
    }
    
    k_mutex_lock(&ir_mutex, K_FOREVER);
    
    if (ir_data.is_transmitting) {
        LOG_WRN("IR transmission in progress");
        k_mutex_unlock(&ir_mutex);
        return -EBUSY;
    }
    
    ir_data.is_transmitting = true;
    ir_status.bits_sent = 0;
    
    LOG_INF("📡 Sending IR: %d bytes", len_bytes);
    
    // Send NEC header
    ir_send_header();
    
    // Send each byte (LSB first for NEC protocol)
    for (uint16_t i = 0; i < len_bytes; i++) {
        uint8_t byte = data[i];
        
        for (int bit = 0; bit < 8; bit++) {
            ir_send_bit((byte >> bit) & 0x01);
            ir_status.bits_sent++;
        }
    }
    
    // Stop bit
    ir_carrier_on(IR_MARK_TIME);
    ir_carrier_off(1000);
    
    ir_data.is_transmitting = false;
    ir_status.last_result = 0;
    
    LOG_INF("✅ IR sent: %d bits", ir_status.bits_sent);
    
    k_mutex_unlock(&ir_mutex);
    return 0;
}



// ⭐ FIXED: GPIO interrupt callback
static void ir_rx_interrupt(const struct device *dev, 
                            struct gpio_callback *cb, 
                            uint32_t pins) {
    uint32_t now_cycles = k_cycle_get_32();
    int pin_state = gpio_pin_get(gpio_dev, IR_RX_PIN);
    
    if (!ir_rx.is_receiving) {
        // Start of new IR signal
        ir_rx.is_receiving = true;
        ir_rx.pulse_count = 0;
        ir_rx.last_edge_cycles = now_cycles;
        LOG_DBG("📥 IR RX started");
        return;
    }
    
    // Calculate pulse duration in microseconds
    uint32_t cycles_elapsed = now_cycles - ir_rx.last_edge_cycles;
    uint32_t duration_us = cycles_to_us(cycles_elapsed);
    
    // Timeout check (100ms = 100000us)
    if (duration_us > IR_RX_TIMEOUT_US) {
        LOG_INF("📥 IR RX complete: %d pulses (timeout)", ir_rx.pulse_count);
        ir_rx.is_receiving = false;
        
        // Convert to bytes
        ir_convert_pulses_to_bytes();
        return;
    }
    
    // Store pulse
    if (ir_rx.pulse_count < IR_MAX_PULSES) {
        ir_rx.pulses[ir_rx.pulse_count].duration_us = duration_us;
        ir_rx.pulses[ir_rx.pulse_count].is_mark = (pin_state == 0);  // TSOP active LOW
        ir_rx.pulse_count++;
        
        // Debug log mỗi 10 pulses
        if (ir_rx.pulse_count % 10 == 0) {
            LOG_DBG("Pulse %d: %d us (%s)", 
                    ir_rx.pulse_count, 
                    duration_us, 
                    pin_state == 0 ? "MARK" : "SPACE");
        }
    } else {
        // Buffer full - force stop
        LOG_WRN("⚠️  Pulse buffer full! Stopping RX");
        ir_rx.is_receiving = false;
        ir_convert_pulses_to_bytes();
        return;
    }
    
    ir_rx.last_edge_cycles = now_cycles;
}

// ⭐ IMPROVED: Pulse to bytes conversion with better logging
static void ir_convert_pulses_to_bytes(void) {
    k_mutex_lock(&ir_rx_mutex, K_FOREVER);
    
    if (ir_rx.pulse_count < 4) {
        LOG_WRN("Too few pulses: %d", ir_rx.pulse_count);
        k_mutex_unlock(&ir_rx_mutex);
        return;
    }
    
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("📊 IR DECODE: %d pulses captured", ir_rx.pulse_count);
    
    // Log first 10 pulses for debugging
    for (int i = 0; i < MIN(10, ir_rx.pulse_count); i++) {
        LOG_INF("  Pulse[%d]: %d us %s", 
                i, 
                ir_rx.pulses[i].duration_us,
                ir_rx.pulses[i].is_mark ? "MARK" : "SPACE");
    }
    
    // Check header (NEC: 9ms mark + 4.5ms space)
    uint32_t header_mark = ir_rx.pulses[0].duration_us;
    uint32_t header_space = ir_rx.pulses[1].duration_us;
    
    LOG_INF("Header: MARK=%d us, SPACE=%d us", header_mark, header_space);
    
    // Tolerant header check (±1000us)
    if (header_mark < 8000 || header_mark > 10000) {
        LOG_WRN("❌ Invalid header mark: %d us (expected ~9000)", header_mark);
        LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        k_mutex_unlock(&ir_rx_mutex);
        return;
    }
    
    if (header_space < 3500 || header_space > 5500) {
        LOG_WRN("❌ Invalid header space: %d us (expected ~4500)", header_space);
        LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        k_mutex_unlock(&ir_rx_mutex);
        return;
    }
    
    LOG_INF("✅ Valid NEC header detected!");
    
    // Decode bits (start from pulse 2)
    uint8_t decoded_data[MAX_IR_DATA_LEN] = {0};
    uint16_t byte_count = 0;
    uint8_t current_byte = 0;
    uint8_t bit_index = 0;
    
    for (int i = 2; i < ir_rx.pulse_count - 1; i += 2) {
        // Each bit = MARK + SPACE
        if (!ir_rx.pulses[i].is_mark) continue;  // Skip if not mark
        
        uint32_t mark_duration = ir_rx.pulses[i].duration_us;
        uint32_t space_duration = ir_rx.pulses[i + 1].duration_us;
        
        // NEC: 560us mark + (560us space = 0, 1690us space = 1)
        bool is_one = (space_duration > 1000);  // Threshold at 1ms
        
        if (is_one) {
            current_byte |= (1 << bit_index);
        }
        
        bit_index++;
        
        if (bit_index == 8) {
            decoded_data[byte_count++] = current_byte;
            LOG_INF("  Byte[%d] = 0x%02X", byte_count - 1, current_byte);
            current_byte = 0;
            bit_index = 0;
            
            if (byte_count >= MAX_IR_DATA_LEN) break;
        }
    }
    
    // Save decoded data
    if (byte_count > 0) {
        memcpy(ir_data.data, decoded_data, byte_count);
        ir_data.data_len = byte_count;
        
        LOG_INF("✅ IR decoded successfully!");
        LOG_INF("   Data: %d bytes", byte_count);
        LOG_HEXDUMP_INF(decoded_data, byte_count, "IR Code");
        
        // Notify app
        uint8_t notify_data[4] = {
            0x02,  // New IR received
            (byte_count >> 8) & 0xFF,
            byte_count & 0xFF,
            0x00
        };
        
        k_mutex_lock(&conn_mutex, K_FOREVER);
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (active_conns[i]) {
                bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[26], 
                              notify_data, sizeof(notify_data));
            }
        }
        k_mutex_unlock(&conn_mutex);
    } else {
        LOG_WRN("❌ No valid data decoded");
    }
    
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    k_mutex_unlock(&ir_rx_mutex);
}

// Start IR learning mode
static int ir_start_learning(void) {
    k_mutex_lock(&ir_rx_mutex, K_FOREVER);
    
    ir_rx.pulse_count = 0;
    ir_rx.is_receiving = false;
    
    LOG_INF("📚 IR learning mode started - point remote and press button");
    
    k_mutex_unlock(&ir_rx_mutex);
    return 0;
}

// Stop IR learning mode
static int ir_stop_learning(void) {
    k_mutex_lock(&ir_rx_mutex, K_FOREVER);
    
    ir_rx.is_receiving = false;
    LOG_INF("⏹️  IR learning mode stopped");
    
    k_mutex_unlock(&ir_rx_mutex);
    return 0;
}



// ============================================================================
// BLE HANDLERS FOR IR RAW DATA
// ============================================================================

static ssize_t read_ir_status(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset) {
    uint8_t data[4] = {
        ir_status.last_result,
        (ir_status.bits_sent >> 8) & 0xFF,
        ir_status.bits_sent & 0xFF,
        ir_data.is_transmitting ? 0x01 : 0x00
    };
    
    LOG_DBG("Read IR status: result=%d, bits=%d, busy=%d",
            data[0], ir_status.bits_sent, data[3]);
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

static ssize_t write_ir_raw_data(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (len < 1 || len > MAX_IR_DATA_LEN) {
        LOG_ERR("Invalid IR data length: %d", len);
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    k_mutex_lock(&ir_mutex, K_FOREVER);
    
    if (ir_data.is_transmitting) {
        LOG_WRN("IR busy - rejecting new data");
        k_mutex_unlock(&ir_mutex);
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }
    
    // Copy data from app
    memcpy(ir_data.data, buf, len);
    ir_data.data_len = len;
    
    k_mutex_unlock(&ir_mutex);
    
    LOG_INF("📥 Received IR data from app: %d bytes", len);
    
    // Log first few bytes for debugging
    if (len >= 4) {
        LOG_INF("   First bytes: 0x%02X 0x%02X 0x%02X 0x%02X",
                ir_data.data[0], ir_data.data[1], ir_data.data[2], ir_data.data[3]);
    }
    
    // Transmit immediately
    int ret = ir_send_raw_data(ir_data.data, ir_data.data_len);
    
    if (ret < 0) {
        ir_status.last_result = 1;  // Error
        LOG_ERR("IR transmission failed: %d", ret);
    } else {
        ir_status.last_result = 0;  // Success
    }
    
    // Notify status update
    uint8_t status_data[4] = {
        ir_status.last_result,
        (ir_status.bits_sent >> 8) & 0xFF,
        ir_status.bits_sent & 0xFF,
        0x00
    };
    
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            bt_gatt_notify(active_conns[i], attr, status_data, sizeof(status_data));
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    return len;
}



// BLE UUID cho IR Learning Control
#define BT_UUID_IR_LEARNING_VAL \
    BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdefe)
#define BT_UUID_IR_LEARNING \
    BT_UUID_DECLARE_128(BT_UUID_IR_LEARNING_VAL)

// Handler: Start/Stop learning mode
static ssize_t write_ir_learning(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = (const uint8_t *)buf;
    uint8_t cmd = data[0];
    
    switch (cmd) {
        case 0x01:  // Start learning
            ir_start_learning();
            break;
            
        case 0x02:  // Stop learning
            ir_stop_learning();
            break;
            
        case 0x03:  // Save learned IR to command DB
            if (len < 5) {
                return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
            }
            
            uint16_t cmd_id = (data[1] << 8) | data[2];
            uint8_t desc_len = data[3];
            
            if (ir_data.data_len == 0) {
                LOG_ERR("No IR data to save");
                return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
            }
            
            // Save to command database
            k_mutex_lock(&ir_mutex, K_FOREVER);
            
            struct ir_command *cmd_ptr = find_ir_command(cmd_id);
            if (cmd_ptr == NULL) {
                if (ir_cmd_count >= MAX_IR_COMMANDS) {
                    k_mutex_unlock(&ir_mutex);
                    return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
                }
                cmd_ptr = &ir_commands[ir_cmd_count++];
            }
            
            cmd_ptr->cmd_id = cmd_id;
            cmd_ptr->data_len = ir_data.data_len;
            memcpy(cmd_ptr->data, ir_data.data, ir_data.data_len);
            memcpy(cmd_ptr->description, &data[4], desc_len);
            cmd_ptr->description[desc_len] = '\0';
            
            save_ir_commands();
            
            k_mutex_unlock(&ir_mutex);
            
            LOG_INF("💾 Learned IR saved as Command %u: '%s'", cmd_id, cmd_ptr->description);
            break;
            
        default:
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    return len;
}

// Handler: App gửi IR code trực tiếp (không học từ remote)
static ssize_t write_ir_from_app(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                  const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    // Format: [cmd_id_h][cmd_id_l][desc_len][description][data...]
    if (len < 4) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = (const uint8_t *)buf;
    uint16_t cmd_id = (data[0] << 8) | data[1];
    uint8_t desc_len = data[2];
    
    if ((3 + desc_len) >= len) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    uint16_t ir_data_len = len - 3 - desc_len;
    
    if (ir_data_len > MAX_IR_DATA_LEN) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }
    
    k_mutex_lock(&ir_mutex, K_FOREVER);
    
    // Find or create command
    struct ir_command *cmd = find_ir_command(cmd_id);
    if (cmd == NULL) {
        if (ir_cmd_count >= MAX_IR_COMMANDS) {
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
        }
        cmd = &ir_commands[ir_cmd_count++];
    }
    
    cmd->cmd_id = cmd_id;
    cmd->data_len = ir_data_len;
    memcpy(cmd->description, &data[3], desc_len);
    cmd->description[desc_len] = '\0';
    memcpy(cmd->data, &data[3 + desc_len], ir_data_len);
    
    save_ir_commands();
    
    k_mutex_unlock(&ir_mutex);
    
    LOG_INF("📲 IR from app saved: Command %u '%s' (%d bytes)", 
            cmd_id, cmd->description, ir_data_len);
    
    return len;
}



// ============================================================================
// BLE HANDLERS FOR IR COMMAND DATABASE
// ============================================================================

static ssize_t read_ir_cmd_db(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                               void *buf, uint16_t len, uint16_t offset) {
    // Format: [cmd_count][cmd1_metadata][cmd2_metadata]...
    // Metadata: [cmd_id_h][cmd_id_l][data_len_h][data_len_l][description_len][description]
    
    uint8_t response[512];
    uint16_t pos = 0;
    
    response[pos++] = ir_cmd_count;
    
    for (int i = 0; i < ir_cmd_count && i < MAX_IR_COMMANDS; i++) {
        struct ir_command *cmd = &ir_commands[i];
        
        response[pos++] = (cmd->cmd_id >> 8) & 0xFF;
        response[pos++] = cmd->cmd_id & 0xFF;
        response[pos++] = (cmd->data_len >> 8) & 0xFF;
        response[pos++] = cmd->data_len & 0xFF;
        
        uint8_t desc_len = strlen(cmd->description);
        response[pos++] = desc_len;
        memcpy(&response[pos], cmd->description, desc_len);
        pos += desc_len;
        
        if (pos > 400) break;  // Avoid overflow
    }
    
    LOG_INF("📖 Read IR command DB: %d commands", ir_cmd_count);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, response, pos);
}

static ssize_t write_ir_cmd_db(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    // Format: [operation][cmd_id_h][cmd_id_l][data...]
    // Operation: 0x01=Add/Update, 0x02=Delete
    
    if (len < 3) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = (const uint8_t *)buf;
    uint8_t operation = data[0];
    uint16_t cmd_id = (data[1] << 8) | data[2];
    
    k_mutex_lock(&ir_mutex, K_FOREVER);
    
    if (operation == 0x01) {  // Add/Update command
        // Format: [0x01][cmd_id_h][cmd_id_l][data_len_h][data_len_l][desc_len][description][ir_data...]
        
        if (len < 6) {
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
        
        uint16_t data_len = (data[3] << 8) | data[4];
        uint8_t desc_len = data[5];
        
        if (data_len > MAX_IR_DATA_LEN || (6 + desc_len + data_len) > len) {
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
        
        // Tìm hoặc tạo mới command
        struct ir_command *cmd = find_ir_command(cmd_id);
        
        if (cmd == NULL) {
            // Tạo mới
            if (ir_cmd_count >= MAX_IR_COMMANDS) {
                k_mutex_unlock(&ir_mutex);
                LOG_ERR("IR command DB full");
                return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
            }
            
            cmd = &ir_commands[ir_cmd_count];
            ir_cmd_count++;
        }
        
        // Update command
        cmd->cmd_id = cmd_id;
        cmd->data_len = data_len;
        memcpy(cmd->description, &data[6], desc_len);
        cmd->description[desc_len] = '\0';
        memcpy(cmd->data, &data[6 + desc_len], data_len);
        
        LOG_INF("💾 IR Command %u: '%s' (%d bytes)", 
                cmd_id, cmd->description, data_len);
        
        // Save to NVS
        save_ir_commands();
        
    } else if (operation == 0x02) {  // Delete command
        
        struct ir_command *cmd = find_ir_command(cmd_id);
        if (cmd == NULL) {
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }
        
        // Shift array to remove
        int index = cmd - ir_commands;
        for (int i = index; i < ir_cmd_count - 1; i++) {
            memcpy(&ir_commands[i], &ir_commands[i + 1], sizeof(struct ir_command));
        }
        ir_cmd_count--;
        
        LOG_INF("🗑️  Deleted IR Command %u", cmd_id);
        
        // Save to NVS
        save_ir_commands();
    }
    else if (operation == 0x03) { // Execute command
        LOG_INF("Attempting to execute IR Command ID %u", cmd_id);
        struct ir_command *cmd = find_ir_command(cmd_id);
        if (cmd == NULL) {
            LOG_ERR("IR Command ID %u not found for execution", cmd_id);
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }
        
        if (cmd->data_len == 0) {
            LOG_WRN("IR Command ID %u has no data to send", cmd_id);
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
        }

        int ret = ir_send_raw_data(cmd->data, cmd->data_len);
        if (ret == 0) {
            LOG_INF("✅ Executed IR Command ID %u: '%s'", cmd_id, cmd->description);
        } else {
            LOG_ERR("❌ Failed to execute IR Command ID %u: %d", cmd_id, ret);
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
        }
    }
    
    k_mutex_unlock(&ir_mutex);
    
    // Notify
    uint8_t notify_data[3] = { operation, (cmd_id >> 8) & 0xFF, cmd_id & 0xFF };
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            bt_gatt_notify(active_conns[i], attr, notify_data, sizeof(notify_data));
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    return len;
}

// ============================================================================
// BLE HANDLERS FOR AUTO RULES
// ============================================================================

static ssize_t read_ir_auto_rules(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    // Format: [active_count][rule0_data][rule1_data]...
    // Rule data: [rule_id][enabled][temp_min_h][temp_min_l][temp_max_h][temp_max_l]
    //            [cmd_id_h][cmd_id_l][interval_ms_bytes_4]
    
    uint8_t response[512];
    uint16_t pos = 0;
    
    uint8_t active_count = 0;
    for (int i = 0; i < MAX_AUTO_RULES; i++) {
        if (ir_auto_rules[i].enabled || ir_auto_rules[i].cmd_id != 0) {
            active_count++;
        }
    }
    
    response[pos++] = active_count;
    
    for (int i = 0; i < MAX_AUTO_RULES; i++) {
        struct ir_auto_rule *rule = &ir_auto_rules[i];
        
        if (!rule->enabled && rule->cmd_id == 0) continue;
        
        response[pos++] = rule->rule_id;
        response[pos++] = rule->enabled ? 0x01 : 0x00;
        response[pos++] = (rule->temp_min >> 8) & 0xFF;
        response[pos++] = rule->temp_min & 0xFF;
        response[pos++] = (rule->temp_max >> 8) & 0xFF;
        response[pos++] = rule->temp_max & 0xFF;
        response[pos++] = (rule->cmd_id >> 8) & 0xFF;
        response[pos++] = rule->cmd_id & 0xFF;
        
        uint32_t interval = rule->min_interval_ms;
        response[pos++] = (interval >> 24) & 0xFF;
        response[pos++] = (interval >> 16) & 0xFF;
        response[pos++] = (interval >> 8) & 0xFF;
        response[pos++] = interval & 0xFF;
        
        if (pos > 400) break;
    }
    
    LOG_INF("📖 Read auto rules: %d active", active_count);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, response, pos);
}

static ssize_t write_ir_auto_rules(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    // Format: [operation][rule_id][data...]
    // Operation: 0x01=Add/Update, 0x02=Delete, 0x03=Toggle Enable
    
    if (len < 2) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = (const uint8_t *)buf;
    uint8_t operation = data[0];
    uint8_t rule_id = data[1];
    
    if (rule_id >= MAX_AUTO_RULES) {
        return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
    }
    
    k_mutex_lock(&ir_mutex, K_FOREVER);
    
    struct ir_auto_rule *rule = &ir_auto_rules[rule_id];
    
    if (operation == 0x01) {  // Add/Update rule
        // Format: [0x01][rule_id][enabled][temp_min_h][temp_min_l][temp_max_h][temp_max_l]
        //         [cmd_id_h][cmd_id_l][interval_4bytes]
        
        if (len < 14) {
            k_mutex_unlock(&ir_mutex);
            return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
        }
        
        rule->rule_id = rule_id;
        rule->enabled = (data[2] != 0);
        rule->temp_min = (int16_t)((data[3] << 8) | data[4]);
        rule->temp_max = (int16_t)((data[5] << 8) | data[6]);
        rule->cmd_id = (data[7] << 8) | data[8];
        rule->min_interval_ms = ((uint32_t)data[9] << 24) | 
                               ((uint32_t)data[10] << 16) |
                               ((uint32_t)data[11] << 8) | 
                               data[12];
        
        // Reset last sent time
        rule->last_sent_time = 0;
        
        // Update active count
        ir_auto_state.active_rule_count = 0;
        for (int i = 0; i < MAX_AUTO_RULES; i++) {
            if (ir_auto_rules[i].enabled) {
                ir_auto_state.active_rule_count++;
            }
        }
        
        LOG_INF("💾 Auto Rule %d: %s", rule_id, rule->enabled ? "ENABLED" : "DISABLED");
        LOG_INF("   Temp range: [%d.%02d, %d.%02d) → Cmd %u",
                rule->temp_min / 100, abs(rule->temp_min % 100),
                rule->temp_max / 100, abs(rule->temp_max % 100),
                rule->cmd_id);
        LOG_INF("   Min interval: %u ms", rule->min_interval_ms);
        
    } else if (operation == 0x02) {  // Delete rule
        
        memset(rule, 0, sizeof(struct ir_auto_rule));
        rule->rule_id = rule_id;
        
        // Update active count
        ir_auto_state.active_rule_count = 0;
        for (int i = 0; i < MAX_AUTO_RULES; i++) {
            if (ir_auto_rules[i].enabled) {
                ir_auto_state.active_rule_count++;
            }
        }
        
        LOG_INF("🗑️  Deleted Auto Rule %d", rule_id);
        
    } else if (operation == 0x03) {  // Toggle enable
        
        rule->enabled = !rule->enabled;
        rule->last_sent_time = 0;  // Reset
        
        // Update active count
        ir_auto_state.active_rule_count = 0;
        for (int i = 0; i < MAX_AUTO_RULES; i++) {
            if (ir_auto_rules[i].enabled) {
                ir_auto_state.active_rule_count++;
            }
        }
        
        LOG_INF("🔄 Rule %d: %s", rule_id, rule->enabled ? "ENABLED" : "DISABLED");
    }
    
    // Save to NVS
    save_ir_auto_rules();
    
    k_mutex_unlock(&ir_mutex);
    
    // Notify
    uint8_t notify_data[3] = { operation, rule_id, rule->enabled ? 0x01 : 0x00 };
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            bt_gatt_notify(active_conns[i], attr, notify_data, sizeof(notify_data));
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    return len;
}

// ============================================================================
// BLE HANDLERS FOR GLOBAL AUTO ENABLE
// ============================================================================

static ssize_t read_ir_auto_enable(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                    void *buf, uint16_t len, uint16_t offset) {
    uint8_t data[3] = {
        ir_auto_state.global_enabled ? 0x01 : 0x00,
        ir_auto_state.active_rule_count,
        ir_auto_state.last_matched_rule
    };
    
    LOG_INF("📖 Auto Control: %s (%d active rules)", 
            ir_auto_state.global_enabled ? "ENABLED" : "DISABLED",
            ir_auto_state.active_rule_count);
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, data, sizeof(data));
}

static ssize_t write_ir_auto_enable(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                     const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    if (len < 1) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    const uint8_t *data = (const uint8_t *)buf;
    bool enabled = (data[0] != 0);
    
    k_mutex_lock(&ir_mutex, K_FOREVER);
    
    bool was_enabled = ir_auto_state.global_enabled;
    ir_auto_state.global_enabled = enabled;
    
    // Reset last matched rule
    if (enabled && !was_enabled) {
        ir_auto_state.last_matched_rule = 0xFF;
        
        // Reset all rule timers
        for (int i = 0; i < MAX_AUTO_RULES; i++) {
            ir_auto_rules[i].last_sent_time = 0;
        }
    }
    
    k_mutex_unlock(&ir_mutex);
    
    // Log status change
    if (was_enabled && !enabled) {
        LOG_WRN("❄️ Global Auto Control: DISABLED ⛔");
    } else if (!was_enabled && enabled) {
        LOG_WRN("❄️ Global Auto Control: ENABLED ✅");
        LOG_WRN("   Active rules: %d", ir_auto_state.active_rule_count);
    }
    
    // Save to NVS
    save_ir_auto_state();
    
    // Notify
    uint8_t notify_data[3] = {
        ir_auto_state.global_enabled ? 0x01 : 0x00,
        ir_auto_state.active_rule_count,
        ir_auto_state.last_matched_rule
    };
    
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            bt_gatt_notify(active_conns[i], attr, notify_data, sizeof(notify_data));
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    return len;
}

// ============================================================================
// SETTINGS STORAGE - NVS
// ============================================================================

// Thêm vào hàm settings_set_all(), sau phần temperature settings:
/*
    // IR commands database
    if (settings_name_steq(name, "ir_cmds", &next) && !next) {
        if (len > sizeof(ir_commands)) return -EINVAL;
        
        if (read_cb(cb_arg, ir_commands, len) != len) {
            return -EINVAL;
        }
        
        ir_cmd_count = len / sizeof(struct ir_command);
        LOG_DBG("Loaded %d IR commands from NVS", ir_cmd_count);
        return 0;
    }
    
    // IR auto rules
    if (settings_name_steq(name, "ir_rules", &next) && !next) {
        if (len != sizeof(ir_auto_rules)) return -EINVAL;
        
        if (read_cb(cb_arg, ir_auto_rules, sizeof(ir_auto_rules)) != sizeof(ir_auto_rules)) {
            return -EINVAL;
        }
        
        LOG_DBG("Loaded IR auto rules from NVS");
        return 0;
    }
    
    // IR auto state
    if (settings_name_steq(name, "ir_state", &next) && !next) {
        if (len != sizeof(ir_auto_state)) return -EINVAL;
        
        if (read_cb(cb_arg, &ir_auto_state, sizeof(ir_auto_state)) != sizeof(ir_auto_state)) {
            return -EINVAL;
        }
        
        LOG_DBG("Loaded IR auto state from NVS");
        return 0;
    }
*/

static int save_ir_commands(void) {
    size_t data_len = ir_cmd_count * sizeof(struct ir_command);
    
    int rc = settings_save_one(SETTINGS_NAME "/ir_cmds", ir_commands, data_len);
    if (rc) {
        LOG_ERR("Failed to save IR commands: %d", rc);
        return rc;
    }
    
    LOG_INF("💾 Saved %d IR commands to NVS", ir_cmd_count);
    return 0;
}

static int save_ir_auto_rules(void) {
    int rc = settings_save_one(SETTINGS_NAME "/ir_rules", ir_auto_rules, sizeof(ir_auto_rules));
    if (rc) {
        LOG_ERR("Failed to save IR auto rules: %d", rc);
        return rc;
    }
    
    LOG_INF("💾 Saved IR auto rules to NVS");
    return 0;
}

static int save_ir_auto_state(void) {
    int rc = settings_save_one(SETTINGS_NAME "/ir_state", &ir_auto_state, sizeof(ir_auto_state));
    if (rc) {
        LOG_ERR("Failed to save IR auto state: %d", rc);
        return rc;
    }
    
    LOG_INF("💾 Saved IR auto state to NVS");
    return 0;
}







// ============================================================================
// Battery Level Handlers (Standard BLE Battery Service)
// ============================================================================

static ssize_t read_battery_level(struct bt_conn *conn, 
                                   const struct bt_gatt_attr *attr,
                                   void *buf, uint16_t len, uint16_t offset) {
    // ⭐ Đọc voltage và convert sang %
    const struct device *battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
    
    if (device_is_ready(battery)) {
        sensor_sample_fetch(battery);
        
        struct sensor_value voltage;
        if (sensor_channel_get(battery, SENSOR_CHAN_VOLTAGE, &voltage) == 0) {
            current_voltage_mv = (voltage.val1 * 1000) + (voltage.val2 / 1000);
            last_battery_percent = voltage_to_percent(current_voltage_mv);
            
            LOG_INF("📱 BLE Battery Level read: %d%% (%d mV)", 
                    last_battery_percent, current_voltage_mv);
        }
    }
    
    // ⭐ CRITICAL: BAS battery level format là uint8 (0-100)
    uint8_t battery_level = last_battery_percent;
    
    // Start auto-updates khi host đọc battery
    start_auto_updates();
    if (!k_work_delayable_is_pending(&update_work)) {
        k_work_reschedule(&update_work, K_MSEC(UPDATE_INTERVAL_MS));
    }
    
    return bt_gatt_attr_read(conn, attr, buf, len, offset, 
                            &battery_level, sizeof(battery_level));
}

// ⭐ Notify helper function
static void notify_battery_level(uint8_t percent) {
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            // attrs[2] sẽ là Battery Level characteristic (xem GATT definition bên dưới)
            bt_gatt_notify(active_conns[i], &bas_svc.attrs[2], 
                          &percent, sizeof(percent));
        }
    }
    k_mutex_unlock(&conn_mutex);
    
    LOG_INF("🔋 Battery Level notified: %d%%", percent);
}




static int settings_set_all(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    
    // Battery settings
    if (settings_name_steq(name, "cfg", &next) && !next) {
        if (len != 9) return -EINVAL;
        
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
        
        LOG_DBG("Loaded battery settings from NVS");  // ⭐ Đổi thành DBG
        return 0;
    }
    
    // Temperature settings
    if (settings_name_steq(name, "temp", &next) && !next) {
        if (len != 12) return -EINVAL;
        
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
        
        LOG_DBG("Loaded temperature settings from NVS");  // ⭐ Đổi thành DBG
        return 0;
    }

     // IR commands database
    if (settings_name_steq(name, "ir_cmds", &next) && !next) {
        if (len > sizeof(ir_commands)) return -EINVAL;
        
        if (read_cb(cb_arg, ir_commands, len) != len) {
            return -EINVAL;
        }
        
        ir_cmd_count = len / sizeof(struct ir_command);
        LOG_DBG("Loaded %d IR commands from NVS", ir_cmd_count);
        return 0;
    }
    
    // IR auto rules
    if (settings_name_steq(name, "ir_rules", &next) && !next) {
        if (len != sizeof(ir_auto_rules)) return -EINVAL;
        
        if (read_cb(cb_arg, ir_auto_rules, sizeof(ir_auto_rules)) != sizeof(ir_auto_rules)) {
            return -EINVAL;
        }
        
        LOG_DBG("Loaded IR auto rules from NVS");
        return 0;
    }
    
    // IR auto state
    if (settings_name_steq(name, "ir_state", &next) && !next) {
        if (len != sizeof(ir_auto_state)) return -EINVAL;
        
        if (read_cb(cb_arg, &ir_auto_state, sizeof(ir_auto_state)) != sizeof(ir_auto_state)) {
            return -EINVAL;
        }
        
        LOG_DBG("Loaded IR auto state from NVS");
        return 0;
    }
    
    // Bond aliases
    if (settings_name_steq(name, "bonds", &next) && !next) {
        if (len > sizeof(bond_list)) {
            return -EINVAL;
        }
        
        if (read_cb(cb_arg, bond_list, len) != len) {
            return -EINVAL;
        }
        
        LOG_DBG("Loaded bond aliases from NVS");  // ⭐ Đổi thành DBG
        return 0;
    }
    
    return -ENOENT;
}

// ============================================================================
// Settings Commit - Apply Loaded Settings
// ============================================================================

static int settings_apply_loaded(void) {
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    LOG_INF("⚙️  APPLYING LOADED SETTINGS FROM NVS");
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    // Battery settings
    LOG_INF("📋 Battery Auto Settings:");
    LOG_INF("   Auto ON: %s at <%d%%", 
            auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_on_percent);
    LOG_INF("   Auto OFF: %s at >%d%%", 
            auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.auto_off_percent);
    LOG_INF("   Storage: %d%%", auto_settings.storage_percent);
    LOG_INF("   Reverse OFF: %s at <%d%%", 
            auto_settings.reverse_off_enabled ? "ENABLED" : "DISABLED",
            auto_settings.reverse_off_percent);
    LOG_INF("   Reverse ON: %s at >%d%%", 
            auto_settings.reverse_on_enabled ? "ENABLED" : "DISABLED",
            auto_settings.reverse_on_percent);
    
    // Temperature settings
    LOG_INF("🌡️  Temperature Protection:");
    LOG_INF("   Internal High: %s at >%d.%02d°C", 
            temp_settings.int_high_enabled ? "ENABLED" : "DISABLED",
            temp_settings.int_high_threshold / 100,
            abs(temp_settings.int_high_threshold % 100));
    LOG_INF("   Internal Low: %s at <%d.%02d°C", 
            temp_settings.int_low_enabled ? "ENABLED" : "DISABLED",
            temp_settings.int_low_threshold / 100,
            abs(temp_settings.int_low_threshold % 100));
    LOG_INF("   External High: %s at >%d.%02d°C", 
            temp_settings.ext_high_enabled ? "ENABLED" : "DISABLED",
            temp_settings.ext_high_threshold / 100,
            abs(temp_settings.ext_high_threshold % 100));
    LOG_INF("   External Low: %s at <%d.%02d°C", 
            temp_settings.ext_low_enabled ? "ENABLED" : "DISABLED",
            temp_settings.ext_low_threshold / 100,
            abs(temp_settings.ext_low_threshold % 100));
    
    // Apply settings with current battery level
    LOG_INF("🔋 Applying auto MOSFET rules...");
    //check_auto_mosfet(last_battery_percent);
    
    LOG_INF("✅ All settings applied successfully");
    LOG_INF("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    
    return 0;
}

// Line ~340 (sau settings_commit)
SETTINGS_STATIC_HANDLER_DEFINE(
    battery_monitor,    // Handler name
    SETTINGS_NAME,      // "btmon"
    NULL,               // Init callback
    settings_set_all,   // Set callback (load from NVS)
    settings_apply_loaded,    // ⭐ Commit callback (apply after load)
    NULL                // Export callback
);

// // ============================================================================
// // Settings Management (using Zephyr Settings API like ZMK Studio)
// // ============================================================================

// static int settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
//     const char *next;
    
//     // Battery settings
//     if (settings_name_steq(name, "cfg", &next) && !next) {
//         if (len != 9) {
//             return -EINVAL;
//         }
        
//         uint8_t data[9];
//         if (read_cb(cb_arg, data, sizeof(data)) != sizeof(data)) {
//             return -EINVAL;
//         }
        
//         auto_settings.auto_on_enabled = (data[0] != 0);
//         auto_settings.auto_off_enabled = (data[1] != 0);
//         auto_settings.auto_on_percent = data[2];
//         auto_settings.auto_off_percent = data[3];
//         auto_settings.storage_percent = data[4];
//         auto_settings.reverse_off_enabled = (data[5] != 0);
//         auto_settings.reverse_off_percent = data[6];
//         auto_settings.reverse_on_enabled = (data[7] != 0);
//         auto_settings.reverse_on_percent = data[8];
        
//         LOG_INF("Battery settings loaded from NVS:");
//         LOG_INF("  Auto ON: %s at <%d%%", 
//                 auto_settings.auto_on_enabled ? "ENABLED" : "DISABLED",
//                 auto_settings.auto_on_percent);
//         LOG_INF("  Auto OFF: %s at >%d%%", 
//                 auto_settings.auto_off_enabled ? "ENABLED" : "DISABLED",
//                 auto_settings.auto_off_percent);
//         LOG_INF("  Storage: %d%%", auto_settings.storage_percent);
//         LOG_INF("  Reverse OFF: %s at <%d%%", 
//                 auto_settings.reverse_off_enabled ? "ENABLED" : "DISABLED",
//                 auto_settings.reverse_off_percent);
//         LOG_INF("  Reverse ON: %s at >%d%%", 
//                 auto_settings.reverse_on_enabled ? "ENABLED" : "DISABLED",
//                 auto_settings.reverse_on_percent);
        
//         return 0;
//     }
    
//     // Temperature settings
//     if (settings_name_steq(name, "temp", &next) && !next) {
//         if (len != 12) {
//             return -EINVAL;
//         }
        
//         uint8_t data[12];
//         if (read_cb(cb_arg, data, sizeof(data)) != sizeof(data)) {
//             return -EINVAL;
//         }
        
//         temp_settings.int_high_enabled = (data[0] != 0);
//         temp_settings.int_high_threshold = (int16_t)((data[1] << 8) | data[2]);
//         temp_settings.int_low_enabled = (data[3] != 0);
//         temp_settings.int_low_threshold = (int16_t)((data[4] << 8) | data[5]);
//         temp_settings.ext_high_enabled = (data[6] != 0);
//         temp_settings.ext_high_threshold = (int16_t)((data[7] << 8) | data[8]);
//         temp_settings.ext_low_enabled = (data[9] != 0);
//         temp_settings.ext_low_threshold = (int16_t)((data[10] << 8) | data[11]);
        
//         LOG_INF("Temperature settings loaded from NVS:");
//         LOG_INF("  Internal High: %s at >%d.%02d°C", 
//                 temp_settings.int_high_enabled ? "ENABLED" : "DISABLED",
//                 temp_settings.int_high_threshold / 100,
//                 abs(temp_settings.int_high_threshold % 100));
//         LOG_INF("  Internal Low: %s at <%d.%02d°C", 
//                 temp_settings.int_low_enabled ? "ENABLED" : "DISABLED",
//                 temp_settings.int_low_threshold / 100,
//                 abs(temp_settings.int_low_threshold % 100));
//         LOG_INF("  External High: %s at >%d.%02d°C", 
//                 temp_settings.ext_high_enabled ? "ENABLED" : "DISABLED",
//                 temp_settings.ext_high_threshold / 100,
//                 abs(temp_settings.ext_high_threshold % 100));
//         LOG_INF("  External Low: %s at <%d.%02d°C", 
//                 temp_settings.ext_low_enabled ? "ENABLED" : "DISABLED",
//                 temp_settings.ext_low_threshold / 100,
//                 abs(temp_settings.ext_low_threshold % 100));
        
//         return 0;
//     }
    
//     return -ENOENT;
// }

// SETTINGS_STATIC_HANDLER_DEFINE(battery_monitor, SETTINGS_NAME, NULL, settings_set, NULL, NULL);

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

// ============================================================================
// Battery Voltage to Percentage Conversion
// ============================================================================

// LiPo voltage curve (typical for single cell)
static uint8_t voltage_to_percent(uint16_t voltage_mv) {
    // Lookup table for more accurate conversion
    static const struct {
        uint16_t mv;
        uint8_t pct;
    } curve[] = {
        {4200, 100}, {4150, 95}, {4110, 90}, {4080, 85}, {4020, 80},
        {3980, 75},  {3950, 70}, {3910, 65}, {3870, 60}, {3850, 55},
        {3840, 50},  {3820, 45}, {3800, 40}, {3790, 35}, {3770, 30},
        {3750, 25},  {3730, 20}, {3710, 15}, {3690, 10}, {3610, 5},
        {3400, 0}
    };
    
    // Handle bounds
    if (voltage_mv >= curve[0].mv) return curve[0].pct;
    if (voltage_mv <= curve[20].mv) return curve[20].pct;
    
    // Linear interpolation between points
    for (int i = 0; i < 20; i++) {
        if (voltage_mv >= curve[i + 1].mv) {
            uint16_t v_high = curve[i].mv;
            uint16_t v_low = curve[i + 1].mv;
            uint8_t p_high = curve[i].pct;
            uint8_t p_low = curve[i + 1].pct;
            
            // Linear interpolation
            return p_low + ((voltage_mv - v_low) * (p_high - p_low)) / (v_high - v_low);
        }
    }
    
    return 0;
}





// // Force battery state update and raise event
// static void force_battery_update(void) {
//     if (current_voltage_mv == 0) return;
    
//     uint8_t new_percent = voltage_to_percent(current_voltage_mv);
    
//     if (new_percent != last_battery_percent) {
//         LOG_INF("🔋 Battery: %d%% (%d mV)", new_percent, current_voltage_mv);
//     }
//         // Method 2: Nếu method 1 không compile, dùng sensor fetch
//         const struct device *battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
//         if (device_is_ready(battery)) {
        
               
                    
//                     // ⭐ CRITICAL: Manually trigger state_of_charge calculation
//                     struct sensor_value soc;
//                     int rc = sensor_channel_get(battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &soc);
//                     if (rc == 0) {
//                         uint8_t new_percent = soc.val1;
                        
//                         if (new_percent != last_battery_percent) {
//                             LOG_INF("🔋 Battery updated: %d%% (%d mV)", 
//                                     new_percent, current_voltage_mv);
//                             }
//                         }
                
//             }        
        
        
//         // ⭐ CRITICAL: Manually raise ZMK battery event
//         struct zmk_battery_state_changed ev = {
//             .state_of_charge = new_percent            
//         };
        
//         // This will trigger battery_level_listener()
//         raise_zmk_battery_state_changed(ev);
        
//         last_battery_percent = new_percent;

//         // ⭐ Notify qua BLE Battery Service
//         notify_battery_level(new_percent);

//         check_auto_mosfet(new_percent);
// }





static void update_all_sensors(void) {
    temp_internal = read_internal_temp();
    int temp_int_int = temp_internal / 100;
    int temp_int_frac = abs(temp_internal % 100);
    LOG_INF("Internal temp: %d.%02d°C", temp_int_int, temp_int_frac);
    
    temp_external = read_ntc_temperature();
    int temp_ext_int = temp_external / 100;
    int temp_ext_frac = abs(temp_external % 100);
    LOG_INF("External temp: %d.%02d°C", temp_ext_int, temp_ext_frac);
    
    // ⭐ Đọc voltage
    read_battery_voltage();
    
    // ⭐ Convert voltage → % và notify BLE
    LOG_INF("Voltage read: %d mV", current_voltage_mv);
    if (current_voltage_mv > 0) {
        uint8_t new_percent = voltage_to_percent(current_voltage_mv);
        
        LOG_INF("🔋 Battery current: %d%% (%d mV)", new_percent, current_voltage_mv);

        LOG_INF("🔋 Battery new: %d%% (%d mV)", new_percent, current_voltage_mv);
            
            last_battery_percent = new_percent;
            
            // ⭐ Notify BLE Battery Service
            notify_battery_level(new_percent);
            
            // Auto MOSFET control
            check_auto_mosfet(new_percent);
    }
    
    // Check temperature protection
    check_temp_protection();


    // Check IR auto control based on temperature
    check_ir_auto_control();
    
    // Notify custom characteristics
    k_mutex_lock(&conn_mutex, K_FOREVER);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (active_conns[i]) {
            bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[5], 
                          &temp_internal, sizeof(temp_internal));
            bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[8], 
                          &current_voltage_mv, sizeof(current_voltage_mv));
            bt_gatt_notify(active_conns[i], &battery_monitor_svc.attrs[11], 
                          &temp_external, sizeof(temp_external));
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
    
    // ⭐ Log event từ ZMK (nếu có external trigger)
    LOG_DBG("ZMK battery event: %d%%", percent);
    
    // ⭐ Sync với custom logic (nếu ZMK update từ nguồn khác)
    if (percent != last_battery_percent) {
        last_battery_percent = percent;
        notify_battery_level(percent);
        check_auto_mosfet(percent);
    }
    
    return 0;
}

ZMK_LISTENER(battery_monitor, battery_level_listener);
ZMK_SUBSCRIPTION(battery_monitor, zmk_battery_state_changed);

// ⭐ CRITICAL: Enable event manager to dispatch to this listener
#define ZMK_EV_EVENT_TYPE(ev) zmk_battery_state_changed
#define ZMK_EV_EVENT_LIST_ITEM(ev) ZMK_LISTENER_CB(battery_monitor, ev)


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

// Thêm sau hàm read_battery_voltage() - Line ~590
// ============================================================================
// Battery Percentage
// ============================================================================




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

// ⭐ SERVICE 1: Standard Battery Service (0x180F)
// Phải define TRƯỚC để có handle thấp (host ưu tiên scan service này)
BT_GATT_SERVICE_DEFINE(bas_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_BAS),
    
    // Battery Level Characteristic (0x2A19)
    // Properties: Read + Notify (theo BLE spec)
    BT_GATT_CHARACTERISTIC(BT_UUID_BAS_BATTERY_LEVEL,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_battery_level, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);





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


     // IR Raw Data - Gửi instant IR command
    BT_GATT_CHARACTERISTIC(BT_UUID_IR_RAW_DATA,
                          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                          BT_GATT_PERM_WRITE,
                          NULL, write_ir_raw_data, NULL),
    
    // IR Status - Đọc trạng thái transmission
    BT_GATT_CHARACTERISTIC(BT_UUID_IR_STATUS,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ,
                          read_ir_status, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // IR Command Database - CRUD operations cho IR codes
    BT_GATT_CHARACTERISTIC(BT_UUID_IR_CMD_DB,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_ir_cmd_db, write_ir_cmd_db, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // IR Auto Rules - CRUD operations cho rules
    BT_GATT_CHARACTERISTIC(BT_UUID_IR_AUTO_RULES,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_ir_auto_rules, write_ir_auto_rules, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    
    // IR Auto Enable - Bật/tắt global auto control
    BT_GATT_CHARACTERISTIC(BT_UUID_IR_AUTO_ENABLE,
                          BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                          read_ir_auto_enable, write_ir_auto_enable, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    // IR Learning Control
    BT_GATT_CHARACTERISTIC(BT_UUID_IR_LEARNING,
                          BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                          BT_GATT_PERM_WRITE,
                          NULL, write_ir_learning, NULL),
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

// SETTINGS_STATIC_HANDLER_DEFINE(bond_aliases, SETTINGS_NAME, NULL, settings_set_bonds, NULL, NULL);

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

    set_power_state(true);

    k_work_init_delayable(&update_work, update_work_handler);
    k_work_init_delayable(&bootloader_work, bootloader_work_handler);


    // Initialize connection array
    memset(active_conns, 0, sizeof(active_conns));
    refresh_bond_list();
    
    // ⭐ THÊM: Đọc battery level ban đầu
    read_battery_voltage();
    if (current_voltage_mv > 0) {
        last_battery_percent = voltage_to_percent(current_voltage_mv);
        LOG_INF("🔋 Initial battery: %d%% (%d mV)", 
                last_battery_percent, current_voltage_mv);
    }

 
    // Configure IR LED
    ret = gpio_pin_configure(gpio_dev, IR_TX_PIN, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure IR LED: %d", ret);
        return ret;
    }


        // Configure IR RX với interrupt
    ret = gpio_pin_configure(gpio_dev, IR_RX_PIN, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure IR RX: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure(gpio_dev, IR_RX_PIN, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Failed to configure IR RX interrupt: %d", ret);
        return ret;
    }

    gpio_init_callback(&ir_rx_cb_data, ir_rx_interrupt, BIT(IR_RX_PIN));
    gpio_add_callback(gpio_dev, &ir_rx_cb_data);

   
    
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