#define DT_DRV_COMPAT zmk_behavior_uptime_reset

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

// Import hàm reset từ peripheral_status.c
extern int uptime_reset(void);

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    LOG_INF("Uptime reset triggered!");
    int rc = uptime_reset();
    if (rc == 0) {
        LOG_INF("Uptime reset successful");
    } else {
        LOG_ERR("Uptime reset failed: %d", rc);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_uptime_reset_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, APPLICATION,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                        &behavior_uptime_reset_driver_api);