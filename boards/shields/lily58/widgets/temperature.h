// File: config/widgets/temperature.h
// Header file cho temperature widget

#pragma once

#include <lvgl.h>

struct zmk_widget_temperature {
    lv_obj_t *obj;
};

int zmk_widget_temperature_init(struct zmk_widget_temperature *widget, lv_obj_t *parent);
