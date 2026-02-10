#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class Weather: public ESP_Brookesia_PhoneApp
{
public:
	Weather(bool use_status_bar = true, bool use_navigation_bar = false);
	~Weather();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;

    bool pause();
    bool resume();
    static void custom_timer_cb_weather_monitor(lv_timer_t * timer);
    static void btn_clicked_event(lv_event_t * e);
    uint16_t _height;
    uint16_t _width;

private:
    static void keyboard_event_cb(lv_event_t *e);
};
