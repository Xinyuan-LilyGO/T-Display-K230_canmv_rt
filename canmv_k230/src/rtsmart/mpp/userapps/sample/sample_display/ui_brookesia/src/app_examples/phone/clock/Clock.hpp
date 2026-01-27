#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class Clock: public ESP_Brookesia_PhoneApp
{
public:
	Clock(bool use_status_bar = true, bool use_navigation_bar = false);
	~Clock();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;

    bool pause();
    bool resume();
    static void hwtimer_handler_timeout(int sig);
    static void btn_clicked_event(lv_event_t * e);
    uint16_t _height;
    uint16_t _width;

private:
    static void keyboard_event_cb(lv_event_t *e);
};
