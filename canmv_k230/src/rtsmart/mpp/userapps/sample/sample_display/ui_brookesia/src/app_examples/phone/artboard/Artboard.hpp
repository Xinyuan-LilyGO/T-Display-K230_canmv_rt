#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class Artboard: public ESP_Brookesia_PhoneApp
{
public:
	Artboard(bool use_status_bar = false, bool use_navigation_bar = false);
	~Artboard();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;

    bool pause();
    bool resume();

    
    uint16_t _height;
    uint16_t _width;

private:
    static void keyboard_event_cb(lv_event_t *e);
};
