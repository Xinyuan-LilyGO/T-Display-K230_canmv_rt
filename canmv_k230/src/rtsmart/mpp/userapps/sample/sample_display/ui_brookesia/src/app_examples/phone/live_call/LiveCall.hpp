#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class LiveCall: public ESP_Brookesia_PhoneApp
{
public:
	LiveCall(bool use_status_bar = true, bool use_navigation_bar = false);
	~LiveCall();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;
    static int camera_venc_open(int _sensor_type,int _rotation);
    static int camera_venc_close();
    static int vdec_open();
    static int vdec_close();
    bool pause();
    bool resume();
    uint16_t _height;
    uint16_t _width;
 static void btn_clicked_event(lv_event_t * e);   
private:
    static void keyboard_event_cb(lv_event_t *e);
};
