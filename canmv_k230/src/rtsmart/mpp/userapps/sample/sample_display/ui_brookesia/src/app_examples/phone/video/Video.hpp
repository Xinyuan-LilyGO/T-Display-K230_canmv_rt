#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class Video: public ESP_Brookesia_PhoneApp
{
public:
	Video(bool use_status_bar = true, bool use_navigation_bar = false);
	~Video();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;

    bool pause();
    bool resume();
    static int camera_venc_open(int _sensor_type,int _rotation);
    static int camera_venc_close();
    static int vdec_open();
    static int vdec_close();
    static void btn_clicked_event(lv_event_t * e);
    uint16_t _height;
    uint16_t _width;

private:
    static void keyboard_event_cb(lv_event_t *e);
};
