#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class Camera: public ESP_Brookesia_PhoneApp
{
public:
	Camera(bool use_status_bar = true, bool use_navigation_bar = false);
	~Camera();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;

    bool pause();
    bool resume();

    uint16_t _height;
    uint16_t _width;
static void btn_clicked_event(lv_event_t * e);

  static  int camera_open(int sensor_type,int _rotation);
   static void camera_close();


private:
    static void keyboard_event_cb(lv_event_t *e);
};
