#pragma once

#include "lvgl.h"
#include "esp_brookesia.hpp"

class Lora: public ESP_Brookesia_PhoneApp
{
public:
	Lora(bool use_status_bar = true, bool use_navigation_bar = false);
	~Lora();

    bool run(void);
    bool back(void);
    bool close(void);

    bool init(void) override;

    bool pause();
    bool resume();

    
    uint16_t _height;
    uint16_t _width;
    
    
 static void custom_timer_cb_lora_send(lv_timer_t * timer);
 static void custom_timer_cb_lora_receive(lv_timer_t * timer);  
 static void btn_clicked_event(lv_event_t * e); 
 static void roller_changed_event(lv_event_t * e);    
private:

     //lv_timer_t * timer_lora_send;
     //lv_timer_t * timer_lora_receive;
       //lv_obj_t * label_lora_rx;

    static void keyboard_event_cb(lv_event_t *e);
};
