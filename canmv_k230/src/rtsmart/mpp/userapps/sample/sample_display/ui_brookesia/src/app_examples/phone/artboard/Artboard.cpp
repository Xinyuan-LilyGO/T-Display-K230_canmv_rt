#include <math.h>
#include <vector>
#include "Artboard.hpp"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "sketchpad/lv_100ask_sketchpad.h"
using namespace std;
#define CANVAS_WIDTH  560
#define CANVAS_HEIGHT 1100
LV_IMG_DECLARE(img_app_artboard);




#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256

Artboard::Artboard(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Artboard", &img_app_artboard, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_artboard, use_status_bar, use_navigation_bar)
    )
{
}

Artboard::~Artboard()
{

}

static void event_handler(lv_event_t * e)
{
    lv_obj_t * sketchpad = lv_event_get_target(e);

    //void * canvas_buf = lv_canvas_get_buf(sketchpad);
    //lv_draw_buf_t * canvas_draw_buf = lv_canvas_get_draw_buf(sketchpad);

    LV_LOG_USER("LV_EVENT_VALUE_CHANGED");

}



bool Artboard::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
    printf("area_point1:(%d,%d)-area_point2:(%d,%d),width:%d,height:%d\n",area.x1,area.y1,area.x2,area.y2,_width,_height);
    //LV_DRAW_BUF_DEFINE(draw_buf, SKETCHPAD_DEFAULT_WIDTH, SKETCHPAD_DEFAULT_HEIGHT, LV_COLOR_FORMAT_ARGB8888);
    static lv_color_t draw_buf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT)];
    
	lv_obj_t * sketchpad = lv_100ask_sketchpad_create(lv_scr_act());

	//lv_canvas_set_draw_buf(sketchpad, &draw_buf);
	lv_canvas_set_buffer(sketchpad, draw_buf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_center(sketchpad);
    lv_canvas_fill_bg(sketchpad, lv_palette_lighten(LV_PALETTE_GREY, 3), LV_OPA_COVER);

    lv_obj_add_event_cb(sketchpad, event_handler, LV_EVENT_VALUE_CHANGED, NULL);
 
 
 
 
   
#if 0
 /*Create a buffer for the canvas*/
    static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT)];

    /*Create a canvas and initialize its palette*/
    lv_obj_t * canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
     lv_obj_center(canvas);
    lv_canvas_fill_bg(canvas, lv_palette_lighten(LV_PALETTE_GREY, 3), LV_OPA_COVER);

    //lv_canvas_draw_rect(canvas, 70, 60, 100, 70, &rect_dsc);



    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_palette_main(LV_PALETTE_ORANGE);
    
    lv_canvas_draw_text(canvas, 40, 20, 100, &label_dsc, "Some text on text canvas");

#endif




  
#if 0
    lv_obj_t *label = lv_label_create(lv_scr_act());
    ESP_BROOKESIA_CHECK_NULL_RETURN(label, false, "Create label failed");
    lv_label_set_text(label, "Top Middle");
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);

    label = lv_label_create(lv_scr_act());
    ESP_BROOKESIA_CHECK_NULL_RETURN(label, false, "Create label failed");
    lv_label_set_text(label, "Bottom Middle");
    lv_obj_align(label, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);

    label = lv_label_create(lv_scr_act());
    ESP_BROOKESIA_CHECK_NULL_RETURN(label, false, "Create label failed");
    lv_label_set_text(label, "Left Middle");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);

    label = lv_label_create(lv_scr_act());
    ESP_BROOKESIA_CHECK_NULL_RETURN(label, false, "Create label failed");
    lv_label_set_text(label, "Right Middle");
    lv_obj_align(label, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);

    label = lv_label_create(lv_scr_act());
    ESP_BROOKESIA_CHECK_NULL_RETURN(label, false, "Create label failed");
    lv_label_set_text(label, "Simple Conf");
    lv_obj_center(label);
    lv_obj_set_style_text_font(label, LV_FONT_DEFAULT, 0);
#endif
    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool Artboard::back(void)
{
 notifyCoreClosed();
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Artboard::close(void)
{
//lv_obj_del(obj);
    return true;
}

bool Artboard::init(void)
{

    return true;
}


 bool Artboard::pause()
{

 printf("artboard pause before \n");
 
 //notifyCoreClosed();
 printf("artboard pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Artboard::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("artboard resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Artboard::keyboard_event_cb(lv_event_t *e)
{
}
