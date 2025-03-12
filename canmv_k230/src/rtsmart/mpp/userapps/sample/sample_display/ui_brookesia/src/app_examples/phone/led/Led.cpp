#include <math.h>
#include <vector>
#include "Led.hpp"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
using namespace std;

LV_IMG_DECLARE(img_app_led);

#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256
typedef union
{
    uint32_t value;
    struct
    {
        uint8_t b;
        uint8_t r;
        uint8_t g;
        uint8_t none;
    };
} ws2812_value;



int fd_ws2812;
static lv_obj_t * create_slider(lv_color_t color);
static void slider_event_cb(lv_event_t * e);

static lv_obj_t * red_slider, * green_slider, * blue_slider, * intense_slider;
static lv_obj_t * img_led;

static void slider_event_cb(lv_event_t * e)
{
    LV_UNUSED(e);

    /*Recolor the image based on the sliders' values*/
    lv_color_t color  = lv_color_make(lv_slider_get_value(red_slider), lv_slider_get_value(green_slider),
                                      lv_slider_get_value(blue_slider));
    //lv_opa_t intense = lv_slider_get_value(intense_slider);
    lv_obj_set_style_img_recolor_opa(img_led, 255, 0);
    lv_obj_set_style_img_recolor(img_led, color, 0);
    
     ws2812_value led_rgb;
     led_rgb.b=lv_slider_get_value(blue_slider);
     led_rgb.r=lv_slider_get_value(red_slider);
     led_rgb.g=lv_slider_get_value(green_slider);
     //led_rgb.value=led_rgb;
     ioctl(fd_ws2812,0x00,&led_rgb);
    
    
    
}

static lv_obj_t * create_slider(lv_color_t color)
{
    lv_obj_t * slider = lv_slider_create(lv_scr_act());
    lv_slider_set_range(slider, 0, 255);
    lv_obj_set_size(slider, 20, 300);
    lv_obj_set_style_bg_color(slider, color, LV_PART_KNOB);
    lv_obj_set_style_bg_color(slider, lv_color_darken(color, LV_OPA_40), LV_PART_INDICATOR);
    lv_obj_add_event_cb(slider, slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    return slider;
}


Led::Led(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Led", &img_app_led, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_led, use_status_bar, use_navigation_bar)
    )
{
}

Led::~Led()
{

}

bool Led::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
   
    fd_ws2812 = open("/dev/ws2812", O_RDWR);
    if (fd_ws2812 < 0) {
        perror("open /dev/ws2812");
        return false;
    }
    red_slider = create_slider(lv_palette_main(LV_PALETTE_RED));
    green_slider = create_slider(lv_palette_main(LV_PALETTE_GREEN));
    blue_slider = create_slider(lv_palette_main(LV_PALETTE_BLUE));
    //intense_slider = create_slider(lv_palette_main(LV_PALETTE_GREY));

    lv_slider_set_value(red_slider, LV_OPA_0, LV_ANIM_OFF);
    lv_slider_set_value(green_slider, LV_OPA_0, LV_ANIM_OFF);
    lv_slider_set_value(blue_slider, LV_OPA_0, LV_ANIM_OFF);
    //lv_slider_set_value(intense_slider, LV_OPA_50, LV_ANIM_OFF);

    lv_obj_align(red_slider, LV_ALIGN_LEFT_MID, 80, 0);
    lv_obj_align_to(green_slider, red_slider, LV_ALIGN_OUT_RIGHT_MID, 80, 0);
    lv_obj_align_to(blue_slider, green_slider, LV_ALIGN_OUT_RIGHT_MID, 80, 0);
    //lv_obj_align_to(intense_slider, blue_slider, LV_ALIGN_OUT_RIGHT_MID, 50, 0);

    /*Now create the actual image*/
    LV_IMG_DECLARE(img_cogwheel_argb)
    img_led = lv_img_create(lv_scr_act());
    lv_img_set_src(img_led, &img_cogwheel_argb);
    lv_obj_align(img_led, LV_ALIGN_RIGHT_MID, -40, 0);

    lv_event_send(intense_slider, LV_EVENT_VALUE_CHANGED, NULL);
  
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
bool Led::back(void)
{
 notifyCoreClosed();
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Led::close(void)
{
    ws2812_value led_rgb;
    led_rgb.b=0;
    led_rgb.r=0;
    led_rgb.g=0;
    ioctl(fd_ws2812,0x00,&led_rgb);
    //close(fd_ws2812);
    return true;
}

bool Led::init(void)
{

    return true;
}


 bool Led::pause()
{

 printf("led pause before \n");
 
 notifyCoreClosed();
 printf("led pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Led::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
 printf("led resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Led::keyboard_event_cb(lv_event_t *e)
{
}
