#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <stdbool.h>
#include "lvgl/lvgl.h"
#include "port/lv_port_disp.h"
#include "port/lv_port_indev.h"
#include "lvgl/src/hal/lv_hal_tick.h"
//#include "rtdevice.h"
//#include "lvgl/demos/lv_demos.h"
#include "cap_vio.h"
#include "esp_brookesia.hpp"
/* These are built-in app examples in `esp-brookesia` library */
#include "app_examples/phone/simple_conf/src/phone_app_simple_conf.hpp"
#include "app_examples/phone/complex_conf/src/phone_app_complex_conf.hpp"
#include "app_examples/phone/squareline/src/phone_app_squareline.hpp"
#include "app_examples/phone/calculator/Calculator.hpp"
#include "app_examples/phone/game_2048/Game_2048.hpp"
#include "app_examples/phone/camera/Camera.hpp"
#include "app_examples/phone/music/Music.hpp"
#include "app_examples/phone/lora/Lora.hpp"
#include "app_examples/phone/hdmi/Hdmi.hpp"
#include "app_examples/phone/setting/Setting.hpp"
#include "app_examples/phone/led/Led.hpp"
#include "app_examples/phone/camera_face_detect/Camera_face_detect.hpp"
#include "app_examples/phone/camera_head_detect/Camera_head_detect.hpp"
#include "app_examples/phone/camera_face_emotion/Camera_face_emotion.hpp"
//#include "app_examples/phone/camera_object_segment/Camera_object_segment.hpp"
#include "app_examples/phone/camera_object_detect/Camera_object_detect.hpp"
#if 1
static void on_clock_update_timer_cb(struct _lv_timer_t *t)
{
    time_t now;
    struct tm timeinfo;
    bool is_time_pm = false;
    ESP_Brookesia_Phone *phone = (ESP_Brookesia_Phone *)t->user_data;

    time(&now);
    localtime_r(&now, &timeinfo);
    is_time_pm = (timeinfo.tm_hour >= 12);

    /* Since this callback is called from LVGL task, it is safe to operate LVGL */
    // Update clock on "Status Bar"
    ESP_BROOKESIA_CHECK_FALSE_EXIT(
        phone->getHome().getStatusBar()->setClock(timeinfo.tm_hour, timeinfo.tm_min, is_time_pm),
        "Refresh status bar failed"
    );
    //printf("driver->screen_transp:%d\n", lv_disp_get_default()->driver->screen_transp);
    //printf("disp_bg_opa:%d\n", lv_disp_get_default()->bg_opa);
}
#endif
/*
static void btn_clicked_event(lv_event_t * e)
{
    //lv_event_code_t code = lv_event_get_code(e);
    //lv_obj_t * label = lv_obj_get_child(btn, 0);
    char *btn_name=lv_event_get_user_data(e);
    printf("btn_name:%s\n",btn_name);
    
}*/
void user_gui_init()
{
 printf("user_gui_init\n");
#if 1
    lv_disp_t*disp=lv_disp_get_default();
 
 /* Create a phone object */
    ESP_Brookesia_Phone *phone = new ESP_Brookesia_Phone(disp);
    ESP_BROOKESIA_CHECK_NULL_EXIT(phone, "Create phone failed");

    /* Try using a stylesheet that corresponds to the resolution */
        ESP_Brookesia_PhoneStylesheet_t *stylesheet = new ESP_Brookesia_PhoneStylesheet_t ESP_BROOKESIA_PHONE_568_1232_DARK_STYLESHEET();
        ESP_BROOKESIA_CHECK_NULL_EXIT(stylesheet, "Create stylesheet failed");
  //      ESP_LOGI(TAG, "Using stylesheet (%s)", stylesheet->core.name);
        ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->addStylesheet(stylesheet), "Add stylesheet failed");
        ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->activateStylesheet(stylesheet), "Activate stylesheet failed");
        delete stylesheet;

   lv_indev_t * indev_touch= lv_indev_get_next(NULL);
    /* Configure and begin the phone */
    ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->setTouchDevice(indev_touch), "Set touch device failed");
    //phone->registerLvLockCallback((ESP_Brookesia_LvLockCallback_t)(bsp_display_lock), 0);
    //phone->registerLvUnlockCallback((ESP_Brookesia_LvUnlockCallback_t)(bsp_display_unlock));
    ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->begin(), "Begin failed");
    // ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->getCoreHome().showContainerBorder(), "Show container border failed");

    /* Install apps */
    PhoneAppSimpleConf *app_simple_conf = new PhoneAppSimpleConf();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_simple_conf, "Create app simple conf failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_simple_conf) >= 0), "Install app simple conf failed");
    PhoneAppComplexConf *app_complex_conf = new PhoneAppComplexConf();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_complex_conf, "Create app complex conf failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_complex_conf) >= 0), "Install app complex conf failed");
    PhoneAppSquareline *app_squareline = new PhoneAppSquareline();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_squareline, "Create app squareline failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_squareline) >= 0), "Install app squareline failed");
    Calculator *app_calculator = new Calculator();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_calculator, "Create app calculator failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_calculator) >= 0), "Install app calculator failed");
    Game2048 *app_game2048 = new Game2048();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_game2048, "Create app game2048 failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_game2048) >= 0), "Install app game2048 failed");
    Camera *app_camera = new Camera();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera, "Create app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera) >= 0), "Install app camera failed");

    Music *app_music = new Music();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_music, "Create app music failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_music) >= 0), "Install app music failed");
    
    Lora *app_lora = new Lora();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_lora, "Create app lora failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_lora) >= 0), "Install app lora failed");

    /*Hdmi *app_hdmi = new Hdmi();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_hdmi, "Create app hdmi failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_hdmi) >= 0), "Install app hdmi failed");
    */
    AppSettings *app_setting = new AppSettings();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_setting, "Create app setting failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_setting) >= 0), "Install app setting failed");
    
    Led *app_led = new Led();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_led, "Create app led failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_led) >= 0), "Install app led failed");

    Camera_face_detect *app_camera_face_detect = new Camera_face_detect();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_face_detect, "camera_face_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_face_detect) >= 0), "Install app camera_face_detect failed");

    /*Camera_head_detect *app_camera_head_detect = new Camera_head_detect();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_head_detect, "camera_head_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_head_detect) >= 0), "Install app camera_head_detect failed");*/
    
    Camera_face_emotion *app_camera_emotiom_detect = new Camera_face_emotion();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_emotiom_detect, "camera_emotion_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_emotiom_detect) >= 0), "Install app camera_emotion_detect failed");

    /*Camera_object_segment *app_camera_segment_detect = new Camera_object_segment();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_segment_detect, "camera_segment_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_segment_detect) >= 0), "Install app camera_segment_detect failed");*///memory limit
    
    
    /*Camera_object_detect *app_camera_object_detect = new Camera_object_detect();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_object_detect, "camera_object_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_object_detect) >= 0), "Install app camera_object_detect failed");*/




    /* Create a timer to update the clock */
    ESP_BROOKESIA_CHECK_NULL_EXIT(lv_timer_create(on_clock_update_timer_cb, 1000, phone), "Create clock update timer failed");
#endif











#if 0

  lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    //lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/
    lv_obj_center(label);
#endif
#if 0
    lv_obj_t * cont_row = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cont_row, 500, 150);
    lv_obj_align(cont_row, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);

   
    char str_btn_arr[5][20] = {"Human\nPosture", "Lora", "Music","Face\nDetect","Object\nDetect"};
    uint32_t i;
    for(i = 0; i < 5; i++) {
        lv_obj_t * obj;
        lv_obj_t * label;

        /*Add items to the row*/
        obj = lv_button_create(cont_row);
        lv_obj_set_size(obj, 100, LV_PCT(100));

        label = lv_label_create(obj);
        lv_label_set_text(label, str_btn_arr[i]);
        lv_obj_center(label); 
        lv_obj_add_event_cb(obj, btn_clicked_event, LV_EVENT_CLICKED, str_btn_arr[i]);   
    }
    //show img desktop
#endif  
   
#if 0
static lv_img_dsc_t my_img_dsc = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 60,
    .data_size = 80 * 60 * LV_COLOR_DEPTH / 8,
    .header.cf = LV_IMG_CF_TRUE_COLOR,          /*Set the color format*/
    .data = png_data,
};
#endif
//lv_obj_t * icon_desktop = lv_img_create(lv_scr_act());

/*From variable*/
//lv_img_set_src(icon, &my_icon_dsc);

/*From file*/
//lv_img_set_src(icon_desktop, "S:/sdcard/desktop.png");
//lv_obj_center(icon_desktop); 
   
   #if 0 
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_s32 chn_num=0;
    //k_u32 pool_id;
    k_vicap_sensor_type sensor_type=GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
    vi_sensor_init(sensor_type);
    
    //vio_vb_config(&pool_id);
    k_u32 display_width=560;
    k_u32 display_height=320;
    k_pixel_format pix_format=PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    vi_chn_config(dev_num,chn_num,display_width,display_height,pix_format);
    vi_bind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;
    vo_layer_config(vo_layer,display_width,display_height,pix_format);
    vio_start_stream(dev_num);
   #endif
   //lv_demos_show_help();
   //char* info[]={"music"};
   //lv_demos_create(info, 1);
   //char* info[]={(char*)"widgets"};
   //lv_demos_create(info, 1);
  
    
    printf("user_gui_init after\n");
    
    
    



}
int main(int argc, char **argv)
{
    printf("hello world ,lvgl version:%d.%d.%d\n",lv_version_major(),lv_version_minor(),lv_version_patch());
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();

    #if 0
    lv_style_t style_scr_act;    
    lv_style_init(&style_scr_act);
    lv_style_set_bg_opa(&style_scr_act, LV_OPA_TRANSP);
    lv_obj_add_style(lv_scr_act(), &style_scr_act, 0);
    //lv_obj_report_style_change(&style_scr_act);
 
    lv_disp_get_default()->driver->screen_transp = 1;
    /* 这里设置屏幕背景是透明的 */
    lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_TRANSP);
   #endif

//rt_device_t tmr_dev_0 = rt_device_find("hwtimer0");



    user_gui_init();








    
    
    #if 0
     /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    #endif
    #if 0
    
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Button");
    lv_obj_center(label);

    lv_obj_t * btn2 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_height(btn2, LV_SIZE_CONTENT);

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Toggle");
    lv_obj_center(label);
    
    
    
    
    #endif





while(1)
    {
    //printf("\nlv_timer_handler before\n");
    lv_tick_inc(1);
    //lv_timer_handler();
    lv_task_handler();
    //printf("\nlv_timer_handler after\n");
    usleep(500);
    }
	
    return 0;
}



















