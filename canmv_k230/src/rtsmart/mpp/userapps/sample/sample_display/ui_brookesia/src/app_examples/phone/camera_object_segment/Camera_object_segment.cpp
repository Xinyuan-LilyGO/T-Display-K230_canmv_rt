#include <math.h>
#include <vector>
#include "Camera_object_segment.hpp"
#include "../../../../../cap_vio.h"
#include "segment_yolov8n/segment_detect.h"
using namespace std;

LV_IMG_DECLARE(img_app_segment_detect);

#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256



Camera_object_segment::Camera_object_segment(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("segment", &img_app_segment_detect, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_segment_detect, use_status_bar, use_navigation_bar)
    )
{
}

Camera_object_segment::~Camera_object_segment()
{

}
int Camera_object_segment::camera_open(int _sensor_type)
{
k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_s32 chn_num=0;
    //k_u32 pool_id;
    k_vicap_sensor_type sensor_type=k_vicap_sensor_type(_sensor_type);//GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
    vi_sensor_init(sensor_type);
    
    //vio_vb_config(&pool_id);
    k_u32 display_width=992;
    k_u32 display_height=560;
    k_pixel_format pix_format=PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    vi_chn_config(dev_num,chn_num,display_width,display_height,pix_format);
    vi_chn_config(dev_num,VICAP_CHN_ID_1,1280,720,PIXEL_FORMAT_BGR_888_PLANAR);//add chn for model
    
    vi_bind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;

    display_width=560;
    display_height=992;
    vo_layer_config(vo_layer,display_width,display_height,pix_format);
    vio_start_stream(dev_num);
return 0;

}
void Camera_object_segment::camera_close()
 {   
 k_vicap_dev dev_num=VICAP_DEV_ID_0;
 k_s32 chn_num=0;
 vio_stop_stream(dev_num); 
 vi_unbind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);       
 }

bool Camera_object_segment::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
    #if 1
    static lv_style_t style_scr_act;    
    lv_style_init(&style_scr_act);
    lv_style_set_bg_opa(&style_scr_act, LV_OPA_TRANSP);
    lv_obj_add_style(lv_scr_act(), &style_scr_act, 0);
    //lv_obj_report_style_change(&style_scr_act);
 
    lv_disp_get_default()->driver->screen_transp = 1;
    /* 这里设置屏幕背景是透明的 */
    lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_TRANSP);
   #endif
     camera_open(GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR);
    //emotion detect
    segment_detect_start();

    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool Camera_object_segment::back(void)
{
 printf("camer back before \n");
 lv_disp_get_default()->driver->screen_transp = 0;
    /* 这里设置屏幕背景是透明的 */
 lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_COVER);
 notifyCoreClosed();
 printf("camer back after \n");
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Camera_object_segment::close(void)
{
 segment_detect_stop();
 camera_close();
 /*k_vicap_dev dev_num=VICAP_DEV_ID_0;
 k_s32 chn_num=0;
 vio_stop_stream(dev_num);
  printf("vi_unbind_vo before \n");
 vi_unbind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1); 
 printf("vi_unbind_vo after \n"); */   
    return true;
}

bool Camera_object_segment::init(void)
{

    return true;
}


 bool Camera_object_segment::pause()
{

 printf("camer pause before \n");
 lv_disp_get_default()->driver->screen_transp = 0;
    /* 这里设置屏幕背景是透明的 */
 lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_COVER);
 notifyCoreClosed();
 printf("camer pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Camera_object_segment::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
 printf("camer resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Camera_object_segment::keyboard_event_cb(lv_event_t *e)
{
}
