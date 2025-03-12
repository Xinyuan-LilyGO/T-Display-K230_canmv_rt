#include <math.h>
#include <vector>
#include "Hdmi.hpp"
#include "../../../../../cap_vio.h"
//#include "face_detect/face_detect.h"
using namespace std;

LV_IMG_DECLARE(img_app_hdmi);

#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256

Hdmi::Hdmi(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Hdmi", &img_app_hdmi, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_hdmi, use_status_bar, use_navigation_bar)
    )
{
}

Hdmi::~Hdmi()
{

}

bool Hdmi::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
    #if 0
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_s32 chn_num=0;
    k_connector_type connector_type=LT9611_MIPI_4LAN_1920X1080_60FPS;
    vo_display_init(connector_type);
    k_u32 pool_id;
    vio_vb_config(&pool_id);
    k_u32 *pic_addr;
    k_video_frame_info vf_info;
    k_vo_osd osd_vo;
    osd_vo=K_VO_OSD3;
    osd_info osd_msg;
    k_u32 display_width=1920;
    k_u32 display_height=1080;
    k_pixel_format pix_format=PIXEL_FORMAT_ARGB_8888;
    vo_osd_config(osd_vo,&osd_msg,display_width,display_height,pix_format);
    memset(&vf_info, 0, sizeof(vf_info));
    vo_config_manage(pool_id,&vf_info,&pic_addr,osd_msg); 
    printf("disp_init,pic_addr is %p \n", pic_addr);   
    
    vo_area sc_area;
    sc_area.x1=0;
    sc_area.x2=400;
    sc_area.y1=0;
    sc_area.y2=400;
    uint32_t * px_map_color =(uint32_t*)0xffff00ff;
    vo_draw_color(osd_vo,&vf_info,&pic_addr,sc_area,px_map_color);
    
    
    
   /* k_vicap_sensor_type sensor_type=GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
    vi_sensor_init(sensor_type);
    k_u32 display_width=1080;
    k_u32 display_height=1920;
    k_pixel_format pix_format=PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    vi_chn_config(dev_num,chn_num,display_width,display_height,pix_format);
    vi_bind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;
    display_width=1920;
    display_height=1080;
    vo_layer_config(vo_layer,display_width,display_height,pix_format);
    vio_start_stream(dev_num);*/
    #endif
    
    
    
       
   #if 0 
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_s32 chn_num=0;
    //k_u32 pool_id;
    k_vicap_sensor_type sensor_type=GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
    vi_sensor_init(sensor_type);
    
    //vio_vb_config(&pool_id);
    k_u32 display_width=992;
    k_u32 display_height=560;
    k_pixel_format pix_format=PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    vi_chn_config(dev_num,chn_num,display_width,display_height,pix_format);
    vi_bind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;
    display_width=560;
    display_height=992;
    vo_layer_config(vo_layer,display_width,display_height,pix_format);
    vio_start_stream(dev_num);
   #endif  
    
     #if 0 
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_s32 chn_num=0;
    //k_u32 pool_id;
    k_vicap_sensor_type sensor_type=GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
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
    //face detect
    face_detect_start();
   #endif
    
    
    
    
#if 1
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
bool Hdmi::back(void)
{
 printf("hdmi back before \n");
 //lv_disp_get_default()->driver->screen_transp = 0;
    /* 这里设置屏幕背景是透明的 */
 //lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_COVER);
 notifyCoreClosed();
 printf("hdmi back after \n");
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Hdmi::close(void)
{
 //face_detect_stop();
 /*k_vicap_dev dev_num=VICAP_DEV_ID_0;
 k_s32 chn_num=0;
 vio_stop_stream(dev_num);
  printf("vi_unbind_vo before \n");
 vi_unbind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1); 
 printf("vi_unbind_vo after \n"); */   
    return true;
}

bool Hdmi::init(void)
{

    return true;
}


 bool Hdmi::pause()
{

 printf("hdmi pause before \n");
 notifyCoreClosed();
 printf("hdmi pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Hdmi::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
 printf("hdmi resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Hdmi::keyboard_event_cb(lv_event_t *e)
{
}
