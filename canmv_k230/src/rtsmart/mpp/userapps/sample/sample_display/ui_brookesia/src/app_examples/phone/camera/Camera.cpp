#include <math.h>
#include <vector>
#include "Camera.hpp"
#include "../../../../../cap_vio.h"
using namespace std;
#include <unistd.h>
#include "fpioa/rt_fpioa.h"
LV_IMG_DECLARE(img_app_camera);

#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256

lv_obj_t * btn1=NULL;
lv_obj_t * btn2=NULL;
lv_obj_t * btn3=NULL;
 void Camera::btn_clicked_event(lv_event_t * e)
{   
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        //lv_label_set_text_fmt(label, "x%d", cnt);
        if(strcmp(lv_label_get_text(label),"1")==0) //3
        {
        fpioa_set_function(7,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);//void i2c4 camera and peripheral(i2c) conflict
        fpioa_set_function(8,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);
        }
        else
        {
        fpioa_set_function(46,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);//void i2c4 camera and peripheral(i2c) conflict
        fpioa_set_function(47,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);            
        }
        camera_close();        
        if(strcmp(lv_label_get_text(label),"1")==0)
        {
            //lv_obj_set_size(btn1, 80, 80);
            //lv_obj_set_size(btn2, 60, 60);
           // lv_obj_set_size(btn3, 60, 60); 
            lv_obj_set_style_bg_opa(btn1, LV_OPA_80, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn3, LV_OPA_20, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn1, 2, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn2, 0, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn3, 0, LV_PART_MAIN);
            
            //pthread_create(&g_pthread_handle, NULL, Camera::camera_thread_preview, NULL);
              
        camera_open(GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR,3); //ratation 0:0 1:90 2:180 3:270      
        }
        else if(strcmp(lv_label_get_text(label),"2")==0)
        {
            //lv_obj_set_size(btn1, 60, 60);
            //lv_obj_set_size(btn2, 80, 80);
            //lv_obj_set_size(btn3, 60, 60);
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_80, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn3, LV_OPA_20, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn1, 0, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn2, 2, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn3, 0, LV_PART_MAIN);
        camera_open(GC2093_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR,1);//0 
             
        }
         else if(strcmp(lv_label_get_text(label),"3")==0)
        {
            //lv_obj_set_size(btn1, 60, 60);
            //lv_obj_set_size(btn2, 60, 60);
            //lv_obj_set_size(btn3, 80, 80);
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn3, LV_OPA_80, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn1, 0, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn2, 0, LV_PART_MAIN);
            //lv_obj_set_style_border_width(btn3, 2, LV_PART_MAIN);
        camera_open(GC2093_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR,1);//1
        }
        
        
        
        
        
        
        
        
    }
}

Camera::Camera(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Camera", &img_app_camera, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_camera, use_status_bar, use_navigation_bar)
    )
{
}

Camera::~Camera()
{

}
int Camera::camera_open(int _sensor_type,int _rotation)
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

    //display_width=560;
    //display_height=992;
    vo_layer_config(vo_layer,display_width,display_height,pix_format,_rotation);
    vio_start_stream(dev_num);
return 0;

}
void Camera::camera_close()
 {   
 k_vicap_dev dev_num=VICAP_DEV_ID_0;
 k_s32 chn_num=0;
 vio_stop_stream(dev_num); 
 vi_unbind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);       
 }

bool Camera::run(void)
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
     //camera_open(GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR);
     //camera_open(GC2093_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR);
    #if 1
    lv_obj_t * label;

   // lv_obj_t * 
    btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, btn_clicked_event, LV_EVENT_CLICKED, NULL);    
    lv_obj_set_pos(btn1, 135, 980);                            /*Set its position*/
    lv_obj_set_size(btn1, 100, 100);  //40                        /*Set its size*/

    label = lv_label_create(btn1);
    lv_label_set_text(label, "1");
    lv_obj_center(label);
    
    //lv_obj_t * 
    btn2 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn2, btn_clicked_event, LV_EVENT_CLICKED, NULL);
    lv_obj_set_pos(btn2, 255, 980);                            /*Set its position*/
    lv_obj_set_size(btn2, 100, 100);                          /*Set its size*/

    label = lv_label_create(btn2);
    lv_label_set_text(label, "2");
    lv_obj_center(label);
    
    //lv_obj_t * 
    btn3 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn3, btn_clicked_event, LV_EVENT_CLICKED, NULL);
    lv_obj_set_pos(btn3, 375, 980);                            /*Set its position*/
    lv_obj_set_size(btn3, 100, 100);                          /*Set its size*/

    label = lv_label_create(btn3);
    lv_label_set_text(label, "3");
    lv_obj_center(label);
    
    lv_obj_set_style_radius(btn1, LV_PCT(20), LV_PART_MAIN);
    lv_obj_set_style_radius(btn2, LV_PCT(20), LV_PART_MAIN);
    lv_obj_set_style_radius(btn3, LV_PCT(20), LV_PART_MAIN);
    
    lv_obj_set_style_bg_color(btn1, lv_color_hex(0xafafaf), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn2, lv_color_hex(0xafafaf), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn3, lv_color_hex(0xafafaf), LV_PART_MAIN);
    
    lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn3, LV_OPA_20, LV_PART_MAIN);
    
    lv_obj_set_style_border_width(btn1, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn2, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn3, 0, LV_PART_MAIN);
    
    lv_event_send(btn1, LV_EVENT_CLICKED, NULL);
    #endif
    
    /*
     lv_obj_set_style_radius(btn1, LV_RADIUS_CIRCLE, 40);
     lv_obj_set_style_radius(btn2, LV_RADIUS_CIRCLE, 40);
     lv_obj_set_style_radius(btn3, LV_RADIUS_CIRCLE, 40);
     static lv_style_t style;
     lv_style_init(&style);
     lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_GREY));
     lv_obj_add_style(btn1, &style, LV_STATE_DEFAULT);
     lv_obj_add_style(btn2, &style, LV_STATE_DEFAULT);
     lv_obj_add_style(btn3, &style, LV_STATE_DEFAULT);
     */
     
    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool Camera::back(void)
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
bool Camera::close(void)
{
 k_vicap_dev dev_num=VICAP_DEV_ID_0;
 k_s32 chn_num=0;
 vio_stop_stream(dev_num);
  printf("vi_unbind_vo before \n");
 vi_unbind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1); 
 printf("vi_unbind_vo after \n"); 
 fpioa_set_function(46,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);//void i2c4 camera and peripheral(i2c) conflict
 fpioa_set_function(47,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);  
    return true;
}

bool Camera::init(void)
{

    return true;
}


 bool Camera::pause()
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

bool Camera::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
 printf("camer resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Camera::keyboard_event_cb(lv_event_t *e)
{
}
