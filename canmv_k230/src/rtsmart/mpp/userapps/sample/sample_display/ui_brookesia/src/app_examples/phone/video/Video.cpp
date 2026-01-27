#include <math.h>
#include <vector>
#include "Video.hpp"
#include "../../../../../cap_vio.h"
#include <unistd.h>
using namespace std;
LV_IMG_DECLARE(img_app_video);
#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256
pthread_t venc_pthread;
int flag_venc_write=1;
static int camera_rotation=3;//csi2:370
extern FILE *venc_file_handler;
static void *task_venc_write(void *arg)
{
   char venc_file[50];
   memset(venc_file,0,50);
   sprintf(venc_file,"/sdcard/record_play.h265");
   printf("task_venc_write sched......\n");
    while (flag_venc_write)
    {
      venc_write_data_to_file(0,venc_file);
      usleep(30000);
    } 
    if(venc_file_handler)
    { 
    printf("fclose...............\n");
    fclose(venc_file_handler); 
    venc_file_handler=NULL;
    }
    return arg;
}

int Video::camera_venc_open(int _sensor_type,int _rotation)
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
   
    venc_chn_config(chn_num,1920, 1080);
    vi_bind_venc(dev_num, chn_num,dev_num, chn_num);  
     
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;

    //display_width=560;
    //display_height=992;
    vo_layer_config(vo_layer,display_width,display_height,pix_format,_rotation);
    vio_start_stream(dev_num);
return 0;

}

int Video::camera_venc_close()
   {
         k_vicap_dev dev_num=VICAP_DEV_ID_0;
 k_s32 chn_num=0;
 vio_stop_stream(dev_num);
 venc_stop_stream(chn_num);
 vi_unbind_venc(dev_num, chn_num, dev_num, chn_num); 
 vi_unbind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1); 
   }
  
  
  
pthread_t vdec_input_pthread;
pthread_t vdec_output_pthread;
extern int flag_vdec_done;
extern FILE *vdec_file_handler;
static void *task_vdec_read(void *arg)
{
   flag_vdec_done=0;
   char vdec_file[50];
   memset(vdec_file,0,50);
   sprintf(vdec_file,"/sdcard/record_play.h265");
   printf("task_vdec_read sched......\n");
   vdec_read_data(0,vdec_file);
    if(vdec_file_handler)
    { 
    printf("fclose...............\n");
    fclose(vdec_file_handler); 
    vdec_file_handler=NULL;
    }
    return arg;
}
  
  static void *task_vdec_output(void *arg)
{


    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    k_pixel_format pix_format=PIXEL_FORMAT_YVU_PLANAR_420;

    //k_u32 display_width=560;
    //k_u32 display_height=992;
    
    k_u32 display_width=992;
    k_u32 display_height=560;
    vo_layer_config(vo_layer,display_width,display_height,pix_format,camera_rotation);
    while (1)
    {
    
       if(flag_vdec_done==1)
      {
        break;
      }else if(vdec_output(0))
       {
       flag_vdec_done=1;
       break;
       }
     usleep(10000);
    }
    
    
}
 
  
  
  
  
  
  
  
  
   
  int Video::vdec_open()
{
int ch = 0;
vdec_chn_config(ch,560,992);
vdec_bind_vo(0, 0, 0);

//












 /*   k_vicap_dev dev_num=VICAP_DEV_ID_0;
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
   
    venc_chn_config(chn_num,1920, 1080);
    vi_bind_venc(dev_num, chn_num,dev_num, chn_num);  
     
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;

    display_width=560;
    display_height=992;
    vo_layer_config(vo_layer,display_width,display_height,pix_format);
    vio_start_stream(dev_num);*/
return 0;

} 
int Video::vdec_close()
{
 k_s32 chn_num=0;
 vdec_unbind_vo(0,0,0);
 vdec_stop_stream(chn_num);
}   
   
   
   
   
   
   
   
   
   
   
   
   
 static lv_obj_t * btn1=NULL;
 static lv_obj_t * btn2=NULL;
 int flag_record=0;
 int flag_record_play=0;
 void Video::btn_clicked_event(lv_event_t * e)
{ 
    
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        //lv_label_set_text_fmt(label, "x%d", cnt);
        //camera_close();        
        if(strcmp(lv_label_get_text(label),"Record")==0)
        {
          if(flag_record_play==1)
          {
          lv_event_send(btn2, LV_EVENT_CLICKED, NULL);
          }
            if(flag_record==0)
            {
            lv_obj_set_style_bg_opa(btn1, LV_OPA_80, LV_PART_MAIN);  
            lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);   
           lv_obj_set_style_bg_color(btn1, lv_color_hex(0xff0000), LV_PART_MAIN);
            camera_venc_open(GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR,camera_rotation);
    	    printf("start pthread_create .....\n");
            flag_venc_write=1;
            pthread_create(&venc_pthread, NULL, task_venc_write, NULL);
            flag_record=1;
            
            }
            else
            {
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
           lv_obj_set_style_bg_color(btn1, lv_color_hex(0xafafaf), LV_PART_MAIN);
            flag_venc_write=0;
            camera_venc_close();
            pthread_cancel(venc_pthread);
            pthread_join(venc_pthread, NULL);
            venc_pthread=NULL;
            flag_record=0;
            }
                          
        }
        else if(strcmp(lv_label_get_text(label),"Play")==0)
        {
         if(flag_record==1)
          {
          lv_event_send(btn1, LV_EVENT_CLICKED, NULL);
          }
            if(flag_record_play==0)
            {
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_80, LV_PART_MAIN);
            vdec_open();
    	    printf("start pthread_create .....\n");
            pthread_create(&vdec_input_pthread, NULL, task_vdec_read, NULL);
            pthread_create(&vdec_output_pthread, NULL, task_vdec_output, NULL);
            flag_record_play=1;
            }else
            {
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
            vdec_close();
            flag_vdec_done=1;           
            pthread_cancel(vdec_input_pthread);
            pthread_join(vdec_input_pthread, NULL);
            pthread_cancel(vdec_output_pthread);
            pthread_join(vdec_output_pthread, NULL);                    
            flag_record_play=0;
            }
            
            
            
                    
        }
    
    }
} 
   
Video::Video(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Video", &img_app_video, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_video, use_status_bar, use_navigation_bar)
    )
{
}

Video::~Video()
{

}
bool Video::run(void)
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
    //camera_venc_open(GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR);
    //usleep(5000000);
    //flag_venc_write=1;
    //pthread_create(&venc_pthread, NULL, task_venc_write, NULL);
        
     #if 1
    lv_obj_t * label;

   // lv_obj_t * 
    btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, btn_clicked_event, LV_EVENT_CLICKED, NULL);    
    lv_obj_set_pos(btn1, 135, 980);                            /*Set its position*/
    lv_obj_set_size(btn1, 100, 100);  //40                        /*Set its size*/

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Record");
    lv_obj_center(label);
    
    //lv_obj_t * 
    btn2 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn2, btn_clicked_event, LV_EVENT_CLICKED, NULL);
    lv_obj_set_pos(btn2, 375, 980);                            /*Set its position*/
    lv_obj_set_size(btn2, 100, 100);                          /*Set its size*/

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Play");
    lv_obj_center(label);
    
   
    
    lv_obj_set_style_radius(btn1, LV_PCT(20), LV_PART_MAIN);
    lv_obj_set_style_radius(btn2, LV_PCT(20), LV_PART_MAIN);
    
    
    lv_obj_set_style_bg_color(btn1, lv_color_hex(0xafafaf), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn2, lv_color_hex(0xafafaf), LV_PART_MAIN);
   
    
    lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
    
    
    lv_obj_set_style_border_width(btn1, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn2, 0, LV_PART_MAIN);
    
    
    //lv_event_send(btn1, LV_EVENT_CLICKED, NULL);
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
bool Video::back(void)
{
  //notifyCoreClosed(); 
  printf("video back before \n");
 lv_disp_get_default()->driver->screen_transp = 0;
    /* 这里设置屏幕背景是透明的 */
 lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_COVER);
 notifyCoreClosed();
 printf("video back after \n");
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Video::close(void)
{  
   if(flag_record==1)
   {
   lv_event_send(btn1, LV_EVENT_CLICKED, NULL);
   }
   if(flag_record_play==1)
   {
   lv_event_send(btn2, LV_EVENT_CLICKED, NULL);
   }


   /*flag_venc_write=0;
      camera_venc_close();
      pthread_cancel(venc_pthread);
      pthread_join(venc_pthread, NULL);*/
      //kd_mpi_venc_close_fd();
    //flag_running=0;
    //pthread_join(pthread_handle_uart_rx, NULL);
    //pthread_mutex_destroy(&mutex_uart_rx);
    return true;
}

bool Video::init(void)
{

    return true;
}


 bool Video::pause()
{
 printf("video pause before \n");
 lv_disp_get_default()->driver->screen_transp = 0;
    /* 这里设置屏幕背景是透明的 */
 lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_COVER);
 notifyCoreClosed();
 printf("video pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Video::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("video resume \n");
     /* Do some operations here if needed */

     return true;
 }

