#include <math.h>
#include <vector>
#include "LiveCall.hpp"
#include <unistd.h>
#include "../lora/lora_lib/RadioLib.h"
#include "../../../../../cap_vio.h"
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "fpioa/rt_fpioa.h"
#include <rtthread.h>
//#include <sched.h>
#include <dirent.h>
#include <fnmatch.h>
#include <unistd.h>
#include <stdio.h>
#include "../../../../../rt_cache.h"
static pkg_cache_t g_video_cache;
/* ioctl */
#define	GPIO_DM_OUTPUT           _IOW('G', 0, int)
#define	GPIO_DM_INPUT            _IOW('G', 1, int)
#define	GPIO_DM_INPUT_PULL_UP    _IOW('G', 2, int)
#define	GPIO_DM_INPUT_PULL_DOWN  _IOW('G', 3, int)
#define	GPIO_WRITE_LOW           _IOW('G', 4, int)
#define	GPIO_WRITE_HIGH          _IOW('G', 5, int)

#define	GPIO_PE_RISING           _IOW('G', 7, int)
#define	GPIO_PE_FALLING          _IOW('G', 8, int)
#define	GPIO_PE_BOTH             _IOW('G', 9, int)
#define	GPIO_PE_HIGH             _IOW('G', 10, int)
#define	GPIO_PE_LOW              _IOW('G', 11, int)
#define RADIO_SCLK_PIN              15
#define RADIO_MISO_PIN              17
#define RADIO_MOSI_PIN              16
#define RADIO_CS_PIN                14


//#define RADIO_DIO0_PIN  RADIOLIB_NC
#define RADIO_RST_PIN   5
//#define RADIO_DIO1_PIN  RADIOLIB_NC


//#define USING_LR2021
#if defined(USING_LR2021)
#define LR20XX_SYSTEM_IRQ_RX_DONE                ( 1 << 18 )  //!< Packet received
#define LR20XX_SYSTEM_IRQ_TX_DONE                ( 1 << 19 )  //!< Packet sent


#define RADIO_DIO1_PIN              RADIOLIB_NC
#define RADIO_BUSY_PIN              RADIOLIB_NC
//#define RADIO_BUSY_PIN              19
static LR2021 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif


#define USING_LR2021_FLRC
#if defined(USING_LR2021_FLRC)
#define LR20XX_SYSTEM_IRQ_RX_DONE                ( 1 << 18 )  //!< Packet received
#define LR20XX_SYSTEM_IRQ_TX_DONE                ( 1 << 19 )  //!< Packet sent


#define RADIO_DIO1_PIN              RADIOLIB_NC
#define RADIO_BUSY_PIN              RADIOLIB_NC
//#define RADIO_BUSY_PIN              19
static LR2021 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

//#define USING_LR2021_FSK
#if defined(USING_LR2021_FSK)
#define LR20XX_SYSTEM_IRQ_RX_DONE                ( 1 << 18 )  //!< Packet received
#define LR20XX_SYSTEM_IRQ_TX_DONE                ( 1 << 19 )  //!< Packet sent


#define RADIO_DIO1_PIN              RADIOLIB_NC
#define RADIO_BUSY_PIN              RADIOLIB_NC
//#define RADIO_BUSY_PIN              19
static LR2021 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif

void unPackData(k_u8*src_data,int len)
{


}
static void clean_video_temp_files(const char* dir_path) {
    DIR *dir;
    struct dirent *entry;
    if ((dir = opendir(dir_path)) == NULL) {
        perror("opendir failed");
        return;
    }

    while ((entry = readdir(dir)) != NULL) {
        if (fnmatch("live_*", entry->d_name, 0) == 0) {
            char file_path[256];
            snprintf(file_path, sizeof(file_path), "%s/%s", dir_path, entry->d_name); 
            if (unlink(file_path) == 0) {
                printf("Deleted: %s\n", file_path);
            } else {
                perror("unlink failed");
            }
        }
    }
    closedir(dir);
}
static int camera_rotation=3;//csi2:370
extern int h265_file_packet_index;
static int last_remain=0;
static char packet[229];
static volatile  int flag_venc_write=1;
static volatile  int flag_venc_read=1;
static rt_thread_t lora_send_tid;
static int is_save_video_file=0;
static volatile int  flag_end_task_lora_send=0;
static void task_lora_send(void *arg)
{    flag_end_task_lora_send=0;
    printf("task_lora_send sched......\n");
   char send_data[229]={0};//lora max:256 flrc max :512 fsk max:65535
   int file_packet_index=0;
   char h265_file[50];
   int num_counter =0;
   bool is_first=true;
   char h265_file_lora_send[50];
   memset(h265_file_lora_send,0,50);
   sprintf(h265_file_lora_send,"/sdcard/liveLora_send.h265");//abc.h265
   FILE *send_file_handler=fopen(h265_file_lora_send, "wb");
   rt_uint8_t video_data[PKG_SIZE];
   k_u32 data_send_size=0;

   k_u32 blk_size=225;//225
    while (flag_venc_read)
    {
  #if defined(USING_LR2021_FLRC) || defined(USING_LR2021_FSK)
  num_counter++;
  //sprintf(send_data, "qwe789#%d", num_counter);
  int state;
     memset(send_data,0,blk_size+4);
     //sprintf(send_data,"%d",num_counter);
     memcpy(send_data,&num_counter,sizeof(int));   
     cache_read(&g_video_cache, video_data);
     memcpy(send_data+4,video_data,225);
     state=radio.startTransmit((uint8_t*)send_data, blk_size+4);
     if(is_save_video_file==1)
     {
     fwrite(send_data+4, 1, blk_size, send_file_handler);
     fflush(send_file_handler); 
     } 
     data_send_size+=blk_size;
 if (state == RADIOLIB_ERR_NONE) {
 
    int i=0;
    uint32_t irq_state;
    while(i<10)
    {   
    i++;
     usleep(10);//2000
    irq_state=radio.get_clear_IrqStatus();
     if(( irq_state & RADIOLIB_LR20xx_IRQ_TX_DONE ) == RADIOLIB_LR20xx_IRQ_TX_DONE)
     {

     break;
     }
     
    }
  
  if(( irq_state & RADIOLIB_LR20xx_IRQ_TX_DONE ) == RADIOLIB_LR20xx_IRQ_TX_DONE)//tx_done
  {
  printf("data_send_size:%d\n",data_send_size);
  }
  else
  {
  printf("[Radio] send packet......!\n");
  }
  }

  #endif
      //usleep(1000);//500000:2rx  200
      rt_thread_mdelay(1);
    }

    
    
     if(send_file_handler)
    { 
    fflush(send_file_handler);
    printf("send_file_handler fclose\n");
    fclose(send_file_handler); 
    send_file_handler=NULL;
    }
    flag_end_task_lora_send=1;
     
}











extern int flag_vdec_done;
//static pthread_t lora_recv_pthread;
static rt_thread_t lora_recv_tid;
//static FILE *to_recv_file_handler=NULL;
static volatile int  flag_end_task_lora_recv=0;
static void task_lora_recv(void *arg)
{
flag_end_task_lora_recv=0;
uint8_t recv_data[229];
char msg_transfer[300];
int state = radio.startReceive();
 char h265_file[50];
 memset(h265_file,0,50);
 sprintf(h265_file,"/sdcard/liveLora_recv.h265");
 FILE *to_recv_file_handler=fopen(h265_file, "wb");
 k_u32 data_recv_size=0;
 k_u32 blk_size=225;//225
 int packet_no_last =999999;
 int packet_no=0;
 uint32_t irq_state;
 int packet_len=blk_size+4;
while(flag_vdec_done==0)
{
    irq_state=radio.get_clear_IrqStatus();
  if(( irq_state & RADIOLIB_LR20xx_IRQ_RX_DONE ) == RADIOLIB_LR20xx_IRQ_RX_DONE)

  { 

    memset(recv_data,0,blk_size+4);
    int state = radio.readData(recv_data, packet_len);//blk_size+4
     packet_no=(recv_data[0]&0xFF) + (recv_data[1] << 8) + (recv_data[2] << 16) + (recv_data[3] << 24);     

     printf("recv packet_no:%d\n",packet_no);
     packet_no=abs(packet_no);
     if(packet_no-packet_no_last>1)
     { 

         if(packet_no-packet_no_last<5)

         {

               printf("packet_no<%d-%d> is lose\n",packet_no_last,packet_no);

               memset(msg_transfer,0,blk_size);

     		for(int i=packet_no_last+1;i<packet_no;i++)
               {
                if(is_save_video_file==1)
     		{
                fwrite(msg_transfer, 1, blk_size, to_recv_file_handler);
                fflush(to_recv_file_handler);
                }
                data_recv_size+=blk_size;
                }
         }
         else//luanma
         {
         printf("packet_%02X,%02X,%02X,%02X\n",recv_data[0]&0xFF,recv_data[1],recv_data[2],recv_data[3]);
         printf("packet_no<%d> is too big\n",packet_no);
         packet_no=packet_no_last+1;
         }
     }     
     packet_no_last=packet_no;
        if (state == RADIOLIB_ERR_NONE) {            
                 int try_cnt=0; 
                while(cache_write(&g_video_cache, recv_data+4)<0)
                      {
                      try_cnt++;
                      if(try_cnt>3)
                      {                
                      break;
                      }
                      usleep(100);
                      //rt_thread_mdelay(1);
                      //printf("cache_write recv  waiting<%d>\n",try_cnt);
                      }         
              if(is_save_video_file==1)
     		{
                    fwrite(recv_data+4, 1, packet_len-4, to_recv_file_handler);//blk_size
                    fflush(to_recv_file_handler);
                    //data_recv_size+=(packet_len-4);//blk_size
                }
                if(flag_vdec_done)
                {
                //fclose(to_recv_file_handler);
                printf("++++++++++++++++++++++++recv data finished\n");
                break;
                }
                
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            printf("[Radio] CRC error!\n");
            //printf("data_recv_size:%d\n",data_recv_size);
              int w_bytes=fwrite(recv_data+4, 1, blk_size, to_recv_file_handler);
              printf("w_bytes:%d\n",w_bytes);
                fflush(to_recv_file_handler);
                data_recv_size+=blk_size;                              
                 if(flag_vdec_done)
                {
                //fclose(to_recv_file_handler);
                printf("++++++++++++++++++++++++recv data finished\n");
                break;
                }                                        
               

        } else {
            // some other error occurred
            printf("[Radio] Failed, code:%d\n",state);
            //printf("data_recv_size:%d\n",data_recv_size);
              int w_bytes=fwrite(recv_data+4, 1, blk_size, to_recv_file_handler);
              printf("w_bytes:%d\n",w_bytes);
                fflush(to_recv_file_handler);
                data_recv_size+=blk_size;
                if(flag_vdec_done)
                {
                //fclose(to_recv_file_handler);
               // to_recv_file_handler=NULL;
                printf("++++++++++++++++++++++++recv data finished\n");
                break;
                } 
               
        }
//radio.finishTransmit(); //single rx use
}

else
{
usleep(10);
}

}

if(to_recv_file_handler)
    { 
    fflush(to_recv_file_handler);
    printf("recv_file fclose...............\n");
    fclose(to_recv_file_handler); 
    to_recv_file_handler=NULL;
    }
    flag_end_task_lora_recv=1;   
    //return arg;
}

static rt_thread_t task_venc_tid;
extern FILE *venc_file_handler;


static void task_venc_write(void *arg)
{
   printf("task_venc_write sched......\n");
    while (flag_venc_write)
    { 
      venc_write_data_to_cache(0,&g_video_cache,0);     
      rt_thread_mdelay(1);//40  5
    } 
    
    if(venc_file_handler)
    { 
    fflush(venc_file_handler);
    printf("task_venc_write fclose...............\n");
    fclose(venc_file_handler); 
    venc_file_handler=NULL;
    }
}





int LiveCall::camera_venc_open(int _sensor_type,int _rotation)
{
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_s32 chn_num=0;
    //k_u32 pool_id;
    
    k_vicap_sensor_type sensor_type=k_vicap_sensor_type(_sensor_type);//GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
    vi_sensor_init(sensor_type);
    
    //vio_vb_config(&pool_id);
    k_u32 display_width=992;//992
    k_u32 display_height=560;//560
    k_pixel_format pix_format=PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    vi_chn_config(dev_num,chn_num,display_width,display_height,pix_format);
    vi_chn_config(dev_num,VICAP_CHN_ID_1,1280,720,PIXEL_FORMAT_BGR_888_PLANAR);//add chn for model
    
    vi_bind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);
   
    venc_chn_config(chn_num,1920, 1080);
    vi_bind_venc(dev_num, chn_num,dev_num, chn_num);  
     
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;

    //display_width=560;//560
    //display_height=992;//992
    vo_layer_config(vo_layer,display_width,display_height,pix_format,_rotation);
    vio_start_stream(dev_num);
return 0;

}

int LiveCall::camera_venc_close()
   {
         k_vicap_dev dev_num=VICAP_DEV_ID_0;
 k_s32 chn_num=0;
 vio_stop_stream(dev_num);
 venc_stop_stream(chn_num);
 vi_unbind_venc(dev_num, chn_num, dev_num, chn_num); 
 vi_unbind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1); 
   }
  
  static rt_thread_t task_vdec_tid;
  static volatile int flag_end_task_vdec_output=0;
  static void task_vdec_output(void *arg)
{
    flag_end_task_vdec_output=0;
    rt_uint8_t video_data[PKG_SIZE];
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    k_pixel_format pix_format=PIXEL_FORMAT_YVU_PLANAR_420;

    k_u32 display_width=992;//560
    k_u32 display_height=560;//992
    k_u32 pool_id=vb_create_pool(6,1920*1088);
    vo_layer_config(vo_layer,display_width,display_height,pix_format,camera_rotation);
    while (flag_vdec_done==0)
    {   
       cache_read(&g_video_cache, video_data);
       
       vdec_process_data(0,pool_id,video_data,PKG_SIZE,flag_vdec_done);//blk_size
      if(vdec_output(0))
       {
       printf("task_vdec_output exit.....\n");
       break;
       }
     usleep(10);
     //rt_thread_mdelay(1);  
    }
    flag_end_task_vdec_output=1;   
}
    
int LiveCall::vdec_open()
{
int ch = 0;
vdec_chn_config(ch,560,992);//560 992
vdec_bind_vo(0, 0, 0);
return 0;

} 
int LiveCall::vdec_close()
{
 k_s32 chn_num=0;
 vdec_unbind_vo(0,0,0);
 vdec_stop_stream(chn_num);
}   

using namespace std;

LV_IMG_DECLARE(img_app_live_call);

#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256
static lv_obj_t * btn1=NULL;
static lv_obj_t * btn2=NULL;
static int flag_record=0;
static int flag_record_play=0;
void LiveCall::btn_clicked_event(lv_event_t * e)
{

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) { 
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        static uint32_t user_data = 35; 
        if(strcmp(lv_label_get_text(label),"Send")==0)
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
    	    printf("start rt_thread_create .....\n");
            flag_venc_write=1;
            flag_venc_read=1;
            cache_init(&g_video_cache);
            //test task_venc_write pthread to rtthread
              if(task_venc_tid!=RT_NULL)
             {
        task_venc_tid=RT_NULL;
             }
               if(task_venc_tid == RT_NULL)
             {
        task_venc_tid= rt_thread_create("venc_write",         // 线程名
                           task_venc_write,    // 线程入口函数
                           RT_NULL,        // 传递给线程入口函数的参数
                           1024,           // 栈大小
                           RT_THREAD_PRIORITY_MAX/2+1, // 线程优先级
                           20);            // 时间片长度（单位tick）
            }
        rt_thread_startup(task_venc_tid); // 启动线程
   
            //pthread_create(&venc_pthread, NULL, task_venc_write, NULL);
            //pthread_create(&lora_send_pthread, NULL, task_lora_send, NULL); 
             if(lora_send_tid!=RT_NULL)
    {
        lora_send_tid=RT_NULL;
    }
               if(lora_send_tid == RT_NULL)
    {
    lora_send_tid= rt_thread_create("lora_send",         // 线程名
                           task_lora_send,    // 线程入口函数
                           RT_NULL,        // 传递给线程入口函数的参数
                           1024,           // 栈大小
                           RT_THREAD_PRIORITY_MAX/2, // 线程优先级
                           20);            // 时间片长度（单位tick）
   }
        rt_thread_startup(lora_send_tid); // 启动线程 
               
            flag_record=1;            
            }
            else
            {        
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
           lv_obj_set_style_bg_color(btn1, lv_color_hex(0xafafaf), LV_PART_MAIN);
           
           flag_venc_read=0;
           //rt_thread_mdelay(100);  // 等待任务完成当前周期  read
           while(flag_end_task_lora_send==0)
           {
                rt_thread_mdelay(100);  // 等待任务完成当前周期  read
                printf("waiting flag_end_task_lora_send==1\n"); 
           }
            flag_venc_write=0;
            //rt_thread_mdelay(100);  // 等待任务完成当前周期 write
            while(venc_file_handler!=NULL)
            {
               rt_thread_mdelay(100);  // 等待任务完成当前周期 write
               printf("waiting venc_file_handler==NULL\n"); 
            }                                
            cache_deinit(&g_video_cache);
            camera_venc_close();                    
            flag_record=0;
            }
           
           
           
         }
         else if(strcmp(lv_label_get_text(label),"Receive")==0)
         {
           if(flag_record==1)
          {
          lv_event_send(btn1, LV_EVENT_CLICKED, NULL);
          }
            if(flag_record_play==0)
            {
            cache_init(&g_video_cache);
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_80, LV_PART_MAIN);
            vdec_open();
    	    printf("start rt_thread_create .....\n");
    	    flag_vdec_done=0;
    //test task_vdec_output pthread to rtthread
              if(task_vdec_tid!=RT_NULL)
    {
        task_vdec_tid=RT_NULL;
    }
               if(task_vdec_tid == RT_NULL)
        {
        task_vdec_tid= rt_thread_create("vdec_output",         // 线程名
                           task_vdec_output,    // 线程入口函数
                           RT_NULL,        // 传递给线程入口函数的参数

                           1024*2,           // 栈大小
                           RT_THREAD_PRIORITY_MAX/2+1, // 线程优先级
                           20);            // 时间片长度（单位tick）
       }
        rt_thread_startup(task_vdec_tid); // 启动线程	    
    	          
              if(lora_recv_tid!=RT_NULL)
    {
       
        lora_recv_tid=RT_NULL;
    }           
             // 创建一个线程
    if(lora_recv_tid == RT_NULL)
    {
    
    lora_recv_tid= rt_thread_create("lora_recv",         // 线程名
                           task_lora_recv,    // 线程入口函数
                           RT_NULL,        // 传递给线程入口函数的参数
                           1024*2,           // 栈大小
                           RT_THREAD_PRIORITY_MAX/2, // 线程优先级
                           20);            // 时间片长度（单位tick）
   }
        rt_thread_startup(lora_recv_tid); // 启动线程
            flag_record_play=1;
            }else
            {
            lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
            lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
            flag_vdec_done=1;                    
            while(flag_end_task_lora_recv==0)
             {
                rt_thread_mdelay(100);  // 等待任务完成当前周期  write
                printf("waiting to_recv_file_handler==NULL\n"); 
           }
            int trycnt=0;
             while(flag_end_task_vdec_output==0)
            {
            char tmp[225];
            memset(tmp,0,225);
            cache_write(&g_video_cache, tmp);//void  read block
            cache_force_recover(&g_video_cache);
            rt_thread_mdelay(100);  // 等待任务完成当前周期  read
            printf("waiting flag_end_task_vdec_output==1\n"); 
            trycnt++;
            if(trycnt>10)
            {                 
               break;//void task_vdec_output waiting for long time
            }
            }
            cache_deinit(&g_video_cache);
            
            vdec_close();                  
            flag_record_play=0;
            
            } 
                    
         }
         
            
            
            
            
            
            
    }
}








LiveCall::LiveCall(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("LiveCall", &img_app_live_call, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_live_call, use_status_bar, use_navigation_bar)
    )
{
}

LiveCall::~LiveCall()
{

}
bool LiveCall::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
    int gpio_fd = open("/dev/gpio", O_RDWR);
    if (gpio_fd>0)
    {
      pin_gpio_t pin_44;
      pin_44.pin = 44;
      ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_44);  //pin44 output
      ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_44);
      //close(gpio_fd);
        
    }
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
   
     #if 1
    lv_obj_t * label;

   // lv_obj_t * 
    btn1 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn1, btn_clicked_event, LV_EVENT_CLICKED, NULL);    
    lv_obj_set_pos(btn1, 135, 980);                            /*Set its position*/
    lv_obj_set_size(btn1, 100, 100);  //40                        /*Set its size*/

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Send");
    lv_obj_center(label);
    
    //lv_obj_t * 
    btn2 = lv_btn_create(lv_scr_act());
    lv_obj_add_event_cb(btn2, btn_clicked_event, LV_EVENT_CLICKED, NULL);
    lv_obj_set_pos(btn2, 375, 980);                            /*Set its position*/
    lv_obj_set_size(btn2, 100, 100);                          /*Set its size*/

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Receive");
    lv_obj_center(label);
    
   
    
    lv_obj_set_style_radius(btn1, LV_PCT(20), LV_PART_MAIN);
    lv_obj_set_style_radius(btn2, LV_PCT(20), LV_PART_MAIN);
    
    
    lv_obj_set_style_bg_color(btn1, lv_color_hex(0xafafaf), LV_PART_MAIN);
    lv_obj_set_style_bg_color(btn2, lv_color_hex(0xafafaf), LV_PART_MAIN);
   
    
    lv_obj_set_style_bg_opa(btn1, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(btn2, LV_OPA_20, LV_PART_MAIN);
    
    
    lv_obj_set_style_border_width(btn1, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(btn2, 0, LV_PART_MAIN);
    #endif
    
  
#if defined(USING_LR2021_FLRC)
 float freq=2450;//915
 float bw=500;//125
 uint8_t sf=7;//10 
 uint8_t cr=7; //8  5
 uint8_t syncWord=0x12; //0x12
 int8_t power=12;//22
 uint16_t preambleLength=8;//12
 float tcxovoltage=3.0;//1.6,2.7,0:normal 32m ,other:tcxo
 radio.beginFLRC(freq,650,5,power,preambleLength,tcxovoltage,0);
#endif
  
#if defined(USING_LR2021_FSK)
   float freq=2400;//915 
   float br=50;
   float freqDev=25;
   float rxBw=111;
   int8_t power=12;//22
   uint16_t preambleLength=8;//12
   float tcxovoltage=0;//1.6,2.7,0:normal 32m ,other:tcxo
  
  radio.beginGFSK(freq,br,freqDev,rxBw,power,preambleLength,tcxovoltage);
#endif  
  
     clean_video_temp_files("/sdcard"); 
    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool LiveCall::back(void)
{
  printf("livecall back before \n");
 lv_disp_get_default()->driver->screen_transp = 0;
    /* 这里设置屏幕背景是透明的 */
 lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_COVER);
 notifyCoreClosed();
 printf("livecall back after \n");
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool LiveCall::close(void)
{
  if(flag_record==1)
   {
   lv_event_send(btn1, LV_EVENT_CLICKED, NULL);
   }
   if(flag_record_play==1)
   {
   lv_event_send(btn2, LV_EVENT_CLICKED, NULL);
   }

   
    return true;
}

bool LiveCall::init(void)
{
   
    return true;
}


 bool LiveCall::pause()
{
    
      printf("livecall pause before \n");
 lv_disp_get_default()->driver->screen_transp = 0;
    /* 这里设置屏幕背景是透明的 */
 lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_COVER);
 notifyCoreClosed();
 printf("livecall pause after \n");

     return true;
}

bool LiveCall::resume()
 {
     printf("livecall resume \n");
     /* Do some operations here if needed */

     return true;
 }

void LiveCall::keyboard_event_cb(lv_event_t *e)
{
}
