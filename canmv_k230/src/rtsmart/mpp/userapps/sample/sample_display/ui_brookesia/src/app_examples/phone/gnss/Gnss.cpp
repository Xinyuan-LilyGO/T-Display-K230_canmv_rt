#include <math.h>
#include <vector>
#include "Gnss.hpp"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>


#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
//#include <rtthread.h>
//#include <rtdevice.h>
#include <poll.h>
#include "../../../../../fpioa/rt_fpioa.h"
using namespace std;

LV_IMG_DECLARE(img_app_gnss);
#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256
#define BUF_SIZE    (1024)
#define UART_DEVICE_NAME    "/dev/uart2"
static char buf_rx[BUF_SIZE];
static int fd_uart=0;
#define	IOC_SET_BAUDRATE            _IOW('U', 0x40, int)
struct uart_configure
{
    uint32_t baud_rate;

    uint32_t data_bits               :4;
    uint32_t stop_bits               :2;
    uint32_t parity                  :2;
    uint32_t fifo_lenth              :2;
    uint32_t auto_flow               :1;
    uint32_t reserved                :21;
};

typedef enum _uart_parity
{
    UART_PARITY_NONE,
    UART_PARITY_ODD,
    UART_PARITY_EVEN
} uart_parity_t;

typedef enum _uart_receive_trigger
{
    UART_RECEIVE_FIFO_1,
    UART_RECEIVE_FIFO_8,
    UART_RECEIVE_FIFO_16,
    UART_RECEIVE_FIFO_30,
} uart_receive_trigger_t;
static int flag_running=1;
static pthread_t pthread_handle_uart_rx = NULL;
static pthread_cond_t cond;
static pthread_mutex_t mutex_uart_rx;
lv_obj_t * label_latitude;
lv_obj_t * label_longitude;
lv_obj_t * label_altitude;
lv_obj_t * label_satellite_speed;
lv_obj_t * label_satellite_date;
lv_obj_t * label_satellite_time;
char gnss_data[20];
lv_obj_t * btn_locate=NULL;

void *  Gnss::thread_uart_rx_entry(void *parameter)
{
    struct pollfd fds[1];
    int ret = 0, cnt = 0;  
    fds[0].fd = fd_uart;
    fds[0].events = POLLIN;
    printf(" [app] ........... poll \n");
    flag_running=1;

    while(flag_running)
    {
        //pthread_mutex_lock(&mutex_uart_rx);
        if (poll(fds, 1, -1) > 0 && fds[0].revents & POLLIN)
        {
            ret = read(fd_uart, &buf_rx[ret], BUF_SIZE-cnt);
            if (ret > 0 && ret <= BUF_SIZE)
            {
                cnt += ret;
            }
            printf("package size %d\n", ret);
            if(ret > 0)
            { 
            	int  is_end=0; 
            	string gnss_arr[7]={""};          	
                printf("[app] read %d byte\n", cnt);
                printf("uart recv msg:%s\n",buf_rx);
                printf("++++++++++++++++++++++\n");
                char *saveptr;
                char*token=strtok_r(buf_rx,"\r\n",&saveptr);
                 while (token != NULL) {
                  printf("%s\n", token);
                  if(strstr(token,"#XGPS:")&&strlen(token)>12)//12
                  { token=token+7;
                    char *saveptr1;
                    char*token1=strtok_r(token,",",&saveptr1);
                    
                    int i=0;
                    printf("************\n");
                     while (token1 != NULL) {
                        printf("%s\n", token1);
                        gnss_arr[i]=token1;
                        token1 = strtok_r(NULL, ",", &saveptr1);
                     i++;
                     }
                    printf("************\n");
                     is_end=1;
                  break;
                  }
                  token = strtok_r(NULL, "\r\n", &saveptr);
                  }
                printf("---------------------\n");
                 if(is_end==1)
                {
                //fuzhi
                lv_label_set_text(label_latitude, gnss_arr[0].c_str());
                lv_label_set_text(label_longitude, gnss_arr[1].c_str());
                memset(gnss_data,0,20);
                sprintf(gnss_data, "%s m", gnss_arr[2].c_str());
                lv_label_set_text(label_altitude, gnss_data);
                 memset(gnss_data,0,20);
                sprintf(gnss_data, "%s m/s", gnss_arr[4].c_str());
                lv_label_set_text(label_satellite_speed, gnss_data);
                if(strlen(gnss_arr[6].c_str())>20) 
                {              
                lv_label_set_text(label_satellite_date, gnss_arr[6].substr(1,10).c_str());
                lv_label_set_text(label_satellite_time, gnss_arr[6].substr(12,8).c_str());
      #if 1          
                //shun dai tong bu   time       
    int year_2d, mon, day, hour, min, sec;
    char tz_sign; // 时区符号（+/-）
    int tz_val;   // 时区数值（示例中是32）
    int parsed = sscanf(gnss_arr[6].substr(1,19).c_str(), "%d-%d-%d %d:%d:%d",
                        &year_2d, &mon, &day, &hour, &min, &sec);
    struct tm at_time={0};
    at_time.tm_year = year_2d-1900;;
    at_time.tm_mon = mon-1;      
    at_time.tm_mday = day;
    at_time.tm_hour = hour;
    at_time.tm_min = min;
    at_time.tm_sec = sec;
    at_time.tm_isdst = -1; 
   time_t at_ts = mktime(&at_time); // 转为time_t             
  struct timespec ts;
  ts.tv_sec = at_ts;    // 秒级时间戳
  ts.tv_nsec = 0;       // 纳秒（设为0即可）
  // 设置CLOCK_REALTIME（实时时钟）
  clock_settime(CLOCK_REALTIME, &ts);  
 #endif
                
                
                             
                } 
                 lv_obj_set_style_bg_color(btn_locate, lv_color_hex(0x2b7bf6), LV_PART_MAIN);//blue
                }  
                memset(buf_rx,0,BUF_SIZE);
                //printf("uart recv msg:%s\n",buf_rx);
                ret = 0;
                cnt = 0;
                //if(is_end==1)
                //{
                //break;
                //}
           
            }
            
            //pthread_mutex_unlock(&mutex_uart_rx); 
            //pthread_cond_signal(&cond);
        }
    }


    //close(fd);
    return NULL;
}




//char aht20_data_tmp[20]={0};
Gnss::Gnss(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Gnss", &img_app_gnss, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_gnss, use_status_bar, use_navigation_bar)
    )
{
}

Gnss::~Gnss()
{

}

void Gnss::btn_clicked_event(lv_event_t * e)
{
 lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
    
    lv_label_set_text(label_latitude, "");
    lv_label_set_text(label_longitude, "");
    lv_label_set_text(label_altitude, "");
    lv_label_set_text(label_satellite_speed, "");
    lv_label_set_text(label_satellite_date, "");
    lv_label_set_text(label_satellite_time, "");
    //
    lv_obj_set_style_bg_color(btn_locate, lv_color_hex(0xc4c4cf), LV_PART_MAIN);//gray
    //lv_obj_set_style_bg_color(btn1, LV_COLOR_GRAY, LV_PART_MAIN);
    //LV_PALETTE_GREEN
    //LV_PALETTE_GREEN
     //gnss set  start
    
    //set system mod gnss
    char msg_send[30]={0};
    sprintf(msg_send, "AT#XGPS=0\r\n");
    write(fd_uart, msg_send, strlen(msg_send));
    usleep(2000000);
    sprintf(msg_send, "AT+CFUN=0\r\n");
    write(fd_uart, msg_send, strlen(msg_send));
    usleep(2000000);
    sprintf(msg_send, "AT%%XSYSTEMMODE=0,0,1,0\r\n");
    write(fd_uart, msg_send, strlen(msg_send));
    usleep(2000000);
    sprintf(msg_send, "AT+CFUN=31\r\n");
    write(fd_uart, msg_send, strlen(msg_send));
    usleep(2000000);
    sprintf(msg_send, "AT#XGPS=1,0,0,0\r\n");
    write(fd_uart, msg_send, strlen(msg_send));
    usleep(2000000);

    }
}
bool Gnss::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1; 
    
    
    
    static lv_coord_t col_dsc[] = {300, 200, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {80,80,80,80,80,80,80, LV_GRID_TEMPLATE_LAST};

    /*Create a container with grid*/
    lv_obj_t * cont = lv_obj_create(lv_scr_act());
    lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    lv_obj_set_size(cont, 550, 600);
    lv_obj_center(cont);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

lv_obj_t * label = lv_label_create(cont);
        lv_label_set_text(label, "latitude:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


 label_latitude = lv_label_create(cont);
        lv_label_set_text(label_latitude, "");
        lv_obj_center(label_latitude);
lv_obj_set_grid_cell(label_latitude, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


label = lv_label_create(cont);
        lv_label_set_text(label, "longitude:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


 label_longitude = lv_label_create(cont);
        lv_label_set_text(label_longitude, "");
        lv_obj_center(label_longitude);
lv_obj_set_grid_cell(label_longitude, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);
  
  label = lv_label_create(cont);
        lv_label_set_text(label, "altitude:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);


 label_altitude = lv_label_create(cont);
        lv_label_set_text(label_altitude, "");
        lv_obj_center(label_altitude);
lv_obj_set_grid_cell(label_altitude, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);
  
    label = lv_label_create(cont);
        lv_label_set_text(label, "speed:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);


 label_satellite_speed = lv_label_create(cont);
        lv_label_set_text(label_satellite_speed, "");
        lv_obj_center(label_satellite_speed);
lv_obj_set_grid_cell(label_satellite_speed, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);
  
      label = lv_label_create(cont);
        lv_label_set_text(label, "date(utc):");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 4, 1);


 label_satellite_date = lv_label_create(cont);
        lv_label_set_text(label_satellite_date, "");
        lv_obj_center(label_satellite_date);
lv_obj_set_grid_cell(label_satellite_date, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 4, 1);
                             
                             
                             
                             
                             
        label = lv_label_create(cont);
        lv_label_set_text(label, "time(utc):");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 5, 1);


 label_satellite_time= lv_label_create(cont);
        lv_label_set_text(label_satellite_time, "");
        lv_obj_center(label_satellite_time);
lv_obj_set_grid_cell(label_satellite_time, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 5, 1);
  
  //lv_obj_t * btn_locate;
  btn_locate= lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn_locate, btn_clicked_event, LV_EVENT_CLICKED, NULL);
  lv_obj_align(btn_locate, LV_ALIGN_CENTER, -20, 360);//160
  lv_obj_set_style_radius(btn_locate, LV_PCT(20), LV_PART_MAIN);
  lv_obj_set_style_border_width(btn_locate, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn_locate, lv_color_hex(0x2b7bf6), LV_PART_MAIN);//blue
  label = lv_label_create(btn_locate);
  lv_label_set_text(label, "Locate");
  lv_obj_center(label);
  lv_obj_set_style_bg_color(btn_locate,lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
  
    fpioa_set_function(5,UART2_TXD,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(6,UART2_RXD,-1,-1,-1,-1,-1,-1,-1);
      
      
      
    printf(" [app] open uart2.....\n");
    fd_uart = open(UART_DEVICE_NAME, O_RDWR);
    if (fd_uart >0)
    {
    struct uart_configure config = {
        .baud_rate = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = UART_PARITY_NONE,
        .fifo_lenth = UART_RECEIVE_FIFO_16,
        .auto_flow = 0,
    };


    if (ioctl(fd_uart, IOC_SET_BAUDRATE, &config))
    {
        printf("uart2 ioctl failed!\n");
    }
    
 //flag = 0;
    pthread_mutex_init(&mutex_uart_rx, NULL);
    pthread_cond_init(&cond, NULL);
    pthread_create(&pthread_handle_uart_rx, NULL, thread_uart_rx_entry, NULL);     
    }//fd_uart
    
 
      
      
        
   
   
   
 /*
static uint32_t user_data = 35; 
timer_weather_monitor = lv_timer_create(custom_timer_cb_weather_monitor, 2000, &user_data);
lv_timer_set_repeat_count(timer_weather_monitor, -1); 
*/
    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool Gnss::back(void)
{
  notifyCoreClosed(); 
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Gnss::close(void)
{ 
    //flag_running=0;
    //pthread_join(pthread_handle_uart_rx, NULL);
    //pthread_mutex_destroy(&mutex_uart_rx);
    return true;
}

bool Gnss::init(void)
{

    return true;
}


 bool Gnss::pause()
{

 printf("gnss pause before \n");
 //notifyCoreClosed();
 printf("gnss pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Gnss::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("gnss resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Gnss::keyboard_event_cb(lv_event_t *e)
{
}
