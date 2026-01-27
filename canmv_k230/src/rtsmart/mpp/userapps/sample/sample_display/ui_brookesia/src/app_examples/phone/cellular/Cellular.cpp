#include <math.h>
#include <vector>
#include "Cellular.hpp"
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

LV_IMG_DECLARE(img_app_cellular);
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
static int is_received=0;
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
lv_obj_t * label_iccid;
lv_obj_t * label_imsi;
lv_obj_t * label_imei;
lv_obj_t * label_modem_fw;
//char cellular_data[20];
string cellular_arr[4]={""};
#if 0
void *  Cellular::thread_uart_rx_entry(void *parameter)
{
    int fd;
    struct pollfd fds[1];
    int ret = 0, cnt = 0; 

    printf(" [app] open uart2.....\n");
    fd = open(UART_DEVICE_NAME, O_RDWR);
    if (fd < 0)
    {
        printf("open dev uart2 failed!\n");
        return NULL;
    }
    fd_uart=fd;
    struct uart_configure config = {
        .baud_rate = 115200,
        .data_bits = 8,
        .stop_bits = 1,
        .parity = UART_PARITY_NONE,
        .fifo_lenth = UART_RECEIVE_FIFO_16,
        .auto_flow = 0,
    };


    if (ioctl(fd, IOC_SET_BAUDRATE, &config))
    {
        printf("uart2 ioctl failed!\n");
    }

    fds[0].fd = fd;
    fds[0].events = POLLIN;
    printf(" [app] ........... poll \n");
    flag_running=1;
    //msg_curr_index=0;
    //msg_curr_pos=0;
    memset(buf_rx,0,BUF_SIZE);
    buf_rx[0]=1;
    while(flag_running)
    {
        pthread_mutex_lock(&mutex_uart_rx);
        if (poll(fds, 1, -1) > 0 && fds[0].revents & POLLIN)
        {
            ret = read(fd, &buf_rx[ret], BUF_SIZE-cnt);
            if (ret > 0 && ret <= BUF_SIZE)
            {
                cnt += ret;
            }
            printf("package size %d\n", ret);
            if(ret > 0)
            {  
            	//
                printf("[app] read %d byte\n", cnt);
                printf("uart recv msg:%s\n",buf_rx);
                for (int i = 0; i < cnt; i++) {
        // 打印十六进制值 + 可见字符（不可见字符显示为.）
        printf("第%2d字节: 0x%02x | %c\n", 
               i, 
               buf_rx[i], 
               (buf_rx[i] >= 0x20 && buf_rx[i] <= 0x7E)?buf_rx[i]:'.');
    }
                
                
                
                 if(buf_rx[0]==0x00)
                {
                buf_rx[0]='.';
                }
                printf("++++++++++++++++++++++\n");
                char *saveptr;
                char*token=strtok_r(buf_rx,"\r\n",&saveptr);
                 while (token != NULL) {
                  printf("%s\n",token);
                  if(strstr(token,"XICCID:")&&strlen(token)>12)//12
                  { 
                    char *saveptr1;
                    char*token1=strtok_r(token,":",&saveptr1);
                    
                    int i=0;
                    printf("************\n");
                     while (token1 != NULL) {
                        printf("%s\n", token1);
                        if(i==1)
                        {
                        cellular_arr[0]=token1;
                        }
                        token1 = strtok_r(NULL, ":", &saveptr1);
                     i++;
                     }
                    printf("************\n");
                  is_received=1;
                  //break;
                  }
                  else if(strstr(token,"mfw"))
                  { 
                    
                    printf("************\n");
                    cellular_arr[3]=token;
                    printf("************\n");
                    is_received=1;
                  //break;
                  }
                  else if(strlen(token)==15)
                  { 
                    
                    printf("************\n");
                    if(cellular_arr[2]=="")//IMEI:no card have IMEI
                    {
                    cellular_arr[2]=token;
                    }
                    else
                    {
                    cellular_arr[1]=token;//no card no IMSI
                    }
                    
                    printf("************\n");
                    is_received=1;
                 // break;
                  }
                  
                  
                  
                  
                  
                  token = strtok_r(NULL, "\r\n", &saveptr);
                  }
                printf("---------------------\n");
                //fuzhi
                lv_label_set_text(label_iccid, cellular_arr[0].c_str());
                lv_label_set_text(label_imsi, cellular_arr[1].c_str());
                lv_label_set_text(label_imei, cellular_arr[2].c_str());
                lv_label_set_text(label_modem_fw, cellular_arr[3].c_str());
                memset(buf_rx,0,BUF_SIZE);
                //printf("uart recv msg:%s\n",buf_rx);
                ret = 0;
                cnt = 0;
            
            }
            
            pthread_mutex_unlock(&mutex_uart_rx); 
            pthread_cond_signal(&cond);
        }
    }


    //close(fd);
    return NULL;
}
#endif
Cellular::Cellular(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Cellular", &img_app_cellular, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_cellular, use_status_bar, use_navigation_bar)
    )
{
}

Cellular::~Cellular()
{

}

void Cellular::btn_clicked_event(lv_event_t * e)
{
 lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
    

    }
}
bool Cellular::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1; 
    
    
    
    static lv_coord_t col_dsc[] = {300, 200, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {80,80,80,80,80,LV_GRID_TEMPLATE_LAST};

    /*Create a container with grid*/
    lv_obj_t * cont = lv_obj_create(lv_scr_act());
    lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    lv_obj_set_size(cont, 550, 480);
    lv_obj_center(cont);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

lv_obj_t * label = lv_label_create(cont);
        lv_label_set_text(label, "ICCID:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


 label_iccid = lv_label_create(cont);
        lv_label_set_text(label_iccid, "");
        lv_obj_center(label_iccid);
lv_obj_set_grid_cell(label_iccid, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


label = lv_label_create(cont);
        lv_label_set_text(label, "IMSI:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


 label_imsi = lv_label_create(cont);
        lv_label_set_text(label_imsi, "");
        lv_obj_center(label_imsi);
lv_obj_set_grid_cell(label_imsi, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);
  
  label = lv_label_create(cont);
        lv_label_set_text(label, "IMEI:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);


 label_imei = lv_label_create(cont);
        lv_label_set_text(label_imei, "");
        lv_obj_center(label_imei);
lv_obj_set_grid_cell(label_imei, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);
  
    label = lv_label_create(cont);
        lv_label_set_text(label, "modem_fw:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);


 label_modem_fw = lv_label_create(cont);
        lv_label_set_text(label_modem_fw, "");
        lv_obj_center(label_modem_fw);
lv_obj_set_grid_cell(label_modem_fw, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);
  
    fpioa_set_function(5,UART2_TXD,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(6,UART2_RXD,-1,-1,-1,-1,-1,-1,-1);
        
      fd_uart = open(UART_DEVICE_NAME, O_RDWR);
    if (fd_uart>0)
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
    
   
    char msg_send[25]={0};
    sprintf(msg_send, "AT%%XICCID\r\n");//ICCID
    write(fd_uart, msg_send, strlen(msg_send));
    int try_cnt=0;
    int ret = 0, cnt = 0; 
    while(try_cnt<10)
    {  usleep(100000);
     ret = read(fd_uart, &buf_rx[ret], BUF_SIZE-cnt);
            if (ret > 0 && ret <= BUF_SIZE)

            {

                cnt += ret;
            }
            printf("package size %d\n", ret);
            if(ret > 0)
            { 
            int  is_end=0;
                 printf("[app] read %d byte\n", cnt);
                printf("uart recv msg:%s\n",buf_rx);           
                 if(buf_rx[0]==0x00)
                {
                buf_rx[0]='.';
                }
                printf("++++++++++++++++++++++\n");
                char *saveptr;
                char*token=strtok_r(buf_rx,"\r\n",&saveptr);
                 while (token != NULL) {
                  printf("%s\n",token);

                  if(strstr(token,"XICCID:")&&strlen(token)>12)//12
                  { 
                    char *saveptr1;
                    char*token1=strtok_r(token,":",&saveptr1);
                    
                    int i=0;
                    printf("************\n");
                     while (token1 != NULL) {
                        printf("%s\n", token1);
                        if(i==1)
                        {
                        cellular_arr[0]=token1;
                        }
                        token1 = strtok_r(NULL, ":", &saveptr1);
                     i++;
                     }
                    printf("************\n");
                 is_end=1;
                  break;
                  }
                   token = strtok_r(NULL, "\r\n", &saveptr);
                  }
                printf("---------------------\n");
                memset(buf_rx,0,BUF_SIZE);
                ret = 0;
                cnt = 0;
                if(is_end==1)
                {
                printf("try_cnt:%d\n",try_cnt);
                break;
                }
            } 
      try_cnt++;
      
    }
    
//////////////////////    
   sprintf(msg_send, "AT+CIMI\r\n");//IMSI
    write(fd_uart, msg_send, strlen(msg_send)); 
    try_cnt=0;
    ret = 0;
    cnt = 0; 
    while(try_cnt<10)
    {  usleep(100000);
     ret = read(fd_uart, &buf_rx[ret], BUF_SIZE-cnt);
            if (ret > 0 && ret <= BUF_SIZE)
            {
                cnt += ret;
            }
            printf("package size %d\n", ret);
            if(ret > 0)
            { 
            int  is_end=0;
                 printf("[app] read %d byte\n", cnt);
                printf("uart recv msg:%s\n",buf_rx);           
                 if(buf_rx[0]==0x00)
                {
                buf_rx[0]='.';
                }
                printf("++++++++++++++++++++++\n");
                char *saveptr;
                char*token=strtok_r(buf_rx,"\r\n",&saveptr);
                 while (token != NULL) {
                  printf("%s\n",token);
                   if(strlen(token)==15)
                  {                     
                    printf("************\n");                   
                    cellular_arr[1]=token;                                 
                    printf("************\n");
                     is_end=1;
                  break;
                  }
                   token = strtok_r(NULL, "\r\n", &saveptr);
                  }
                printf("---------------------\n");
                memset(buf_rx,0,BUF_SIZE);
                ret = 0;
                cnt = 0;
                if(is_end==1)
                {
                printf("try_cnt:%d\n",try_cnt);
                break;
                }
            } 
      try_cnt++;
      
    }
 //////////////////////////////////////////////
    sprintf(msg_send, "AT+CGSN\r\n");//IMEI
    write(fd_uart, msg_send, strlen(msg_send)); 
    try_cnt=0;
    ret = 0;
    cnt = 0; 
    while(try_cnt<10)
    {  usleep(100000);
     ret = read(fd_uart, &buf_rx[ret], BUF_SIZE-cnt);
            if (ret > 0 && ret <= BUF_SIZE)
            {
                cnt += ret;
            }
            printf("package size %d\n", ret);
            if(ret > 0)
            { 
            int  is_end=0;
                 printf("[app] read %d byte\n", cnt);
                printf("uart recv msg:%s\n",buf_rx);           
                 if(buf_rx[0]==0x00)
                {
                buf_rx[0]='.';
                }
                printf("++++++++++++++++++++++\n");
                char *saveptr;
                char*token=strtok_r(buf_rx,"\r\n",&saveptr);
                 while (token != NULL) {
                  printf("%s\n",token);
                  if(strlen(token)==15)
                  {                     
                    printf("************\n");                   
                    cellular_arr[2]=token;                                 
                    printf("************\n");
                     is_end=1;
                  break;
                  }
                   token = strtok_r(NULL, "\r\n", &saveptr);
                  }
                printf("---------------------\n");
                memset(buf_rx,0,BUF_SIZE);
                ret = 0;
                cnt = 0;
                if(is_end==1)
                {
                printf("try_cnt:%d\n",try_cnt);
                break;
                }
            } 
      try_cnt++;
      
    }
 
////////////////////////////////////////////////////////
    sprintf(msg_send, "AT+CGMR\r\n");//modem_fw
    write(fd_uart, msg_send, strlen(msg_send)); 
    try_cnt=0;
    ret = 0;
    cnt = 0; 
    while(try_cnt<10)
    {  usleep(100000);
     ret = read(fd_uart, &buf_rx[ret], BUF_SIZE-cnt);
            if (ret > 0 && ret <= BUF_SIZE)
            {
                cnt += ret;
            }
            printf("package size %d\n", ret);
            if(ret > 0)
            { 
            int  is_end=0;
                 printf("[app] read %d byte\n", cnt);
                printf("uart recv msg:%s\n",buf_rx);           
                 if(buf_rx[0]==0x00)
                {
                buf_rx[0]='.';
                }
                printf("++++++++++++++++++++++\n");
                char *saveptr;
                char*token=strtok_r(buf_rx,"\r\n",&saveptr);
                 while (token != NULL) {
                  printf("%s\n",token);
                  if(strstr(token,"mfw"))
                  { 
                    
                    printf("************\n");

                    cellular_arr[3]=token;
                    printf("************\n");
                    is_end=1;
                  break;
                  }
                   token = strtok_r(NULL, "\r\n", &saveptr);
                  }
                printf("---------------------\n");
                memset(buf_rx,0,BUF_SIZE);
                ret = 0;
                cnt = 0;
                if(is_end==1)
                {
                printf("try_cnt:%d\n",try_cnt);
                break;
                }
            } 
      try_cnt++;
      
    } 
 //////////////////////////////////////////
 //fuzhi
                lv_label_set_text(label_iccid, cellular_arr[0].c_str());
                lv_label_set_text(label_imsi, cellular_arr[1].c_str());
                lv_label_set_text(label_imei, cellular_arr[2].c_str());
                lv_label_set_text(label_modem_fw, cellular_arr[3].c_str());
 
  } //fd_uart 
        
  
    #if 0
    pthread_mutex_init(&mutex_uart_rx, NULL);
    pthread_cond_init(&cond, NULL);
    pthread_create(&pthread_handle_uart_rx, NULL, thread_uart_rx_entry, NULL);
    usleep(600000);
    
    lv_label_set_text(label_iccid, "");
    lv_label_set_text(label_imsi, "");
    lv_label_set_text(label_imei, "");
    lv_label_set_text(label_modem_fw, "");
    
    //set
    int try_cnt=0;
    char msg_send[25]={0};
    is_received=0;
    sprintf(msg_send, "AT%%XICCID\r\n");//ICCID
    write(fd_uart, msg_send, strlen(msg_send));
    
    //usleep(2000000);
    try_cnt=0;
    while(is_received!=1)
    {
    if(try_cnt>100)
    {
    printf("uart send fail<try_cnt:%d>\n",try_cnt);
    break;
    }
    usleep(20000);
    try_cnt++;
    }

    is_received=0;
    sprintf(msg_send, "AT+CGSN\r\n");//IMEI
    write(fd_uart, msg_send, strlen(msg_send));
    //usleep(2000000);
    try_cnt=0;
    while(is_received!=1)
    {
    if(try_cnt>100)
    {
    printf("uart send fail<try_cnt:%d>\n",try_cnt);
    break;
    }
    usleep(20000);
    try_cnt++;
    }
    is_received=0;
    sprintf(msg_send, "AT+CIMI\r\n");//IMSI
    write(fd_uart, msg_send, strlen(msg_send));
    //usleep(2000000);
    try_cnt=0;
     while(is_received!=1)
    {
    if(try_cnt>100)
    {
    printf("uart send fail<try_cnt:%d>\n",try_cnt);
    break;
    }
    usleep(20000);
    try_cnt++;
    }
    is_received=0;
    sprintf(msg_send, "AT+CGMR\r\n");//modem_fw
    write(fd_uart, msg_send, strlen(msg_send));
    //usleep(2000000);
    try_cnt=0;
     while(is_received!=1)
    {
    if(try_cnt>100)
    {
    printf("uart send fail<try_cnt:%d>\n",try_cnt);
    break;
    }
    usleep(20000);
    try_cnt++;
    }
    #endif
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
bool Cellular::back(void)
{
  notifyCoreClosed(); 
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Cellular::close(void)
{ 
    //flag_running=0;
    //pthread_join(pthread_handle_uart_rx, NULL);
    //pthread_mutex_destroy(&mutex_uart_rx);
    return true;
}

bool Cellular::init(void)
{

    return true;
}


 bool Cellular::pause()
{

 printf("cellular pause before \n");
 //notifyCoreClosed();
 printf("cellular pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Cellular::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("cellular resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Cellular::keyboard_event_cb(lv_event_t *e)
{
}
