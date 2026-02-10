#include <math.h>
#include <vector>
#include "Weather.hpp"
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

LV_IMG_DECLARE(img_app_weather);
#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256


#define RT_I2C_DEV_CTRL_10BIT (0x800 + 0x01)
#define RT_I2C_DEV_CTRL_TIMEOUT (0x800 + 0x03)
#define RT_I2C_DEV_CTRL_RW (0x800 + 0x04)
#define RT_I2C_DEV_CTRL_CLK (0x800 + 0x05)

typedef struct {
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t *buf;

} i2c_msg_t;

typedef struct {
    i2c_msg_t *msgs;
    size_t number;
} i2c_priv_data_t;



static int fd_i2c0=-1;

static int aht20_read_status_data(int fd,uint8_t *reg_data, int length)
{
    int ret=0;
        i2c_msg_t msgs[1]=
        {
        {
        .addr=0x38, 
        .flags=1, //1  ,0x41,0x11,0x21
        .len=(uint16_t)length, 
        .buf=reg_data
        }
        };
        i2c_priv_data_t privdata={msgs, 1};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }

    return ret;
}
static int aht20_write_cmd_params(int fd, uint8_t*cmd_params,int length)
{
    int ret=0;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x38, 
        .flags=0, 
        .len=(uint16_t)length, 
        .buf=cmd_params
        }
        
        };
        i2c_priv_data_t privdata={msgs, 1};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }
    return ret;
}
lv_obj_t * label_temperature;
lv_obj_t * label_humidity;







lv_timer_t * timer_weather_monitor=NULL;
char aht20_data_tmp[20]={0};
Weather::Weather(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Weather", &img_app_weather, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_weather, use_status_bar, use_navigation_bar)
    )
{
}

Weather::~Weather()
{

}
void Weather::custom_timer_cb_weather_monitor(lv_timer_t * timer) {
uint32_t * user_data = (uint32_t*)timer->user_data;
    uint8_t cmd_params[3]={0xAC,0x33,0x00};
    int ret=aht20_write_cmd_params(fd_i2c0,cmd_params,3);//trigger measure
    //printf("aht20_write_cmd_params,ret=%d\n",ret);
    usleep(80000);  
    uint8_t aht20_data[6];
    memset(aht20_data,0,6);
    aht20_read_status_data(fd_i2c0,aht20_data,6);
    //printf("aht20_raw_data:%02x %02x %02x %02x %02x %02x\n",aht20_data[0],aht20_data[1],aht20_data[2],aht20_data[3],aht20_data[4],aht20_data[5]);
    uint32_t raw_temperature=((aht20_data[3]&0x0F)<<16)|(aht20_data[4]<<8)|aht20_data[5];
    uint32_t raw_humidity=(aht20_data[1]<<12)|(aht20_data[2]<<4)|(aht20_data[3]>>4);
    float temperature=((float)raw_temperature/pow(2,20)*200-50);
    float humidity=(float)raw_humidity/pow(2,20)*100;
    //printf("temperature:%f,humidity:%f\n",temperature,humidity);
    memset(aht20_data_tmp,0,20);
    sprintf(aht20_data_tmp, "%f deg C", temperature);
    lv_label_set_text(label_temperature, aht20_data_tmp);
    memset(aht20_data_tmp,0,20);
    sprintf(aht20_data_tmp, "%f %%", humidity);
    lv_label_set_text(label_humidity, aht20_data_tmp);
  
}
void Weather::btn_clicked_event(lv_event_t * e)
{

 }
bool Weather::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1; 
    
    
    
    static lv_coord_t col_dsc[] = {300, 200, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {80,80,80,80,80,80,80,80,80, LV_GRID_TEMPLATE_LAST};

    /*Create a container with grid*/
    lv_obj_t * cont = lv_obj_create(lv_scr_act());
    lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    lv_obj_set_size(cont, 550, 850);
    lv_obj_center(cont);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

lv_obj_t * label = lv_label_create(cont);
        lv_label_set_text(label, "Temperature:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


 label_temperature = lv_label_create(cont);
        lv_label_set_text(label_temperature, "");
        lv_obj_center(label_temperature);
lv_obj_set_grid_cell(label_temperature, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


label = lv_label_create(cont);
        lv_label_set_text(label, "Humidity:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


 label_humidity = lv_label_create(cont);
        lv_label_set_text(label_humidity, "");
        lv_obj_center(label_humidity);
lv_obj_set_grid_cell(label_humidity, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);
  
   
    //init aht20 i2c

    fpioa_set_function(46,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(47,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);    
    fd_i2c0 = open("/dev/i2c4", O_RDWR);
    if (fd_i2c0> 0)
    {
        uint32_t spped=100000;
     if (ioctl(fd_i2c0, RT_I2C_DEV_CTRL_CLK, &spped) != 0){ //设置速率
        printf("set speed %d failed!\n",spped);
    }
    usleep(40000);
    uint8_t aht20_status;
    int ret=aht20_read_status_data(fd_i2c0,&aht20_status,1);
    printf("aht20_read_status_data,ret=%d\n",ret);
    if((aht20_status&0x08)>>3==0x0)//not calib
    {
    printf("aht20 init start\n");
    uint8_t cmd_params[3]={0xBE,0x08,0x00};
    aht20_write_cmd_params(fd_i2c0,cmd_params,3);
    }
    //reset
    uint8_t reset_cmd[1]={0xBA};
    aht20_write_cmd_params(fd_i2c0,reset_cmd,1);//init
    usleep(40000);
    
    do
    {
    usleep(1000);
    aht20_read_status_data(fd_i2c0,&aht20_status,1);
    }while(!((aht20_status==0x08)||(aht20_status==0x18)));
    printf("reset_cmd exec ok\n");
    
    
    uint8_t cmd_params[3]={0xBE,0x08,0x00};
    aht20_write_cmd_params(fd_i2c0,cmd_params,3);
    usleep(20000);
    
    do
    {
    usleep(1000);
    aht20_read_status_data(fd_i2c0,&aht20_status,1);
    }while(!((aht20_status==0x08)||(aht20_status==0x18)));
    printf("init_cmd exec ok\n");
    }
static uint32_t user_data = 35; 
timer_weather_monitor = lv_timer_create(custom_timer_cb_weather_monitor, 5000, &user_data);
lv_timer_set_repeat_count(timer_weather_monitor, -1); 

    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool Weather::back(void)
{
  notifyCoreClosed(); 
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Weather::close(void)
{ 
    //flag_running=0;
    //pthread_join(pthread_handle_uart_rx, NULL);
    //pthread_mutex_destroy(&mutex_uart_rx);
    return true;
}

bool Weather::init(void)
{

    return true;
}


 bool Weather::pause()
{

 printf("weather pause before \n");
 //notifyCoreClosed();
 printf("weather pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Weather::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("weather resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Weather::keyboard_event_cb(lv_event_t *e)
{
}
