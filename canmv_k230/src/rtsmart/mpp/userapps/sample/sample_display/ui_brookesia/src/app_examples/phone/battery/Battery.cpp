#include <math.h>
#include <vector>
#include "Battery.hpp"
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

LV_IMG_DECLARE(img_app_battery);
#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256

#define USING_BQ25896
#define USING_BQ27220

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
#if defined(USING_BQ25896)
static int bq25896_read_reg(int fd, uint8_t reg_addr,
        uint8_t *reg_data, int length)
{
    int ret=0;
        i2c_msg_t msgs[2]=
        {
        {
        .addr=0x6B, 
        .flags=0, 
        .len=1, 
        .buf=&reg_addr
        },
        {
        .addr=0x6B, 
        .flags=1, 
        .len=(uint16_t)length, 
        .buf=reg_data
        }
        };
        i2c_priv_data_t privdata={msgs, 2};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }

    return ret;
}

static int bq25896_write_reg(int fd, uint8_t reg_addr,
        uint8_t reg_data)
{
    int ret;
    uint8_t buff[2];
    buff[0]=reg_addr;
    buff[1]=reg_data;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x6B, 
        .flags=0, 
        .len=2, 
        .buf=buff
        }
        
        };
        i2c_priv_data_t privdata={msgs, 1};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }
    return ret;
}
#endif


/*
static int bq27220_read_reg(int fd, uint8_t reg_addr,
        uint8_t *reg_data, int length)
{
    int ret=0;
        i2c_msg_t msgs[2]=
        {
        {
        .addr=0x55, 
        .flags=0, 
        .len=1, 
        .buf=&reg_addr
        },
        {
        .addr=0x55, 
        .flags=1, 
        .len=(uint16_t)length, 
        .buf=reg_data
        }
        };
        i2c_priv_data_t privdata={msgs, 2};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }

    return ret;
}

static int bq27220_write_reg(int fd, uint8_t reg_addr,
        uint8_t reg_data)
{
    int ret;
    uint8_t buff[2];
    buff[0]=reg_addr;
    buff[1]=reg_data;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x55, 
        .flags=0, 
        .len=2, 
        .buf=buff
        }
        
        };
        i2c_priv_data_t privdata={msgs, 1};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }
    return ret;
}

static int bq27220_write_reg2(int fd, uint8_t reg_addr,
        uint8_t reg_data)
{
    int ret;
    uint8_t buff[2];
    buff[0]=reg_addr;
    buff[1]=reg_data;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x55, 
        .flags=0, 
        .len=2, 
        .buf=buff
        }
        
        };
        i2c_priv_data_t privdata={msgs, 1};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }
    return ret;
}
*/
#if defined(USING_BQ27220)
static int bq27220_read_cmd_data(int fd,uint8_t reg_addr,uint8_t *reg_data, int length)
{
    int ret=0;
        i2c_msg_t msgs[2]=
        {
         {
        .addr=0x55, 
        .flags=0, 
        .len=1, 
        .buf=&reg_addr
        },
        {
        .addr=0x55, 
        .flags=1, 
        .len=(uint16_t)length, 
        .buf=reg_data
        }
        };
        i2c_priv_data_t privdata={msgs, 2};
        if (ioctl(fd, RT_I2C_DEV_CTRL_RW, &privdata) != 0){ //读写数据
            ret=-1;
        }

    return ret;
}
static int bq27220_write_cmd_params(int fd, uint8_t*cmd_params,int length)
{
    int ret=0;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x55, 
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

#endif

//#define BUF_SIZE    (1024)
//#define UART_DEVICE_NAME    "/dev/uart1"
//static char buf_tx[BUF_SIZE];
//int g_ble_cnt=0;

lv_obj_t * label_bq25896_vbus;
lv_obj_t * label_bq25896_vsys;
lv_obj_t * label_bq25896_vbat;
lv_obj_t * label_bq25896_charge_current;
lv_obj_t * label_bq25896_charge_status;

lv_obj_t * label_bq25896_precharge_currentl_limit;
lv_obj_t * label_bq25896_fast_charge_currentl_limit;
lv_obj_t * label_bq25896_terminate_current_limit;
lv_obj_t * label_bq25896_charge_voltage_limit;



lv_obj_t * label_bq27220_full_charge_capacity;
lv_obj_t * label_bq27220_design_capacity;
lv_obj_t * label_bq27220_voltage;
lv_obj_t * label_bq27220_current;


lv_obj_t * label_bq27220_temperature_c;
lv_obj_t * label_bq27220_remaining_capacity;
lv_obj_t * label_bq27220_soc;
lv_obj_t * label_bq27220_soh;
lv_obj_t * label_bq27220_bat_state;







lv_timer_t * timer_battery_capacity_monitor=NULL;
lv_timer_t * timer_battery_charge_monitor=NULL;
char bq25896_data[20]={0};
char bq27220_data_s[20]={0};
Battery::Battery(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Battery", &img_app_battery, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_battery, use_status_bar, use_navigation_bar)
    )
{
}

Battery::~Battery()
{

}
void Battery::custom_timer_cb_battery_charge_monitor(lv_timer_t * timer) {
uint32_t * user_data = (uint32_t*)timer->user_data;
#if defined(USING_BQ25896)
// 执行一些LVGL相关的操作
//char send_data[20]={0};
bq25896_write_reg(fd_i2c0, 0x04,0x22);//0x2F:3008ma 0x28:2.5ma 0x22:2.1ma
uint8_t reg_data;

    bq25896_read_reg(fd_i2c0, 0x04,&reg_data,1);
    reg_data=reg_data&(~(1<<7));
    //printf("Fast Charge Current Limit:%dmA\n", reg_data*64+0);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmA", reg_data*64+0); 
    lv_label_set_text(label_bq25896_fast_charge_currentl_limit, bq25896_data); 
    
    //Termination Current Limit   Default: 256mA (0011)
    bq25896_read_reg(fd_i2c0, 0x05,&reg_data,1);
    reg_data=reg_data&(~(0xF<<4));
    //printf("Termination Current Limit:%dmA\n", reg_data*64+64);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmA", reg_data*64+64); 
    lv_label_set_text(label_bq25896_terminate_current_limit, bq25896_data);


   // Charge Voltage Limit   Default: 4.208V (010111)
    bq25896_read_reg(fd_i2c0, 0x06,&reg_data,1);
    reg_data=reg_data>>2;
    //printf("Charge Voltage Limit:%dmV\n", reg_data*16+3840);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmV", reg_data*16+3840); 
    lv_label_set_text(label_bq25896_charge_voltage_limit, bq25896_data);
    
    
    
    
     
    printf("*************************************\n");
    bq25896_read_reg(fd_i2c0, 0x02,&reg_data,1);
    printf("addr 0x02 and data=%#X\n", reg_data);
    reg_data=reg_data|(1<<7);
    printf("reg_data:%#X\n",reg_data);
    bq25896_write_reg(fd_i2c0, 0x02,reg_data);
    bq25896_read_reg(fd_i2c0, 0x02,&reg_data,1);
    printf("addr 0x02 and data=%#X\n", reg_data);
printf("--------------------------------------\n");
    bq25896_read_reg(fd_i2c0, 0x0E,&reg_data,1);
    reg_data=reg_data&(~(1<<7));
    //printf("vbat voltage:%dmV\n", reg_data*20+2304);
     memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmV", reg_data*20+2304);
    lv_label_set_text(label_bq25896_vbat, bq25896_data);
    
    
    bq25896_read_reg(fd_i2c0, 0x0F,&reg_data,1);
    reg_data=reg_data&(~(1<<7));
    //printf("sys voltage:%dmV\n", reg_data*20+2304);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmV", reg_data*20+2304);
    lv_label_set_text(label_bq25896_vsys, bq25896_data);
    
    //bq25896_read_reg(fd_i2c0, 0x10,&reg_data,1);
    //reg_data=reg_data&(~(1<<7));
    //printf("ts:%f%%\n", reg_data*0.465+21);
    
    
    
    bq25896_read_reg(fd_i2c0, 0x11,&reg_data,1);
    reg_data=reg_data&(~(1<<7));
    //printf("vbus:%dmV\n", reg_data*100+2600);
     memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmV", reg_data*100+2600);
    lv_label_set_text(label_bq25896_vbus, bq25896_data);
    
    

    bq25896_read_reg(fd_i2c0, 0x12,&reg_data,1);
    reg_data=reg_data&(~(1<<7));
    //printf("bat charge current:%dmA\n", reg_data*50+0);
     memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmA", reg_data*50+0); 
    lv_label_set_text(label_bq25896_charge_current, bq25896_data);
    
    
    bq25896_read_reg(fd_i2c0, 0x0B,&reg_data,1);
    //printf("charge status,addr 0x0B,data=%#X\n", reg_data);
    reg_data=(reg_data>>3)&0x03;
    memset(bq25896_data,0,20);
    if(reg_data==0x00)
    {
    sprintf(bq25896_data, "Not Charging");
    }
    else if(reg_data==0x01)
    {
    sprintf(bq25896_data, "Pre-charge");
    }
    else if(reg_data==0x02)
    {
    sprintf(bq25896_data, "Fast Charging");
    }
    else if(reg_data==0x03)
    {
    sprintf(bq25896_data, "Charge Termination Done");
    }
     
    lv_label_set_text(label_bq25896_charge_status, bq25896_data);
      
  
  printf("++++++++++++++++++++++++++++++++++++++++++++++++\n"); 

#endif




}

void Battery::custom_timer_cb_battery_capacity_monitor(lv_timer_t * timer) {
uint32_t * user_data = (uint32_t*)timer->user_data;
#if defined(USING_BQ27220)
uint8_t cmd_params[3]={0x00,0x01,0x00};
    int ret=bq27220_write_cmd_params(fd_i2c0,cmd_params,3);//trigger measure
    printf("bq27220_write_cmd_params,ret=%d\n",ret);
    usleep(15000);
    uint8_t bq27220_data[2];
    bq27220_read_cmd_data(fd_i2c0,0x40,bq27220_data,2);
    printf("bq27220 device_number:%02x %02x\n",bq27220_data[0],bq27220_data[1]);//0x20 0x02 0x00 0x00
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x12,bq27220_data,2);
    int full_charge_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 full charge capacity:%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],full_charge_capacity);
    memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%dmAh", full_charge_capacity); 
    lv_label_set_text(label_bq27220_full_charge_capacity, bq27220_data_s); 
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3C,bq27220_data,2);
    int design_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 design capacity:%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],design_capacity); 
    memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%dmAh", design_capacity); 
    lv_label_set_text(label_bq27220_design_capacity, bq27220_data_s);
    
    
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x08,bq27220_data,2);
    int bq27220_voltage=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_voltage():%02x %02x,value:%dmV\n",bq27220_data[0],bq27220_data[1],bq27220_voltage);
     memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%dmV", bq27220_voltage); 
    lv_label_set_text(label_bq27220_voltage, bq27220_data_s); 
    
     memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x0C,bq27220_data,2);
    int16_t bq27220_current=(int16_t)((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_current():%02x %02x,value:%dmA\n",bq27220_data[0],bq27220_data[1],bq27220_current);
    memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%dmA", bq27220_current); 
    lv_label_set_text(label_bq27220_current, bq27220_data_s); 
    
    
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x06,bq27220_data,2);
    float bq27220_temperature_k=(float)((bq27220_data[1]<<8)|bq27220_data[0])/10.0;
    float bq27220_temperature_c=bq27220_temperature_k-273.15;
    //printf("bq27220 get_temperature():%02x %02x,value:%.2fK,value:%.2fC\n",bq27220_data[0],bq27220_data[1],bq27220_temperature_k,bq27220_temperature_c);
    memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%.2fC", bq27220_temperature_c); 
    lv_label_set_text(label_bq27220_temperature_c, bq27220_data_s); 
    
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x10,bq27220_data,2);
    int bq27220_remaining_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_remaining_capacity():%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],bq27220_remaining_capacity);
       memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%dmAh", bq27220_remaining_capacity); 
    lv_label_set_text(label_bq27220_remaining_capacity, bq27220_data_s);
    
    
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2C,bq27220_data,2);
    int bq27220_soc=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_soc():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soc);
      memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%d%%", bq27220_soc); 
    lv_label_set_text(label_bq27220_soc, bq27220_data_s); 
    
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2E,bq27220_data,2);
    int bq27220_soh=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_soh():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soh);
     memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%d%%", bq27220_soh); 
    lv_label_set_text(label_bq27220_soh, bq27220_data_s); 
    
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x0A,bq27220_data,2);
    uint16_t bq27220_bat_state=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_bat_state():%02x %02x,value:%d\n",bq27220_data[0],bq27220_data[1],bq27220_bat_state);
    //printf("bq27220 bat_charge_state:%s\n",(bq27220_bat_state&0x0001)==1?"Discharging":"charging");
      memset(bq27220_data_s,0,20);
    sprintf(bq27220_data_s, "%s", (bq27220_bat_state&0x0001)==1?"Discharging":"charging"); 
    lv_label_set_text(label_bq27220_bat_state, bq27220_data_s); 
#endif
}




void Battery::btn_clicked_event(lv_event_t * e)
{

 }
bool Battery::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1; 
    
     /*Create a Tab view object*/
    lv_obj_t * tabview;
    tabview = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 80);

    /*Add 3 tabs (the tabs are page (lv_page) and can be scrolled*/
    lv_obj_t * tab_charge = lv_tabview_add_tab(tabview, "Charge");
    lv_obj_t * tab_coulometer = lv_tabview_add_tab(tabview, "Coulometer");
    lv_obj_t * label;
    static lv_coord_t col_dsc[] = {300, 200, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {80,80,80,80,80,80,80,80,80, LV_GRID_TEMPLATE_LAST};
#if defined(USING_BQ25896)
    /*Create a container with grid*/
    lv_obj_t * cont = lv_obj_create(tab_charge);
    lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    lv_obj_set_size(cont, 550, 850);
    lv_obj_center(cont);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

 label = lv_label_create(cont);
        lv_label_set_text(label, "VBUS:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


 label_bq25896_vbus = lv_label_create(cont);
        lv_label_set_text(label_bq25896_vbus, "");
        lv_obj_center(label_bq25896_vbus);
lv_obj_set_grid_cell(label_bq25896_vbus, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


label = lv_label_create(cont);
        lv_label_set_text(label, "VSYS:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


 label_bq25896_vsys = lv_label_create(cont);
        lv_label_set_text(label_bq25896_vsys, "");
        lv_obj_center(label_bq25896_vsys);
lv_obj_set_grid_cell(label_bq25896_vsys, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);
  
  
label = lv_label_create(cont);
        lv_label_set_text(label, "VBAT:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);


 label_bq25896_vbat = lv_label_create(cont);
        lv_label_set_text(label_bq25896_vbat, "");
        lv_obj_center(label_bq25896_vbat);
lv_obj_set_grid_cell(label_bq25896_vbat, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);  
  
  
 label = lv_label_create(cont);
        lv_label_set_text(label, "charge_current:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);


 label_bq25896_charge_current = lv_label_create(cont);
        lv_label_set_text(label_bq25896_charge_current, "");
        lv_obj_center(label_bq25896_charge_current);
lv_obj_set_grid_cell(label_bq25896_charge_current, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);   
  
  
  
  label = lv_label_create(cont);
        lv_label_set_text(label, "charge_status:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 4, 1);


 label_bq25896_charge_status = lv_label_create(cont);
        lv_label_set_text(label_bq25896_charge_status, "");
        lv_obj_center(label_bq25896_charge_status);
lv_obj_set_grid_cell(label_bq25896_charge_status, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 4, 1);
  
 
   label = lv_label_create(cont);
        lv_label_set_text(label, "precharge_current_limit:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 5, 1);


 label_bq25896_precharge_currentl_limit = lv_label_create(cont);
        lv_label_set_text(label_bq25896_precharge_currentl_limit, "");
        lv_obj_center(label_bq25896_precharge_currentl_limit);
lv_obj_set_grid_cell(label_bq25896_precharge_currentl_limit, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 5, 1); 
  
  
     label = lv_label_create(cont);
        lv_label_set_text(label, "fast_charge_current_limit:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 6, 1);


 label_bq25896_fast_charge_currentl_limit = lv_label_create(cont);
        lv_label_set_text(label_bq25896_fast_charge_currentl_limit, "");
        lv_obj_center(label_bq25896_fast_charge_currentl_limit);
lv_obj_set_grid_cell(label_bq25896_fast_charge_currentl_limit, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 6, 1); 
  
  label = lv_label_create(cont);
        lv_label_set_text(label, "terminate_current_limit:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 7, 1);


 label_bq25896_terminate_current_limit = lv_label_create(cont);
        lv_label_set_text(label_bq25896_terminate_current_limit, "");
        lv_obj_center(label_bq25896_terminate_current_limit);
lv_obj_set_grid_cell(label_bq25896_terminate_current_limit, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 7, 1); 
  
   label = lv_label_create(cont);
        lv_label_set_text(label, "charge_voltage_limit:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 8, 1);


 label_bq25896_charge_voltage_limit = lv_label_create(cont);
        lv_label_set_text(label_bq25896_charge_voltage_limit, "");
        lv_obj_center(label_bq25896_charge_voltage_limit);
lv_obj_set_grid_cell(label_bq25896_charge_voltage_limit, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 8, 1);    
  #endif
  ////////////////////////////////////////////////////
  
  #if defined(USING_BQ27220)
  /*Create a container with grid*/
    static lv_coord_t col_dsc1[] = {300, 200, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc1[] = {80,80,80,80,80,80,80,80,80, LV_GRID_TEMPLATE_LAST};
    lv_obj_t * cont1 = lv_obj_create(tab_coulometer);
    lv_obj_set_style_grid_column_dsc_array(cont1, col_dsc1, 0);
    lv_obj_set_style_grid_row_dsc_array(cont1, row_dsc1, 0);
    lv_obj_set_size(cont1, 550, 850);
    lv_obj_center(cont1);
    lv_obj_set_layout(cont1, LV_LAYOUT_GRID);

label = lv_label_create(cont1);
        lv_label_set_text(label, "full_charge_capacity:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


 label_bq27220_full_charge_capacity = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_full_charge_capacity, "");
        lv_obj_center(label_bq27220_full_charge_capacity);
lv_obj_set_grid_cell(label_bq27220_full_charge_capacity, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


label = lv_label_create(cont1);
        lv_label_set_text(label, "design_capacity:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


 label_bq27220_design_capacity = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_design_capacity, "");
        lv_obj_center(label_bq27220_design_capacity);
lv_obj_set_grid_cell(label_bq27220_design_capacity, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);
  
  
label = lv_label_create(cont1);
        lv_label_set_text(label, "voltage:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);


 label_bq27220_voltage = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_voltage, "");
        lv_obj_center(label_bq27220_voltage);
lv_obj_set_grid_cell(label_bq27220_voltage, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 2, 1);  
  
  
 label = lv_label_create(cont1);
        lv_label_set_text(label, "current:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);


 label_bq27220_current = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_current, "");
        lv_obj_center(label_bq27220_current);
lv_obj_set_grid_cell(label_bq27220_current, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 3, 1);   
  
  
  
  label = lv_label_create(cont1);
        lv_label_set_text(label, "temperature_c:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 4, 1);


 label_bq27220_temperature_c = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_temperature_c, "");
        lv_obj_center(label_bq27220_temperature_c);
lv_obj_set_grid_cell(label_bq27220_temperature_c, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 4, 1);
  
 
   label = lv_label_create(cont1);
        lv_label_set_text(label, "remaining_capacity:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 5, 1);


 label_bq27220_remaining_capacity = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_remaining_capacity, "");
        lv_obj_center(label_bq27220_remaining_capacity);
lv_obj_set_grid_cell(label_bq27220_remaining_capacity, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 5, 1); 
  
  
     label = lv_label_create(cont1);
        lv_label_set_text(label, "soc:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 6, 1);


 label_bq27220_soc = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_soc, "");
        lv_obj_center(label_bq27220_soc);
lv_obj_set_grid_cell(label_bq27220_soc, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 6, 1); 
  
  label = lv_label_create(cont1);
        lv_label_set_text(label, "soh:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 7, 1);


 label_bq27220_soh = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_soh, "");
        lv_obj_center(label_bq27220_soh);

lv_obj_set_grid_cell(label_bq27220_soh, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 7, 1); 
  
   label = lv_label_create(cont1);
        lv_label_set_text(label, "bat_state:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 8, 1);


 label_bq27220_bat_state = lv_label_create(cont1);
        lv_label_set_text(label_bq27220_bat_state, "");
        lv_obj_center(label_bq27220_bat_state);
lv_obj_set_grid_cell(label_bq27220_bat_state, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 8, 1);
  #endif
  
   ////////////////////////////////////////// 
    
    /////test
    //fpioa_set_function(46,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);
    //fpioa_set_function(47,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);    
    //fd_i2c0 = open("/dev/i2c4", O_RDWR);


/////test
    
    
    //init bq25896 i2c
    fpioa_set_function(46,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(47,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);    
    fd_i2c0 = open("/dev/i2c4", O_RDWR);
    if (fd_i2c0> 0)
    {
        uint32_t spped=100000;
     if (ioctl(fd_i2c0, RT_I2C_DEV_CTRL_CLK, &spped) != 0){ //设置速率
        printf("set speed %d failed!\n",spped);
    }
    static uint32_t user_data = 35; 
    uint8_t reg_data;
#if defined(USING_BQ25896)
    bq25896_read_reg(fd_i2c0, 0x03,&reg_data,1);
    printf("chg_config addr 0x03 and data=%#X\n", reg_data);

   //# Precharge Current Limit   Default: 128mA (0001)
    bq25896_read_reg(fd_i2c0, 0x05,&reg_data,1);
    reg_data=reg_data>>4;
    //printf("Precharge Current Limit:%dmA\n", reg_data*64+64);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmA", reg_data*64+64); 
    lv_label_set_text(label_bq25896_precharge_currentl_limit, bq25896_data);

    
    bq25896_write_reg(fd_i2c0, 0x04,0x22);//0x2F:3008ma
    //ICHG=000000 (0mA) disables charge
    bq25896_read_reg(fd_i2c0, 0x04,&reg_data,1);
    reg_data=reg_data&(~(1<<7));
    //printf("Fast Charge Current Limit:%dmA\n", reg_data*64+0);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmA", reg_data*64+0); 
    lv_label_set_text(label_bq25896_fast_charge_currentl_limit, bq25896_data);







   //Termination Current Limit   Default: 256mA (0011)
    bq25896_read_reg(fd_i2c0, 0x05,&reg_data,1);
    reg_data=reg_data&(~(0xF<<4));
    //printf("Termination Current Limit:%dmA\n", reg_data*64+64);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmA", reg_data*64+64); 
    lv_label_set_text(label_bq25896_terminate_current_limit, bq25896_data);


   // Charge Voltage Limit   Default: 4.208V (010111)
    bq25896_read_reg(fd_i2c0, 0x06,&reg_data,1);
    reg_data=reg_data>>2;
    //printf("Charge Voltage Limit:%dmV\n", reg_data*16+3840);
    memset(bq25896_data,0,20);
    sprintf(bq25896_data, "%dmV", reg_data*16+3840); 
    lv_label_set_text(label_bq25896_charge_voltage_limit, bq25896_data);
timer_battery_charge_monitor = lv_timer_create(custom_timer_cb_battery_charge_monitor, 2000, &user_data);
lv_timer_set_repeat_count(timer_battery_charge_monitor, -1); 
#endif

#if defined(USING_BQ27220)
//////////////init  modify capacity
    uint8_t bq27220_data1[2];
    printf("-----get operation status\n");
    //get operation status  0x3:seal 0x2:unseal 0x1: full access
    memset(bq27220_data1,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data1,2);
    int flag_seal=((bq27220_data1[0]>>1)&0x03);
    int init_comp=((bq27220_data1[0]>>5)&0x01);
    int cfg_update=((bq27220_data1[1]>>2)&0x01);
    printf("bq27220 operation status:%02x %02x,flag_seal:%d,init_comp:%d,cfg_update:%d\n",bq27220_data1[0],bq27220_data1[1],flag_seal,init_comp,cfg_update);
    

    printf("-----0x0414\n");
    uint8_t cmd_params1[3]={0x00,0x14,0x04};
    int ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);

    printf("-----0x3672\n");
    cmd_params1[0]=0x00;
    cmd_params1[1]=0x72;
    cmd_params1[2]=0x36;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);

    
    //get operation status
    printf("-----get operation status\n");
    memset(bq27220_data1,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data1,2);
    flag_seal=((bq27220_data1[0]>>1)&0x03);
    init_comp=((bq27220_data1[0]>>5)&0x01);
    cfg_update=((bq27220_data1[1]>>2)&0x01);
    printf("bq27220 operation status:%02x %02x,flag_seal:%d,init_comp:%d,cfg_update:%d\n",bq27220_data1[0],bq27220_data1[1],flag_seal,init_comp,cfg_update);

//reset
    printf("-----reset\n");
    cmd_params1[0]=0x00;
    cmd_params1[1]=0x41;
    cmd_params1[2]=0x00;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(3500000);


 //get operation status
    printf("-----get operation status\n");
    memset(bq27220_data1,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data1,2);
    flag_seal=((bq27220_data1[0]>>1)&0x03);
    init_comp=((bq27220_data1[0]>>5)&0x01);
    cfg_update=((bq27220_data1[1]>>2)&0x01);
    printf("bq27220 operation status:%02x %02x,flag_seal:%d,init_comp:%d,cfg_update:%d\n",bq27220_data1[0],bq27220_data1[1],flag_seal,init_comp,cfg_update);
    
    ////full_access
    printf("-----start full access\n");
    cmd_params1[0]=0x00;
    cmd_params1[1]=0xFF;
    cmd_params1[2]=0xFF;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);
    cmd_params1[0]=0x00;
    cmd_params1[1]=0xFF;
    cmd_params1[2]=0xFF;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);
 //get operation status
    printf("-----get operation status\n");
    memset(bq27220_data1,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data1,2);
    flag_seal=((bq27220_data1[0]>>1)&0x03);
    init_comp=((bq27220_data1[0]>>5)&0x01);
    cfg_update=((bq27220_data1[1]>>2)&0x01);
    printf("bq27220 operation status:%02x %02x,flag_seal:%d,init_comp:%d,cfg_update:%d\n",bq27220_data1[0],bq27220_data1[1],flag_seal,init_comp,cfg_update);
//cfg_update
    printf("-----start cfg_update\n");
    cmd_params1[0]=0x00;
    cmd_params1[1]=0x90;
    cmd_params1[2]=0x00;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);
    
 //get operation status
    printf("-----get operation status\n");
    memset(bq27220_data1,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data1,2);
    flag_seal=((bq27220_data1[0]>>1)&0x03);
    init_comp=((bq27220_data1[0]>>5)&0x01);
    cfg_update=((bq27220_data1[1]>>2)&0x01);
    printf("bq27220 operation status:%02x %02x,flag_seal:%d,init_comp:%d,cfg_update:%d\n",bq27220_data1[0],bq27220_data1[1],flag_seal,init_comp,cfg_update);   
//////////////////////////////////////////////////
//set ffc
   {
   int full_charge_capacity_new=10000;
    cmd_params1[0]=0x3E;
    cmd_params1[1]=0x9D;
    cmd_params1[2]=0x92;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);

    cmd_params1[0]=0x40;
    cmd_params1[1]=(full_charge_capacity_new >> 8) & 0xff;
    cmd_params1[2]=(full_charge_capacity_new >> 0) & 0xff;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);

int offset_fcc=255;
int check_sum_fcc=255-((cmd_params1[1]+cmd_params1[2]+offset_fcc)%256);
 printf("check_sum_fcc:%d\n",check_sum_fcc); 
    cmd_params1[0]=0x60;
    cmd_params1[1]=check_sum_fcc;//check_sum:5000:0x65,10000:0xC9
    cmd_params1[2]=0x24;//data_len
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);




  }
  usleep(500000);
//set design capacity
{
int design_capacity_new=10000;
    cmd_params1[0]=0x3E;
    cmd_params1[1]=0x9F;
    cmd_params1[2]=0x92;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);

    
    cmd_params1[0]=0x40;
    cmd_params1[1]=(design_capacity_new >> 8) & 0xff;
    cmd_params1[2]=(design_capacity_new >> 0) & 0xff;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);
int offset_dc=155;
int check_sum_dc=255-((cmd_params1[1]+cmd_params1[2]+offset_dc)%256);
  printf("check_sum_dc:%d\n",check_sum_dc);
    cmd_params1[0]=0x60;
    cmd_params1[1]=check_sum_dc;//check_sum:5000:0xC9,10000:0x2D
    cmd_params1[2]=0x24;//data_len
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);



}
//////////////////////////////////////////////////
    printf("-----exit cfg_update\n");
    cmd_params1[0]=0x00;
    cmd_params1[1]=0x91;
    cmd_params1[2]=0x00;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(50000);
    
    
 //get operation status
    printf("-----get operation status\n");
    memset(bq27220_data1,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data1,2);
    flag_seal=((bq27220_data1[0]>>1)&0x03);
    init_comp=((bq27220_data1[0]>>5)&0x01);
    cfg_update=((bq27220_data1[1]>>2)&0x01);
    printf("bq27220 operation status:%02x %02x,flag_seal:%d,init_comp:%d,cfg_update:%d\n",bq27220_data1[0],bq27220_data1[1],flag_seal,init_comp,cfg_update);       
//seal
 printf("-----exit seal\n");
    cmd_params1[0]=0x00;
    cmd_params1[1]=0x30;
    cmd_params1[2]=0x00;
     ret1=bq27220_write_cmd_params(fd_i2c0,cmd_params1,3);
    printf("bq27220_write_cmd_params,ret=%d\n",ret1);
    usleep(1000000);

//get operation status
    printf("-----get operation status\n");
    memset(bq27220_data1,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3A,bq27220_data1,2);
    flag_seal=((bq27220_data1[0]>>1)&0x03);
    init_comp=((bq27220_data1[0]>>5)&0x01);
    cfg_update=((bq27220_data1[1]>>2)&0x01);
    printf("bq27220 operation status:%02x %02x,flag_seal:%d,init_comp:%d,cfg_update:%d\n",bq27220_data1[0],bq27220_data1[1],flag_seal,init_comp,cfg_update); 



#endif

#if defined(USING_BQ27220)
    //BQ27220
    /*bq27220_write_reg(fd_i2c0,0x00,0x00);
    bq27220_write_reg(fd_i2c0,0x01,0x01);
    usleep(15000);
    uint8_t bq27220_data[4];
    bq27220_read_reg(fd_i2c0,0x3E,bq27220_data,4);
    printf("bq27220 device_number:%02x %02x %02x %02x\n",bq27220_data[0],bq27220_data[1],bq27220_data[2],bq27220_data[3]);*/
    
    
     /*uint8_t cmd_params[3]={0x00,0x01,0x00};
    int ret=bq27220_write_cmd_params(fd_i2c0,cmd_params,3);//trigger measure
    printf("bq27220_write_cmd_params,ret=%d\n",ret);*/
    
    uint8_t cmd_params[3]={0x00,0x01,0x00};
    int ret=bq27220_write_cmd_params(fd_i2c0,cmd_params,3);//trigger measure
    printf("bq27220_write_cmd_params,ret=%d\n",ret);
    usleep(15000);
    uint8_t bq27220_data[2];
    bq27220_read_cmd_data(fd_i2c0,0x40,bq27220_data,2);
    printf("bq27220 device_number:%02x %02x\n",bq27220_data[0],bq27220_data[1]);//0x20 0x02 0x00 0x00
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x12,bq27220_data,2);
    int full_charge_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 full charge capacity:%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],full_charge_capacity);
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x3C,bq27220_data,2);
    int design_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 design capacity:%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],design_capacity); 
    
    
    
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x08,bq27220_data,2);
    int bq27220_voltage=((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 get_voltage():%02x %02x,value:%dmV\n",bq27220_data[0],bq27220_data[1],bq27220_voltage);
     memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x0C,bq27220_data,2);
    int16_t bq27220_current=(int16_t)((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 get_current():%02x %02x,value:%dmA\n",bq27220_data[0],bq27220_data[1],bq27220_current);
    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x06,bq27220_data,2);
    float bq27220_temperature_k=(float)((bq27220_data[1]<<8)|bq27220_data[0])/10.0;
    float bq27220_temperature_c=bq27220_temperature_k-273.15;
    printf("bq27220 get_temperature():%02x %02x,value:%.2fK,value:%.2fC\n",bq27220_data[0],bq27220_data[1],bq27220_temperature_k,bq27220_temperature_c);
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x10,bq27220_data,2);
    int bq27220_remaining_capacity=((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 get_remaining_capacity():%02x %02x,value:%dmAh\n",bq27220_data[0],bq27220_data[1],bq27220_remaining_capacity);
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2C,bq27220_data,2);
    int bq27220_soc=((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 get_soc():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soc);
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2E,bq27220_data,2);
    int bq27220_soh=((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 get_soh():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soh);
     
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x0A,bq27220_data,2);
    uint16_t bq27220_bat_state=((bq27220_data[1]<<8)|bq27220_data[0]);
    printf("bq27220 get_bat_state():%02x %02x,value:%d\n",bq27220_data[0],bq27220_data[1],bq27220_bat_state);
    printf("bq27220 bat_charge_state:%s\n",(bq27220_bat_state&0x0001)==1?"Discharging":"charging");
    
    timer_battery_capacity_monitor= lv_timer_create(custom_timer_cb_battery_capacity_monitor, 2000, &user_data);
    lv_timer_set_repeat_count(timer_battery_capacity_monitor, -1);  
#endif
      
    }
     
    
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
bool Battery::back(void)
{
  notifyCoreClosed(); 
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Battery::close(void)
{ 
    //flag_running=0;
    //pthread_join(pthread_handle_uart_rx, NULL);
    //pthread_mutex_destroy(&mutex_uart_rx);
    return true;
}

bool Battery::init(void)
{

    return true;
}


 bool Battery::pause()
{

 printf("battery pause before \n");
 //notifyCoreClosed();
 printf("battery pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Battery::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("battery resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Battery::keyboard_event_cb(lv_event_t *e)
{
}
