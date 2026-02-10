#include <math.h>
//#include <vector>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <rtthread.h>
//#include <rtdevice.h>
#include <poll.h>
#include "fpioa/rt_fpioa.h"
#include "keyboard_ext.h"
//using namespace std;
#include<lvgl.h>
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

#define	KD_PWM_CMD_ENABLE           _IOW('P', 0, int)
#define	KD_PWM_CMD_DISABLE          _IOW('P', 1, int)
#define	KD_PWM_CMD_SET              _IOW('P', 2, int)
#define	KD_PWM_CMD_GET              _IOW('P', 3, int)
typedef struct
{
    unsigned int channel; /* 0-n */
    unsigned int period;  /* unit:ns 1ns~4.29s:1Ghz~0.23hz */
    unsigned int pulse;   /* unit:ns (pulse<=period) */
} pwm_config_t;
int flag_press_fn=0;
static int flag_press_shift=0;
int flag_press_caps_lock=0;
static int fd_keyboard=-1;

static int tca8418_read_reg(int fd, uint8_t reg_addr,
        uint8_t *reg_data, int length)
{
    int ret=0;
        i2c_msg_t msgs[2]=
        {
        {
        .addr=0x34, 
        .flags=0, 
        .len=1, 
        .buf=&reg_addr
        },
        {
        .addr=0x34, 
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

static int tca8418_write_reg(int fd, uint8_t reg_addr,
        uint8_t reg_data)
{
    int ret=0;
    uint8_t buff[2];
    buff[0]=reg_addr;
    buff[1]=reg_data;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x34, 
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

static int xl9555_write_reg(int fd, uint8_t reg_addr,
        uint8_t reg_data)
{
    int ret=0;
    uint8_t buff[2];
    buff[0]=reg_addr;
    buff[1]=reg_data;
    i2c_msg_t msgs[1]=
        {
        {
        .addr=0x20, 
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
static const uint32_t tca8418_keyboard_lvgl[] =
    {
        LV_KEY_RIGHT,LV_KEY_LEFT,0x8E,0x00,' ',LV_KEY_NEXT,0x90,0x8F,0x8E,0x8B,
	  0xA2,LV_KEY_DOWN,'m',' ','v','c','x','z',0x8C,'q',
	  LV_KEY_ENTER,LV_KEY_UP,0x8D,'n','b','f','d','s','a',0x00,
	   0x00,'l','k','j','h','g','r','e','w',LV_KEY_ESC,
	   LV_KEY_BACKSPACE,'p','o','i','u','y','t','2','1',0x81,
	   '0','9','8','7','6','5','4','3',0x83,0x82,
	   0x91,0x8A,0x89,0x88,0x87,0x86,0x85,0x84
	   };
static const uint32_t tca8418_keyboard_lvgl_shift[] =
    {
        LV_KEY_RIGHT,LV_KEY_LEFT,0x8E,0x00,' ',LV_KEY_NEXT,0x90,0x8F,0x8E,0x8B,
	  0xA2,LV_KEY_DOWN,'>',' ','v','c','x','z',0x8C,'\'',
	  LV_KEY_ENTER,LV_KEY_UP,0x8D,'<','.','{',']','[','~',0x00,
	   0x00,'?','/','`','\'','}','+','-','_',LV_KEY_ESC,
	   LV_KEY_BACKSPACE,'"',':',';','|','\\','=','@','!',0x81,
	   ')','(','*','&','^','%','$','#',0x83,0x82,
	   0x91,0x8A,0x89,0x88,0x87,0x86,0x85,0x84
	   };
void tca8418_init()
{
//return;
 int gpio_fd = open("/dev/gpio", O_RDWR);
    if (gpio_fd>0)
    {
      //reset tca8418
      pin_gpio_t pin_43;
      pin_43.pin = 43;
      ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_43);  //pin43 output
      ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_43);//
      usleep(10000);
      ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_43);//
      usleep(10000);
      ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_43);//
      close(gpio_fd);          
    }
 

    fpioa_set_function(46,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(47,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);    
    fd_keyboard = open("/dev/i2c4", O_RDWR);
    if (fd_keyboard> 0)
    {
        uint32_t spped=100000;
     if (ioctl(fd_keyboard, RT_I2C_DEV_CTRL_CLK, &spped) != 0){ //设置速率
        printf("set speed %d failed!\n",spped);
    }
    //init_tca8418
    int ret;
    uint8_t reg_data;
    /////////////////////////////
    reg_data=0x00;
    ret=tca8418_write_reg(fd_keyboard, 0x23,reg_data);//
    printf("tca8418_write_reg 0x23,ret:%d\n",ret);
    reg_data=0x00;
    ret=tca8418_write_reg(fd_keyboard, 0x24,reg_data);//
    reg_data=0x00;
    ret=tca8418_write_reg(fd_keyboard, 0x25,reg_data);//
    
    
    
    
    reg_data=0xFF;
    ret=tca8418_write_reg(fd_keyboard, 0x20,reg_data);//
    reg_data=0xFF;
    ret=tca8418_write_reg(fd_keyboard, 0x21,reg_data);//
    reg_data=0xFF;
    ret=tca8418_write_reg(fd_keyboard, 0x22,reg_data);//
    
    
    
    
    
    
    reg_data=0x00;
    ret=tca8418_write_reg(fd_keyboard, 0x26,reg_data);//   
    reg_data=0x00;
    ret=tca8418_write_reg(fd_keyboard, 0x27,reg_data);//
    reg_data=0x00;
    ret=tca8418_write_reg(fd_keyboard, 0x28,reg_data);//
    
    
    reg_data=0xFF;
    ret=tca8418_write_reg(fd_keyboard, 0x1A,reg_data);
    reg_data=0xFF;
    ret=tca8418_write_reg(fd_keyboard, 0x1B,reg_data);
    reg_data=0xFF;
    ret=tca8418_write_reg(fd_keyboard, 0x1C,reg_data);    
    
    
    
    
    
    
    
    /////////////////////////////////////////////////////////////////////////
    reg_data=0x7F;
    ret=tca8418_write_reg(fd_keyboard, 0x1D,reg_data);//KP_GPIO1 row0-6
    //printf("tca8418_write_reg ret=%d\n",ret);
    reg_data=0xFF;
    tca8418_write_reg(fd_keyboard, 0x1E,reg_data);//KP_GPIO2 col0-7
    reg_data=0x03;
    tca8418_write_reg(fd_keyboard, 0x1F,reg_data);//KP_GPIO3 col8-9

    reg_data=0x00; 
    ret=tca8418_read_reg(fd_keyboard, 0x01,&reg_data,1);//CFG   read
    printf("tca8418_read_reg 0x01 ret=%d\n",ret);
    //test keyboard exist
    if(ret==-1)
    {
    close(fd_keyboard);
    fd_keyboard=-1;
    printf("keyboard is not exist\n");
    return;
    }
    printf("detect keyboard device\n");
    //printf("addr1 0x01 and data=%#X\n", reg_data);
    reg_data=(reg_data&0xF0)|0x01;//key_event
    //printf("addr2 0x01 and data=%#X\n", reg_data);
    tca8418_write_reg(fd_keyboard, 0x01,reg_data);//CFG
    printf("addr3 0x01 and data=%#X\n", reg_data);
    reg_data=0x01;
    tca8418_write_reg(fd_keyboard, 0x02,reg_data);//
	}

}
uint32_t tca8418_get_key()
{
    if(fd_keyboard<=0)
     {
        return 0;
     }
   uint32_t key_code=0x00;
    //int ret;
    uint8_t reg_data;    
    tca8418_read_reg(fd_keyboard, 0x02,&reg_data,1);//_INT_STAT
    if((reg_data&0x01)==0x01)
    { 
    tca8418_read_reg(fd_keyboard, 0x03,&reg_data,1);//KEY_LCK_EC
    int event_num=reg_data&0x0F;
    //printf("event_num:%d\n",event_num);
    for(int i=0;i<event_num;i++)
    {
     tca8418_read_reg(fd_keyboard, 0x04,&reg_data,1);//KEY_EVENT_A
     //printf("addr 0x04 and data=%#X\n", reg_data); 
     int flag_press=reg_data>>7;
     int code_index=reg_data&0x7F;
     if(code_index>96)
     {
     printf("flag_press:%d,code_index:%d,code:%s\n",flag_press,code_index,"GPIO");
     }
     else
     {
     printf("flag_press:%d,code_index:%d\n",flag_press,code_index);
     if(flag_press)
     {
     if(code_index==7)//shift
     {
     flag_press_shift=1;
     }
     if(code_index==3||code_index==9)//fn
     {
     flag_press_fn=1;
     }
     if(code_index==10)//CapsLock
     {
     flag_press_caps_lock=(flag_press_caps_lock==1)?0:1;
     }
     if(flag_press_shift==1)
     {
     key_code=tca8418_keyboard_lvgl_shift[code_index-1];
     }
     else
     {
       char ch=tca8418_keyboard_lvgl[code_index-1];
       if((flag_press_caps_lock==1)&&(ch <= 'z') && (ch >= 'a'))
       {
              ch = ch + 'A' - 'a';
              key_code=ch;           
       }
       else
       {
             key_code=tca8418_keyboard_lvgl[code_index-1];
       }
     }
     }
     else//flag_press==0
     {
     
      if(code_index==7)//shift
     {
     flag_press_shift=0;
     }
     if(code_index==3||code_index==9)//fn
     {
     flag_press_fn=0;
     }
     
     
     }
     }
     
     
    }
    //clear int_flag
    reg_data=0x01;
    tca8418_write_reg(fd_keyboard, 0x02,reg_data);//
    }
    
    
    return key_code;

}
 void control_keyboard_backlight(int freq,int duty)//hz  50%
 {
    fpioa_set_function(52,PWM4,-1,-1,-1,-1,-1,-1,-1);
    int fd_pwm = open("/dev/pwm", O_RDWR);
    if (fd_pwm>0)
    {
    pwm_config_t config;
    config.channel = 4;
    config.period = pow(10,9)/freq;//ns  50000
    config.pulse = config.period*duty/100;  //20khz    10%
    ioctl(fd_pwm, KD_PWM_CMD_SET, &config); 
    ioctl(fd_pwm, KD_PWM_CMD_ENABLE,&config); 
    close(fd_pwm);    
    }
 }
 
void control_keyboard_indicator_led_caps_lock(int is_open)
{

    int ret;
    uint8_t reg_data;
    if(fd_keyboard>0)
    {
    reg_data=0x00;
    ret=xl9555_write_reg(fd_keyboard, 0x06,reg_data);//P00-P07   output
    printf("xl9555_write_reg ret=%d\n",ret);
    if(is_open)
    {
    reg_data=0xF7;
    }
    else
    {
    reg_data=0xFF;
    }
    xl9555_write_reg(fd_keyboard, 0x02,reg_data);//P00-P07   output high  low  led light on
    }
}













