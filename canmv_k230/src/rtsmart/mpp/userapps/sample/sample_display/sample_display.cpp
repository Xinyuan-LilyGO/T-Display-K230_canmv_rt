#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <stdbool.h>
#include "lvgl/lvgl.h"
#include "port/lv_port_disp.h"
#include "port/lv_port_indev.h"
#include "lvgl/src/hal/lv_hal_tick.h"
//#include "rtdevice.h"
//#include "lvgl/demos/lv_demos.h"
#include "cap_vio.h"
#include "esp_brookesia.hpp"
#include <math.h>
/* These are built-in app examples in `esp-brookesia` library */
#include "app_examples/phone/simple_conf/src/phone_app_simple_conf.hpp"
#include "app_examples/phone/complex_conf/src/phone_app_complex_conf.hpp"
#include "app_examples/phone/squareline/src/phone_app_squareline.hpp"
#include "app_examples/phone/calculator/Calculator.hpp"
#include "app_examples/phone/game_2048/Game_2048.hpp"
#include "app_examples/phone/camera/Camera.hpp"
#include "app_examples/phone/music/Music.hpp"
#include "app_examples/phone/lora/Lora.hpp"
#include "app_examples/phone/hdmi/Hdmi.hpp"
#include "app_examples/phone/setting/Setting.hpp"
#include "app_examples/phone/led/Led.hpp"
#include "app_examples/phone/camera_face_detect/Camera_face_detect.hpp"
#include "app_examples/phone/camera_head_detect/Camera_head_detect.hpp"
#include "app_examples/phone/camera_face_emotion/Camera_face_emotion.hpp"
//#include "app_examples/phone/camera_object_segment/Camera_object_segment.hpp"
#include "app_examples/phone/camera_object_detect/Camera_object_detect.hpp"
#include "app_examples/phone/clock/Clock.hpp"
#include "app_examples/phone/artboard/Artboard.hpp"
#include "app_examples/phone/ble/Ble.hpp"
#include "app_examples/phone/battery/Battery.hpp"
#include "app_examples/phone/video/Video.hpp"
#include "app_examples/phone/live_call/LiveCall.hpp"
#include "app_examples/phone/live_call_test/LiveCallTest.hpp"
#include "app_examples/phone/live_call_send/LiveCallSend.hpp"
#include "app_examples/phone/live_call_recv/LiveCallRecv.hpp"
#include "app_examples/phone/weather/Weather.hpp"
#include "app_examples/phone/keyboard/KeyBoard.hpp"
#include "app_examples/phone/gnss/Gnss.hpp"
#include "app_examples/phone/cellular/Cellular.hpp"
#define USING_BLE
#define USING_WEATHER
#define USING_KEYBOARD
#define USING_BATTERY
#define USING_FAN
#define USING_LTE_GNSS
static int gpio_fd;
static int fd_ts;
#include <fcntl.h>
#include<sys/ioctl.h>
#include "fpioa/rt_fpioa.h"
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


//////////////uart   to use nRF9151 LTE-m
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <poll.h>
using namespace std;
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

//////////////////////////////
static int fd_i2c0=-1;
#if defined(USING_BATTERY)
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
uint8_t bq27220_data[2];
//int bat_percent=0;
#if 1
static void on_clock_update_timer_cb(struct _lv_timer_t *t)
{
    time_t now;
    struct tm timeinfo;
    bool is_time_pm = false;
    ESP_Brookesia_Phone *phone = (ESP_Brookesia_Phone *)t->user_data;

    time(&now);
    //printf("当前系统时间戳：%lld（应与at_ts一致)\n", (long long)now);
    localtime_r(&now, &timeinfo);
    is_time_pm = (timeinfo.tm_hour >= 12);

    /* Since this callback is called from LVGL task, it is safe to operate LVGL */
    // Update clock on "Status Bar"
    ESP_BROOKESIA_CHECK_FALSE_EXIT(
        phone->getHome().getStatusBar()->setClock(timeinfo.tm_hour, timeinfo.tm_min, is_time_pm),
        "Refresh status bar failed"
    );
    //printf("driver->screen_transp:%d\n", lv_disp_get_default()->driver->screen_transp);
    //printf("disp_bg_opa:%d\n", lv_disp_get_default()->bg_opa);
    //printf("+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-\n");
    #if defined(USING_BATTERY)    
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2C,bq27220_data,2);
    int bq27220_soc=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_soc():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soc);
     ESP_BROOKESIA_CHECK_FALSE_EXIT(
        phone->getHome().getStatusBar()->setBatteryPercent(false,bq27220_soc),
        "Refresh status bar failed"
    ); //true
    #endif
    //temperature control fan
    #if defined(USING_FAN)
    //read temperature if temperature>60 open fan else close fan
    uint32_t buffer[1] = {0};
    uint32_t ts_val = 0;
    double code = 0, temp = 0;
    int ret=read(fd_ts,buffer,0);
    if(ret>0)
    {
    ts_val = *(uint32_t *)buffer;
    code = (double)(ts_val & 0xfff);
    temp = (1e-10 * pow(code, 4) * 1.01472 - 1e-6 * pow(code, 3) * 1.10063 + 4.36150 * 1e-3 * pow(code, 2) - 7.10128 * code + 3565.87);
    //printf("ts_val: 0x%x, TS = %lf C\n", ts_val, temp); 
    }
    if (gpio_fd>0)
    {
      pin_gpio_t pin_42;
      pin_42.pin = 42;
      //ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_42);  //pin42 output
      if(temp>50)
      {
      ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_42);
      }
      else if(temp<40)
      {
      ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_42);
      }
      //close(gpio_fd);
        
    }
    #endif 
}
#endif


#if defined(USING_LTE_GNSS)
static void on_clock_ltem_time_cb(struct _lv_timer_t *t)
{
printf("on_clock_ltem_time_cb start\n");
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
    sprintf(msg_send, "AT+CCLK?\r\n");
    write(fd_uart, msg_send, strlen(msg_send));
    int try_cnt=0;
    int ret = 0, cnt = 0; 
    string cclk_arr[3]={""};
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
                  if(strstr(token,"+CCLK:")&&strlen(token)>12)//12
                  { 
                    char *saveptr1;
                    char*token1=strtok_r(token,"\"",&saveptr1);
                    
                    int i=0;
                    printf("************\n");
                     while (token1 != NULL) {
                        printf("%s\n", token1);
                        cclk_arr[i]=token1;
                        token1 = strtok_r(NULL, "\"", &saveptr1);
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
//////////////////////////////////////////
 //fuzhi
                if(cclk_arr[0]!="") 
                {           
    int year_2d, mon, day, hour, min, sec;
    char tz_sign; // 时区符号（+/-）
    int tz_val;   // 时区数值（示例中是32）
    int parsed = sscanf(cclk_arr[1].c_str(), "%d/%d/%d,%d:%d:%d%c%d",
                        &year_2d, &mon, &day, &hour, &min, &sec, &tz_sign, &tz_val);
    struct tm at_time={0};
    at_time.tm_year = year_2d + 100;;
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
  }


}

// 执行完后删除定时器，避免重复触发
    lv_timer_del(t);
printf("on_clock_ltem_time_cb end\n");

}





#endif

#if 0
#include <signal.h>
typedef enum {
    HWTIMER_CTRL_FREQ_SET = 19 * 0x100 + 0x01,           /* set the count frequency */
    HWTIMER_CTRL_STOP = 19 * 0x100 + 0x02,               /* stop timer */
    HWTIMER_CTRL_INFO_GET = 19 * 0x100 + 0x03,           /* get a timer feature information */
    HWTIMER_CTRL_MODE_SET = 19 * 0x100 + 0x04,           /* Setting the timing mode(oneshot/period) */
    HWTIMER_CTRL_IRQ_SET = 19 * 0x100 + 0x10,
} hwtimer_ctrl_t;

typedef struct {
    int32_t sec;      /* second */
    int32_t usec;     /* microsecond */
} hwtimerval_t;

typedef struct {
    uint8_t enable;
    uint8_t signo;
    void *sigval;
} hwtimer_irqcfg_t;

typedef enum {
    HWTIMER_MODE_ONESHOT = 0x01,
    HWTIMER_MODE_PERIOD
} hwtimer_mode_t;

//#define TIMER1_SIG SIGUSR1
//#define TIMER2_SIG SIGUSR2
#define TIMER1_SIG SIGUSR2
#define errExit(msg)        \
    do {                    \
        perror(msg);        \
        exit(EXIT_FAILURE); \
    } while (0)






void hwtimer_handler_timeout(int sig)
{
    /* Note: calling printf() from a signal handler is not safe
       (and should not be done in production programs), since
       printf() is not async-signal-safe; see signal-safety(7).
       Nevertheless, we use printf() here as a simple way of
       showing that the handler was called. */

    printf("Caught2 signal %d\n", sig);
    //hwtimerval_t val;
    //read(fd_hwtimer1, &val, sizeof(val));
}

#endif



/*
static void btn_clicked_event(lv_event_t * e)
{
    //lv_event_code_t code = lv_event_get_code(e);
    //lv_obj_t * label = lv_obj_get_child(btn, 0);
    char *btn_name=lv_event_get_user_data(e);
    printf("btn_name:%s\n",btn_name);
    
}*/
void user_gui_init()
{
 printf("user_gui_init\n");
#if 1
    lv_disp_t*disp=lv_disp_get_default();
 
 /* Create a phone object */
    ESP_Brookesia_Phone *phone = new ESP_Brookesia_Phone(disp);
    ESP_BROOKESIA_CHECK_NULL_EXIT(phone, "Create phone failed");

    /* Try using a stylesheet that corresponds to the resolution */
        ESP_Brookesia_PhoneStylesheet_t *stylesheet = new ESP_Brookesia_PhoneStylesheet_t ESP_BROOKESIA_PHONE_568_1232_DARK_STYLESHEET();
        ESP_BROOKESIA_CHECK_NULL_EXIT(stylesheet, "Create stylesheet failed");
  //      ESP_LOGI(TAG, "Using stylesheet (%s)", stylesheet->core.name);
        ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->addStylesheet(stylesheet), "Add stylesheet failed");
        ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->activateStylesheet(stylesheet), "Activate stylesheet failed");
        delete stylesheet;

   lv_indev_t * indev_touch= lv_indev_get_next(NULL);
    /* Configure and begin the phone */
    ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->setTouchDevice(indev_touch), "Set touch device failed");
    //phone->registerLvLockCallback((ESP_Brookesia_LvLockCallback_t)(bsp_display_lock), 0);
    //phone->registerLvUnlockCallback((ESP_Brookesia_LvUnlockCallback_t)(bsp_display_unlock));
    ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->begin(), "Begin failed");
    // ESP_BROOKESIA_CHECK_FALSE_EXIT(phone->getCoreHome().showContainerBorder(), "Show container border failed");

    /* Install apps */
    /*
    PhoneAppSimpleConf *app_simple_conf = new PhoneAppSimpleConf();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_simple_conf, "Create app simple conf failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_simple_conf) >= 0), "Install app simple conf failed");
    PhoneAppComplexConf *app_complex_conf = new PhoneAppComplexConf();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_complex_conf, "Create app complex conf failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_complex_conf) >= 0), "Install app complex conf failed");
    */
    PhoneAppSquareline *app_squareline = new PhoneAppSquareline();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_squareline, "Create app squareline failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_squareline) >= 0), "Install app squareline failed");
    Calculator *app_calculator = new Calculator();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_calculator, "Create app calculator failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_calculator) >= 0), "Install app calculator failed");
    Game2048 *app_game2048 = new Game2048();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_game2048, "Create app game2048 failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_game2048) >= 0), "Install app game2048 failed");
    Camera *app_camera = new Camera();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera, "Create app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera) >= 0), "Install app camera failed");
#if 1
    Music *app_music = new Music();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_music, "Create app music failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_music) >= 0), "Install app music failed");
    
    Lora *app_lora = new Lora();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_lora, "Create app lora failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_lora) >= 0), "Install app lora failed");

    /*Hdmi *app_hdmi = new Hdmi();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_hdmi, "Create app hdmi failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_hdmi) >= 0), "Install app hdmi failed");
    */
    AppSettings *app_setting = new AppSettings();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_setting, "Create app setting failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_setting) >= 0), "Install app setting failed");
    
    Led *app_led = new Led();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_led, "Create app led failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_led) >= 0), "Install app led failed");

    Camera_face_detect *app_camera_face_detect = new Camera_face_detect();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_face_detect, "camera_face_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_face_detect) >= 0), "Install app camera_face_detect failed");

    /*Camera_head_detect *app_camera_head_detect = new Camera_head_detect();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_head_detect, "camera_head_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_head_detect) >= 0), "Install app camera_head_detect failed");*/
    #if 0
    Camera_face_emotion *app_camera_emotiom_detect = new Camera_face_emotion();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_emotiom_detect, "camera_emotion_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_emotiom_detect) >= 0), "Install app camera_emotion_detect failed");
    #endif
    /*Camera_object_segment *app_camera_segment_detect = new Camera_object_segment();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_segment_detect, "camera_segment_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_segment_detect) >= 0), "Install app camera_segment_detect failed");*///memory limit
    
    
    /*Camera_object_detect *app_camera_object_detect = new Camera_object_detect();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_camera_object_detect, "camera_object_detect app camera failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_camera_object_detect) >= 0), "Install app camera_object_detect failed");*/

    Clock *app_clock = new Clock();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_clock, "Create app clock failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_clock) >= 0), "Install app clock failed");

    Artboard *app_artboard = new Artboard();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_artboard, "Create app artboard failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_artboard) >= 0), "Install app artboard failed");
    #endif
    #if defined(USING_BLE)  
    Ble *app_ble = new Ble();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_ble, "Create app ble failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_ble) >= 0), "Install app ble failed");
    #endif
    #if defined(USING_BATTERY)    
    Battery *app_battery = new Battery();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_battery, "Create app battery failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_battery) >= 0), "Install app battery failed");
    #endif

    Video *app_video = new Video();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_video, "Create app video failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_video) >= 0), "Install app video failed");

    LiveCall *app_live_call = new LiveCall();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_live_call, "Create app live call failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_live_call) >= 0), "Install app live call failed");

/*
    LiveCallTest *app_live_call_test = new LiveCallTest();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_live_call_test, "Create app live call test failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_live_call_test) >= 0), "Install app live call test failed");
    
    
    LiveCallSend *app_live_call_send = new LiveCallSend();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_live_call_send, "Create app live call send failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_live_call_send) >= 0), "Install app live call send failed");
    
    LiveCallRecv *app_live_call_recv = new LiveCallRecv();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_live_call_recv, "Create app live call recv failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_live_call_recv) >= 0), "Install app live call recv failed");
*/
   #if defined(USING_WEATHER)    
    Weather *app_weather = new Weather();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_weather, "Create app weather failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_weather) >= 0), "Install app weather failed");
    #endif
    
    #if defined(USING_KEYBOARD)    
    KeyBoard *app_keyboard = new KeyBoard();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_keyboard, "Create app keyboard failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_keyboard) >= 0), "Install app keyboard failed");
    #endif

   #if defined(USING_LTE_GNSS)
    Gnss *app_gnss = new Gnss();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_gnss, "Create app gnss failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_gnss) >= 0), "Install app gnss failed");

    Cellular *app_cellular = new Cellular();
    ESP_BROOKESIA_CHECK_NULL_EXIT(app_cellular, "Create app cellular failed");
    ESP_BROOKESIA_CHECK_FALSE_EXIT((phone->installApp(app_cellular) >= 0), "Install app cellular failed");
   #endif







//init bq25896 i2c
#if defined(USING_BATTERY)
/////test
    fpioa_set_function(46,IIC4_SCL,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(47,IIC4_SDA,-1,-1,-1,-1,-1,-1,-1);    
    fd_i2c0 = open("/dev/i2c4", O_RDWR);
    if (fd_i2c0> 0)
    {
        uint32_t spped=100000;
     if (ioctl(fd_i2c0, RT_I2C_DEV_CTRL_CLK, &spped) != 0){ //设置速率
        printf("set speed %d failed!\n",spped);
    }
    
    //////////////bq27220  init  modify capacity
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
    //usleep(500000);

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
    cmd_params1[1]=check_sum_fcc;//check_sum
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
    cmd_params1[1]=check_sum_dc;//check_sum
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

 //first get battery soc
 
    memset(bq27220_data,0,2);
    bq27220_read_cmd_data(fd_i2c0,0x2C,bq27220_data,2);
    int bq27220_soc=((bq27220_data[1]<<8)|bq27220_data[0]);
    //printf("bq27220 get_soc():%02x %02x,value:%d%%\n",bq27220_data[0],bq27220_data[1],bq27220_soc);
     ESP_BROOKESIA_CHECK_FALSE_EXIT(
        phone->getHome().getStatusBar()->setBatteryPercent(false,bq27220_soc),
        "Refresh status bar failed"
    );  
    }
   
  printf("++++++++++++++++++++++++++++++++++++++++++++++++\n");  

#endif    
   gpio_fd = open("/dev/gpio", O_RDWR);
    //temperature control fan
    #if defined(USING_FAN)
    
    if (gpio_fd>0)
    {
      pin_gpio_t pin_42;
      pin_42.pin = 42;
      ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_42);  //pin42 output
      ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_42);
      //close(gpio_fd);
        
    }  
    fd_ts = open("/dev/ts", O_RDWR);
    if (fd_ts<=0)
    {
    printf("open /dev/ts fail\n");
    }
    
    #endif 

//nRF9151 enable 3.3v
      fpioa_set_function(2,GPIO2,-1,-1,-1,-1,-1,-1,-1); 
      pin_gpio_t pin_2;
      pin_2.pin = 2;
      ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_2);  //pin2 output
      ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_2);//

//close wifi
        pin_gpio_t pin_45;
	pin_45.pin = 45;
  	ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_45);  //pin45 output
  	ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_45);

    /* Create a timer to update the clock */
    ESP_BROOKESIA_CHECK_NULL_EXIT(lv_timer_create(on_clock_update_timer_cb, 1000, phone), "Create clock update timer failed");
    
    #if defined(USING_LTE_GNSS)
     fpioa_set_function(5,UART2_TXD,-1,-1,-1,-1,-1,-1,-1);
     fpioa_set_function(6,UART2_RXD,-1,-1,-1,-1,-1,-1,-1);    
     lv_timer_create(on_clock_ltem_time_cb, 60 * 1000, NULL);
    #endif
#endif


   
#if 0

  lv_obj_t * btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                          /*Set its size*/
    //lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    lv_obj_t * label = lv_label_create(btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Button");                     /*Set the labels text*/
    lv_obj_center(label);
#endif
#if 0
    lv_obj_t * cont_row = lv_obj_create(lv_screen_active());
    lv_obj_set_size(cont_row, 500, 150);
    lv_obj_align(cont_row, LV_ALIGN_BOTTOM_MID, 0, -20);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);

   
    char str_btn_arr[5][20] = {"Human\nPosture", "Lora", "Music","Face\nDetect","Object\nDetect"};
    uint32_t i;
    for(i = 0; i < 5; i++) {
        lv_obj_t * obj;
        lv_obj_t * label;

        /*Add items to the row*/
        obj = lv_button_create(cont_row);
        lv_obj_set_size(obj, 100, LV_PCT(100));

        label = lv_label_create(obj);
        lv_label_set_text(label, str_btn_arr[i]);
        lv_obj_center(label); 
        lv_obj_add_event_cb(obj, btn_clicked_event, LV_EVENT_CLICKED, str_btn_arr[i]);   
    }
    //show img desktop
#endif  
   
#if 0
static lv_img_dsc_t my_img_dsc = {
    .header.always_zero = 0,
    .header.w = 80,
    .header.h = 60,
    .data_size = 80 * 60 * LV_COLOR_DEPTH / 8,
    .header.cf = LV_IMG_CF_TRUE_COLOR,          /*Set the color format*/
    .data = png_data,
};
#endif
//lv_obj_t * icon_desktop = lv_img_create(lv_scr_act());

/*From variable*/
//lv_img_set_src(icon, &my_icon_dsc);

/*From file*/
//lv_img_set_src(icon_desktop, "S:/sdcard/desktop.png");
//lv_obj_center(icon_desktop); 
   
   #if 0 
    k_vicap_dev dev_num=VICAP_DEV_ID_0;
    k_s32 chn_num=0;
    //k_u32 pool_id;
    k_vicap_sensor_type sensor_type=GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
    vi_sensor_init(sensor_type);
    
    //vio_vb_config(&pool_id);
    k_u32 display_width=560;
    k_u32 display_height=320;
    k_pixel_format pix_format=PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    vi_chn_config(dev_num,chn_num,display_width,display_height,pix_format);
    vi_bind_vo(dev_num, chn_num, K_VO_DISPLAY_CHN_ID1);
    k_vo_layer vo_layer=K_VO_LAYER1;
    //k_pixel_format 
    pix_format=PIXEL_FORMAT_YVU_PLANAR_420;
    vo_layer_config(vo_layer,display_width,display_height,pix_format);
    vio_start_stream(dev_num);
   #endif
   //lv_demos_show_help();
   //char* info[]={"music"};
   //lv_demos_create(info, 1);
   //char* info[]={(char*)"widgets"};
   //lv_demos_create(info, 1);
  
    
    printf("user_gui_init after\n");
    
    
    



}

int main(int argc, char **argv)
{
    printf("hello world ,lvgl version:%d.%d.%d\n",lv_version_major(),lv_version_minor(),lv_version_patch());
    lv_init();
    lv_port_disp_init();
    lv_port_indev_init();
    
    

    #if 0
    lv_style_t style_scr_act;    
    lv_style_init(&style_scr_act);
    lv_style_set_bg_opa(&style_scr_act, LV_OPA_TRANSP);
    lv_obj_add_style(lv_scr_act(), &style_scr_act, 0);
    //lv_obj_report_style_change(&style_scr_act);
 
    lv_disp_get_default()->driver->screen_transp = 1;
    /* 这里设置屏幕背景是透明的 */
    lv_disp_set_bg_opa(lv_disp_get_default(), LV_OPA_TRANSP);
   #endif

//rt_device_t tmr_dev_0 = rt_device_find("hwtimer0");
    user_gui_init();






#if 0
static int fd_hwtimer1=0;
 struct sigaction sa;   
    sa.sa_flags = SA_SIGINFO;
    sa.sa_handler=hwtimer_handler_timeout;
    sigemptyset(&sa.sa_mask);
    if (sigaction(TIMER1_SIG, &sa, NULL) == -1)
        errExit("sigaction");
    if(fd_hwtimer1==0)
    {
        fd_hwtimer1 = open("/dev/hwtimer1", O_RDWR);
	if (fd_hwtimer1 < 0) {
        perror("open /dev/hwtimer1");
        return false;
         }
    }
    hwtimer_irqcfg_t irqcfg;
    irqcfg.enable = 1;
    irqcfg.signo = TIMER1_SIG;
    irqcfg.sigval = (void *)(uint64_t)1;
    ioctl(fd_hwtimer1, HWTIMER_CTRL_IRQ_SET, &irqcfg);
    hwtimer_mode_t mode = HWTIMER_MODE_PERIOD;
    ioctl(fd_hwtimer1, HWTIMER_CTRL_MODE_SET, &mode);
     hwtimerval_t val;
    	val.sec = 0;
        val.usec = 1000000;
        write(fd_hwtimer1, &val, sizeof(val));
#endif




    
    #if 0
     /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello world");
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    #endif
    #if 0
    
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Button");
    lv_obj_center(label);

    lv_obj_t * btn2 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_height(btn2, LV_SIZE_CONTENT);

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Toggle");
    lv_obj_center(label);   
    #endif

uint8_t reg_data;
while(1)
    {
    //LV_LOG_TRACE("This is a trace log");
    //printf("\nlv_timer_handler before\n");
    lv_tick_inc(1);
    //lv_timer_handler();
    lv_task_handler();
    //printf("\nlv_timer_handler after\n");
    usleep(500);//500
    }
	
    return 0;
}



















