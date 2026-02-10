/**
 * @file lv_port_indev_templ.c
 *
 */

/*Copy this file as "lv_port_indev.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_indev.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include "sys/ioctl.h"
#include <stdio.h>
#include "stdio.h"
#include "../keyboard_ext.h"
//static int touch_flag=0;
#define	GPIO_WRITE_LOW           _IOW('G', 4, int)
#define	GPIO_WRITE_HIGH          _IOW('G', 5, int)

#define KD_GPIO_SET_MODE _IOW('G', 20, int)
#define KD_GPIO_GET_MODE _IOWR('G', 21, int)
#define KD_GPIO_SET_VALUE _IOW('G', 22, int)
#define KD_GPIO_GET_VALUE _IOWR('G', 23, int)
#define KD_GPIO_SET_IRQ _IOW('G', 24, int)
#define KD_GPIO_GET_IRQ _IOWR('G', 25, int)

typedef enum _gpio_pin_edge {
    GPIO_PE_RISING,
    GPIO_PE_FALLING,
    GPIO_PE_BOTH,
    GPIO_PE_HIGH,
    GPIO_PE_LOW,
} gpio_pin_edge_t;

typedef enum _gpio_drive_mode {
    GPIO_DM_OUTPUT,
    GPIO_DM_INPUT,
    GPIO_DM_INPUT_PULL_UP,
    GPIO_DM_INPUT_PULL_DOWN,
} gpio_drive_mode_t;

typedef enum _gpio_pin_value {
    GPIO_PV_LOW,
    GPIO_PV_HIGH
} gpio_pin_value_t;

typedef struct {
    uint16_t pin;
    uint16_t value;
} gpio_cfg_t;

typedef struct {
    uint16_t pin;
    uint8_t enable;
    uint8_t mode;
    uint16_t debounce;
    uint8_t signo;
    void* sigval;
} gpio_irqcfg_t;

#define KEY1_SIG SIGUSR1
#define KEY2_SIG SIGUSR2

#define errExit(msg)        \
    do {                    \
        perror(msg);        \
        exit(EXIT_FAILURE); \
    } while (0)
#if 0
static void touch_irq_handler(int sig)
{
    /* Note: calling printf() from a signal handler is not safe
       (and should not be done in production programs), since
       printf() is not async-signal-safe; see signal-safety(7).
       Nevertheless, we use printf() here as a simple way of
       showing that the handler was called. */
    touch_flag=1;
    //printf("Caught2 signal %d\n", sig);
    printf("touched...\n");
}
#endif
/*static const uint32_t tca8418_keyboard_lvgl[] =
    {
        LV_KEY_RIGHT,LV_KEY_LEFT,0x8E,0x00,' ',LV_KEY_NEXT,0x90,0x8F,0x8E,0x8B,
	  0xA2,LV_KEY_DOWN,'m',' ','v','c','x','z',0x8C,'q',
	  LV_KEY_ENTER,LV_KEY_UP,0x8D,'N','B','F','D','S','A',0x00,
	   0x00,'l','k','j','h','g','r','e','w',LV_KEY_ESC,
	   LV_KEY_BACKSPACE,'p','o','i','u','y','t','2','1',0x81,
	   '0','9','8','7','6','5','4','3',0x83,0x82,
	   0x91,0x8A,0x89,0x88,0x87,0x86,0x85,0x84
	   };
	   */
/*********************
 *      DEFINES
 *********************/
int fd_touch;
struct rt_touch_data {
    uint8_t event; /* The touch event of the data */
    uint8_t track_id; /* Track id of point */
    uint8_t width; /* Point of width */
    uint16_t x_coordinate; /* Point of x coordinate */
    uint16_t y_coordinate; /* Point of y coordinate */
    uint32_t timestamp; /* The timestamp when the data was received */
};
extern int flag_press_caps_lock;
extern int flag_press_fn;
/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void touchpad_init(void);
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static bool touchpad_is_pressed(void);
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y);


static void keypad_init(void);
static void keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
static uint32_t keypad_get_key(void);
/**********************
 *  STATIC VARIABLES
 **********************/
lv_indev_t * indev_touchpad;
lv_indev_t * indev_keypad;
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_indev_init(void)
{
    /**
     * Here you will find example implementation of input devices supported by LittelvGL:
     *  - Touchpad
     *  - Mouse (with cursor support)
     *  - Keypad (supports GUI usage only with key)
     *  - Encoder (supports GUI usage only with: left, right, push)
     *  - Button (external buttons to press points on the screen)
     *
     *  The `..._read()` function are only examples.
     *  You should shape them according to your hardware
     */

    static lv_indev_drv_t indev_drv;
    static lv_indev_drv_t indev_drv2;
    
    /*------------------
     * Keypad
     * -----------------*/

    /*Initialize your keypad or keyboard if you have*/
    keypad_init();
    
    /*Register a keypad input device*/
    lv_indev_drv_init(&indev_drv2);
    indev_drv2.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv2.read_cb = keypad_read;
    indev_keypad = lv_indev_drv_register(&indev_drv2);
    
    /*------------------
     * Touchpad
     * -----------------*/

    /*Initialize your touchpad if you have*/
    touchpad_init();

    /*Register a touchpad input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touchpad_read;
    indev_touchpad = lv_indev_drv_register(&indev_drv);
    
     
 
     lv_group_t * group = lv_group_create();
     //lv_group_add_obj(group, lv_scr_act()); 
     lv_group_set_default(group);	
     lv_indev_set_group(indev_keypad, group);
    
    
    
    
    
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*------------------
 * Touchpad
 * -----------------*/

/*Initialize your touchpad*/
static void touchpad_init(void)
{
    /*Your code comes here*/
    int index=1;
    char dev_name[16] = "/dev/touch0";
    dev_name[10] = '0' + index;
    fd_touch = open(dev_name, O_RDWR);
    
    #if 0
    //interrupt
    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_handler=touch_irq_handler;
    sigemptyset(&sa.sa_mask);
    if (sigaction(KEY1_SIG, &sa, NULL) == -1)
    {
       printf("sigaction KEY1_SIG fail ...\n");
        errExit("sigaction"); 
        }
    int fd_irq = open("/dev/gpio", O_RDWR);
    gpio_cfg_t cfg;
    cfg.pin = 23;//29
    cfg.value = GPIO_DM_INPUT;
    ioctl(fd_irq, KD_GPIO_SET_MODE, &cfg);
    gpio_irqcfg_t irqcfg;
    irqcfg.pin = 23;//29
    irqcfg.enable = 1;
    irqcfg.mode = GPIO_PE_RISING;
    irqcfg.debounce = 60000;//200
    irqcfg.signo = KEY1_SIG;
    irqcfg.sigval = (void *)(uint64_t)irqcfg.pin;
    ioctl(fd_irq, KD_GPIO_SET_IRQ, &irqcfg);
    #endif
    
    
    
    
}

/*Will be called by the library to read the touchpad*/
static void touchpad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static lv_coord_t last_x = 0;
    static lv_coord_t last_y = 0;

    /*Save the pressed coordinates and the state*/
    if(touchpad_is_pressed()) {
        touchpad_get_xy(&last_x, &last_y);
        data->state = LV_INDEV_STATE_PR;
    }
    else {
        data->state = LV_INDEV_STATE_REL;
    }

    /*Set the last pressed coordinates*/
    data->point.x = last_x;
    data->point.y = last_y;
}

/*Return true is the touchpad is pressed*/
static bool touchpad_is_pressed(void)
{
    /*Your code comes here*/
     #if 0
    if(touch_flag==1)
    { touch_flag=0;    
    return true;
    }
    else
    {
    return false;
    }
    #endif
    #if 1
     int point_number =1;
    struct rt_touch_data touch_data[point_number];
    point_number = read(fd_touch, touch_data, sizeof(touch_data));
    point_number /= sizeof(struct rt_touch_data);
    if(point_number>0)
    {
    return true;
    }
    else
    {
    return false;
    }
    #endif
}

/*Get the x and y coordinates if the touchpad is pressed*/
static void touchpad_get_xy(lv_coord_t * x, lv_coord_t * y)
{
    /*Your code comes here*/

    int point_number =1;
 struct rt_touch_data touch_data[point_number];
    point_number = read(fd_touch, touch_data, sizeof(touch_data));
    point_number /= sizeof(struct rt_touch_data);
  if(point_number)
  {
  (*x)=touch_data[0].x_coordinate;
  (*y)=touch_data[0].y_coordinate;
  }
}
#if 1

/*------------------
 * Keypad
 * -----------------*/

/*Initialize your keypad*/
static void keypad_init(void)
{
    /*Your code comes here*/
    tca8418_init(); 
    control_keyboard_backlight(20000,5);
}

/*Will be called by the library to read the mouse*/
static void keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint32_t last_key = 0;

    /*Get the current x and y coordinates*/
    //mouse_get_xy(&data->point.x, &data->point.y);
     touchpad_get_xy(&data->point.x, &data->point.y);
    /*Get whether the a key is pressed and save the pressed key*/
    uint32_t act_key =keypad_get_key();
    if(act_key != 0) {
        data->state = LV_INDEV_STATE_PR;

        /*Translate the keys to LVGL control characters according to your key definitions*/
        switch(act_key) {
            case 1:
                act_key = LV_KEY_NEXT;
                break;
            case 2:
                act_key = LV_KEY_PREV;
                break;
            case 3:
                act_key = LV_KEY_LEFT;
                break;
            case 4:
                act_key = LV_KEY_RIGHT;
                break;
            case 5:
                act_key = LV_KEY_ENTER;
                break;
            case 0x83:
            	break;
            case 0x84:
            	if(flag_press_fn==1)
            	{
                 control_keyboard_backlight(20000,0);
                 flag_press_fn=0;
                }
            	break;
            case 0x85:
                if(flag_press_fn==1)
            	{
                 control_keyboard_backlight(20000,5);
                 flag_press_fn=0;
                }
            	break;
            case 0x86:
            	break;     
            case 0x8B:
                control_keyboard_indicator_led_caps_lock(flag_press_caps_lock);
                break;
            default:
                break;
            
        }

        last_key = act_key;
    }
    else {
        data->state = LV_INDEV_STATE_REL;
    }

    data->key = last_key;
}

/*Get the currently being pressed key.  0 if no key is pressed*/
static uint32_t keypad_get_key(void)
{
    /*Your code comes here*/   
return tca8418_get_key();
    //return 0;
}

#endif
#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
