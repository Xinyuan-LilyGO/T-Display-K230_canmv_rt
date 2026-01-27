#include <signal.h>
#include <math.h>
#include <vector>
#include "Clock.hpp"
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
using namespace std;

LV_IMG_DECLARE(img_app_clock);

LV_IMG_DECLARE(ui_img_clock_hour);
LV_IMG_DECLARE(ui_img_clock_min);
LV_IMG_DECLARE(ui_img_clock_sec);
#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256




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

int fd_hwtimer1=0;
int stopwatch_min,stopwatch_sec,stopwatch_ms;
lv_obj_t * label_stopwatch_min;
lv_obj_t * label_stopwatch_sec;
lv_obj_t * label_stopwatch_ms;
char str_min[5];
char str_sec[5];
char str_ms[5];

lv_obj_t * btn_stopwatch_start_stop;
lv_obj_t * btn_stopwatch_reset;

void Clock::btn_clicked_event(lv_event_t * e)
{

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {    
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        static uint32_t user_data = 35; 
        if(strcmp(lv_label_get_text(label),"Start")==0)
        {
             //lv_color_t color = lv_obj_get_style_bg_color(btn, LV_BTN_PART_MAIN);
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
        lv_label_set_text(label, "Stop");
        
        hwtimerval_t val;
    	val.sec = 0;
        val.usec = 100000;
        write(fd_hwtimer1, &val, sizeof(val)); 
         }
         else if(strcmp(lv_label_get_text(label),"Stop")==0)
         {
           lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
           lv_label_set_text(label, "Start");
           
           ioctl(fd_hwtimer1, HWTIMER_CTRL_STOP, 0);
           
         
         }
         else if(strcmp(lv_label_get_text(label),"Reset")==0)
         { 
           ioctl(fd_hwtimer1, HWTIMER_CTRL_STOP, 0);
           stopwatch_min=0;
           stopwatch_sec=0;
           stopwatch_ms=0;       
           lv_label_set_text(label_stopwatch_min, "00");
           lv_label_set_text(label_stopwatch_sec, "00");
           lv_label_set_text(label_stopwatch_ms, "0");                                     
            lv_obj_set_style_bg_color(btn_stopwatch_start_stop, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            lv_label_set_text(lv_obj_get_child(btn_stopwatch_start_stop, 0), "Start");                               
         }
        
    }
}

void Clock::hwtimer_handler_timeout(int sig)
{
    /* Note: calling printf() from a signal handler is not safe
       (and should not be done in production programs), since
       printf() is not async-signal-safe; see signal-safety(7).
       Nevertheless, we use printf() here as a simple way of
       showing that the handler was called. */

    //printf("Caught2 signal %d\n", sig);
    //hwtimerval_t val;
    //read(fd_hwtimer1, &val, sizeof(val));
    stopwatch_ms++;
    if(stopwatch_ms==10)
    {
       stopwatch_ms=0;
       stopwatch_sec++;
      if(stopwatch_sec==60)
       {
          stopwatch_sec=0;
          stopwatch_min++; 
       }
    
    }
    sprintf(str_min, "%02d", stopwatch_min);
    sprintf(str_sec, "%02d", stopwatch_sec);
    sprintf(str_ms, "%d", stopwatch_ms);
    lv_label_set_text(label_stopwatch_min, str_min);
    lv_label_set_text(label_stopwatch_sec, str_sec);
    lv_label_set_text(label_stopwatch_ms, str_ms);
}

Clock::Clock(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Clock", &img_app_clock, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_clock, use_status_bar, use_navigation_bar)
    )
{
}

Clock::~Clock()
{

}

bool Clock::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
   
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
   
  
    stopwatch_min=0;
    stopwatch_sec=0;
    stopwatch_ms=0;
    
    #if 1       
     static lv_coord_t col_dsc[] = {150,10, 150,10,150, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {80, LV_GRID_TEMPLATE_LAST};

    /*Create a container with grid*/
    lv_obj_t * cont = lv_obj_create(lv_scr_act());
    lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    lv_obj_set_size(cont, 550, 120);
    lv_obj_center(cont);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

    label_stopwatch_min = lv_label_create(cont);
    lv_obj_set_style_text_font(label_stopwatch_min, &lv_font_montserrat_48, 0); 
    lv_label_set_text(label_stopwatch_min, "00");
    lv_obj_center(label_stopwatch_min);
    lv_obj_set_grid_cell(label_stopwatch_min, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);

    lv_obj_t * label_split_symbol_1 = lv_label_create(cont);
    lv_obj_set_style_text_font(label_split_symbol_1, &lv_font_montserrat_48, 0); 
    lv_label_set_text(label_split_symbol_1, ":");
    lv_obj_center(label_split_symbol_1);
    lv_obj_set_grid_cell(label_split_symbol_1, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


    label_stopwatch_sec = lv_label_create(cont);
    lv_obj_set_style_text_font(label_stopwatch_sec, &lv_font_montserrat_48, 0); 
    lv_label_set_text(label_stopwatch_sec, "00");
    lv_obj_center(label_stopwatch_sec);
lv_obj_set_grid_cell(label_stopwatch_sec, LV_GRID_ALIGN_STRETCH, 2, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);
                             
       lv_obj_t * label_split_symbol_2 = lv_label_create(cont);
    lv_obj_set_style_text_font(label_split_symbol_2, &lv_font_montserrat_48, 0); 
    lv_label_set_text(label_split_symbol_2, ".");
    lv_obj_center(label_split_symbol_2);
    lv_obj_set_grid_cell(label_split_symbol_2, LV_GRID_ALIGN_STRETCH, 3, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);                      
                                                                                  
    label_stopwatch_ms = lv_label_create(cont);
    lv_obj_set_style_text_font(label_stopwatch_ms, &lv_font_montserrat_48, 0); 
    lv_label_set_text(label_stopwatch_ms, "0");
    lv_obj_center(label_stopwatch_ms);
lv_obj_set_grid_cell(label_stopwatch_ms, LV_GRID_ALIGN_STRETCH, 4, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1); 
                             
      //lv_obj_t * btn1 
  btn_stopwatch_start_stop= lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn_stopwatch_start_stop, btn_clicked_event, LV_EVENT_CLICKED, NULL);
  lv_obj_align(btn_stopwatch_start_stop, LV_ALIGN_CENTER, -100, 160);
  lv_obj_t * label = lv_label_create(btn_stopwatch_start_stop);
  lv_label_set_text(label, "Start");
  lv_obj_center(label);
  lv_obj_set_style_bg_color(btn_stopwatch_start_stop,lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
  
  //lv_obj_t * btn2 
  btn_stopwatch_reset= lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn_stopwatch_reset, btn_clicked_event, LV_EVENT_CLICKED, NULL);
  lv_obj_align(btn_stopwatch_reset, LV_ALIGN_CENTER, 100, 160);
  label = lv_label_create(btn_stopwatch_reset);
  lv_label_set_text(label, "Reset");
  lv_obj_center(label);
  lv_obj_set_style_bg_color(btn_stopwatch_reset, lv_palette_main(LV_PALETTE_BLUE),LV_PART_MAIN);                        
                             
                             
                             
                             
  #endif  
    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool Clock::back(void)
{
  notifyCoreClosed(); 
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Clock::close(void)
{
    lv_event_send(btn_stopwatch_reset,LV_EVENT_CLICKED,NULL);    
    return true;
}

bool Clock::init(void)
{

    return true;
}


 bool Clock::pause()
{

 printf("clock pause before \n");
 //notifyCoreClosed();
 printf("clock pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Clock::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("clock resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Clock::keyboard_event_cb(lv_event_t *e)
{
}
