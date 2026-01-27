#include <math.h>
#include <vector>
#include "Ble.hpp"
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

LV_IMG_DECLARE(img_app_ble);
#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256
lv_obj_t * ta_ble_send;
#define BUF_SIZE    (1024)
#define UART_DEVICE_NAME    "/dev/uart1"
static char buf_rx[BUF_SIZE];
//static char buf_tx[BUF_SIZE];
//int g_ble_cnt=0;
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
static int msg_curr_index=0;
static int msg_curr_pos=0;
lv_obj_t * ui_SmartGadgetChat_Panel_Chat_container_ble;
lv_obj_t * ui_Label_Small_Label_create(lv_obj_t * comp_parent)
{

    lv_obj_t * cui_Label_Small_Label;
    cui_Label_Small_Label = lv_label_create(comp_parent);
    lv_obj_set_width(cui_Label_Small_Label, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(cui_Label_Small_Label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(cui_Label_Small_Label, 0);
    lv_obj_set_y(cui_Label_Small_Label, 72);
    lv_obj_set_align(cui_Label_Small_Label, LV_ALIGN_TOP_MID);
    lv_label_set_text(cui_Label_Small_Label, "12");
    lv_obj_set_style_text_color(cui_Label_Small_Label, lv_color_hex(0x000746), LV_PART_MAIN);
    lv_obj_set_style_text_opa(cui_Label_Small_Label, 255, LV_PART_MAIN);
    lv_obj_set_style_text_font(cui_Label_Small_Label, LV_FONT_DEFAULT, LV_PART_MAIN);
    return cui_Label_Small_Label;
}

void ui_SmartGadgetChat_init()
{
   lv_obj_t* ui_SmartGadgetChat = lv_obj_create(lv_scr_act());
    lv_obj_clear_flag(ui_SmartGadgetChat, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_width(ui_SmartGadgetChat, 567);
    lv_obj_set_height(ui_SmartGadgetChat,600);
    //lv_obj_set_style_bg_color(ui_SmartGadgetChat, lv_color_hex(0xFF0000),
                              //LV_PART_MAIN);

    ui_SmartGadgetChat_Panel_Chat_container_ble = lv_obj_create(ui_SmartGadgetChat);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_Chat_container_ble, lv_pct(100));
    lv_obj_set_height(ui_SmartGadgetChat_Panel_Chat_container_ble, lv_pct(100));
    lv_obj_set_align(ui_SmartGadgetChat_Panel_Chat_container_ble, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_Chat_container_ble, lv_color_hex(0xFFFFFF),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_Chat_container_ble, 0, LV_PART_MAIN);
    
    
    lv_obj_t* ui_SmartGadgetChat_Small_label_Chat_title = ui_Label_Small_Label_create(ui_SmartGadgetChat);
    lv_obj_set_x(ui_SmartGadgetChat_Small_label_Chat_title, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Small_label_Chat_title, 17);
    lv_obj_set_align(ui_SmartGadgetChat_Small_label_Chat_title, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_SmartGadgetChat_Small_label_Chat_title, "Nordic_UART_Service");
    lv_obj_set_style_text_color(ui_SmartGadgetChat_Small_label_Chat_title, lv_color_hex(0x000746),LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_SmartGadgetChat_Small_label_Chat_title, 255, LV_PART_MAIN);
    
    

}



void ui_SmartGadgetChat_Panel_add(lv_obj_t * obj_container,int msg_index,int flag_send_recv,char*msg)
{

    lv_obj_t* ui_SmartGadgetChat_Panel_C1 = lv_obj_create(obj_container);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_C1, 100);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_C1, lv_pct(100));
    lv_obj_set_x(ui_SmartGadgetChat_Panel_C1, 0);
    msg_curr_pos=msg_index*100+10;
    lv_obj_set_y(ui_SmartGadgetChat_Panel_C1, msg_curr_pos);
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_C1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_C1, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_C1, 0, LV_PART_MAIN );
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_C1, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_C1, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_C1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_C1, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(ui_SmartGadgetChat_Panel_C1, 0,LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Panel_Chat_Panel1 = lv_obj_create(ui_SmartGadgetChat_Panel_C1);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_Chat_Panel1, 169);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_Chat_Panel1, 81);
    if(flag_send_recv==1)
    {
    lv_obj_set_align(ui_SmartGadgetChat_Panel_Chat_Panel1, LV_ALIGN_TOP_RIGHT);
    }
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_Chat_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_SmartGadgetChat_Panel_Chat_Panel1, 12, LV_PART_MAIN);
    if(flag_send_recv==0)
    {
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_Chat_Panel1, lv_color_hex(0x9C9CD9),LV_PART_MAIN);
        }
    else if(flag_send_recv==1)
          {              
              lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_Chat_Panel1, lv_color_hex(0x293062),LV_PART_MAIN);             
            }                                                                                                      
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_Chat_Panel1, 255, LV_PART_MAIN);
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Small_label_Chat1 = ui_Label_Small_Label_create(ui_SmartGadgetChat_Panel_Chat_Panel1);
    lv_obj_set_width(ui_SmartGadgetChat_Small_label_Chat1, lv_pct(100));
    lv_obj_set_height(ui_SmartGadgetChat_Small_label_Chat1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SmartGadgetChat_Small_label_Chat1, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Small_label_Chat1, 0);
    lv_obj_set_align(ui_SmartGadgetChat_Small_label_Chat1, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(ui_SmartGadgetChat_Small_label_Chat1, msg);
    lv_obj_set_style_text_color(ui_SmartGadgetChat_Small_label_Chat1, lv_color_hex(0xFFFFFF),
                                LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_SmartGadgetChat_Small_label_Chat1, 255, LV_PART_MAIN);






}
void *  Ble::thread_uart_rx_entry(void *parameter)
{
    int fd;
    struct pollfd fds[1];
    int ret = 0, cnt = 0; 

    printf(" [app] open uart1.....\n");
    fd = open(UART_DEVICE_NAME, O_RDWR);
    if (fd < 0)
    {
        printf("open dev uart1 failed!\n");
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
        printf("uart1 ioctl failed!\n");
    }

    fds[0].fd = fd;
    fds[0].events = POLLIN;
    printf(" [app] ........... poll \n");
    flag_running=1;
    msg_curr_index=0;
    msg_curr_pos=0;
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
            /*if (cnt >= BUF_SIZE)
            {
             ret = 0;
             cnt = 0;
            }*/
            if(ret > 0)
            {
                
            	//flag = 1;
            	ret = 0;
                printf("[app] read %d byte\n", cnt);
                ui_SmartGadgetChat_Panel_add(ui_SmartGadgetChat_Panel_Chat_container_ble,msg_curr_index++,0,buf_rx);
                cnt = 0;
            
            }
            
            pthread_mutex_unlock(&mutex_uart_rx); 
            pthread_cond_signal(&cond);
        }
    }


    //close(fd);
    return NULL;
}


void Ble::ta_event_cb(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    lv_obj_t * kb = (lv_obj_t *)lv_event_get_user_data(e);
    if(code == LV_EVENT_FOCUSED) {
        lv_keyboard_set_textarea(kb, ta);
        lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }

    if(code == LV_EVENT_DEFOCUSED) {
        lv_keyboard_set_textarea(kb, NULL);
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
    
    if(code == LV_EVENT_READY) {
        char* msg_send=(char*)lv_textarea_get_text(ta_ble_send);
        if(strlen(msg_send)>0)
        {
        ui_SmartGadgetChat_Panel_add(ui_SmartGadgetChat_Panel_Chat_container_ble,msg_curr_index++,1,msg_send);
        write(fd_uart, msg_send, strlen(msg_send));
        }
    }
    
}

Ble::Ble(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Ble", &img_app_ble, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_ble, use_status_bar, use_navigation_bar)
    )
{
}

Ble::~Ble()
{

}

void Ble::btn_clicked_event(lv_event_t * e)
{

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {    
         char* msg_send=(char*)lv_textarea_get_text(ta_ble_send);
        if(strlen(msg_send)>0)
        {
        ui_SmartGadgetChat_Panel_add(ui_SmartGadgetChat_Panel_Chat_container_ble,msg_curr_index++,1,msg_send);
        write(fd_uart, msg_send, strlen(msg_send));
        }
    }
 }
bool Ble::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1; 
    ui_SmartGadgetChat_init();
    
    
    
     /*Create a keyboard to use it with an of the text areas*/
    lv_obj_t * kb = lv_keyboard_create(lv_scr_act());
    lv_obj_set_size(kb, lv_pct(100),350);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -50);
    
   
    
    /*lv_obj_set_width(ui_KeyboardScreenSettingVerification, 558);
    lv_obj_set_height(ui_KeyboardScreenSettingVerification, 350);
    lv_obj_set_x(ui_KeyboardScreenSettingVerification, -1);
    lv_obj_set_y(ui_KeyboardScreenSettingVerification, 320);//120
    lv_obj_set_align(ui_KeyboardScreenSettingVerification, LV_ALIGN_CENTER);*/
    
    
    
    

    /*Create a text area. The keyboard will write here*/
    
    ta_ble_send = lv_textarea_create(lv_scr_act());
    lv_obj_align(ta_ble_send, LV_ALIGN_CENTER, -50, 50);
    lv_obj_add_event_cb(ta_ble_send, ta_event_cb, LV_EVENT_ALL, kb);
    lv_textarea_set_placeholder_text(ta_ble_send, "send msg");
    lv_obj_set_size(ta_ble_send, 400, 50);   
    lv_keyboard_set_textarea(kb, ta_ble_send);
    
  lv_obj_t * btn_ble_send= lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn_ble_send, btn_clicked_event, LV_EVENT_CLICKED, NULL);
  lv_obj_align(btn_ble_send, LV_ALIGN_CENTER, 200, 50);
  lv_obj_t * label = lv_label_create(btn_ble_send);
  lv_label_set_text(label, "Send");
  lv_obj_center(label);
  lv_obj_set_style_bg_color(btn_ble_send,lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
    
    
    
    
    
    
    
    
    
    
    
    /*label_ble_rx = lv_label_create(lv_scr_act());
    lv_label_set_text(label_ble_rx, "");
    lv_obj_align(label_ble_rx, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_text_font(label_ble_rx, LV_FONT_DEFAULT, 0);
    */
    //char msg_transfer[50]={0};
     //sprintf(msg_transfer, "data:14,snr:47,rssi:%d",10);
    
    
    //ui_SmartGadgetChat_Panel_add(ui_SmartGadgetChat_Panel_Chat_container_ble,msg_curr_index++,0,msg_transfer);
    //ui_SmartGadgetChat_Panel_add(ui_SmartGadgetChat_Panel_Chat_container_ble,msg_curr_index++,0,msg_transfer);
    //ui_SmartGadgetChat_Panel_add(ui_SmartGadgetChat_Panel_Chat_container_ble,msg_curr_index++,0,msg_transfer);
    
    #if 0
    lv_obj_t* ui_SmartGadgetChat = lv_obj_create(lv_scr_act());
    lv_obj_clear_flag(ui_SmartGadgetChat, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    //lv_obj_set_style_bg_img_src(ui_SmartGadgetChat, &ui_img_pattern_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    //lv_obj_set_style_bg_img_tiled(ui_SmartGadgetChat, true, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_width(ui_SmartGadgetChat, 567);
    lv_obj_set_height(ui_SmartGadgetChat,300);
    //lv_obj_set_style_bg_color(ui_SmartGadgetChat, lv_color_hex(0xFF0000),
                              //LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Panel_Chat_container = lv_obj_create(ui_SmartGadgetChat);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_Chat_container, lv_pct(100));
    lv_obj_set_height(ui_SmartGadgetChat_Panel_Chat_container, lv_pct(100));
    lv_obj_set_align(ui_SmartGadgetChat_Panel_Chat_container, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_Chat_container, lv_color_hex(0xFFFFFF),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_Chat_container, 0, LV_PART_MAIN);
    /*
    lv_obj_t* ui_SmartGadgetChat_Panel_C1 = lv_obj_create(ui_SmartGadgetChat_Panel_Chat_container);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_C1, 100);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_C1, lv_pct(100));
    lv_obj_set_x(ui_SmartGadgetChat_Panel_C1, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Panel_C1, 60);
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_C1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_C1, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_C1, 0, LV_PART_MAIN );
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_C1, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_C1, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_C1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_C1, 0, LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Panel_Chat_Panel1 = lv_obj_create(ui_SmartGadgetChat_Panel_C1);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_Chat_Panel1, 169);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_Chat_Panel1, 81);
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_Chat_Panel1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_SmartGadgetChat_Panel_Chat_Panel1, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_Chat_Panel1, lv_color_hex(0x9C9CD9),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_Chat_Panel1, 255, LV_PART_MAIN);
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_Chat_Panel1, 8, LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Small_label_Chat1 = ui_Label_Small_Label_create(ui_SmartGadgetChat_Panel_Chat_Panel1);
    lv_obj_set_width(ui_SmartGadgetChat_Small_label_Chat1, lv_pct(100));
    lv_obj_set_height(ui_SmartGadgetChat_Small_label_Chat1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SmartGadgetChat_Small_label_Chat1, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Small_label_Chat1, 0);
    lv_obj_set_align(ui_SmartGadgetChat_Small_label_Chat1, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(ui_SmartGadgetChat_Small_label_Chat1, "Let's get some dinner, how about pizza?");
    lv_obj_set_style_text_color(ui_SmartGadgetChat_Small_label_Chat1, lv_color_hex(0xFFFFFF),
                                LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_SmartGadgetChat_Small_label_Chat1, 255, LV_PART_MAIN);

    
    lv_obj_t* ui_SmartGadgetChat_Panel_C2 = lv_obj_create(ui_SmartGadgetChat_Panel_Chat_container);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_C2, 100);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_C2, lv_pct(100));
    lv_obj_set_x(ui_SmartGadgetChat_Panel_C2, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Panel_C2, 160);
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_C2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_C2, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_C2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_C2, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_C2, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_C2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_C2, 0, LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Panel_Chat_Panel2 = lv_obj_create(ui_SmartGadgetChat_Panel_C2);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_Chat_Panel2, 169);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_Chat_Panel2, 81);
    lv_obj_set_align(ui_SmartGadgetChat_Panel_Chat_Panel2, LV_ALIGN_TOP_RIGHT);
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_Chat_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_SmartGadgetChat_Panel_Chat_Panel2, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_Chat_Panel2, lv_color_hex(0x293062),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_Chat_Panel2, 255, LV_PART_MAIN);
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_Chat_Panel2, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_Chat_Panel2, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_Chat_Panel2, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_Chat_Panel2, 8, LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Small_label_Chat2 = ui_Label_Small_Label_create(ui_SmartGadgetChat_Panel_Chat_Panel2);
    lv_obj_set_width(ui_SmartGadgetChat_Small_label_Chat2, lv_pct(100));
    lv_obj_set_height(ui_SmartGadgetChat_Small_label_Chat2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SmartGadgetChat_Small_label_Chat2, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Small_label_Chat2, 0);
    lv_obj_set_align(ui_SmartGadgetChat_Small_label_Chat2, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(ui_SmartGadgetChat_Small_label_Chat2, "Sounds good! What about James?");
    lv_obj_set_style_text_color(ui_SmartGadgetChat_Small_label_Chat2, lv_color_hex(0xFFFFFF),
                                LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_SmartGadgetChat_Small_label_Chat2, 255, LV_PART_MAIN);


    lv_obj_t* ui_SmartGadgetChat_Panel_C3 = lv_obj_create(ui_SmartGadgetChat_Panel_Chat_container);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_C3, 100);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_C3, lv_pct(100));
    lv_obj_set_x(ui_SmartGadgetChat_Panel_C3, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Panel_C3, 270);
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_C3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_C3, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_C3, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_C3, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_C3, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_C3, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_C3, 0, LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Panel_Chat_Panel3 = lv_obj_create(ui_SmartGadgetChat_Panel_C3);
    lv_obj_set_width(ui_SmartGadgetChat_Panel_Chat_Panel3, 169);
    lv_obj_set_height(ui_SmartGadgetChat_Panel_Chat_Panel3, 81);
    lv_obj_clear_flag(ui_SmartGadgetChat_Panel_Chat_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_SmartGadgetChat_Panel_Chat_Panel3, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_SmartGadgetChat_Panel_Chat_Panel3, lv_color_hex(0x9C9CD9),
                              LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_SmartGadgetChat_Panel_Chat_Panel3, 255, LV_PART_MAIN);
    lv_obj_set_style_pad_left(ui_SmartGadgetChat_Panel_Chat_Panel3, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_right(ui_SmartGadgetChat_Panel_Chat_Panel3, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_top(ui_SmartGadgetChat_Panel_Chat_Panel3, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(ui_SmartGadgetChat_Panel_Chat_Panel3, 8, LV_PART_MAIN);

    lv_obj_t* ui_SmartGadgetChat_Small_label_Chat3 = ui_Label_Small_Label_create(ui_SmartGadgetChat_Panel_Chat_Panel3);
    lv_obj_set_width(ui_SmartGadgetChat_Small_label_Chat3, lv_pct(100));
    lv_obj_set_height(ui_SmartGadgetChat_Small_label_Chat3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SmartGadgetChat_Small_label_Chat3, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Small_label_Chat3, 0);
    lv_obj_set_align(ui_SmartGadgetChat_Small_label_Chat3, LV_ALIGN_TOP_LEFT);
    lv_label_set_text(ui_SmartGadgetChat_Small_label_Chat3, "He likes it too! Where do we meet?");
    lv_obj_set_style_text_color(ui_SmartGadgetChat_Small_label_Chat3, lv_color_hex(0xFFFFFF),
                                LV_PART_MAIN);
    lv_obj_set_style_text_opa(ui_SmartGadgetChat_Small_label_Chat3, 255, LV_PART_MAIN);
    
    */
    
    

















































    /*ui_SmartGadgetChat_Image_Chat_Icon3 = lv_img_create(ui_SmartGadgetChat_Panel_C3);
    lv_img_set_src(ui_SmartGadgetChat_Image_Chat_Icon3, &ui_img_chatbox_png);
    lv_obj_set_width(ui_SmartGadgetChat_Image_Chat_Icon3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_SmartGadgetChat_Image_Chat_Icon3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_SmartGadgetChat_Image_Chat_Icon3, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Image_Chat_Icon3, 80);
    lv_obj_add_flag(ui_SmartGadgetChat_Image_Chat_Icon3, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_SmartGadgetChat_Image_Chat_Icon3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_img_recolor(ui_SmartGadgetChat_Image_Chat_Icon3, lv_color_hex(0x9C9CD9),
                                 LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_img_recolor_opa(ui_SmartGadgetChat_Image_Chat_Icon3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    */

    /*ui_SmartGadgetChat_Scrolldots_Scrolldots2 = ui_Panel_Scrolldots_create(ui_SmartGadgetChat);
    lv_obj_set_x(ui_SmartGadgetChat_Scrolldots_Scrolldots2, 0);
    lv_obj_set_y(ui_SmartGadgetChat_Scrolldots_Scrolldots2, -8);

    lv_obj_set_width(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D1), 4);
    lv_obj_set_height(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D1), 4);

    lv_obj_set_x(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D2), 10);
    lv_obj_set_y(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D2), 0);

    lv_obj_set_width(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D3), 8);
    lv_obj_set_height(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D3), 8);
    lv_obj_set_x(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D3), 21);
    lv_obj_set_y(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D3), 0);
    lv_obj_set_style_bg_color(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2,
                                                UI_COMP_PANEL_SCROLLDOTS_PANEL_D3), lv_color_hex(0x101C52), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_comp_get_child(ui_SmartGadgetChat_Scrolldots_Scrolldots2, UI_COMP_PANEL_SCROLLDOTS_PANEL_D3),
                            255, LV_PART_MAIN | LV_STATE_DEFAULT);
                            */
    #endif
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    fpioa_set_function(3,UART1_TXD,-1,-1,-1,-1,-1,-1,-1);
    fpioa_set_function(4,UART1_RXD,-1,-1,-1,-1,-1,-1,-1);
        
    //flag = 0;
    pthread_mutex_init(&mutex_uart_rx, NULL);
    pthread_cond_init(&cond, NULL);
    pthread_create(&pthread_handle_uart_rx, NULL, thread_uart_rx_entry, NULL);


    /*while(1)
    {
        if (!pthread_cond_wait(&cond, &mutex_uart_rx))
        {
            if (flag)
            {
                lv_label_set_text(label_ble_rx, buf_rx);
                flag = 0;
            }

            pthread_mutex_unlock(&mutex_uart_rx); 
        }
    }
	*/

    //pthread_join(pthread_handle_uart_rx, NULL);

    //pthread_mutex_destroy(&mutex_uart_rx);

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
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
bool Ble::back(void)
{
  notifyCoreClosed(); 
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Ble::close(void)
{ 
    flag_running=0;
    //pthread_join(pthread_handle_uart_rx, NULL);
    //pthread_mutex_destroy(&mutex_uart_rx);
    return true;
}

bool Ble::init(void)
{

    return true;
}


 bool Ble::pause()
{

 printf("ble pause before \n");
 //notifyCoreClosed();
 printf("ble pause after \n");
     //ESP_BROOKESIA_LOGD("Pause");

     /* Do some operations here if needed */

     return true;
}

bool Ble::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
      printf("ble resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Ble::keyboard_event_cb(lv_event_t *e)
{
}
