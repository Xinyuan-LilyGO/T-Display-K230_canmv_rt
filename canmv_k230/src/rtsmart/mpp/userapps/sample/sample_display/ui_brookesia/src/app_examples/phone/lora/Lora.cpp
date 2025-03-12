#include <math.h>
#include <vector>
#include "Lora.hpp"
#include <unistd.h>
#include "lora_lib/RadioLib.h"

#define RADIO_SCLK_PIN              15
#define RADIO_MISO_PIN              17
#define RADIO_MOSI_PIN              16
#define RADIO_CS_PIN                14


#define RADIO_DIO0_PIN  RADIOLIB_NC
#define RADIO_RST_PIN   5
#define RADIO_DIO1_PIN  RADIOLIB_NC

//#define USING_SX1276
#if defined(USING_SX1276)
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
#endif

#define USING_SX1262
#if defined(USING_SX1262)
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
#endif







int num_counter;
lv_obj_t * label_lora_tx;
lv_obj_t * label_lora_rx;
void Lora::custom_timer_cb_lora_send(lv_timer_t * timer) {
uint32_t * user_data = (uint32_t*)timer->user_data;
//printf("custom_timer_cb_lora_send called with user data: %d\n", *user_data);
// 执行一些LVGL相关的操作
char send_data[20]={0};
printf("--------------------------------------\n");
  num_counter++;
  sprintf(send_data, "qwe789#%d", num_counter);
  #if defined(USING_SX1276)
 int state=radio.startTransmit((uint8_t*)send_data, strlen(send_data));
 if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start  send packet failed!\n");
    }else
    {
    
    printf("[Radio] start send packet!\n");
    }

  usleep(1000000);
  uint16_t irq_state=radio.getIRQFlags();
  printf("irq_state:0x%04X\n",irq_state);
  if(irq_state==0x08)//tx_done
  {
  printf("[Radio] send packet dada:[%s]finished!\n",send_data);
  lv_label_set_text(label_lora_tx, send_data);
  }
  else
  {
  printf("[Radio] send packet......!\n");
  }
  #endif
  
  
  #if defined(USING_SX1262)
  int state=radio.startTransmit((uint8_t*)send_data, strlen(send_data));
 if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start  send packet failed!\n");
    }else
    {
    
    printf("[Radio] start send packet!\n");
    }

  //usleep(1000000);
  usleep(500000);
  uint16_t irq_state=radio.getIrqStatus();
  printf("irq_state:0x%04X\n",irq_state);
  if(irq_state==0x01)//tx_done
  {
  printf("[Radio] send packet dada:[%s]finished!\n",send_data);
  lv_label_set_text(label_lora_tx, send_data);
  }
  else
  {
  printf("[Radio] send packet......!\n");
  }
  
  
  
  
  
  
  
  #endif
  
  
  
  
  
  
  
  
  
  
  
}

void Lora::custom_timer_cb_lora_receive(lv_timer_t * timer) {
uint32_t * user_data = (uint32_t*)timer->user_data;
//printf("custom_timer_cb_lora_receive called with user data: %d\n", *user_data);
// 执行一些LVGL相关的操作
float           radioRSSI   =   0;
float           radioSNR   =   0;
uint8_t recv_data[20]={0};
char msg_transfer[50]={0};
printf("++++++++++++++++++++++++++++++++++++++\n");
#if defined(USING_SX1276)
//printf("[Radio] Starting to listen ... \n");
int state = radio.startReceive(2000,0,0,0);
    if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start Received packet failed!\n");
    }else
    {
    
    printf("[Radio]start Received packet!\n");
    }
usleep(2000000);
  uint16_t irq_state=radio.getIRQFlags();
  printf("irq_state:0x%04X\n",irq_state);
  if(irq_state==0x50)//rx_done|valid_header
  {
   memset(recv_data,0,20);
    int state = radio.readData(recv_data, 20);
        if (state == RADIOLIB_ERR_NONE) {
            // packet was successfully received
            //printf("[Radio] Received packet!\n");
            
            //printf("[Radio] Received packet!data:%s\n",recv_data);
            
            radioSNR=radio.getSNR();
            radioRSSI = radio.getRSSI();
            //printf("[Radio] Received packet,radioRSSI:%f!\n",radioRSSI);
            
            sprintf(msg_transfer, "data:%s,snr:%d,rssi:%d", recv_data,(int)round(radioSNR),(int)round(radioRSSI));
            lv_label_set_text(label_lora_rx, msg_transfer);
            

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            printf("[Radio] CRC error!\n");

        } else {
            // some other error occurred
            printf("[Radio] Failed, code:%d\n",state);
        }

}
else if(irq_state==0x80)
{
printf("rx timeout!\n");
sprintf(msg_transfer, "timeout...");
lv_label_set_text(label_lora_rx, msg_transfer);
}
else
{
printf("...!\n");
sprintf(msg_transfer, "...");
lv_label_set_text(label_lora_rx, msg_transfer);
}
#endif

#if defined(USING_SX1262)

int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start Received packet failed!\n");
    }else
    {
    
    printf("[Radio]start Received packet!\n");
    }
usleep(500000);
 uint16_t irq_state=radio.getIrqStatus();
 printf("irq_state:0x%04X\n",irq_state);
  if(irq_state==0x02||irq_state==0x00)//rx_done|valid_header
  {
   memset(recv_data,0,20);
    int state = radio.readData(recv_data, 20);
        if (state == RADIOLIB_ERR_NONE) {

            printf("[Radio] Received packet!data:%s\n",recv_data);
            
            radioSNR=radio.getSNR();
            radioRSSI = radio.getRSSI(true);
            //printf("[Radio] Received packet,radioRSSI:%f!\n",radioRSSI);
            if(recv_data[0]=='\0')
            {
            sprintf(msg_transfer, "......");
            }
            else
            {
            sprintf(msg_transfer, "data:%s,snr:%d,rssi:%d", recv_data,(int)round(radioSNR),(int)round(radioRSSI));
            }
            lv_label_set_text(label_lora_rx, msg_transfer);
            

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            printf("[Radio] CRC error!\n");

        } else {
            // some other error occurred
            printf("[Radio] Failed, code:%d\n",state);
        }

}
/*else if(irq_state==0x00)
{
memset(recv_data,0,20);
int state = radio.readData(recv_data, 20);
        if (state == RADIOLIB_ERR_NONE) {

            printf("[Radio] Received packet!data:%s\n",recv_data);
            }


printf("rx timeout!\n");
sprintf(msg_transfer, "timeout...");
lv_label_set_text(label_lora_rx, msg_transfer);
}*/
else
{
printf("...!\n");
sprintf(msg_transfer, "...");
lv_label_set_text(label_lora_rx, msg_transfer);
}
#endif







}




using namespace std;

LV_IMG_DECLARE(img_app_lora);

#define KEYBOARD_H_PERCENT      65
#define KEYBOARD_FONT           &lv_font_montserrat_48
#define KEYBOARD_SPECIAL_COLOR  lv_color_make(0, 0x99, 0xff)

#define LABEL_PAD               5
#define LABEL_FONT_SMALL        &lv_font_montserrat_34
#define LABEL_FONT_BIG          &lv_font_montserrat_48
#define LABEL_COLOR             lv_color_make(170, 170, 170)
#define LABEL_FORMULA_LEN_MAX   256

lv_obj_t * btn_send;
lv_obj_t * btn_recv;
lv_timer_t * timer_lora_send=NULL;
lv_timer_t * timer_lora_receive=NULL;
void Lora::btn_clicked_event(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        static uint32_t user_data = 35; 
        if(strcmp(lv_label_get_text(label),"Send")==0)
        {
             //lv_color_t color = lv_obj_get_style_bg_color(btn, LV_BTN_PART_MAIN);
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
        lv_label_set_text(label, "Sending");
        if(timer_lora_send==NULL)
            {
           timer_lora_send = lv_timer_create(custom_timer_cb_lora_send, 500, &user_data);
            }
        lv_timer_set_repeat_count(timer_lora_send, -1); 
        
        
        lv_obj_set_style_bg_color(btn_recv, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
        lv_label_set_text(lv_obj_get_child(btn_recv, 0), "Receive"); 
         if(timer_lora_receive!=NULL)
            {
              lv_timer_set_repeat_count(timer_lora_receive, 0);
              timer_lora_receive=NULL;
              lv_label_set_text(label_lora_rx, "");
            }          
         }
         else if(strcmp(lv_label_get_text(label),"Sending")==0)
         {
           lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
           lv_label_set_text(label, "Send");
           lv_timer_set_repeat_count(timer_lora_send, 0);
           timer_lora_send=NULL;
           lv_label_set_text(label_lora_tx, "");
         
         }
         else if(strcmp(lv_label_get_text(label),"Receive")==0)
         {
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
            lv_label_set_text(label, "Receiving");
            if(timer_lora_receive==NULL)
            {
            timer_lora_receive = lv_timer_create(custom_timer_cb_lora_receive, 500, &user_data);
            }
            lv_timer_set_repeat_count(timer_lora_receive, -1); 
            
            lv_obj_set_style_bg_color(btn_send, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            lv_label_set_text(lv_obj_get_child(btn_send, 0), "Send");
            if(timer_lora_send!=NULL)
            {
              lv_timer_set_repeat_count(timer_lora_send, 0);
              timer_lora_send=NULL;
              lv_label_set_text(label_lora_tx, "");
            }         
                     
         }
          else if(strcmp(lv_label_get_text(label),"Receiving")==0)
         {
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
            lv_label_set_text(label, "Receive");
            lv_timer_set_repeat_count(timer_lora_receive, 0); 
            timer_lora_receive=NULL;
            lv_label_set_text(label_lora_rx, "");
         }
            
            
            
            
            
            
    }
}








Lora::Lora(bool use_status_bar, bool use_navigation_bar):
    ESP_Brookesia_PhoneApp(
        ESP_BROOKESIA_CORE_APP_DATA_DEFAULT("Lora", &img_app_lora, true),
        ESP_BROOKESIA_PHONE_APP_DATA_DEFAULT(&img_app_lora, use_status_bar, use_navigation_bar)
    )
{
}

Lora::~Lora()
{

}

bool Lora::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
    num_counter=0;
  #if defined(USING_SX1276)  
 float freq=915;
 float bw=125;
 uint8_t sf=8; 
 uint8_t cr=8; 
 uint8_t syncWord=0x12; 
 int8_t power=17;
 uint16_t preambleLength=12;
 uint8_t gain=0;

radio.begin(freq,bw,sf,cr,syncWord,power,preambleLength,gain);
if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        printf("Selected CRC is invalid for this module!\n");
        while (true);
    }
  #endif
  
  #if defined(USING_SX1262)  
 float freq=915;
 float bw=125;
 uint8_t sf=8; 
 uint8_t cr=8; 
 uint8_t syncWord=0xAB; 
 int8_t power=22;
 uint16_t preambleLength=15;//12
 //uint8_t gain=0;
 float tcxovoltage=1.6;
 bool useRegulatorLDO=false;

radio.begin(freq,bw,sf,cr,syncWord,power,preambleLength,tcxovoltage,useRegulatorLDO);
if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        printf("Selected CRC is invalid for this module!\n");
        while (true);
    }
  #endif
  
  
       
    
#if 1 
    static lv_coord_t col_dsc[] = {100, 400, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {100, 100, LV_GRID_TEMPLATE_LAST};

    /*Create a container with grid*/
    lv_obj_t * cont = lv_obj_create(lv_scr_act());
    lv_obj_set_style_grid_column_dsc_array(cont, col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(cont, row_dsc, 0);
    lv_obj_set_size(cont, 550, 250);
    lv_obj_center(cont);
    lv_obj_set_layout(cont, LV_LAYOUT_GRID);

lv_obj_t * label = lv_label_create(cont);
        lv_label_set_text(label, "Send:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


 label_lora_tx = lv_label_create(cont);
        lv_label_set_text(label_lora_tx, "");
        lv_obj_center(label_lora_tx);
lv_obj_set_grid_cell(label_lora_tx, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


label = lv_label_create(cont);
        lv_label_set_text(label, "Received:");
        lv_obj_center(label);
lv_obj_set_grid_cell(label, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


 label_lora_rx = lv_label_create(cont);
        lv_label_set_text(label_lora_rx, "");
        lv_obj_center(label_lora_rx);
lv_obj_set_grid_cell(label_lora_rx, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);
  
 /////////////////////////////////////////////////////////////////
  //lv_obj_t * btn1 
  btn_send= lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn_send, btn_clicked_event, LV_EVENT_CLICKED, NULL);
  lv_obj_align(btn_send, LV_ALIGN_CENTER, -100, 160);
  label = lv_label_create(btn_send);
  lv_label_set_text(label, "Send");
  lv_obj_center(label);
  //lv_obj_set_style_bg_color(btn_send, lv_color_hex(0x0000ff),LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn_send,lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN);
  
  //lv_obj_t * btn2 
  btn_recv= lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn_recv, btn_clicked_event, LV_EVENT_CLICKED, NULL);
  lv_obj_align(btn_recv, LV_ALIGN_CENTER, 100, 160);
  label = lv_label_create(btn_recv);
  lv_label_set_text(label, "Receive");
  lv_obj_center(label);
  //lv_obj_set_style_bg_color(btn_recv, lv_color_hex(0x0000ff),LV_PART_MAIN);
  lv_obj_set_style_bg_color(btn_recv, lv_palette_main(LV_PALETTE_BLUE),LV_PART_MAIN);
///////////////////////////////////////////////////////////////////  
   
//static uint32_t user_data = 35; 
//timer_lora_send = lv_timer_create(custom_timer_cb_lora_send, 500, &user_data);
//lv_timer_set_repeat_count(timer_lora_send, -1); 
//timer_lora_receive = lv_timer_create(custom_timer_cb_lora_receive, 500, &user_data);
//lv_timer_set_repeat_count(timer_lora_receive, -1);   
 //lv_timer_set_repeat_count(timer_lora_send, 0);
//lv_timer_set_repeat_count(timer_lora_receive, 0); 
 #endif  
 
 #if 0
  

    
 
 
 
 
 
    lv_obj_t * cont1 = lv_obj_create(lv_scr_act());
    lv_obj_set_flex_flow(cont1, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_size(cont1, lv_pct(40), lv_pct(80));
    lv_obj_add_event_cb(cont1, radio_event_handler, LV_EVENT_CLICKED, &active_index_1);

 
 
 
 
 
 
 
 
 
 
 
 #endif
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
    return true;
}

/**
 * @brief The function will be called when the left button of navigate bar is clicked.
 */
bool Lora::back(void)
{
 notifyCoreClosed();
    return true;
}

/**
 * @brief The function will be called when app should be closed.
 */
bool Lora::close(void)
{
   if(strcmp(lv_label_get_text(lv_obj_get_child(btn_send, 0)), "Sending")==0)
     {
     lv_timer_set_repeat_count(timer_lora_send, 0);
     }
     if(strcmp(lv_label_get_text(lv_obj_get_child(btn_recv, 0)), "Receiving")==0)
     {
     lv_timer_set_repeat_count(timer_lora_receive, 0);
     }  
//lv_timer_set_repeat_count(timer_lora_send, 0);
//lv_timer_set_repeat_count(timer_lora_receive, 0); 
    return true;
}

bool Lora::init(void)
{
    num_counter=0;
    return true;
}


 bool Lora::pause()
{
     //ESP_BROOKESIA_LOGD("Pause");
     if(strcmp(lv_label_get_text(lv_obj_get_child(btn_send, 0)), "Sending")==0)
     {
     lv_timer_pause(timer_lora_send);
     }
     if(strcmp(lv_label_get_text(lv_obj_get_child(btn_recv, 0)), "Receiving")==0)
     {
     lv_timer_pause(timer_lora_receive);
     }
     
     
     
     //lv_timer_pause(timer_lora_send);
     //lv_timer_pause(timer_lora_receive);
     printf("lora pause \n");
     /* Do some operations here if needed */

     return true;
}

bool Lora::resume()
 {
     //ESP_BROOKESIA_LOGD("Resume");
     
      if(strcmp(lv_label_get_text(lv_obj_get_child(btn_send, 0)), "Sending")==0)
     {
     lv_timer_resume(timer_lora_send);
     }
     if(strcmp(lv_label_get_text(lv_obj_get_child(btn_recv, 0)), "Receiving")==0)
     {
     lv_timer_resume(timer_lora_receive);
     }
     
     
     
     
     
     
     //lv_timer_resume(timer_lora_send);
    // lv_timer_resume(timer_lora_receive);
     printf("lora resume \n");
     /* Do some operations here if needed */

     return true;
 }

void Lora::keyboard_event_cb(lv_event_t *e)
{
}
