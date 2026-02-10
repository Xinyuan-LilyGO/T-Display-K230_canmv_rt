#include <math.h>
#include <vector>
#include "Lora.hpp"
#include <unistd.h>
#include "lora_lib/RadioLib.h"
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
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
#define RADIO_SCLK_PIN              15
#define RADIO_MISO_PIN              17
#define RADIO_MOSI_PIN              16
#define RADIO_CS_PIN                14


//#define RADIO_DIO0_PIN  RADIOLIB_NC
#define RADIO_RST_PIN   5
//#define RADIO_DIO1_PIN  RADIOLIB_NC

//#define USING_SX1276
#if defined(USING_SX1276)
SX1276 radio = new Module(RADIO_CS_PIN, RADIOLIB_NC, RADIO_RST_PIN, RADIOLIB_NC);
#endif

//#define USING_SX1262
#if defined(USING_SX1262)
#define RADIO_DIO1_PIN              RADIOLIB_NC
//#define RADIO_BUSY_PIN              RADIOLIB_NC
#define RADIO_BUSY_PIN              19
SX1262 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif


#define USING_LR2021
#if defined(USING_LR2021)
#define LR20XX_SYSTEM_IRQ_RX_DONE                ( 1 << 18 )  //!< Packet received
#define LR20XX_SYSTEM_IRQ_TX_DONE                ( 1 << 19 )  //!< Packet sent


#define RADIO_DIO1_PIN              RADIOLIB_NC
#define RADIO_BUSY_PIN              RADIOLIB_NC
//#define RADIO_BUSY_PIN              19
LR2021 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
#endif


//#define USING_LR2021_FLRC
#if defined(USING_LR2021_FLRC)
#define LR20XX_SYSTEM_IRQ_RX_DONE                ( 1 << 18 )  //!< Packet received
#define LR20XX_SYSTEM_IRQ_TX_DONE                ( 1 << 19 )  //!< Packet sent


#define RADIO_DIO1_PIN              RADIOLIB_NC
#define RADIO_BUSY_PIN              RADIOLIB_NC
//#define RADIO_BUSY_PIN              19
LR2021 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
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
  //sprintf(send_data, "qwe789ab");
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
  //radio.finishTransmit();(sf>10 tx_done not trigger ,have it receive better than not it receive)
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
  
  #if defined(USING_LR2021)
  //radio.finishTransmit();(sf>10 tx_done not trigger ,have it receive better than not it receive)
  int state=radio.startTransmit((uint8_t*)send_data, strlen(send_data));
 if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start  send packet failed!\n");
    }else
    {
    
    printf("[Radio] start send packet!\n");
    }

  //usleep(1000000);
  usleep(500000);
  uint32_t irq_state=radio.get_clear_IrqStatus();
  printf("get_clear_IrqStatus,irq_state:0x%08X\n",irq_state);  
  if(( irq_state & RADIOLIB_LR20xx_IRQ_TX_DONE ) == RADIOLIB_LR20xx_IRQ_TX_DONE)//tx_done
  {
  printf("[Radio] send packet dada:[%s]finished!\n",send_data);
  lv_label_set_text(label_lora_tx, send_data);
  }
  else
  {
  printf("[Radio] send packet......!\n");
  }
  
  
  
  
  
  
  
  #endif
  
  #if defined(USING_LR2021_FLRC)
  //radio.finishTransmit();(sf>10 tx_done not trigger ,have it receive better than not it receive)
  int state=radio.startTransmit((uint8_t*)send_data, strlen(send_data));
 if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start  send packet failed!\n");
    }else
    {
    
    printf("[Radio] start send packet!\n");
    }
  //usleep(500000);
  uint32_t irq_state=radio.get_clear_IrqStatus();
  printf("get_clear_IrqStatus,irq_state:0x%08X\n",irq_state);  
  if(( irq_state & RADIOLIB_LR20xx_IRQ_TX_DONE ) == RADIOLIB_LR20xx_IRQ_TX_DONE)//tx_done
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

int state = radio.startReceive(640000, RADIOLIB_SX126X_IRQ_RX_DEFAULT, RADIOLIB_SX126X_IRQ_RX_DONE|RADIOLIB_SX126X_IRQ_TIMEOUT|RADIOLIB_SX126X_IRQ_HEADER_VALID, 0);
    if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start Received packet failed!\n");
    }else
    {
    
    printf("[Radio]start Received packet!\n");
    }
 usleep(800000);//500000
 uint16_t irq_state=radio.getIrqStatus();
 printf("irq_state:0x%04X\n",irq_state);
  //if(irq_state==0x02||irq_state==0x00)//rx_done|valid_header
  if(irq_state==0x02)
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
else if(irq_state==0x00)//sending
{


}
else if(irq_state==0x0200)
{
radio.finishTransmit();
   memset(recv_data,0,20);
   int state = radio.readData(recv_data, 20);
        if (state == RADIOLIB_ERR_NONE) {

            printf("[Radio] Received packet!data:%s\n",recv_data);
            }


printf("rx timeout!\n");
radioSNR=radio.getSNR();
radioRSSI = radio.getRSSI(true);
sprintf(msg_transfer, "timeout...,legacy_data:%s,snr:%d,rssi:%d", recv_data,(int)round(radioSNR),(int)round(radioRSSI));
lv_label_set_text(label_lora_rx, msg_transfer);
}
else
{
radio.finishTransmit();
printf("...!\n");
sprintf(msg_transfer, "...,irq_state:0x%04X",irq_state);

lv_label_set_text(label_lora_rx, msg_transfer);
}
#endif

#if defined(USING_LR2021)
int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start Received packet failed!\n");
    }else
    {
    
    printf("[Radio]start Received packet!\n");
    }
 usleep(800000);//500000
 uint32_t irq_state=radio.get_clear_IrqStatus();
 printf("get_clear_IrqStatus,irq_state:0x%08X\n",irq_state);
  if(( irq_state & RADIOLIB_LR20xx_IRQ_RX_DONE ) == RADIOLIB_LR20xx_IRQ_RX_DONE)
  {
    memset(recv_data,0,20);
    int state = radio.readData(recv_data, 20);
        if (state == RADIOLIB_ERR_NONE) {

            printf("[Radio] Received packet!data:%s\n",recv_data);
            
            radioSNR=radio.getSNR();
            radioRSSI = radio.getRSSI();
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
radio.finishTransmit();
}
else
{
radio.finishTransmit();
printf("...!\n");
sprintf(msg_transfer, "...,irq_state:0x%08X",irq_state);

lv_label_set_text(label_lora_rx, msg_transfer);
}
#endif


#if defined(USING_LR2021_FLRC)
int state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        printf("[Radio]start Received packet failed!\n");
    }else
    {
    
    printf("[Radio]start Received packet!\n");
    }
 usleep(800000);//500000
 uint32_t irq_state=radio.get_clear_IrqStatus();
 printf("get_clear_IrqStatus,irq_state:0x%08X\n",irq_state);
 
  if(( irq_state & RADIOLIB_LR20xx_IRQ_RX_DONE ) == RADIOLIB_LR20xx_IRQ_RX_DONE)
  {
    memset(recv_data,0,20);
    int state = radio.readData(recv_data, 20);
        if (state == RADIOLIB_ERR_NONE) {

            printf("[Radio] Received packet!data:%s\n",recv_data);
            
            radioSNR=radio.getSNR();
            radioRSSI = radio.getRSSI();
            printf("[Radio] Received packet,radioRSSI:%f!\n",radioRSSI);
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
radio.finishTransmit();
}
else
{
radio.finishTransmit();
printf("...!\n");
sprintf(msg_transfer, "...,irq_state:0x%08X",irq_state);

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
lv_obj_t * roller_freq;
lv_obj_t * roller_bw;
lv_obj_t * roller_sf;
lv_obj_t * roller_cr;
lv_obj_t * btn_send;
lv_obj_t * btn_recv;
lv_timer_t * timer_lora_send=NULL;
lv_timer_t * timer_lora_receive=NULL;
void Lora::btn_clicked_event(lv_event_t * e)
{

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * btn = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
    
       char roll_selected_buf[10];
       memset(roll_selected_buf,0,10);
       lv_roller_get_selected_str(roller_freq, roll_selected_buf, sizeof(roll_selected_buf));
       float freq= (float)strtol(roll_selected_buf, NULL, 10);
       
       memset(roll_selected_buf,0,10);
       lv_roller_get_selected_str(roller_bw, roll_selected_buf, sizeof(roll_selected_buf));
        char *p = NULL;
       float bw = strtof(roll_selected_buf,&p);
       
       memset(roll_selected_buf,0,10);
       lv_roller_get_selected_str(roller_sf, roll_selected_buf, sizeof(roll_selected_buf));
        uint8_t sf= strtol(roll_selected_buf, NULL, 10); 
       
       memset(roll_selected_buf,0,10);
       lv_roller_get_selected_str(roller_cr, roll_selected_buf, sizeof(roll_selected_buf));
        uint8_t cr= strtol(roll_selected_buf, NULL, 10); 
       
       
#if defined(USING_LR2021)|| defined(USING_SX1262)||defined(USING_SX1276)
printf("freq:%f,bw:%f,sf:%d,cr:%d\n",freq,bw,sf,cr);
/*
 float freq=868;//915
 float bw=125;
 uint8_t sf=7;//10 
 uint8_t cr=5; //8
 uint8_t syncWord=0x12; //0x12
 int8_t power=22;//22
 uint16_t preambleLength=8;//12
 float tcxovoltage=0;//1.6,2.7,0:normal 32m ,other:tcxo
 */
if (radio.setFrequency(freq) != 0) {
        printf("setFrequency[%d] fail\n",freq);
        return;        
    } 
     
    if (radio.setBandwidth(bw) != 0) {
        printf("setBandwidth[%d] fail\n",bw);
        return;
    }  
    if (radio.setSpreadingFactor(sf) != 0) {
        printf("setSpreadingFactor[%d] fail\n",sf);
        return;
    }  
    if (radio.setCodingRate(cr) != 0) {
        printf("setCodingRate[%d] fail\n",cr);
        return;
    }
 #endif
 
        lv_obj_t * label = lv_obj_get_child(btn, 0);
        static uint32_t user_data = 35; 
        if(strcmp(lv_label_get_text(label),"Send")==0)
        {
             //lv_color_t color = lv_obj_get_style_bg_color(btn, LV_BTN_PART_MAIN);
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN);
        lv_label_set_text(label, "Sending");
        if(timer_lora_send==NULL)
            {
           timer_lora_send = lv_timer_create(custom_timer_cb_lora_send, 50, &user_data);//500,1000  50:OK
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
            timer_lora_receive = lv_timer_create(custom_timer_cb_lora_receive, 100, &user_data);//500
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
void Lora::roller_changed_event(lv_event_t * e)
{

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_roller_get_selected_str(obj, buf, sizeof(buf));
        LV_LOG_USER("Selected value: %s", buf);
    }




}
bool Lora::run(void)
{
    lv_area_t area = getVisualArea();
    _width = area.x2 - area.x1;
    _height = area.y2 - area.y1;
    num_counter=0;
    int gpio_fd = open("/dev/gpio", O_RDWR);
    if (gpio_fd>0)
    {
      pin_gpio_t pin_44;
      pin_44.pin = 44;
      ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_44);  //pin44 output
      ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_44);
      //close(gpio_fd);
        
    }
    #if 1 
    static lv_coord_t setting_col_dsc[] = {80,160,80,160, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t setting_row_dsc[] = {80,80,80, LV_GRID_TEMPLATE_LAST};

    /*Create a container with grid*/
    lv_obj_t * setting_cont = lv_obj_create(lv_scr_act());
    lv_obj_set_style_grid_column_dsc_array(setting_cont, setting_col_dsc, 0);
    lv_obj_set_style_grid_row_dsc_array(setting_cont, setting_row_dsc, 0);
    lv_obj_set_size(setting_cont, 550, 300);
    lv_obj_set_pos(setting_cont, 10, 100);
    lv_obj_set_layout(setting_cont, LV_LAYOUT_GRID);

lv_obj_t * label1 = lv_label_create(setting_cont);
        lv_label_set_text(label1, "Freq:");
        lv_obj_center(label1);
lv_obj_set_grid_cell(label1, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);

string freq_arr[5]={"915MHZ","868MHZ","923MHZ","433MHZ","2450MHZ"};
char roller_options[50];
memset(roller_options,0,50);
sprintf(roller_options,"%s\n%s\n%s\n%s\n%s",freq_arr[0].c_str(),freq_arr[1].c_str(),freq_arr[2].c_str(),freq_arr[3].c_str(),freq_arr[4].c_str());
 roller_freq = lv_roller_create(setting_cont);
    lv_roller_set_options(roller_freq,roller_options,LV_ROLLER_MODE_NORMAL);//LV_ROLLER_MODE_NORMAL LV_ROLLER_MODE_INFINITE

    lv_roller_set_visible_row_count(roller_freq, 2);
    lv_obj_center(roller_freq);
    //lv_obj_add_event_cb(roller_freq, event_handler, LV_EVENT_ALL, NULL);
lv_obj_set_grid_cell(roller_freq, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


 label1 = lv_label_create(setting_cont);
        lv_label_set_text(label1, "BW:");
        lv_obj_center(label1);
lv_obj_set_grid_cell(label1, LV_GRID_ALIGN_STRETCH, 2, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);

string bw_arr[4]={"62.5","125","250","500"};
memset(roller_options,0,50);
sprintf(roller_options,"%s\n%s\n%s\n%s",bw_arr[0].c_str(),bw_arr[1].c_str(),bw_arr[2].c_str(),bw_arr[3].c_str());
 roller_bw = lv_roller_create(setting_cont);
    lv_roller_set_options(roller_bw,roller_options,LV_ROLLER_MODE_NORMAL);//LV_ROLLER_MODE_NORMAL LV_ROLLER_MODE_INFINITE

    lv_roller_set_visible_row_count(roller_bw, 2);
    lv_obj_center(roller_bw);
    //lv_obj_add_event_cb(roller_bw, event_handler, LV_EVENT_ALL, NULL);
lv_obj_set_grid_cell(roller_bw, LV_GRID_ALIGN_STRETCH, 3, 1,
                             LV_GRID_ALIGN_STRETCH, 0, 1);


label1 = lv_label_create(setting_cont);
        lv_label_set_text(label1, "SF:");
        lv_obj_center(label1);
lv_obj_set_grid_cell(label1, LV_GRID_ALIGN_STRETCH, 0, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


string sf_arr[8]={"5","6","7","8","9","10","11","12" };
memset(roller_options,0,50);
sprintf(roller_options,"%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s",sf_arr[0].c_str(),sf_arr[1].c_str(),sf_arr[2].c_str(),sf_arr[3].c_str(),sf_arr[4].c_str(),sf_arr[5].c_str(),sf_arr[6].c_str(),sf_arr[7].c_str());
 roller_sf = lv_roller_create(setting_cont);
    lv_roller_set_options(roller_sf,roller_options,LV_ROLLER_MODE_NORMAL);

    lv_roller_set_visible_row_count(roller_sf, 2);
    lv_obj_center(roller_sf);
lv_obj_set_grid_cell(roller_sf, LV_GRID_ALIGN_STRETCH, 1, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);
                             
                             
                             
                             
label1 = lv_label_create(setting_cont);
        lv_label_set_text(label1, "CR:");
        lv_obj_center(label1);
lv_obj_set_grid_cell(label1, LV_GRID_ALIGN_STRETCH, 2, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);


string cr_arr[4]={"5","6","7","8"};
memset(roller_options,0,50);
sprintf(roller_options,"%s\n%s\n%s\n%s",cr_arr[0].c_str(),cr_arr[1].c_str(),cr_arr[2].c_str(),cr_arr[3].c_str());
 roller_cr = lv_roller_create(setting_cont);
    lv_roller_set_options(roller_cr,roller_options,LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(roller_cr, 2);
    lv_obj_center(roller_cr);
lv_obj_set_grid_cell(roller_cr, LV_GRID_ALIGN_STRETCH, 3, 1,
                             LV_GRID_ALIGN_STRETCH, 1, 1);                             
                                                       
  #endif
  
  
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
 float freq=915;//915
 float bw=125;
 uint8_t sf=10; 
 uint8_t cr=8; 
 cr=6;//
 uint8_t syncWord=0x12; //0x12
 int8_t power=22;//22
 uint16_t preambleLength=16;//12
 float tcxovoltage=3.3;//1.6,2.7
 bool useRegulatorLDO=false;

radio.begin(freq,bw,sf,cr,syncWord,power,preambleLength,tcxovoltage,useRegulatorLDO);
 
if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        printf("Selected CRC is invalid for this module!\n");
        while (true);
    }  
//radio.transmitDirect();    
  #endif
  
  #if defined(USING_LR2021)

 float freq=2450;//915  2450
 float bw=500;//125  250
 uint8_t sf=7;//10 
 uint8_t cr=7; //8 7
 uint8_t syncWord=0x12; //0x12
 int8_t power=12;//22
 uint16_t preambleLength=8;//12
 float tcxovoltage=3.0;//1.6,2.7,0:normal 32m ,other:tcxo

 radio.begin(freq,bw,sf,cr,syncWord,power,preambleLength,tcxovoltage);

/*if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        printf("Selected CRC is invalid for this module!\n");
        //while (true);
        return false;
    } */
 /*radio.transmitDirect(freq*1000000);
 while(1)
 {
 printf("test cw...\n");
 }*/
 
 
 
 /*float freq=915;//915
 float bw=125;//125  250
 uint8_t sf=7;//10 
 uint8_t cr=5; //8 7
 uint8_t syncWord=0x12; //0x12
 int8_t power=22;//22
 uint16_t preambleLength=8;//12
 float tcxovoltage=3.0;//1.6,2.7,0:normal 32m ,other:tcxo

 radio.begin(freq,bw,sf,cr,syncWord,power,preambleLength,tcxovoltage);*/
 
 
 
 
 
 
 
  #endif
  
  #if defined(USING_LR2021_FLRC)

 float freq=2450;//915
 float bw=500;
 uint8_t sf=7;//10 
 uint8_t cr=7; //8
 uint8_t syncWord=0x12; //0x12
 int8_t power=12;//22
 uint16_t preambleLength=8;//12
 float tcxovoltage=3.0;//1.6,2.7,0:normal 32m ,other:tcxo
 //int16_t beginFLRC(float freq, uint16_t br_bw, uint8_t cr, int8_t power, uint16_t preambleLength,float tcxoVoltage, uint8_t dataShaping);
//int16_t beginFLRC(uint16_t br, uint8_t cr,uint16_t preambleLength,float tcxoVoltage, uint8_t dataShaping);
 radio.beginFLRC(freq,650,5,power,preambleLength,tcxovoltage,0);
 
/*if (radio.setCRC(false) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
        printf("Selected CRC is invalid for this module!\n");
        //while (true);
        return false;
    } */ 
 /*radio.transmitDirect(freq*1000000);
    while(1) 
 {
 printf("test flrc cw...\n");
 }*/
 
 /*float freq=915;//915
 float bw=125;//125  250
 uint8_t sf=7;//10 
 uint8_t cr=5; //8 7
 uint8_t syncWord=0x12; //0x12
 int8_t power=22;//22
 uint16_t preambleLength=8;//12
 float tcxovoltage=3.0;//1.6,2.7,0:normal 32m ,other:tcxo
radio.beginFLRC(freq,650,5,power,preambleLength,tcxovoltage,0);*/
 
  /*radio.transmitDirect(freq*1000000);
    while(1) 
 {
 printf("test flrc cw...\n");
 }*/
 
 
 
 
 
  #endif
  
  
  
  
  
  
   for(int index=0;index<sizeof(freq_arr)/sizeof(freq_arr[0]);index++)
    {
    float item= (float)strtol(freq_arr[index].c_str(), NULL, 10);
    if(item==freq)
    {
    lv_roller_set_selected(roller_freq, index, LV_ANIM_ON); 
    break;
    }
    }
    char *p = NULL;
     for(int index=0;index<sizeof(bw_arr)/sizeof(bw_arr[0]);index++)
    {
    float item = strtof(bw_arr[index].c_str(),&p);
    if(item==bw)
    {
    lv_roller_set_selected(roller_bw, index, LV_ANIM_ON); 
    break;
    }
    }
    
    for(int index=0;index<sizeof(sf_arr)/sizeof(sf_arr[0]);index++)
    {
    uint8_t item= strtol(sf_arr[index].c_str(), NULL, 10);
    if(item==sf)
    {
    lv_roller_set_selected(roller_sf, index, LV_ANIM_ON); 
    break;
    }
    }
    
    for(int index=0;index<sizeof(cr_arr)/sizeof(cr_arr[0]);index++)
    {
    uint8_t item= strtol(cr_arr[index].c_str(), NULL, 10);
    if(item==cr)
    {
    lv_roller_set_selected(roller_cr, index, LV_ANIM_ON); 
    break;
    }
    }
  
  
  
  
  
  
  
  
  
  #if 0
  lv_obj_t * table = lv_table_create(lv_scr_act());

    /*Fill the first column*/
    lv_table_set_cell_value(table, 0, 0, "Freq");
    lv_table_set_cell_value(table, 1, 0, "SF");
    lv_table_set_cell_value(table, 2, 0, "Sync_Word");

    

    /*Fill the second column*/
    lv_table_set_cell_value(table, 0, 1, "868");
    lv_table_set_cell_value(table, 1, 1, "7");
    lv_table_set_cell_value(table, 2, 1, "0x12");

    
     /*Fill the second column*/
    lv_table_set_cell_value(table, 0, 2, "BW");
    lv_table_set_cell_value(table, 1, 2, "CR");
    lv_table_set_cell_value(table, 2, 2, "Power");
  
    
     /*Fill the second column*/
    lv_table_set_cell_value(table, 0, 3, "125K");
    lv_table_set_cell_value(table, 1, 3, "1");
    lv_table_set_cell_value(table, 2, 3, "22");
    

    /*Set a smaller height to the table. It'll make it scrollable*/
     lv_obj_set_width(table, 550);
    lv_obj_set_height(table, 300);
    lv_obj_set_pos(table, 10, 100);
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
