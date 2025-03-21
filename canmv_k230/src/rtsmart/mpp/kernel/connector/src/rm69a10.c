/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "connector_dev.h"
#include "io.h"
#include "drv_gpio.h"
#include "k_vo_comm.h"
#include "k_connector_comm.h"

#define DELAY_MS_BACKLIGHT_DEFAULT     200
#define DELAY_MS_BACKLIGHT_FIRST       1

static k_u8 rm69a10_y_mirror = 0;

static k_s32 g_blacklight_delay_ms = DELAY_MS_BACKLIGHT_FIRST;

static void rm69a10_OLD_init(k_u8 test_mode_en)
{
    k_u8 param1[] = {0xB9, 0xFF, 0x83, 0x99};
    k_u8 param21[] = {0xD2, 0xAA};
    k_u8 param2[] = {0xB1, 0x02, 0x04, 0x71, 0x91, 0x01, 0x32, 0x33, 0x11, 0x11, 0xab, 0x4d, 0x56, 0x73, 0x02, 0x02};
    k_u8 param3[] = {0xB2, 0x00, 0x80, 0x80, 0xae, 0x05, 0x07, 0x5a, 0x11, 0x00, 0x00, 0x10, 0x1e, 0x70, 0x03, 0xd4};
    k_u8 param4[] = {0xB4, 0x00, 0xFF, 0x02, 0xC0, 0x02, 0xc0, 0x00, 0x00, 0x08, 0x00, 0x04, 0x06, 0x00, 0x32, 0x04, 0x0a, 0x08, 0x21, 0x03, 0x01, 0x00, 0x0f, 0xb8, 0x8b, 0x02, 0xc0, 0x02, 0xc0, 0x00, 0x00, 0x08, 0x00, 0x04, 0x06, 0x00, 0x32, 0x04, 0x0a, 0x08, 0x01, 0x00, 0x0f, 0xb8, 0x01};
    k_u8 param5[] = {0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x10, 0x04, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x05, 0x05, 0x07, 0x00, 0x00, 0x00, 0x05, 0x40};
    k_u8 param6[] = {0xD5, 0x18, 0x18, 0x19, 0x19, 0x18, 0x18, 0x21, 0x20, 0x01, 0x00, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x2f, 0x2f, 0x30, 0x30, 0x31, 0x31, 0x18, 0x18, 0x18, 0x18};
    k_u8 param7[] = {0xD6, 0x18, 0x18, 0x19, 0x19, 0x40, 0x40, 0x20, 0x21, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x2f, 0x2f, 0x30, 0x30, 0x31, 0x31, 0x40, 0x40, 0x40, 0x40};
    k_u8 param8[] = {0xD8, 0xa2, 0xaa, 0x02, 0xa0, 0xa2, 0xa8, 0x02, 0xa0, 0xb0, 0x00, 0x00, 0x00, 0xb0, 0x00, 0x00, 0x00};
    k_u8 param9[] = {0xBD, 0x01};
    k_u8 param10[] = {0xD8, 0xB0, 0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00, 0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0};
    k_u8 param11[] = {0xBD, 0x02};
    k_u8 param12[] = {0xD8, 0xE2, 0xAA, 0x03, 0xF0, 0xE2, 0xAA, 0x03, 0xF0};
    k_u8 param13[] = {0xBD, 0x00};
    k_u8 param14[] = {0xB6, 0x8D, 0x8D};
    k_u8 param15[] = {0xCC, 0x04}; // 0x05
    k_u8 param16[] = {0xC6, 0xFF, 0xF9};
    k_u8 param22[] = {0xE0, 0x00, 0x12, 0x1f, 0x1a, 0x40, 0x4a, 0x59, 0x55, 0x5e, 0x67, 0x6f, 0x75, 0x7a, 0x82, 0x8b, 0x90, 0x95, 0x9f, 0xa3, 0xad, 0xa2, 0xb2, 0xB6, 0x5e, 0x5a, 0x65, 0x77, 0x00, 0x12, 0x1f, 0x1a, 0x40, 0x4a, 0x59, 0x55, 0x5e, 0x67, 0x6f, 0x75, 0x7a, 0x82, 0x8b, 0x90, 0x95, 0x9f, 0xa3, 0xad, 0xa2, 0xb2, 0xB6, 0x5e, 0x5a, 0x65, 0x77};
    k_u8 param23[] = {0x11};
    k_u8 param24[] = {0x29};
    k_u8 pag20[50] = {0xB2, 0x0b, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77, 0x77};               // 蓝色

    connecter_dsi_send_pkg(param1, sizeof(param1));
    connecter_dsi_send_pkg(param21, sizeof(param21));
    connecter_dsi_send_pkg(param2, sizeof(param2));
    connecter_dsi_send_pkg(param3, sizeof(param3));
    connecter_dsi_send_pkg(param4, sizeof(param4));
    connecter_dsi_send_pkg(param5, sizeof(param5));
    connecter_dsi_send_pkg(param6, sizeof(param6));
    connecter_dsi_send_pkg(param7, sizeof(param7));
    connecter_dsi_send_pkg(param8, sizeof(param8));
    connecter_dsi_send_pkg(param9, sizeof(param9));

    if (test_mode_en == 1)
    {
        connecter_dsi_send_pkg(pag20, 10);                   // test  mode
    }

    connecter_dsi_send_pkg(param10, sizeof(param10));
    connecter_dsi_send_pkg(param11, sizeof(param11));
    connecter_dsi_send_pkg(param12, sizeof(param12));
    connecter_dsi_send_pkg(param13, sizeof(param13));
    connecter_dsi_send_pkg(param14, sizeof(param14));
    if(rm69a10_y_mirror != 0)
        param15[1] = rm69a10_y_mirror;
    connecter_dsi_send_pkg(param15, sizeof(param15));

    connecter_dsi_send_pkg(param16, sizeof(param16));
    connecter_dsi_send_pkg(param22, sizeof(param22));
    connecter_dsi_send_pkg(param23, 1);
    connector_delay_us(300);
    connecter_dsi_send_pkg(param24, 1);
    connector_delay_us(100);
}

static void rm69a10_568x1232_init(k_u8 test_mode_en)
{
k_u32 ret=111;

    rt_kprintf("rm69a10_568x1232_init \n");
  /*  k_u8 param1[]={0x01,0xFE,0xFD};
k_u8 param2[]={0x01,0x80,0xFC};
k_u8 param3[]={0x01,0xFE,0x00};
k_u8 param4[]={0x01,0x2A,0x00,0x00,0x02,0x37};
k_u8 param5[]={0x01,0x2B,0x00,0x00,0x04,0xCF};
k_u8 param6[]={0x01,0x31,0x00,0x03,0x02,0x34};
k_u8 param7[]={0x01,0x30,0x00,0x00,0x04,0xCF};
k_u8 param8[]={0x01,0x12,0x00};
k_u8 param9[]={0x01,0x35,0x00};
k_u8 param10[]={0x01,0x51,0xFE};
k_u8 param11[]={0x01,0x11};
k_u8 param12[]={0x01,0x29};
*/
k_u8 param00[]={0xFF,0x98,0x81,0x00};
k_u8 param01[]={0x11};
k_u8 param1[]={0xFE,0xFD};
k_u8 param2[]={0x80,0xFC};
k_u8 param3[]={0xFE,0x00};
k_u8 param4[]={0x2A,0x00,0x00,0x02,0x37};
k_u8 param5[]={0x2B,0x00,0x00,0x04,0xCF};
k_u8 param6[]={0x31,0x00,0x03,0x02,0x34};
k_u8 param7[]={0x30,0x00,0x00,0x04,0xCF};
k_u8 param8[]={0x12,0x00};
k_u8 param9[]={0x35,0x00};
k_u8 param10[]={0x51,0xFE};
k_u8 param11[]={0x11};
k_u8 param12[]={0x29};

//k_u8 param13[]={0xFF,0x98,0x81x0x00};//madctl
k_u8 param14[]={0x3A,0x77};//colmod





  //connecter_dsi_send_pkg(param00, sizeof(param00));
  //connecter_dsi_send_pkg(param01, sizeof(param01));
  //connector_delay_us(120000);
  ret=connecter_dsi_send_pkg(param1, sizeof(param1));
  rt_kprintf("connecter_dsi_send_pkg: %d \n",ret);
  connecter_dsi_send_pkg(param2, sizeof(param2));
  connecter_dsi_send_pkg(param3, sizeof(param3));
  connecter_dsi_send_pkg(param4, sizeof(param4));
  connecter_dsi_send_pkg(param5, sizeof(param5));
  connecter_dsi_send_pkg(param6, sizeof(param6));
  connecter_dsi_send_pkg(param7, sizeof(param7));
  connecter_dsi_send_pkg(param8, sizeof(param8));
  connecter_dsi_send_pkg(param9, sizeof(param9));
  connecter_dsi_send_pkg(param10, sizeof(param10));
   connector_delay_us(2000);
  connecter_dsi_send_pkg(param11, sizeof(param11));
  connector_delay_us(120000);
  connecter_dsi_send_pkg(param12, sizeof(param12));
  //connector_delay_us(20);
  
  connecter_dsi_send_pkg(param14, sizeof(param14));
  
  ret=222;
  ret=connecter_dsi_read_pkg(0x05);
  rt_kprintf("connecter_dsi_read_pkg(0x05): %d \n",ret);
  ret=333;
  ret=connecter_dsi_read_pkg(0x0C);
  rt_kprintf("connecter_dsi_read_pkg(0x0C): %d \n",ret);
    
    
/*
    k_u8 param1[] = {0x11, 0x00};
    k_u8 param2[] = {0xFF, 0x77,0x01,0x00,0x00,0x10};
    k_u8 param3[] = {0xC0, 0xE9,0x03};                      // (105 + 1) * 8 + 3 x 2 = 854//480X854
    k_u8 param4[] = {0xC1, 0x12,0x02};                      // vbp = 0x12  =18   vfp = 0x2
    k_u8 param5[] = {0xC2, 0x31,0x08};                      // pclk = 512 + (0x8 x 16) = 640
    k_u8 param6[] = {0xCC, 0x10};
    k_u8 param7[] = {0xB0, 0x00,0x0A,0x13,0x0E,0x12,0x07,0x05,0x08,0x08,0x1F,0x07,0x15,0x13,0xE3,0x2A,0x11};
    k_u8 param8[] = {0xB1, 0x00,0x0A,0x12,0x0E,0x12,0x07,0x04,0x07,0x07,0x1E,0x04,0x13,0x10,0x23,0x29,0x11};
    k_u8 param9[] = {0xFF, 0x77,0x01,0x00,0x00,0x11};
    k_u8 param10[] = {0xB0, 0x4D};
    k_u8 param11[] = {0xB1, 0x1C};
    k_u8 param12[] = {0xB2, 0x07};
    k_u8 param13[] = {0xB3, 0x80};
    k_u8 param14[] = {0xB5, 0x47};
    k_u8 param15[] = {0xB7, 0x85}; // 0x05
    k_u8 param16[] = {0xB8, 0x21};

    k_u8 param22[] = {0xB9, 0x10};
    k_u8 param23[] = {0xC1, 0x78};
    k_u8 param24[] = {0xC2, 0x78};
    k_u8 param25[] = {0xD0, 0x88};
    k_u8 param26[] = {0xE0, 0x00,0x00,0x02};
    k_u8 param27[] = {0xE1, 0x0B,0x00,0x0D,0x00,0x0C,0x00,0x0E,0x00,0x00,0x44,0x44};
    k_u8 param28[] = {0xE2, 0x33,0x33,0x44,0x44,0x64,0x00,0x66,0x00,0x65,0x00,0x67,0x00,0x00};
    k_u8 param29[] = {0xE3, 0x00,0x00,0x33,0x33};
    k_u8 param30[] = {0xE4, 0x44,0x44}; // 0x05
    k_u8 param31[] = {0xE5, 0x0C,0x78,0xA0,0xA0,0x0E,0x78,0xA0,0xA0,0x10,0x78,0xA0,0xA0,0x12,0x78,0xA0,0xA0};
    k_u8 param32[] = {0xE6, 0x00,0x00,0x33,0x33};
    k_u8 param33[] = {0xE7, 0x44,0x44};
    k_u8 param34[] = {0xE8, 0x0D,0x78,0xA0,0xA0,0x0F,0x78,0xA0,0xA0,0x11,0x78,0xA0,0xA0,0x13,0x78,0xA0,0xA0};
    k_u8 param35[] = {0xEB, 0x02,0x00,0x39,0x39,0xEE,0x44,0x00};
    k_u8 param36[] = {0xEC, 0x00,0x00};
    k_u8 param37[] = {0xED, 0xFF,0xF1,0x04,0x56,0x72,0x3F,0xFF,0xFF,0xFF,0xFF,0xF3,0x27,0x65,0x40,0x1F,0xFF};
    k_u8 param38[] = {0xFF, 0x77,0x01,0x00,0x00,0x00};
    k_u8 param39[] = {0x29, 0x00};

    connecter_dsi_send_pkg(param1, sizeof(param1));
    usleep(100000);
    connecter_dsi_send_pkg(param2, sizeof(param2));
    connecter_dsi_send_pkg(param3, sizeof(param3));
    connecter_dsi_send_pkg(param4, sizeof(param4));
    connecter_dsi_send_pkg(param5, sizeof(param5));
    connecter_dsi_send_pkg(param6, sizeof(param6));
    connecter_dsi_send_pkg(param7, sizeof(param7));
    connecter_dsi_send_pkg(param8, sizeof(param8));
    connecter_dsi_send_pkg(param9, sizeof(param9));
    connecter_dsi_send_pkg(param10, sizeof(param10));
    connecter_dsi_send_pkg(param11, sizeof(param11));
    connecter_dsi_send_pkg(param12, sizeof(param12));
    connecter_dsi_send_pkg(param13, sizeof(param13));
    connecter_dsi_send_pkg(param14, sizeof(param14));
    connecter_dsi_send_pkg(param15, sizeof(param15));
    connecter_dsi_send_pkg(param16, sizeof(param16));

    connecter_dsi_send_pkg(param22, sizeof(param22));
    connecter_dsi_send_pkg(param23, sizeof(param23));
    connecter_dsi_send_pkg(param24, sizeof(param24));
    connecter_dsi_send_pkg(param25, sizeof(param25));
    usleep(100000);
    connecter_dsi_send_pkg(param26, sizeof(param26));
    connecter_dsi_send_pkg(param27, sizeof(param27));
    connecter_dsi_send_pkg(param28, sizeof(param28));
    connecter_dsi_send_pkg(param29, sizeof(param29));

    connecter_dsi_send_pkg(param30, sizeof(param30));
    connecter_dsi_send_pkg(param31, sizeof(param31));
    connecter_dsi_send_pkg(param32, sizeof(param32));
    connecter_dsi_send_pkg(param33, sizeof(param33));
    connecter_dsi_send_pkg(param34, sizeof(param34));
    connecter_dsi_send_pkg(param35, sizeof(param35));
    connecter_dsi_send_pkg(param36, sizeof(param36));
    connecter_dsi_send_pkg(param37, sizeof(param37));
    connecter_dsi_send_pkg(param38, sizeof(param38));
    connecter_dsi_send_pkg(param39, sizeof(param39));


    connecter_dsi_read_pkg(0x5);
    */
}


static void rm69a10_power_reset(k_s32 on)
{
    rt_kprintf("rm69a10_power_reset \n");
    k_u8 rst_gpio;
    if(0 > (rst_gpio = CONFIG_MPP_DSI_LCD_RESET_PIN)) {
			return;
		}
		 rt_kprintf("rm69a10 rst_gpio is %d \n",rst_gpio);
    kd_pin_mode(rst_gpio, GPIO_DM_OUTPUT);
    
    if (on)
        kd_pin_write(rst_gpio, GPIO_PV_HIGH); // GPIO_PV_LOW  GPIO_PV_HIGH
    else
        kd_pin_write(rst_gpio, GPIO_PV_LOW); // GPIO_PV_LOW  GPIO_PV_HIGH

}

static void rm69a10_set_backlight(k_s32 on)
{
    rt_kprintf("rm69a10_set_backlight \n");
    k_u8 backlight_gpio;
    if(0 > (backlight_gpio = CONFIG_MPP_DSI_LCD_BACKLIGHT_PIN)) {
		   return;
	   }

    kd_pin_mode(backlight_gpio, GPIO_DM_OUTPUT);
    if (on)
        kd_pin_write(backlight_gpio, GPIO_PV_HIGH);
    else
        kd_pin_write(backlight_gpio, GPIO_PV_LOW);

}


static k_s32 rm69a10_power_on(void* ctx, k_s32 on)
{
rt_kprintf("rm69a10_power_on \n");
    k_s32 ret = 0;
    struct connector_driver_dev* dev = ctx;

    if (on) {
        // rst vo;
        k230_display_rst();
        // rst rm69a10 
        rm69a10_power_reset(1);
        rt_thread_mdelay(g_blacklight_delay_ms);
        rm69a10_power_reset(0);
        rt_thread_mdelay(10);
        rm69a10_power_reset(1);
        rt_thread_mdelay(20);
        g_blacklight_delay_ms = DELAY_MS_BACKLIGHT_DEFAULT;
        
        //enable backlight
        rm69a10_set_backlight(1);
    } else {
        rm69a10_set_backlight(0);
    }
    
    return ret;
}


static k_s32 rm69a10_set_phy_freq(k_connectori_phy_attr *phy_attr)
{
rt_kprintf("rm69a10_set_phy_freq \n");
    k_vo_mipi_phy_attr mipi_phy_attr;

    memset(&mipi_phy_attr, 0, sizeof(k_vo_mipi_phy_attr));

    mipi_phy_attr.m = phy_attr->m;
    mipi_phy_attr.n = phy_attr->n;
    mipi_phy_attr.hs_freq = phy_attr->hs_freq;
    mipi_phy_attr.voc = phy_attr->voc;
    mipi_phy_attr.phy_lan_num = K_DSI_2LAN;
    connector_set_phy_freq(&mipi_phy_attr);

    return 0;
}


static k_s32 rm69a10_dsi_resolution_init(k_connector_info *info)
{   rt_kprintf("rm69a10_dsi_resolution_init \n");
    k_vo_dsi_attr attr;
    k_vo_display_resolution resolution;

    memset(&attr, 0, sizeof(k_vo_dsi_attr));
    attr.lan_num = info->lan_num;
    attr.cmd_mode = info->cmd_mode;
    attr.lp_div = 8;
    attr.work_mode = info->work_mode;
    memcpy(&resolution, &info->resolution, sizeof(k_vo_display_resolution));
    memcpy(&attr.resolution, &resolution, sizeof(k_vo_display_resolution));
    connector_set_dsi_attr(&attr);

	if(info->type == RM69A10_MIPI_2LAN_568X1232_60FPS)
	   {
		   if(info->screen_test_mode)
			   rm69a10_568x1232_init(1);
		   else
			   rm69a10_568x1232_init(0);
	   }
    connector_set_dsi_enable(1);

    if(info->dsi_test_mode == 1)
        connector_set_dsi_test_mode();

    return 0;
}


static k_s32 rm69a10_vo_resolution_init(k_vo_display_resolution *resolution, k_u32 bg_color, k_u32 intr_line)
{
rt_kprintf("rm69a10_vo_resolution_init \n");
    k_vo_display_resolution vo_resolution;
    k_vo_pub_attr attr;

    memset(&attr, 0, sizeof(k_vo_pub_attr));
    attr.bg_color = bg_color;
    attr.intf_sync = K_VO_OUT_1080P30;
    attr.intf_type = K_VO_INTF_MIPI;
    attr.sync_info = resolution;

    connector_set_vo_init();
    connector_set_vtth_intr(1, intr_line);
    connector_set_vo_param(&attr);
    connector_set_vo_enable();

    return 0;
}
   

k_s32 rm69a10_init(void *ctx, k_connector_info *info)
{
rt_kprintf("rm69a10_init \n");
    k_s32 ret = 0;
    struct connector_driver_dev* dev = ctx;

    if(info->pixclk_div != 0)
        connector_set_pixclk(info->pixclk_div);

    ret |= rm69a10_set_phy_freq(&info->phy_attr);
    ret |= rm69a10_dsi_resolution_init(info);
    ret |= rm69a10_vo_resolution_init(&info->resolution, info->bg_color, info->intr_line);

    return ret;
}

static k_s32 rm69a10_get_chip_id(void* ctx, k_u32* chip_id)
{
rt_kprintf("rm69a10_get_chip_id \n");
    k_s32 ret = 0;

    return ret;
}


static k_s32 rm69a10_conn_check(void* ctx, k_s32* conn)
{
rt_kprintf("rm69a10_conn_check \n");
    k_s32 ret = 0;

    *conn = 1;

    return ret;
}


static k_s32 rm69a10_set_mirror(void* ctx, k_connector_mirror *mirror)
{
    k_connector_mirror rm69a10_y_mirror;

    rm69a10_y_mirror = *mirror;

    switch(rm69a10_y_mirror)
    {
        case K_CONNECTOR_MIRROR_HOR:
            break;
        case K_CONNECTOR_MIRROR_VER: 
            //rm69a10_y_mirror = 0x09;
            break;
        case K_CONNECTOR_MIRROR_BOTH: 
            break;
        default :
            rt_kprintf("rm69a10_y_mirror(%d) is not support \n", rm69a10_y_mirror);
            break;
    }
    return 0;
}


struct connector_driver_dev rm69a10_connector_drv = {
    .connector_name = "rm69a10",
    .connector_func = {
        .connector_power = rm69a10_power_on,
        .connector_init = rm69a10_init,
        .connector_get_chip_id = rm69a10_get_chip_id,
        .connector_conn_check = rm69a10_conn_check,
        .connector_set_mirror = rm69a10_set_mirror,
    },
};

