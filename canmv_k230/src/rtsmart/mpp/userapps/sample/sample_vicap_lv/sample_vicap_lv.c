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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>

#include "k_module.h"
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_vicap_comm.h"
#include "k_video_comm.h"
#include "k_sys_comm.h"
#include "mpi_vb_api.h"
#include "mpi_vicap_api.h"
#include "mpi_isp_api.h"
#include "mpi_sys_api.h"
#include "k_vo_comm.h"
#include "mpi_vo_api.h"
#include "vo_test_case.h"

#include "k_connector_comm.h"
#include "mpi_connector_api.h"

#include "mpi_sensor_api.h"

//#define VICAP_OUTPUT_BUF_NUM 6
//#define VICAP_INPUT_BUF_NUM 4

//#define DISPLAY_WITDH  1088
//#define DISPLAY_HEIGHT 1920
#define LAYER_TEST                              1
// #define OSD_TEST                                1

#if LAYER_TEST

#define PRIVATE_POLL_SZE                        (1920 * 1080 * 3 / 2) + (4096 * 2)
#define PRIVATE_POLL_NUM                        (4)

#else

#define PRIVATE_POLL_SZE                        (320 * 240 * 4)
#define PRIVATE_POLL_NUM                        (4)

#endif




void vo_display_init(void)
{
// rst display subsystem
    kd_display_reset();
    // set hardware reset;
    kd_display_set_backlight();
    k_connector_type connector_type=RM69A10_MIPI_2LAN_568X1232_60FPS;
	 
    k_s32 connector_fd;
    //k_u32 chip_id = 0x00;
    k_connector_info connector_info;
    memset(&connector_info, 0, sizeof(k_connector_info));

    //connector get sensor info
    k_u32 ret = kd_mpi_get_connector_info(connector_type, &connector_info);
    if (ret) {
        printf("sample_vicap, the sensor type not supported!\n");     
        return;
    }

    connector_fd = kd_mpi_connector_open(connector_info.connector_name);
    if (connector_fd < 0) {
        printf("%s, connector open failed.\n", __func__);
        return;
    }

    // set connect power
    kd_mpi_connector_power_set(connector_fd, 1);
    // set connect get id
    //kd_mpi_connector_id_get(connector_fd, &chip_id);
    // connector init
    kd_mpi_connector_init(connector_fd, connector_info);


}
void vi_config_manage()
{
    vo_display_init();
//k_vicap_vo_layer_conf layer_conf;
  // memset(&layer_conf, 0 , sizeof(k_vicap_vo_layer_conf));

   //k_video_frame_info dump_info;

   k_vicap_dev dev_num=0;
    //k_bool dev_enable;
   k_vicap_sensor_type sensor_type=GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
   //vicap get sensor info
   k_vicap_sensor_info sensor_info;
   k_u32 ret = kd_mpi_vicap_get_sensor_info(sensor_type, &sensor_info);
		if (ret) {
				  printf("sample_vicap, the sensor type not supported!\n");
				  return;
			  }

        printf("kd_mpi_vicap_set_dev_attr before\n");
	k_vicap_dev_attr dev_attr;		   
	memset(&dev_attr, 0, sizeof(k_vicap_dev_attr));	
	dev_attr.input_type = VICAP_INPUT_TYPE_SENSOR;
	memcpy(&dev_attr.sensor_info, &sensor_info, sizeof(k_vicap_sensor_info));
	//vicap device attr set
	dev_attr.acq_win.h_start = 0;
	dev_attr.acq_win.v_start = 0;
	dev_attr.acq_win.width = 1920;
	dev_attr.acq_win.height = 1080;	  
	dev_attr.mode = VICAP_WORK_ONLINE_MODE;

	k_u32 pipe_ctrl = 0xFFFFFFFF;
	dev_attr.pipe_ctrl.data = pipe_ctrl;
	
	dev_attr.pipe_ctrl.bits.af_enable = 0;
	dev_attr.pipe_ctrl.bits.ae_enable = 0;
	dev_attr.pipe_ctrl.bits.awb_enable = 0;
	dev_attr.pipe_ctrl.bits.dnr3_enable = 0;
	dev_attr.pipe_ctrl.bits.ahdr_enable = 0;
	//dev_attr.pipe_ctrl.bits.af_enable = 0;
	//dev_attr.pipe_ctrl.bits.ae_enable = device_obj[dev_num].ae_enable;
	//dev_attr.pipe_ctrl.bits.awb_enable = device_obj[dev_num].awb_enable;
	//dev_attr.pipe_ctrl.bits.dnr3_enable = device_obj[dev_num].dnr3_enable;
	//dev_attr.pipe_ctrl.bits.ahdr_enable = device_obj[dev_num].hdr_enable;
	
	dev_attr.cpature_frame = 0;
	dev_attr.dw_enable = 0;
	dev_attr.mirror = 0;
	ret = kd_mpi_vicap_set_dev_attr(dev_num, dev_attr);
		if (ret) {
		   printf("sample_vicap, kd_mpi_vicap_set_dev_attr failed.\n");
			   return;
		   }

       printf("kd_mpi_vicap_set_dev_attr after\n");
//vo_init

		//vo_display_init();



//vb_init
		k_vb_config config;
		memset(&config, 0, sizeof(config));
		config.max_pool_cnt = 10;
		config.comm_pool[0].blk_cnt = 6;
		config.comm_pool[0].blk_size = PRIVATE_POLL_SZE;		  // osd0 - 3 argb 320 x 240
		config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;//VB_REMAP_MODE_NOCACHE;
			
		kd_mpi_vb_set_config(&config);
		kd_mpi_vb_init();
		
				//set pool_config
		/*k_vb_pool_config pool_config;
		memset(&pool_config, 0, sizeof(pool_config));
		pool_config.blk_cnt = PRIVATE_POLL_NUM;
		pool_config.blk_size = PRIVATE_POLL_SZE;
		pool_config.mode = VB_REMAP_MODE_NONE;
		k_u32 pool_id = kd_mpi_vb_create_pool(&pool_config);		// osd0 - 3 argb 320 x 240
		*/

//vo
   printf("kd_mpi_vicap_set_chn_attr before\n");
		int chn_num=1;
		//kd_mpi_vicap_set_dump_reserved(dev_num, chn_num, K_TRUE);
		k_vicap_chn_attr chn_attr;
		memset(&chn_attr, 0, sizeof(k_vicap_chn_attr));
		chn_attr.out_win.width = 560;
		chn_attr.out_win.height = 320;
        	//chn_attr.crop_win.width = 560; 
		//chn_attr.crop_win.height = 320;
		//chn_attr.crop_win.h_start =0;
		//chn_attr.crop_win.v_start =0;	
		//chn_attr.scale_win = chn_attr.out_win;
		chn_attr.crop_enable = K_FALSE;
		chn_attr.scale_enable = K_FALSE;
		chn_attr.chn_enable = K_TRUE;

		chn_attr.pix_format =PIXEL_FORMAT_YUV_SEMIPLANAR_420;
		//chn_attr.pix_format=PIXEL_FORMAT_ARGB_8888;
		chn_attr.buffer_num = 10;
		chn_attr.buffer_size = PRIVATE_POLL_SZE;
		//chn_attr.fps = device_obj[dev_num].fps[chn_num];
		   ret = kd_mpi_vicap_set_chn_attr(dev_num, chn_num, chn_attr);
		   if (ret) {
				printf("sample_vicap, kd_mpi_vicap_set_chn_attr failed.\n");
					return;
					  }

               printf("kd_mpi_vicap_set_chn_attr after\n");
		   //bind vicap to vo, only support bind two vo chn(K_VO_DISPLAY_CHN_ID1 & K_VO_DISPLAY_CHN_ID2)	
		   printf("kd_mpi_sys_bind before\n");
		k_mpp_chn vicap_mpp_chn, vo_mpp_chn;
		vicap_mpp_chn.mod_id = K_ID_VI;
		vicap_mpp_chn.dev_id = dev_num;
		vicap_mpp_chn.chn_id = chn_num;
		
		vo_mpp_chn.mod_id = K_ID_VO;
		vo_mpp_chn.dev_id = K_VO_DISPLAY_DEV_ID;
		vo_mpp_chn.chn_id = K_VO_DISPLAY_CHN_ID1;
		ret = kd_mpi_sys_bind(&vicap_mpp_chn, &vo_mpp_chn);
		if (ret) {
			printf("kd_mpi_sys_bind failed:0x%x\n", ret);
		}
               printf("kd_mpi_sys_bind after\n");


//set layer
 layer_info layer_msg;
		
		// config lyaer
		   layer_msg.act_size.width = 560;//1080;//640;//1080;
		   layer_msg.act_size.height = 320;//1920;//480;//1920;
		   layer_msg.format = PIXEL_FORMAT_YVU_PLANAR_420;
		   layer_msg.func = 0;
		   layer_msg.global_alptha = 0xff;
		   layer_msg.offset.x = 0;
		   layer_msg.offset.y = 0;
		   layer_msg.attr.out_size.width = 560;//640;
		   layer_msg.attr.out_size.height = 320;//480;


        //set attr
        k_vo_layer chn_id = K_VO_LAYER1;//K_VO_LAYER1;//K_VO_LAYER2;
		k_vo_video_layer_attr attr;
		   memset(&attr, 0, sizeof(attr));
		
		   // set offset
		   attr.display_rect = layer_msg.offset;
		   // set act
		   attr.img_size = layer_msg.act_size;
		   // sget size
		   layer_msg.size = layer_msg.act_size.height * layer_msg.act_size.width * 3 / 2;
		   //set pixel format
		   attr.pixel_format = layer_msg.format;
		   if (layer_msg.format != PIXEL_FORMAT_YVU_PLANAR_420)
		   {
			   printf("input pix format failed \n");
			   return;
		   }
		   // set stride
		   attr.stride = (layer_msg.act_size.width / 8 - 1) + ((layer_msg.act_size.height - 1) << 16);
		   // set function
		   attr.func = layer_msg.func;
		   // set scaler attr
		   attr.scaler_attr = layer_msg.attr;
		
		   // set video layer atrr
		   kd_mpi_vo_set_video_layer_attr(chn_id, &attr);
		
		   // enable layer
		   kd_mpi_vo_enable_video_layer(chn_id);
        













/*
//set osd_msg
		osd_info osd_msg;
		osd_msg.act_size.width = 560 ;
		osd_msg.act_size.height = 320;
		osd_msg.offset.x = 0;
		osd_msg.offset.y = 0;
		osd_msg.global_alptha = 0xff;
		osd_msg.format = PIXEL_FORMAT_ARGB_8888;//PIXEL_FORMAT_ARGB_4444; //PIXEL_FORMAT_ARGB_1555;//PIXEL_FORMAT_ARGB_8888;

		// set attr
		k_vo_video_osd_attr attr;
		attr.global_alptha = osd_msg.global_alptha;
		if (osd_msg.format == PIXEL_FORMAT_ABGR_8888 || osd_msg.format == PIXEL_FORMAT_ARGB_8888)
		  {
			  osd_msg.size = osd_msg.act_size.width  * osd_msg.act_size.height * 4;
			  osd_msg.stride  = osd_msg.act_size.width * 4 / 8;
		  }
		   
		attr.stride = osd_msg.stride;
		attr.pixel_format = osd_msg.format;
		attr.display_rect = osd_msg.offset;
		attr.img_size = osd_msg.act_size;
		
		k_vo_osd osd_vo = K_VO_OSD3;
		osd_vo = K_VO_OSD3;
		kd_mpi_vo_set_video_osd_attr(osd_vo, &attr);
		
		kd_mpi_vo_osd_enable(osd_vo);
	*/	
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		

                 printf("kd_mpi_vicap_init before\n");
		ret = kd_mpi_vicap_init(dev_num);
			  if (ret) {
				  printf("sample_vicap, vicap dev(%d) init failed.\n", dev_num);
				  return;
			  }
			  printf("kd_mpi_vicap_init after\n");
			  printf("sample_vicap, vicap dev(%d) start stream\n", dev_num);
			  
	    ret = kd_mpi_vicap_start_stream(dev_num);
			  if (ret) {
				  printf("sample_vicap, vicap dev(%d) start stream failed.\n", dev_num);
				  return;
			  	}
		  kd_mpi_vo_enable();
}










int main(int argc, char *argv[])
{

	vi_config_manage();

    k_char select = 0;
    while(K_TRUE)
    {
        if(select != '\n')
        {
            printf("---------------------------------------\n");
            printf(" Input character to select test option\n");
            printf("---------------------------------------\n");
            printf(" d: dump data addr test\n");
            printf(" h: dump hdr ddr buffer.\n");
            printf(" s: set isp ae roi test\n");
            printf(" g: get isp ae roi test\n");
            printf(" t: toggle TPG\n");
            printf(" r: dump register config to file.\n");
            printf(" q: to exit\n");
            printf("---------------------------------------\n");
            printf("please Input:\n\n");
        }
        select = (k_char)getchar();
    
        sleep(1);
    }
    printf("Press Enter to exit!!!!\n");
    getchar();

    /*Allow one frame time for the VO to release the VB block*/
    k_u32 display_ms = 1000 / 33;
    usleep(1000 * display_ms);
    return 0;
}

