#include "cap_vio.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <stdbool.h>
#include "k_module.h"
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_vicap_comm.h"
#include "k_video_comm.h"
#include "k_sys_comm.h"
#include "mpi_vb_api.h"
#include "mpi_vicap_api.h"
#include "mpi_isp_api.h"
#include "mpi_vvi_api.h"
#include "mpi_sys_api.h"
#include "k_video_comm.h"
#include "mpi_vo_api.h"
#include "k_vo_comm.h"
#include "k_connector_comm.h"
#include "mpi_connector_api.h"

#include "mpi_sensor_api.h"
#define LAYER_TEST                              1
// #define OSD_TEST                                1

#if LAYER_TEST

#define PRIVATE_POLL_SZE                        (1920 * 1080 * 3 / 2) + (4096 * 2)
#define PRIVATE_POLL_NUM                        (4)

#else

#define PRIVATE_POLL_SZE                        (320 * 240 * 4)
#define PRIVATE_POLL_NUM                        (4)

#endif

#define SENSOR_CHANNEL (3)     // 通道数
#define SENSOR_HEIGHT (720)    // sensor ch1输出高度，AI输入
#define SENSOR_WIDTH (1280)    // sensor ch1输出宽度，AI输入
#define ISP_CHN0_WIDTH  (1920) // sensor ch0输出宽度，vo
#define ISP_CHN0_HEIGHT (1080) // sensor ch0输出高度，vo
// extern k_video_frame_info vf_info;
 //extern k_vo_osd osd_vo;

 k_u8 *pic_addr=NULL;
 k_video_frame_info vf_info;
 k_vo_osd osd_vo;

void vo_display_init(k_connector_type connector_type)
{
// rst display subsystem
    kd_display_reset();
    // set hardware reset;
    kd_display_set_backlight();
	 
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
    kd_mpi_connector_power_set(connector_fd, K_TRUE);
    // set connect get id
    //kd_mpi_connector_id_get(connector_fd, &chip_id);
    // connector init
    kd_mpi_connector_init(connector_fd, connector_info);


}


void vio_vb_config(k_u32* pool_id)
{

//set config
		k_vb_config config;
		memset(&config, 0, sizeof(config));
		config.max_pool_cnt = 64;//10
		config.comm_pool[0].blk_cnt = 36;//6
		//config.comm_pool[0].blk_size = PRIVATE_POLL_SZE;		  // osd0 - 3 argb 320 x 240
		config.comm_pool[0].blk_size = VICAP_ALIGN_UP((ISP_CHN0_WIDTH * ISP_CHN0_HEIGHT * 3 / 2), VICAP_ALIGN_1K);
		config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;//VB_REMAP_MODE_NOCACHE;
	
		config.comm_pool[1].blk_cnt = 6;//6
		config.comm_pool[1].blk_size = VICAP_ALIGN_UP((SENSOR_HEIGHT * SENSOR_WIDTH * 3 ), VICAP_ALIGN_1K);
		config.comm_pool[1].mode = VB_REMAP_MODE_NOCACHE;//VB_REMAP_MODE_NOCACHE;
	
	
	
		kd_mpi_vb_set_config(&config);
		kd_mpi_vb_init();

		//set pool_config
		k_vb_pool_config pool_config;
		memset(&pool_config, 0, sizeof(pool_config));
		pool_config.blk_cnt = PRIVATE_POLL_NUM;
		pool_config.blk_size = PRIVATE_POLL_SZE;
		pool_config.mode = VB_REMAP_MODE_NONE;
		*pool_id = kd_mpi_vb_create_pool(&pool_config);		// osd0 - 3 argb 320 x 240
}
void vo_layer_config(k_vo_layer vo_layer,k_u32 display_width, k_u32 display_height,k_pixel_format pix_format)
{

//set layer
 layer_info layer_msg;
		
		// config lyaer
		   layer_msg.act_size.width = display_width;//1080;//640;//1080;
		   layer_msg.act_size.height = display_height;//1920;//480;//1920;
		   layer_msg.format = pix_format;
		   layer_msg.func = K_ROTATION_270;//K_ROTATION_90
		   layer_msg.global_alptha = 0xff;
		   layer_msg.offset.x = 0;
		   layer_msg.offset.y = 0;
		   //layer_msg.attr.out_size.width = display_width;//640;
		   //layer_msg.attr.out_size.height = display_height;//480;


        //set attr
               //k_vo_layer chn_id = K_VO_LAYER1;//K_VO_LAYER1;//K_VO_LAYER2;
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
		   //attr.scaler_attr = layer_msg.attr;
		
		   // set video layer atrr
		   kd_mpi_vo_set_video_layer_attr(vo_layer, &attr);
		
		   // enable layer
		   kd_mpi_vo_enable_video_layer(vo_layer);
}

void vo_osd_config(k_vo_osd osd_vo,osd_info* osd_msg,k_u32 display_width, k_u32 display_height,k_pixel_format pix_format)
{
//config osd
        //set osd_msg
		//osd_info osd_msg;
		osd_msg->act_size.width = display_width ;
		osd_msg->act_size.height = display_height;
		osd_msg->offset.x = 0;
		osd_msg->offset.y = 0;
		osd_msg->global_alptha = 0xff;
		osd_msg->format = pix_format;

		// set attr
		k_vo_video_osd_attr attr;
		attr.global_alptha = osd_msg->global_alptha;
		if (osd_msg->format == PIXEL_FORMAT_ABGR_8888 || osd_msg->format == PIXEL_FORMAT_ARGB_8888)
		  {
			  osd_msg->size = osd_msg->act_size.width  * osd_msg->act_size.height * 4;
			  osd_msg->stride  = osd_msg->act_size.width * 4 / 8;
		  }
		   else if (osd_msg->format == PIXEL_FORMAT_RGB_565 || osd_msg->format == PIXEL_FORMAT_BGR_565)
                 {
                        osd_msg->size = osd_msg->act_size.width  * osd_msg->act_size.height * 2;
                        osd_msg->stride  = osd_msg->act_size.width * 2 / 8;
                }
               else if (osd_msg->format == PIXEL_FORMAT_RGB_888 || osd_msg->format == PIXEL_FORMAT_BGR_888)
                 {
                        osd_msg->size = osd_msg->act_size.width  * osd_msg->act_size.height * 3;
                        osd_msg->stride  = osd_msg->act_size.width * 3 / 8;
                }
		   
		attr.stride = osd_msg->stride;
		attr.pixel_format = osd_msg->format;
		attr.display_rect = osd_msg->offset;
		attr.img_size = osd_msg->act_size;

		//osd_vo = K_VO_OSD3;
		kd_mpi_vo_set_video_osd_attr(osd_vo, &attr);
		kd_mpi_vo_osd_enable(osd_vo);
}

/* 
 */
void vo_config_manage(k_u32 pool_id,k_video_frame_info* vf_info,k_u8 **pic_addr_cur,osd_info osd_msg)
{

		// set frame
		
		//k_video_frame_info vf_info;
		//memset(&vf_info, 0, sizeof(vf_info));
		vf_info->v_frame.width = osd_msg.act_size.width;
		vf_info->v_frame.height = osd_msg.act_size.height;
		vf_info->v_frame.stride[0] = osd_msg.act_size.width;
		vf_info->v_frame.pixel_format = osd_msg.format;
		k_s32 size = 0;
		if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ABGR_8888 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_8888)
			{
				size = vf_info->v_frame.height * vf_info->v_frame.width * 4;
		    }
		     else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_565 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_BGR_565)
		     {
                              size = vf_info->v_frame.height * vf_info->v_frame.width * 2;
                   }
                  else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_888 || vf_info->v_frame.pixel_format == PIXEL_FORMAT_BGR_888)
                  {
                          size = vf_info->v_frame.height * vf_info->v_frame.width * 3;
                  }
		else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_PLANAR_420)
			   {
			   size = vf_info->v_frame.height * vf_info->v_frame.width * 3 / 2;
		   	   }
		
		size = size + 4096;		   // 强制4K ，后边得删了
		
		printf("vb block size is %x \n", size);
		k_vb_blk_handle block_handle;
		block_handle = kd_mpi_vb_get_block(pool_id, size, NULL);
		if (block_handle == VB_INVALID_HANDLE)
			{
			   printf("%s get vb block error\n", __func__);
			   //return K_FAILED;
			   return;
			}
		k_u64 phys_addr = kd_mpi_vb_handle_to_phyaddr(block_handle);
		if (phys_addr == 0)
		   {
			   printf("%s get phys addr error\n", __func__);
			   //return K_FAILED;
			   return;
		   }
		k_u8 *virt_addr;
		virt_addr = (k_u8 *)kd_mpi_sys_mmap(phys_addr, size);
		   // virt_addr = (k_u32 *)kd_mpi_sys_mmap_cached(phys_addr, size);
		printf("vo_config_manage,virt_addr is %p \n", virt_addr);
		if (virt_addr == NULL)
		  {
			   printf("%s mmap error\n", __func__);
			   //return K_FAILED;
			   return;
		  }
		
		vf_info->mod_id = K_ID_VO;
		vf_info->pool_id = pool_id;
		vf_info->v_frame.phys_addr[0] = phys_addr;
		if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_YVU_PLANAR_420)
			{ 
			vf_info->v_frame.phys_addr[1] = phys_addr + (vf_info->v_frame.height * vf_info->v_frame.stride[0]);
			}
		 printf("phys_addr is %lx \n", phys_addr);
		 *pic_addr_cur =virt_addr;
}

void vo_draw_color(k_vo_osd osd_vo_cur,k_video_frame_info* vf_info,k_u8 **pic_addr_cur,vo_area area, void * _px_map_color)
{
    		 		 			 
			 if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_ARGB_8888)
			 {
			     k_u8* px_map_color=(k_u8*)_px_map_color;
				int i=0;
				for(int y = area.y1; y <= area.y2; y++) 
				{
            			     for(int x = area.x1; x <= area.x2; x++) 
            			     {
            			        i=y*vf_info->v_frame.width+x;
            			        k_u8 val_correct=px_map_color[0];
            			        px_map_color[0]=px_map_color[3];
            			        px_map_color[3]=val_correct;            			        
            			        val_correct=px_map_color[1];
            			        px_map_color[1]=px_map_color[2];
            			        px_map_color[2]=val_correct;
            			        
            		                memcpy(((k_u8*)(*pic_addr_cur)+(i*4)),px_map_color,4);
            		                memcpy(pic_addr+(i*4),px_map_color,4);            			       
            				px_map_color=px_map_color+4;	
            			     }
        			} 	 
			 }
			  else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_888)
			 {
			    k_u8* px_map_color=(k_u8*)_px_map_color;
			 
				int i=0;
				for(int y = area.y1; y <= area.y2; y++) 
				{
            			     for(int x = area.x1; x <= area.x2; x++) 
            			     {
            			        i=y*vf_info->v_frame.width+x;
            			         /*struct {
                                            uint8_t blue;
                                            uint8_t green;
                                            uint8_t red;
                                            uint8_t alpha;
                                           } ch;*/
            			        
            			        k_u8 val_correct=px_map_color[0];
            			        px_map_color[0]=px_map_color[2];
            			        px_map_color[2]=val_correct;
            			        memcpy(((k_u8*)(*pic_addr_cur)+(i*3)),px_map_color,3);
            		                memcpy(pic_addr+(i*3),px_map_color,3);            			       
            				px_map_color=px_map_color+4;
		
            			     }
        			} 	 
			 }
			 else if (vf_info->v_frame.pixel_format == PIXEL_FORMAT_RGB_565)
			 {			      
			      k_u8* px_map_color=(k_u8*)_px_map_color;
				int i=0;
				for(int y = area.y1; y <= area.y2; y++) 
				{
            			     for(int x = area.x1; x <= area.x2; x++) 
            			     {
            			        i=y*vf_info->v_frame.width+x;
            			        /*struct {
                                       #if LV_COLOR_16_SWAP == 0
                                           uint16_t blue : 5;
                                           uint16_t green : 6;
                                           uint16_t red : 5;
                                           }*/
            			        k_u8 val_correct=px_map_color[0];
            			        px_map_color[0]=(px_map_color[1]<<3)|(px_map_color[0]&0x07);
            			        px_map_color[1]=(val_correct>>3)|(px_map_color[1]&0xE0);
            			        
            			        memcpy(((k_u8*)(*pic_addr_cur)+(i*2)),px_map_color,2);
            		                memcpy(pic_addr+(i*2),px_map_color,2);            			       
            				px_map_color=px_map_color+2;	
            			     }
        			} 	 
			 }
			 
			 
			 
			 
			 
			 
			 
			 
			 
			 
			 
		kd_mpi_vo_chn_insert_frame(osd_vo_cur+3, vf_info);	//K_VO_OSD0

}

void vi_sensor_init(k_vicap_sensor_type sensor_type)
{
   k_vicap_dev dev_num=VICAP_DEV_ID_0;
    //k_bool dev_enable;
   //k_vicap_sensor_type sensor_type=GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
   //vicap get sensor info
   k_vicap_sensor_info sensor_info;
   k_u32 ret = kd_mpi_vicap_get_sensor_info(sensor_type, &sensor_info);
		if (ret) {
				  printf("sample_vicap, the sensor type not supported!\n");
				  return;
			  }


	k_vicap_dev_attr dev_attr;		   
	memset(&dev_attr, 0, sizeof(k_vicap_dev_attr));	
	dev_attr.input_type = VICAP_INPUT_TYPE_SENSOR;
	memcpy(&dev_attr.sensor_info, &sensor_info, sizeof(k_vicap_sensor_info));
	//vicap device attr set
	dev_attr.acq_win.h_start = 0;
	dev_attr.acq_win.v_start = 0;
	dev_attr.acq_win.width = sensor_info.width;
	dev_attr.acq_win.height = sensor_info.height;	  
	dev_attr.mode = VICAP_WORK_ONLINE_MODE;

	k_u32 pipe_ctrl = 0xFFFFFFFF;
	dev_attr.pipe_ctrl.data = pipe_ctrl;
	dev_attr.pipe_ctrl.bits.af_enable = 0;
	dev_attr.pipe_ctrl.bits.ae_enable = 0;
	dev_attr.pipe_ctrl.bits.awb_enable = 0;
	dev_attr.pipe_ctrl.bits.dnr3_enable = 0;
	dev_attr.pipe_ctrl.bits.ahdr_enable = 0;
	dev_attr.cpature_frame = 0;
	dev_attr.dw_enable = K_FALSE;
	//dev_attr.mirror = VICAP_MIRROR_NONE;
	dev_attr.mirror = VICAP_MIRROR_BOTH;
	ret = kd_mpi_vicap_set_dev_attr(dev_num, dev_attr);
		if (ret) {
		   printf("sample_vicap, kd_mpi_vicap_set_dev_attr failed.\n");
			   return;
		   }
}






void vi_chn_config(k_s32 dev_num, k_s32 chn_num,k_u32 display_width, k_u32 display_height,k_pixel_format pix_format)
{
		//kd_mpi_vicap_set_dump_reserved(dev_num, chn_num, K_TRUE);
		k_vicap_chn_attr chn_attr;
		memset(&chn_attr, 0, sizeof(k_vicap_chn_attr));
		chn_attr.out_win.width = display_width;
		chn_attr.out_win.height = display_height;
        	//chn_attr.crop_win.width = 560; 
		//chn_attr.crop_win.height = 320;
		//chn_attr.crop_win.h_start =0;
		//chn_attr.crop_win.v_start =0;	
		//chn_attr.scale_win = chn_attr.out_win;
		chn_attr.crop_enable = K_FALSE;
		chn_attr.scale_enable = K_FALSE;
		chn_attr.chn_enable = K_TRUE;

		chn_attr.pix_format =pix_format;
		//chn_attr.pix_format=PIXEL_FORMAT_ARGB_8888;
		
		
		//chn_attr.buffer_num = 10;
		//chn_attr.buffer_size = PRIVATE_POLL_SZE;
		if(pix_format==PIXEL_FORMAT_YUV_SEMIPLANAR_420)
		{
		   chn_attr.buffer_num = 10;
		   chn_attr.buffer_size = VICAP_ALIGN_UP((ISP_CHN0_WIDTH * ISP_CHN0_HEIGHT * 3 / 2), VICAP_ALIGN_1K);;
		}
		else if(pix_format==PIXEL_FORMAT_BGR_888_PLANAR)
		{
		   chn_attr.buffer_num = 5;
		   chn_attr.buffer_size = VICAP_ALIGN_UP((SENSOR_HEIGHT * SENSOR_WIDTH * 3 ), VICAP_ALIGN_1K);
		}
		
		//chn_attr.fps = device_obj[dev_num].fps[chn_num];
		   k_u32 ret = kd_mpi_vicap_set_chn_attr((k_vicap_dev)dev_num, (k_vicap_chn)chn_num, chn_attr);
		   if (ret) {
				printf("sample_vicap, kd_mpi_vicap_set_chn_attr failed.\n");
					return;
					  }

               
		  
}
void vi_bind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn)
{
 //bind vicap to vo, only support bind two vo chn(K_VO_DISPLAY_CHN_ID1 & K_VO_DISPLAY_CHN_ID2)	
		   printf("kd_mpi_sys_bind before\n");		 			   
		k_mpp_chn vicap_mpp_chn, vo_mpp_chn;
		vicap_mpp_chn.mod_id = K_ID_VI;
		vicap_mpp_chn.dev_id = vicap_dev;
		vicap_mpp_chn.chn_id = vicap_chn;
		
		vo_mpp_chn.mod_id = K_ID_VO;
		vo_mpp_chn.dev_id = K_VO_DISPLAY_DEV_ID;
		vo_mpp_chn.chn_id = K_VO_DISPLAY_CHN_ID1;
		k_u32 ret = kd_mpi_sys_bind(&vicap_mpp_chn, &vo_mpp_chn);
		if (ret) {
			printf("kd_mpi_sys_bind failed:0x%x\n", ret);
		}
               printf("kd_mpi_sys_bind after\n");
}
void vi_unbind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn)
{
 //bind vicap to vo, only support bind two vo chn(K_VO_DISPLAY_CHN_ID1 & K_VO_DISPLAY_CHN_ID2)	
		   printf("kd_mpi_sys_unbind before\n");		 			   
		k_mpp_chn vicap_mpp_chn, vo_mpp_chn;
		vicap_mpp_chn.mod_id = K_ID_VI;
		vicap_mpp_chn.dev_id = vicap_dev;
		vicap_mpp_chn.chn_id = vicap_chn;
		
		vo_mpp_chn.mod_id = K_ID_VO;
		vo_mpp_chn.dev_id = K_VO_DISPLAY_DEV_ID;
		vo_mpp_chn.chn_id = K_VO_DISPLAY_CHN_ID1;
		k_u32 ret = kd_mpi_sys_unbind(&vicap_mpp_chn, &vo_mpp_chn);
		if (ret) {
			printf("kd_mpi_sys_unbind failed:0x%x\n", ret);
		}
               printf("kd_mpi_sys_unbind after\n");
}

void vio_start_stream(k_s32 vicap_dev)
{
    k_u32 ret = kd_mpi_vicap_init((k_vicap_dev)vicap_dev);
			  if (ret) {
				  printf("sample_vicap, vicap dev(%d) init failed.\n", vicap_dev);
				  return;
			  }
			  printf("kd_mpi_vicap_init after\n");
			  printf("sample_vicap, vicap dev(%d) start stream\n", vicap_dev);
			  
	    ret = kd_mpi_vicap_start_stream((k_vicap_dev)vicap_dev);
			  if (ret) {
				  printf("sample_vicap, vicap dev(%d) start stream failed.\n", vicap_dev);
				  return;
			  	}
		  kd_mpi_vo_enable();
    



}

void vio_stop_stream(k_s32 vicap_dev)
{

      k_vicap_dev dev_num=vicap_dev;
        printf("sample_vicap, vicap dev(%d) stop stream\n", dev_num);
     k_u32   ret = kd_mpi_vicap_stop_stream(dev_num);
        if (ret) {
            printf("sample_vicap, vicap dev(%d) stop stream failed.\n", dev_num);
        }
       

        printf("sample_vicap, vicap dev(%d) deinit\n", dev_num);
        ret = kd_mpi_vicap_deinit(dev_num);
        if (ret) {
            printf("sample_vicap, vicap dev(%d) deinit failed.\n", dev_num);
        }
        
        printf("kd_mpi_vicap_deinit finished \n");
        
        
        kd_mpi_vo_disable_video_layer(K_VO_LAYER1);
         printf("vio_stop_stream finished \n");

}


