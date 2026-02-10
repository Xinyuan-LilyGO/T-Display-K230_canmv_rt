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
#include "mpi_venc_api.h"
#include "k_venc_comm.h"
#include "mpi_vdec_api.h"
#include "k_vdec_comm.h"
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
FILE *venc_file_inner_handler=NULL;
FILE *venc_file_handler=NULL;
FILE *vdec_file_handler=NULL;
int flag_vdec_done=0;
 k_u8 *pic_addr=NULL;
 k_video_frame_info vf_info;
 k_vo_osd osd_vo;
k_s32 connector_fd;//lcd
int h265_file_packet_index=0;
char h265_file_path[50];
uint8_t video_cache_data[225];
int last_remain_data_len=0;
void vo_display_init(k_connector_type connector_type)
{
//enable display
fpioa_set_function(35,GPIO35,-1,-1,-1,-1,-1,-1,-1);
 
  int gpio_fd = open("/dev/gpio", O_RDWR);
    if (gpio_fd>0)
    {
      pin_gpio_t pin_35;
      pin_35.pin = 35;
      ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_35);  //pin35 output
      ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_35);
      close(gpio_fd);
        
    }
  
  
// rst display subsystem
    kd_display_reset();
    // set hardware reset;
    kd_display_set_backlight();
	 
    //k_s32 connector_fd;
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
void vo_display_set_brightness(int bright_val)
{
        kd_mpi_connector_set_brightness(connector_fd,bright_val);	
}
k_u32 vb_create_pool(k_u32 pool_blk_cnt,k_u64 pool_blk_size)
{
    k_vb_pool_config pool_config;
    memset(&pool_config, 0, sizeof(pool_config));
    pool_config.blk_cnt = pool_blk_cnt;
    pool_config.blk_size = pool_blk_size;
    pool_config.mode = VB_REMAP_MODE_NOCACHE;
    k_u32 pool_id = kd_mpi_vb_create_pool(&pool_config);
    return pool_id;
}

void vio_vb_config(k_u32* pool_id)
{

//set config
		k_vb_config config;
		memset(&config, 0, sizeof(config));
		config.max_pool_cnt = 64;//10
		config.comm_pool[0].blk_cnt = 36;//6 36
		//config.comm_pool[0].blk_size = PRIVATE_POLL_SZE;		  // osd0 - 3 argb 320 x 240
		config.comm_pool[0].blk_size = VICAP_ALIGN_UP((ISP_CHN0_WIDTH * ISP_CHN0_HEIGHT * 3 / 2), VICAP_ALIGN_1K);
		config.comm_pool[0].mode = VB_REMAP_MODE_NOCACHE;//VB_REMAP_MODE_NOCACHE;
	
		config.comm_pool[1].blk_cnt = 6;//6
		config.comm_pool[1].blk_size = VICAP_ALIGN_UP((SENSOR_HEIGHT * SENSOR_WIDTH * 3 ), VICAP_ALIGN_1K);
		config.comm_pool[1].mode = VB_REMAP_MODE_NOCACHE;//VB_REMAP_MODE_NOCACHE;
	
	
	
	        //config.max_pool_cnt = 64;//10
		config.comm_pool[0].blk_cnt = 32;//6 36  26
		config.comm_pool[0].blk_size = ((1920*1080*2 + 0xfff) & ~0xfff);
		
	
		config.comm_pool[1].blk_cnt = 6;//6
		config.comm_pool[1].blk_size = ((1920*1080/2 + 0xfff) & ~0xfff);;
	
	
	
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
void vo_layer_config(k_vo_layer vo_layer,k_u32 display_width, k_u32 display_height,k_pixel_format pix_format,k_u16 _rotation)//rotation:0:0 1:90 2:180 3:270
{           k_u32 tmp;
            k_u32 rotation=0;
            switch (_rotation) {
            case 0:
                rotation = K_ROTATION_0;
                break;
            case 1:
                tmp=display_height;
                display_height=display_width;
                display_width=tmp;
                rotation = K_ROTATION_90;
                break;
            case 2:           
                rotation = K_ROTATION_180;
                break;
            case 3:
                tmp=display_height;
                display_height=display_width;
                display_width=tmp;
                rotation = K_ROTATION_270;
                break;
            default:
                break;
            }



//set layer
 layer_info layer_msg;
		
		// config lyaer
		   layer_msg.act_size.width = display_width;//1080;//640;//1080;
		   layer_msg.act_size.height = display_height;//1920;//480;//1920;
		   layer_msg.format = pix_format;
		   //layer_msg.func = K_ROTATION_270;//K_ROTATION_90
		   layer_msg.func =rotation; 
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
		chn_attr.alignment = 12;//for kd_mpi_venc_get_stream
		chn_attr.pix_format =pix_format;
		//chn_attr.pix_format=PIXEL_FORMAT_ARGB_8888;
		
		
		//chn_attr.buffer_num = 10;
		//chn_attr.buffer_size = PRIVATE_POLL_SZE;
		if(pix_format==PIXEL_FORMAT_YUV_SEMIPLANAR_420)
		{
		   chn_attr.buffer_num = 10;
		   chn_attr.buffer_size = VICAP_ALIGN_UP((ISP_CHN0_WIDTH * ISP_CHN0_HEIGHT * 3 / 2), VICAP_ALIGN_1K);
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



void vi_bind_venc(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 venc_dev,k_s32 venc_chn)
{
    k_mpp_chn vi_mpp_chn;
    vi_mpp_chn.mod_id = K_ID_VI;
    vi_mpp_chn.dev_id = vicap_dev;
    vi_mpp_chn.chn_id = vicap_chn;

    k_mpp_chn venc_mpp_chn;
    venc_mpp_chn.mod_id = K_ID_VENC;
    venc_mpp_chn.dev_id = venc_dev;
    venc_mpp_chn.chn_id = venc_chn;

   
   k_s32 ret = kd_mpi_sys_bind(&vi_mpp_chn, &venc_mpp_chn);
    if (ret)
    {
        printf("kd_mpi_sys_bind failed:0x%x\n", ret);
    }

    return;
}
void vi_unbind_venc(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 venc_dev,k_s32 venc_chn)
{
    k_mpp_chn vi_mpp_chn;
    vi_mpp_chn.mod_id = K_ID_VI;
    vi_mpp_chn.dev_id = vicap_dev;
    vi_mpp_chn.chn_id = vicap_chn;
        
    k_mpp_chn venc_mpp_chn;
    venc_mpp_chn.mod_id = K_ID_VENC;
    venc_mpp_chn.dev_id = venc_dev;
    venc_mpp_chn.chn_id = venc_chn;
 
    kd_mpi_sys_unbind(&vi_mpp_chn, &venc_mpp_chn);

}

void venc_chn_config(k_s32 chn_num,k_u32 display_width, k_u32 display_height)
{
    k_u32 bitrate   = 800;   //kbps 4000
    int width       = 1920;
    int height      = 1080;
    k_venc_rc_mode rc_mode  = K_VENC_RC_MODE_CBR;
    k_payload_type type     = K_PT_H265;
    k_venc_profile profile  = VENC_PROFILE_H265_MAIN;

    k_venc_chn_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.venc_attr.pic_width = width;
    attr.venc_attr.pic_height = height;
    attr.venc_attr.stream_buf_size =((1920*1080/2 + 0xfff) & ~0xfff);
    attr.venc_attr.stream_buf_cnt = 6;//15

    attr.rc_attr.rc_mode = rc_mode;
    attr.rc_attr.cbr.src_frame_rate = 30;//30
    attr.rc_attr.cbr.dst_frame_rate = 15;//30
    attr.rc_attr.cbr.bit_rate = bitrate;

    attr.venc_attr.type = type;
    attr.venc_attr.profile = profile;
    
    kd_mpi_venc_create_chn(chn_num, &attr);
    /*if (intbuf_size > 0)
    {
        kd_mpi_venc_set_intbuf_size(chn_num, intbuf_size);
        //printf("%s>intbuf_size %d\n", __func__, intbuf_size);
    }*/

   kd_mpi_venc_start_chn(chn_num);
}

void venc_write_data_to_file(k_s32 chn_num,char* venc_file)
{     k_s32 ret;
       if(!venc_file_handler)
        {       
        venc_file_handler  = fopen(venc_file, "wb");       
        }
        k_venc_stream output;
        k_venc_chn_status status;
        ret=kd_mpi_venc_query_status(chn_num, &status);
       printf("kd_mpi_venc_query_status,ret:%d,cur_packs:%d\n",ret,status.cur_packs);
        if (status.cur_packs > 0)
        {
            output.pack_cnt = status.cur_packs;
            }
        else
        {
            output.pack_cnt = 1;
            }

        output.pack = malloc(sizeof(k_venc_pack) * output.pack_cnt);
        //printf("kd_mpi_venc_get_stream before,pack_cnt:%d\n",output.pack_cnt);
        ret=kd_mpi_venc_get_stream(chn_num, &output, -1);
        //printf("kd_mpi_venc_get_stream after,ret:%d,pack_cnt:%d\n",ret,output.pack_cnt);
        for (int i = 0; i < output.pack_cnt; i++)
        {
            if (output.pack[i].type != K_VENC_HEADER)
            {
                
            }
            printf("output.pack[i].type=%d\n",output.pack[i].type);
            printf("output.pack[i].len=%d\n",output.pack[i].len);

            k_u8 *pData;
            pData = (k_u8 *)kd_mpi_sys_mmap(output.pack[i].phys_addr, output.pack[i].len);
                                                                                               
                  if(venc_file_handler)
                    {
                      //int w_bytes=
                      fwrite(pData, 1, output.pack[i].len, venc_file_handler);
                      //printf("w_bytes:%d\n",w_bytes);
                      fflush(venc_file_handler);
                    }
            kd_mpi_sys_munmap(pData, output.pack[i].len);
        }

        kd_mpi_venc_release_stream(chn_num, &output);
        free(output.pack);

}

void venc_write_data(k_s32 chn_num)
{     k_s32 ret; 
       printf("aaaaaaaaaaaaaaaaa\n"); 
       if (!venc_file_inner_handler)
          {
              memset(h265_file_path,0,50);
              sprintf(h265_file_path,"/sdcard/live_%d.h265",h265_file_packet_index++);//abc.h265
              venc_file_inner_handler  = fopen(h265_file_path, "wb");
          }
        printf("bbbbbbbbbbbbbbbbb\n");        
        k_venc_stream output;
        k_venc_chn_status status;
        ret=kd_mpi_venc_query_status(chn_num, &status);
       //printf("kd_mpi_venc_query_status,ret:%d,cur_packs:%d\n",ret,status.cur_packs);
        if (status.cur_packs > 0)
        {
            output.pack_cnt = status.cur_packs;
            }
        else
        {
            output.pack_cnt = 1;
            }
        printf("cccccccccccccccccccc\n"); 
        output.pack = malloc(sizeof(k_venc_pack) * output.pack_cnt);
        //printf("kd_mpi_venc_get_stream before,pack_cnt:%d\n",output.pack_cnt);
        ret=kd_mpi_venc_get_stream(chn_num, &output, -1);
        //printf("kd_mpi_venc_get_stream after,ret:%d,pack_cnt:%d\n",ret,output.pack_cnt);
        printf("dddddddddddddddd\n");
        for (int i = 0; i < output.pack_cnt; i++)
        {
            if (output.pack[i].type != K_VENC_HEADER)
            {
                
            }
            //printf("output.pack[i].type=%d\n",output.pack[i].type);
            //printf("output.pack[i].len=%d\n",output.pack[i].len);
            printf("111111111111111\n");
            k_u8 *pData;
            pData = (k_u8 *)kd_mpi_sys_mmap(output.pack[i].phys_addr, output.pack[i].len);
               printf("222222222222222222\n");                                              
                     if(venc_file_inner_handler)
                     {
                     printf("333333333333333333\n");
                      //int w_bytes=
                      fwrite(pData, 1, output.pack[i].len, venc_file_inner_handler);
                      printf("4444444444444444444444\n");
                      //printf("w_bytes:%d\n",w_bytes);
                      fflush(venc_file_inner_handler);
                      printf("55555555555555555555555555\n");
                      }
                      else
                      {
                       printf("%s fopen fail\n",h265_file_path);
                       h265_file_packet_index--;
                      
                      }
                      printf("66666666666666666666\n");  
            kd_mpi_sys_munmap(pData, output.pack[i].len);
            printf("7777777777777777777777777777\n");
        }
       printf("eeeeeeeeeeeeeeeeeeeeeeeee\n");
        kd_mpi_venc_release_stream(chn_num, &output);
        free(output.pack);
printf("fffffffffffffffffffffff\n");
     if (venc_file_inner_handler)
     {        
        fclose(venc_file_inner_handler);
        venc_file_inner_handler=NULL;
     }
     printf("gggggggggggggggggg\n");
     (void)ret;
     printf("hhhhhhhhhhhhhhhhhhhh\n");
}
void venc_write_data_to_cache(k_s32 chn_num,pkg_cache_t *cache,int is_save_file)
{     k_s32 ret; 
       if(!venc_file_handler)
        {       
        venc_file_handler  = fopen("/sdcard/abc_cache.h265", "wb");       
        }
        k_venc_stream output;
        k_venc_chn_status status;
        ret=kd_mpi_venc_query_status(chn_num, &status);
       //printf("kd_mpi_venc_query_status,ret:%d,cur_packs:%d\n",ret,status.cur_packs);
        if (status.cur_packs > 0)
        {
            output.pack_cnt = status.cur_packs;
            }
        else
        {
            output.pack_cnt = 1;
            }
   
        output.pack = malloc(sizeof(k_venc_pack) * output.pack_cnt);
        //printf("kd_mpi_venc_get_stream before,pack_cnt:%d\n",output.pack_cnt);
        ret=kd_mpi_venc_get_stream(chn_num, &output, -1);
        //printf("kd_mpi_venc_get_stream after,ret:%d,pack_cnt:%d\n",ret,output.pack_cnt);
        int pData_move;
        int pkt_len;
        for (int i = 0; i < output.pack_cnt; i++)
        {
            if (output.pack[i].type != K_VENC_HEADER)
            {
                
            }
            //printf("output.pack[i].type=%d\n",output.pack[i].type);
            //printf("output.pack[i].len=%d\n",output.pack[i].len);
           
            k_u8 *pData;
            pData = (k_u8 *)kd_mpi_sys_mmap(output.pack[i].phys_addr, output.pack[i].len);
               //pan  duan  225  write fouze  wait
               pData_move=0;
               pkt_len=output.pack[i].len; 
               if(last_remain_data_len>0)
               {
                  if(pkt_len>=(225-last_remain_data_len))
                    {
                      memcpy(video_cache_data+last_remain_data_len,pData,(225-last_remain_data_len));
                      pkt_len=pkt_len-(225-last_remain_data_len);
                      //pData+=(225-last_remain_data_len);
                      pData_move=(225-last_remain_data_len);
                      //cache_write(cache, video_cache_data);
                      int try_cnt=0;
                      while(cache_write(cache, video_cache_data)<0)
                      {
                      try_cnt++;
                      if(try_cnt>3)
                      {
                      break;
                      }
                      usleep(100);
                      //rt_thread_mdelay(1);
                      //printf("cache_write1 waiting<%d>\n",try_cnt);
                      }
                      if(venc_file_handler&&(is_save_file==1))
                     {
                 
                      fwrite(video_cache_data, 1,225, venc_file_handler);
                   
                      fflush(venc_file_handler);
                      }
                    }
                    else
                    {
                    memcpy(video_cache_data+last_remain_data_len,pData,pkt_len);
                    last_remain_data_len+=pkt_len;
                    /*if(venc_file_handler)
                     {
                      //int w_bytes=
                      fwrite(pData, 1, output.pack[i].len, venc_file_handler);
                      //printf("w_bytes:%d\n",w_bytes);
                      fflush(venc_file_handler);
                      }   */
             kd_mpi_sys_munmap(pData, output.pack[i].len);  
                    continue;
                    }
               
               }
               for(int j=0;j<pkt_len/225;j++)
               {
               memcpy(video_cache_data,pData+pData_move+(j*225),225);
               //cache_write(cache, video_cache_data);
               int try_cnt=0; 
                while(cache_write(cache, video_cache_data)<0)
                      {
                      try_cnt++;
                      if(try_cnt>3)
                      {
                      break;
                      }
                      usleep(100);
                      //rt_thread_mdelay(1);
                      //printf("cache_write2 waiting<%d>\n",try_cnt);
                      }
                   if(venc_file_handler&&(is_save_file==1))
                     {
                      
                      fwrite(video_cache_data, 1,225, venc_file_handler);                      
                      fflush(venc_file_handler);
                      }
               }
              last_remain_data_len=pkt_len%225;
              if(last_remain_data_len>0)
              {
              memcpy(video_cache_data,pData+pData_move+((pkt_len/225)*225),last_remain_data_len);
              }
             // pData-=pData_move;
         /* if(venc_file_handler)
            {
                      //int w_bytes=
                      fwrite(pData, 1, output.pack[i].len, venc_file_handler);
                      //printf("w_bytes:%d\n",w_bytes);
                      fflush(venc_file_handler);
             }  */                                                                                                       
            kd_mpi_sys_munmap(pData, output.pack[i].len);
      
        }
 
        kd_mpi_venc_release_stream(chn_num, &output);
        free(output.pack);
     (void)ret;
}
void venc_stop_stream(k_s32 venc_chn)
{
 kd_mpi_venc_stop_chn(venc_chn);
 kd_mpi_venc_destroy_chn(venc_chn);
 kd_mpi_venc_close_fd();
}


void vdec_chn_config(k_s32 chn_num,k_u32 display_width, k_u32 display_height)
{
    k_payload_type type= K_PT_H265;
    int width       = 1088;
    int height      = 1920;
    k_vdec_chn_attr attr;
    attr.pic_width = width;
    attr.pic_height = height;
    attr.frame_buf_cnt = 6;
    attr.frame_buf_size = width*height*2;
    attr.stream_buf_size = width*height;
    attr.type = type;
    
    
    
    k_u32 pool_id=vb_create_pool(6,width*height*2);
    printf("vb_create_pool,pool_id=%d\n",pool_id);
    attr.frame_buf_pool_id = pool_id;
    kd_mpi_vdec_create_chn(chn_num, &attr);
    

   kd_mpi_vdec_start_chn(chn_num);
}
void vdec_bind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn)
{
 //bind vicap to vo, only support bind two vo chn(K_VO_DISPLAY_CHN_ID1 & K_VO_DISPLAY_CHN_ID2)	
		   printf("kd_mpi_sys_bind before\n");		 			   
		k_mpp_chn vdec_mpp_chn, vo_mpp_chn;
		vdec_mpp_chn.mod_id = K_ID_VDEC;
		vdec_mpp_chn.dev_id = vicap_dev;
		vdec_mpp_chn.chn_id = vicap_chn;
		
		vo_mpp_chn.mod_id = K_ID_VO;
		vo_mpp_chn.dev_id = K_VO_DISPLAY_DEV_ID;
		vo_mpp_chn.chn_id = K_VO_DISPLAY_CHN_ID1;
		k_u32 ret = kd_mpi_sys_bind(&vdec_mpp_chn, &vo_mpp_chn);
		if (ret) {
			printf("kd_mpi_sys_bind failed:0x%x\n", ret);
		}
               printf("kd_mpi_sys_bind after\n");
}
void vdec_unbind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn)
{
 //bind vicap to vo, only support bind two vo chn(K_VO_DISPLAY_CHN_ID1 & K_VO_DISPLAY_CHN_ID2)	
		   printf("kd_mpi_sys_unbind before\n");		 			   
		k_mpp_chn vdec_mpp_chn, vo_mpp_chn;
		vdec_mpp_chn.mod_id = K_ID_VDEC;
		vdec_mpp_chn.dev_id = vicap_dev;
		vdec_mpp_chn.chn_id = vicap_chn;
		
		vo_mpp_chn.mod_id = K_ID_VO;
		vo_mpp_chn.dev_id = K_VO_DISPLAY_DEV_ID;
		vo_mpp_chn.chn_id = K_VO_DISPLAY_CHN_ID1;
		k_u32 ret = kd_mpi_sys_unbind(&vdec_mpp_chn, &vo_mpp_chn);
		if (ret) {
			printf("kd_mpi_sys_unbind failed:0x%x\n", ret);
		}
               printf("kd_mpi_sys_unbind after\n");
}
void vdec_read_data(k_s32 chn_num,char* venc_file)
{
    //k_s32 ret;
     if(!vdec_file_handler)
     {
      vdec_file_handler=fopen(venc_file, "rb");
     }
      fseek(vdec_file_handler, 0L, SEEK_END);
      int file_size = ftell(vdec_file_handler);
      fseek(vdec_file_handler, 0, SEEK_SET);
      printf("file_size:%dbytes\n",file_size);

     int width       = 1088;
    int height      = 1920;
    k_u32 pool_id=vb_create_pool(6,width*height);
    printf("vb_create_pool,pool_id-%d\n",pool_id);
    k_vdec_stream stream;
    k_u32 blk_size, stream_len;
   
    k_u32 data_send_size = 0;
    blk_size = 1920*1088;
     k_u64 phys_addr = 0;
    k_u8 *virt_addr;
 while (data_send_size < file_size)
    {
      k_vb_blk_handle handle = kd_mpi_vb_get_block(pool_id, blk_size, NULL);

        if (handle == VB_INVALID_HANDLE)
        {
            usleep(30000);
            continue;
        }

        pool_id = kd_mpi_vb_handle_to_pool_id(handle);
        if (pool_id == VB_INVALID_POOLID)
        {           
            break;
        }
        //if(i >= INPUT_BUF_CNT)
         //   i = 0;
        //vdec_conf->pool_id = pool_id;
        //vdec_conf->vb_handle[i] = handle;

        phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);
        if (phys_addr == 0)
        {
            //vdec_debug("%s get phys addr error\n", __func__);
            break;
        }
    
      virt_addr = (k_u8 *)kd_mpi_sys_mmap_cached(phys_addr, blk_size);

        if (virt_addr == NULL)
        {
            //vdec_debug("%s mmap error\n", __func__);
            break;
        }
    
    
    
    
        memset(&stream, 0, sizeof(k_vdec_stream));
       
        if (data_send_size + blk_size > file_size)
        {
            fread(virt_addr, 1, (file_size - data_send_size), vdec_file_handler);
            stream_len =file_size - data_send_size;
            stream.end_of_stream = K_TRUE;
        }
        else
        {
            fread(virt_addr, 1, blk_size, vdec_file_handler);
            stream_len = blk_size;
        }
        kd_mpi_sys_mmz_flush_cache(phys_addr, virt_addr, stream_len);
        

        data_send_size += stream_len;

        stream.phy_addr = phys_addr;
        stream.len = stream_len;

        kd_mpi_vdec_send_stream(chn_num, &stream, -1);

        kd_mpi_sys_munmap((void *)virt_addr, blk_size);
       

        kd_mpi_vb_release_block(handle);
        if(flag_vdec_done)
        {
        break;
        }
        usleep(40000);
       
    }


/*
      size_t size =1920 * 1088;
    
    
    // alloc memory
    k_u64 paddr = 0;
    void *vaddr = NULL;
    ret = kd_mpi_sys_mmz_alloc_cached(&paddr, &vaddr, "allocate", "anonymous", size);
    if (ret)
    {
        //std::cerr << "physical_memory_block::allocate failed: ret = " << ret << ", errno = " << strerror(errno) << std::endl;
        //std::abort();
    }
    printf("");
    k_vdec_stream stream;
    k_u32 blk_size, stream_len;
   
    k_u32 data_send_size = 0;
    blk_size = 1920*1088;
 while (data_send_size < file_size)
    {
        memset(&stream, 0, sizeof(k_vdec_stream));
       
        if (data_send_size + blk_size > file_size)
        {
            fread(vaddr, 1, (file_size - data_send_size), vdec_file_handler);
            stream_len =file_size - data_send_size;
            stream.end_of_stream = K_TRUE;
        }
        else
        {
            fread(vaddr, 1, blk_size, vdec_file_handler);
            stream_len = blk_size;
        }
        ret = kd_mpi_sys_mmz_flush_cache(paddr, vaddr, stream_len);
        

        data_send_size += stream_len;

        stream.phy_addr = paddr;
        stream.len = stream_len;

        ret = kd_mpi_vdec_send_stream(chn_num, &stream, -1);

        //ret = kd_mpi_sys_munmap((void *)vaddr, blk_size);
      
        //ret = kd_mpi_sys_mmz_free(paddr, vaddr);
       
    }
    */

}



void vdec_process_data(k_s32 chn_num,k_u32 pool_id,uint8_t * data,int data_len,int is_end)
{
     k_u64 phys_addr = 0;
     k_u8 *virt_addr;
     k_vdec_stream stream;
     k_u32 blk_size, stream_len;
     blk_size=256;
     //printf("aaaaaaaaaaaa\n");
      k_vb_blk_handle handle = kd_mpi_vb_get_block(pool_id, blk_size, NULL);

        if (handle == VB_INVALID_HANDLE)
        {
            return;
        }
        //printf("bbbbbbbbbbbbbbbbb\n");
        phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);
        if (phys_addr == 0)
        {
            return;
        }
    //printf("cccccccccccccccccc\n");
      virt_addr = (k_u8 *)kd_mpi_sys_mmap_cached(phys_addr, blk_size);

        if (virt_addr == NULL)
        {
            return;
        }          
        memset(&stream, 0, sizeof(k_vdec_stream));
         //printf("ddddddddddddd\n");       
        memcpy(virt_addr,data,data_len);
        stream_len =data_len;
        stream.end_of_stream =(is_end>0)?K_TRUE:K_FALSE;
        //k_s32 ret=
        kd_mpi_sys_mmz_flush_cache(phys_addr, virt_addr, stream_len); 
        //printf("kd_mpi_sys_mmz_flush_cache,ret_code:%d\n",ret);             
        stream.phy_addr = phys_addr;
        stream.len = stream_len;
        //printf("eeeeeeeeeeeeee\n");
        //printf("stream.phy_addr:%p,stream.len:%d\n",virt_addr,stream_len); 
       //k_s32 ret_code= 
       kd_mpi_vdec_send_stream(chn_num, &stream, -1);
        //printf("kd_mpi_vdec_send_stream,ret_code:%d\n",ret_code);
        
        //printf("fffffffffffffffffffff\n"); 
        kd_mpi_sys_munmap((void *)virt_addr, blk_size);
        //printf("gggggggggggggggggggggg\n"); 
        kd_mpi_vb_release_block(handle);
        //printf("hhhhhhhhhhhhhhhhhh\n"); 
}

void vdec_read_data_live(k_s32 chn_num,char* venc_file)
{
    k_s32 ret;
     if(!vdec_file_handler)
     {
      vdec_file_handler=fopen(venc_file, "rb");
     }
      fseek(vdec_file_handler, 0L, SEEK_END);
      int file_size = ftell(vdec_file_handler);
      fseek(vdec_file_handler, 0, SEEK_SET);
      printf("file_size:%dbytes\n",file_size);

    int width       = 1088;
    int height      = 1920;
    k_u32 pool_id=vb_create_pool(6,width*height);
    printf("vb_create_pool,pool_id-%d\n",pool_id);
    k_vdec_stream stream;
    k_u32 blk_size, stream_len;
   
    k_u32 data_send_size = 0;
    blk_size = 992*560;//1920*1088
    //blk_size =560;
     k_u64 phys_addr = 0;
    k_u8 *virt_addr;
 while (data_send_size < file_size)
    {
     printf("ssssssssssssssssssssss\n");
      k_vb_blk_handle handle = kd_mpi_vb_get_block(pool_id, blk_size, NULL);

        if (handle == VB_INVALID_HANDLE)
        {
            usleep(30000);
            continue;
        }

        pool_id = kd_mpi_vb_handle_to_pool_id(handle);
        if (pool_id == VB_INVALID_POOLID)
        {           
            break;
        }
        //if(i >= INPUT_BUF_CNT)
         //   i = 0;
        //vdec_conf->pool_id = pool_id;
        //vdec_conf->vb_handle[i] = handle;

        phys_addr = kd_mpi_vb_handle_to_phyaddr(handle);
        if (phys_addr == 0)
        {
            //vdec_debug("%s get phys addr error\n", __func__);
            break;
        }
    
      virt_addr = (k_u8 *)kd_mpi_sys_mmap_cached(phys_addr, blk_size);

        if (virt_addr == NULL)
        {
            //vdec_debug("%s mmap error\n", __func__);
            break;
        }
    
    
    
    
        memset(&stream, 0, sizeof(k_vdec_stream));
       
        if (data_send_size + blk_size > file_size)
        {
            fread(virt_addr, 1, (file_size - data_send_size), vdec_file_handler);
            stream_len =file_size - data_send_size;
            stream.end_of_stream = K_TRUE;
        }
        else
        {
            fread(virt_addr, 1, blk_size, vdec_file_handler);
            stream_len = blk_size;
        }
        ret = kd_mpi_sys_mmz_flush_cache(phys_addr, virt_addr, stream_len);
        

        data_send_size += stream_len;

        stream.phy_addr = phys_addr;
        stream.len = stream_len;

        ret = kd_mpi_vdec_send_stream(chn_num, &stream, -1);

        ret = kd_mpi_sys_munmap((void *)virt_addr, blk_size);
       

        ret = kd_mpi_vb_release_block(handle);
        printf("ttttttttttttttttttttttttttt:%d\n",ret);
        if(flag_vdec_done)
        {
        break;
        }
        usleep(40000);
       
    }
}








int vdec_output(k_s32 chn_num)
{

      k_vdec_chn_status status;

      kd_mpi_vdec_query_status(chn_num, &status);
      //printf("status.width:%d status.height:%d \n",status.width, status.height);
      return status.end_of_stream;

}
void vdec_stop_stream(k_s32 vdec_chn)
{
     kd_mpi_vo_disable_video_layer(K_VO_LAYER1);
     kd_mpi_vdec_stop_chn(vdec_chn);            
     kd_mpi_vdec_destroy_chn(vdec_chn);
     kd_mpi_vdec_close_fd();
}









