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

#define VICAP_OUTPUT_BUF_NUM 6
#define VICAP_INPUT_BUF_NUM 4


typedef struct {
    k_vicap_dev dev_num;
    k_bool dev_enable;
    k_vicap_sensor_type sensor_type;
    k_vicap_sensor_info sensor_info;

    k_u16 in_width;
    k_u16 in_height;

    //for mcm
    k_vicap_work_mode mode;
    k_u32 in_size;
    k_pixel_format in_format;

    k_vicap_input_type input_type;
    k_vicap_image_pattern pattern;
    const char *file_path;//input raw image file
    const char *calib_file;
    void *image_data;
    k_u32 dalign;

    k_bool ae_enable;
    k_bool awb_enable;
    k_bool dnr3_enable;
    k_bool hdr_enable;

    k_vicap_chn chn_num[VICAP_CHN_ID_MAX];

    k_bool chn_enable[VICAP_CHN_ID_MAX];
    k_pixel_format out_format[VICAP_CHN_ID_MAX];
    
    k_bool crop_enable[VICAP_CHN_ID_MAX];

    k_vicap_window out_win[VICAP_CHN_ID_MAX];

    k_vicap_window crop_win[VICAP_CHN_ID_MAX];

    k_u32 buf_size[VICAP_CHN_ID_MAX];

    k_video_frame_info dump_info[VICAP_CHN_ID_MAX];

    k_bool preview[VICAP_CHN_ID_MAX];
    k_u16 rotation[VICAP_CHN_ID_MAX];
    k_u8 fps[VICAP_CHN_ID_MAX];
    k_bool dw_enable;
    k_vicap_mirror sensor_mirror;
} vicap_device_obj;

#define MAX_VO_LAYER_NUM 3

typedef struct {
    k_u16 width[MAX_VO_LAYER_NUM];
    k_u16 height[MAX_VO_LAYER_NUM];
    k_u16 rotation[MAX_VO_LAYER_NUM];
    k_vo_layer layer[MAX_VO_LAYER_NUM];
    k_bool enable[MAX_VO_LAYER_NUM];
} k_vicap_vo_layer_conf;

static k_s32 sample_vicap_vo_init(k_connector_type connector_type)
{

    k_u32 ret = 0;
    k_s32 connector_fd;
    k_connector_info connector_info;

    memset(&connector_info, 0, sizeof(k_connector_info));

    //connector get sensor info
    ret = kd_mpi_get_connector_info(connector_type, &connector_info);
    if (ret) {
        printf("sample_vicap, the sensor type not supported!\n");
        return ret;
    }

    connector_fd = kd_mpi_connector_open(connector_info.connector_name);
    if (connector_fd < 0) {
        printf("%s, connector open failed.\n", __func__);
        return K_ERR_VO_NOTREADY;
    }

    // set connect power
    kd_mpi_connector_power_set(connector_fd, 1);
    // connector init
    kd_mpi_connector_init(connector_fd, connector_info);

    return 0;

}

static k_s32 sample_vicap_vo_layer_init(k_vicap_vo_layer_conf *layer_conf, k_u32 display_width, k_u32 display_height)
{
    k_s32 ret = 0;
    layer_info info[MAX_VO_LAYER_NUM];
    //k_u16 margin = 0;
    //k_u16 rotation = 0;
   // k_u16 relative_height = 0;
    //k_u16 total_height = 0;
    osd_info osd_info;

    memset(&info, 0, sizeof(info));
    memset(&osd_info, 0, sizeof(osd_info));
/*
    for (int i = 0; i < MAX_VO_LAYER_NUM; i++) {
        if (layer_conf->enable[i]) {
            rotation = layer_conf->rotation[i];
            switch (rotation) {
            case 0:
                info[i].act_size.width = layer_conf->width[i];
                info[i].act_size.height = layer_conf->height[i];
                info[i].func = K_ROTATION_0;
                break;
            case 1:
                info[i].act_size.width = layer_conf->height[i];
                info[i].act_size.height = layer_conf->width[i];
                info[i].func = K_ROTATION_90;
                break;
            case 2:
                info[i].act_size.width = layer_conf->width[i];
                info[i].act_size.height = layer_conf->height[i];
                info[i].func = K_ROTATION_180;
                break;
            case 3:
                info[i].act_size.width = layer_conf->height[i];
                info[i].act_size.height = layer_conf->width[i];
                info[i].func = K_ROTATION_270;
                break;
            case 4:
                info[i].act_size.width = layer_conf->width[i];
                info[i].act_size.height = layer_conf->height[i];
                info[i].func = 0;
                break;
            default:
                printf("invalid roation paramters.\n");
                return -1;
            }
            total_height += info[i].act_size.height;
            margin = ((display_height - total_height) / (i+2));
            if ((total_height > display_height) || (info[i].act_size.width > display_width)) {
                printf("%s, the preview window size[%dx%d] exceeds the display window size[%dx%d].\n", \
                    __func__, info[i].act_size.width, total_height, display_width, display_height);
                return -1;
            }
            printf("%s, width(%d), height(%d), margin(%d), total_height(%d)\n", \
                __func__, info[i].act_size.width, info[i].act_size.height, margin, total_height);
        }
    }
*/
  /*  for (int i = 0; i < MAX_VO_LAYER_NUM - 1; i++) {
        if (layer_conf->enable[i]) {
            info[i].offset.x = (display_width - info[i].act_size.width)/2;
            info[i].offset.y = margin + relative_height;
            printf("%s, layer(%d), offset.x(%d), offset.y(%d), relative_height(%d)\n", __func__, layer_conf->layer[i], info[i].offset.x, info[i].offset.y, relative_height);
            relative_height += info[i].act_size.height + margin;

            info[i].format = PIXEL_FORMAT_YVU_PLANAR_420;
            info[i].global_alptha = 0xff;

            vo_creat_layer_test(layer_conf->layer[i], &info[i]);
        }
    }
*/
    // osd enable 
    /*if(layer_conf->enable[2])
    {
        osd_info.act_size.width = layer_conf->width[2]; ;
        osd_info.act_size.height = layer_conf->height[2];;
        osd_info.offset.x = (display_width - layer_conf->width[2])/2;;
        osd_info.offset.y = margin + relative_height;
        osd_info.global_alptha = 0xff;
        osd_info.format = PIXEL_FORMAT_RGB_888;//PIXEL_FORMAT_ARGB_4444; //PIXEL_FORMAT_ARGB_1555;//PIXEL_FORMAT_ARGB_8888;

        vo_creat_osd_test(layer_conf->layer[2], &osd_info);
    }*/

//lv write
/*
        osd_info.act_size.width = layer_conf->width[2];
        osd_info.act_size.height = layer_conf->height[2];;
        //osd_info.offset.x = (display_width - layer_conf->width[2])/2;;
        //osd_info.offset.y = margin + relative_height;
        osd_info.global_alptha = 0xff;
        osd_info.format = PIXEL_FORMAT_ARGB_8888;//PIXEL_FORMAT_ARGB_4444; //PIXEL_FORMAT_ARGB_1555;//PIXEL_FORMAT_ARGB_8888;

        vo_creat_osd_test(layer_conf->layer[2], &osd_info);
        */
        
        
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
		
		   // check layer
		  // if ((chn_id >= K_MAX_VO_LAYER_NUM) || ((layer_msg.func & K_VO_SCALER_ENABLE) && (chn_id != K_VO_LAYER0))
			//	   || ((layer_msg.func != 0) && (chn_id == K_VO_LAYER2)))
		  // {
			//   printf("input layer num failed \n");
			//   return;
		   //}
		
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
			   return -1;
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
        
        
        
        
        
        







    return ret;
}
/*
static void sample_vicap_vo_enable(void)
{
    kd_mpi_vo_enable();
}
*/
static k_s32 sample_vicap_vb_init(vicap_device_obj *dev_obj)
{
    k_s32 ret = 0;
    k_vb_config config;
    //k_vb_supplement_config supplement_config;

    memset(&config, 0, sizeof(config));
    config.max_pool_cnt = 64;
	     int i=0;
		int j=0;
              int k = 0;
            config.comm_pool[k].blk_cnt = VICAP_OUTPUT_BUF_NUM;
            config.comm_pool[k].mode = VB_REMAP_MODE_NOCACHE;

            k_pixel_format pix_format = dev_obj[i].out_format[j];
            k_u16 out_width = dev_obj[i].out_win[j].width;
            k_u16 out_height = dev_obj[i].out_win[j].height;
            k_u16 in_width = dev_obj[i].in_width;
            k_u16 in_height = dev_obj[i].in_height;

            switch (pix_format) {
            case PIXEL_FORMAT_YUV_SEMIPLANAR_420:
                config.comm_pool[k].blk_size = VICAP_ALIGN_UP((out_width * out_height * 3 / 2), VICAP_ALIGN_1K);
                break;
            case PIXEL_FORMAT_RGB_888:
            case PIXEL_FORMAT_BGR_888_PLANAR:
                config.comm_pool[k].blk_size = VICAP_ALIGN_UP((out_width * out_height * 3), VICAP_ALIGN_1K);
                break;
            case PIXEL_FORMAT_RGB_BAYER_10BPP:
                config.comm_pool[k].blk_size = VICAP_ALIGN_UP((in_width * in_height * 2), VICAP_ALIGN_1K);
                break;
            default:
                dev_obj[i].out_format[j] = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
                config.comm_pool[k].blk_size = VICAP_ALIGN_UP((out_width * out_height * 3 / 2), VICAP_ALIGN_1K);
                break;
            }
            dev_obj[i].buf_size[j] = config.comm_pool[k].blk_size;
            printf("%s, dev(%d) chn(%d) pool(%d) buf_size(%d) blk_cnt(%d)\n", __func__, i, j, k ,dev_obj[i].buf_size[j], config.comm_pool[k].blk_cnt);
    ret = kd_mpi_vb_set_config(&config);
    if (ret) {
        printf("vb_set_config failed ret:%d\n", ret);
        return ret;
    }

    /*memset(&supplement_config, 0, sizeof(supplement_config));
    supplement_config.supplement_config |= VB_SUPPLEMENT_JPEG_MASK;

    ret = kd_mpi_vb_set_supplement_config(&supplement_config);
    if (ret) {
        printf("vb_set_supplement_config failed ret:%d\n", ret);
        return ret;
    }
*/
    ret = kd_mpi_vb_init();
    if (ret) {
        printf("vb_init failed ret:%d\n", ret);
        return ret;
    }

    return 0;
}

static void sample_vicap_bind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn)
{
    k_s32 ret;

    k_mpp_chn vicap_mpp_chn, vo_mpp_chn;

    vicap_mpp_chn.mod_id = K_ID_VI;
    vicap_mpp_chn.dev_id = vicap_dev;
    vicap_mpp_chn.chn_id = vicap_chn;

    vo_mpp_chn.mod_id = K_ID_VO;
    vo_mpp_chn.dev_id = K_VO_DISPLAY_DEV_ID;
    vo_mpp_chn.chn_id = vo_chn;

    ret = kd_mpi_sys_bind(&vicap_mpp_chn, &vo_mpp_chn);
    if (ret) {
        printf("kd_mpi_sys_unbind failed:0x%x\n", ret);
    }

    return;
}

extern k_u32 display_cnt;
extern k_u32 drop_cnt;
extern uint32_t hdr_buf_base_phy_addr;
extern void *hdr_buf_base_vir_addr;

static void vb_exit() {
    kd_mpi_vb_exit();
}


int main(int argc, char *argv[])
{
    
    k_s32 ret = 0;
    //k_u8 salve_en = 0;

    //k_u32 work_mode = VICAP_WORK_ONLINE_MODE;
    k_connector_type connector_type = RM69A10_MIPI_2LAN_568X1232_60FPS;
    k_connector_info connector_info;
    k_u32 display_width;
    k_u32 display_height;

    vicap_device_obj device_obj[VICAP_DEV_ID_MAX];
    memset(&device_obj, 0 , sizeof(device_obj));

    k_vicap_vo_layer_conf layer_conf;
    memset(&layer_conf, 0 , sizeof(k_vicap_vo_layer_conf));

    k_vicap_dev_attr dev_attr;
    k_vicap_chn_attr chn_attr;

    //k_video_frame_info dump_info;

    k_u8 dev_count = 0, cur_dev = 0;
    k_u8 chn_count = 0, cur_chn = 0;
    k_u8 vo_count = 0;
    k_u32 pipe_ctrl = 0xFFFFFFFF;
    memset(&dev_attr, 0, sizeof(k_vicap_dev_attr));

    
            chn_count = 0;
            cur_dev = 0;
            dev_count++;
            printf("cur_dev(%d), dev_count(%d)\n", cur_dev, dev_count);
            device_obj[cur_dev].sensor_type =  GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR;
            device_obj[cur_dev].dev_num = cur_dev;
            device_obj[cur_dev].dev_enable = K_TRUE;
            device_obj[cur_dev].ae_enable = K_TRUE;//default enable ae
            device_obj[cur_dev].awb_enable = K_TRUE;//default enable awb
            device_obj[cur_dev].dnr3_enable = K_FALSE;//default disable 3ndr
            device_obj[cur_dev].hdr_enable = K_FALSE;//default disable hdr
            
            cur_chn =0;

            chn_count++;
            printf("cur_chn(%d), chn_count(%d)\n", cur_chn ,chn_count);
            device_obj[cur_dev].chn_num[cur_chn] = cur_chn;
            device_obj[cur_dev].chn_enable[cur_chn] = K_TRUE;
            device_obj[cur_dev].preview[cur_chn] = K_TRUE;//default enable preview
            device_obj[cur_dev].out_win[cur_chn].width = 560;
            device_obj[cur_dev].out_win[cur_chn].height = 320;          

    printf("sample_vicap: dev_count(%d), chn_count(%d)\n", dev_count, chn_count);
   

    ret = kd_mpi_get_connector_info(connector_type, &connector_info);
    if (ret) {
        printf("sample_vicap, the sensor type not supported!\n");
        return ret;
    }
    display_width = connector_info.resolution.hdisplay;
    display_height = connector_info.resolution.vdisplay;
    display_width = VICAP_ALIGN_UP(display_width, 16);
	int dev_num = 0;
            dev_attr.input_type = VICAP_INPUT_TYPE_SENSOR;
            //vicap get sensor info
            k_vicap_sensor_info sensor_info;
            ret = kd_mpi_vicap_get_sensor_info(device_obj[dev_num].sensor_type, &sensor_info);
            if (ret) {
                printf("sample_vicap, the sensor type not supported!\n");
                return ret;
            }
            memcpy(&dev_attr.sensor_info, &sensor_info, sizeof(k_vicap_sensor_info));

            device_obj[dev_num].in_width = sensor_info.width;
            device_obj[dev_num].in_height =sensor_info.height;
    
        printf("sample_vicap, dev[%d] in size[%dx%d]\n", \
            dev_num, device_obj[dev_num].in_width, device_obj[dev_num].in_height);

        //vicap device attr set
        dev_attr.acq_win.h_start = 0;
        dev_attr.acq_win.v_start = 0;
        dev_attr.acq_win.width = device_obj[dev_num].in_width;
        dev_attr.acq_win.height = device_obj[dev_num].in_height;
        dev_attr.mode = VICAP_WORK_ONLINE_MODE;
        

        dev_attr.pipe_ctrl.data = pipe_ctrl;
        dev_attr.pipe_ctrl.bits.af_enable = 0;
        dev_attr.pipe_ctrl.bits.ae_enable = device_obj[dev_num].ae_enable;
        dev_attr.pipe_ctrl.bits.awb_enable = device_obj[dev_num].awb_enable;
        dev_attr.pipe_ctrl.bits.dnr3_enable = device_obj[dev_num].dnr3_enable;
        dev_attr.pipe_ctrl.bits.ahdr_enable = device_obj[dev_num].hdr_enable;

        dev_attr.cpature_frame = 0;
        dev_attr.dw_enable = device_obj[dev_num].dw_enable;

        dev_attr.mirror = device_obj[cur_dev].sensor_mirror;

        ret = kd_mpi_vicap_set_dev_attr(dev_num, dev_attr);
        if (ret) {
            printf("sample_vicap, kd_mpi_vicap_set_dev_attr failed.\n");
            return ret;
        }
        int chn_num = 0;      
            //set default value
            device_obj[dev_num].out_format[chn_num] = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
            /*
            if (!device_obj[dev_num].out_format[chn_num]) {
                device_obj[dev_num].out_format[chn_num] = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
            }

            if (!device_obj[dev_num].out_win[chn_num].width) {
                device_obj[dev_num].out_win[chn_num].width = device_obj[dev_num].in_width;
            }

            if (!device_obj[dev_num].out_win[chn_num].height) {
                device_obj[dev_num].out_win[chn_num].height = device_obj[dev_num].in_height;
            }

            if ( device_obj[dev_num].out_win[chn_num].h_start || device_obj[dev_num].out_win[chn_num].v_start) {
                device_obj[dev_num].crop_enable[chn_num] = K_TRUE;
            }

            if ((device_obj[dev_num].out_win[chn_num].width > display_width)
                && (device_obj[dev_num].out_win[chn_num].height > display_height)) {
                device_obj[dev_num].preview[chn_num] = K_FALSE;
            }

            if (!device_obj[dev_num].rotation[chn_num]
                && ((device_obj[dev_num].out_win[chn_num].width > display_width)
                && (device_obj[dev_num].out_win[chn_num].width < display_height))) {
                device_obj[dev_num].rotation[chn_num] = 1;
            }

            printf("sample_vicap, dev_num(%d), chn_num(%d), in_size[%dx%d], out_offset[%d:%d], out_size[%dx%d]\n", \
                dev_num, chn_num, device_obj[dev_num].in_width, device_obj[dev_num].in_height, \
                device_obj[dev_num].out_win[chn_num].h_start, device_obj[dev_num].out_win[chn_num].v_start, \
                device_obj[dev_num].out_win[chn_num].width, device_obj[dev_num].out_win[chn_num].height);
        */
    

    ret = sample_vicap_vo_init(connector_type);
    if (ret) {
        printf("sample_vicap_vo_init failed\n");
        return -1;
    }

    ret = sample_vicap_vb_init(device_obj);
    if (ret) {
        printf("sample_vicap_vb_init failed\n");
        return -1;
    }
    atexit(vb_exit);

    //vicap channel attr set

            kd_mpi_vicap_set_dump_reserved(dev_num, chn_num, K_TRUE);

            memset(&chn_attr, 0, sizeof(k_vicap_chn_attr));
            chn_attr.out_win.width = device_obj[dev_num].out_win[chn_num].width;
            chn_attr.out_win.height = device_obj[dev_num].out_win[chn_num].height;
            //chn_attr.crop_win.width = device_obj[dev_num].in_width;
            //chn_attr.crop_win.height = device_obj[dev_num].in_height;
            /*if (device_obj[dev_num].out_format[chn_num] == PIXEL_FORMAT_RGB_BAYER_10BPP) {
                chn_attr.out_win.width = device_obj[dev_num].in_width;
                chn_attr.out_win.height = device_obj[dev_num].in_height;
            } else {
                chn_attr.out_win.width = device_obj[dev_num].out_win[chn_num].width;
                chn_attr.out_win.height = device_obj[dev_num].out_win[chn_num].height;
            }

            if (device_obj[dev_num].crop_enable[chn_num]) {
                chn_attr.crop_win.width = device_obj[dev_num].crop_win[chn_num].width;  //chn_attr.out_win;1166;// 
                chn_attr.crop_win.height = device_obj[dev_num].crop_win[chn_num].height; //1944;//
                chn_attr.crop_win.h_start =device_obj[dev_num].out_win[chn_num].h_start;  //713;
                chn_attr.crop_win.v_start =device_obj[dev_num].out_win[chn_num].v_start;  //0;//
            } else {
                chn_attr.crop_win.width = device_obj[dev_num].in_width;
                chn_attr.crop_win.height = device_obj[dev_num].in_height;
            }
*/
            chn_attr.scale_win = chn_attr.out_win;
            //chn_attr.crop_enable = device_obj[dev_num].crop_enable[chn_num];
            chn_attr.scale_enable = K_FALSE;
            chn_attr.chn_enable = K_TRUE;

            chn_attr.pix_format = device_obj[dev_num].out_format[chn_num];
            chn_attr.buffer_num = VICAP_OUTPUT_BUF_NUM;
            chn_attr.buffer_size = device_obj[dev_num].buf_size[chn_num];
            //chn_attr.fps = device_obj[dev_num].fps[chn_num];

            printf("sample_vicap, set dev(%d) chn(%d) attr, buffer_size(%d), out size[%dx%d]\n", \
                dev_num, chn_num, chn_attr.buffer_size, chn_attr.out_win.width, chn_attr.out_win.height);

            printf("sample_vicap out_win h_start is %d ,v_start is %d \n", chn_attr.out_win.h_start, chn_attr.out_win.v_start);

            ret = kd_mpi_vicap_set_chn_attr(dev_num, chn_num, chn_attr);
            if (ret) {
                printf("sample_vicap, kd_mpi_vicap_set_chn_attr failed.\n");
                goto vb_exit;
            }

            //bind vicap to vo, only support bind two vo chn(K_VO_DISPLAY_CHN_ID1 & K_VO_DISPLAY_CHN_ID2)
            if (device_obj[dev_num].preview[chn_num]) {//true
                k_s32 vo_chn;
                k_vo_layer layer;
                k_u16 rotation;
                if (vo_count == 0) {
                    vo_chn = K_VO_DISPLAY_CHN_ID1;
                    layer = K_VO_LAYER1;
                    rotation = device_obj[dev_num].rotation[chn_num];
                } else if (vo_count == 1) {
                    vo_chn = K_VO_DISPLAY_CHN_ID2;
                    layer = K_VO_LAYER2;
                    rotation = 4;//layer2 unsupport roation
                } 
                else if (vo_count == 2) {
                    vo_chn = K_VO_DISPLAY_CHN_ID3;
                    layer = K_VO_OSD0;
                    rotation = 4;//layer2 unsupport roation
                } 
                printf("sample_vicap, vo_count(%d), dev(%d) chn(%d) bind vo chn(%d) layer(%d) rotation(%d)\n", vo_count, dev_num, chn_num, vo_chn, layer, rotation);
                sample_vicap_bind_vo(dev_num, chn_num, vo_chn);
                layer_conf.enable[vo_count] = K_TRUE;
                layer_conf.width[vo_count] = chn_attr.out_win.width;
                layer_conf.height[vo_count] = chn_attr.out_win.height;
                layer_conf.rotation[vo_count] = rotation;
                layer_conf.layer[vo_count] = layer;
                
            }

    ret = sample_vicap_vo_layer_init(&layer_conf, display_width, display_height);
    if (ret) {
        printf("sample_vicap, vo layer init failed.\n");
        goto vb_exit;
    }
        printf("sample_vicap, vicap dev(%d) init\n", dev_num);
        ret = kd_mpi_vicap_init(dev_num);
        if (ret) {
            printf("sample_vicap, vicap dev(%d) init failed.\n", dev_num);
            //goto app_exit;
        }   
        printf("sample_vicap, vicap dev(%d) start stream\n", dev_num);
        ret = kd_mpi_vicap_start_stream(dev_num);
        if (ret) {
            printf("sample_vicap, vicap dev(%d) start stream failed.\n", dev_num);
            //goto app_exit;
        }
    kd_mpi_vo_enable();
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

vb_exit:

    return ret;
}
