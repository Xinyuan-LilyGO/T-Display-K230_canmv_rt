/**
 * @file lv_port_disp_templ.h
 *
 */

/*Copy this file as "lv_port_disp.h" and set this value to "1" to enable content*/

#ifndef CAP_VIO_H
#define CAP_VIO_H
/*********************
 *      INCLUDES
 *********************/
 #include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "k_type.h"
#include "k_vicap_comm.h"
#include "k_connector_comm.h"
#include "rt_cache.h"
#ifdef __cplusplus
extern "C" {
#endif


/*********************
 *      DEFINES
 *********************/


/**********************
 *      TYPEDEFS
 **********************/
 typedef struct {
    int x1;
    int y1;
    int x2;
    int y2;
} vo_area;
typedef struct
{
    k_u64 osd_phy_addr;
    void *osd_virt_addr;
    k_pixel_format format;
    k_vo_point offset;
    k_vo_size act_size;
    k_u32 size;
    k_u32 stride;
    k_u8 global_alptha;
} osd_info;

typedef struct
{
    k_u64 layer_phy_addr;
    k_pixel_format format;
    k_vo_point offset;
    k_vo_size act_size;
    k_u32 size;
    k_u32 stride;
    k_u8 global_alptha;

    //only layer0、layer1
    k_u32 func;
    // only layer0
    k_vo_scaler_attr attr;

} layer_info;
/**********************
 * GLOBAL PROTOTYPES
 **********************/
/* Initialize low level display driver */
void vo_display_init(k_connector_type connector_type);
void vo_display_set_brightness(int bright_val);
k_u32 vb_create_pool(k_u32 pool_blk_cnt,k_u64 pool_blk_size);
void vio_vb_config(k_u32* pool_id);
void vo_layer_config(k_vo_layer vo_layer,k_u32 display_width, k_u32 display_height,k_pixel_format pix_format,k_u16 _rotation);

void vo_osd_config(k_vo_osd osd_vo,osd_info* osd_msg,k_u32 display_width, k_u32 display_height,k_pixel_format pix_format);
/* 
 */
void vo_config_manage(k_u32 pool_id,k_video_frame_info* vf_info,k_u8 **pic_addr_cur,osd_info osd_msg);

void vo_draw_color(k_vo_osd osd_vo_cur,k_video_frame_info* vf_info,k_u8 **pic_addr_cur,vo_area area, void * px_map_color);

void vi_sensor_init(k_vicap_sensor_type sensor_type);
void vi_chn_config(k_s32 dev_num, k_s32 chn_num,k_u32 display_width, k_u32 display_height,k_pixel_format pix_format);
void vi_bind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn);
void vi_unbind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn);
void vio_start_stream(k_s32 vicap_dev);
void vio_stop_stream(k_s32 vicap_dev);

void vi_bind_venc(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 venc_dev,k_s32 venc_chn);
void vi_unbind_venc(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 venc_dev,k_s32 venc_chn);

void venc_chn_config(k_s32 chn_num,k_u32 display_width, k_u32 display_height);
void venc_write_data(k_s32 chn_num);
void venc_write_data_to_file(k_s32 chn_num,char* venc_file);
void venc_stop_stream(k_s32 venc_chn);
void vdec_chn_config(k_s32 chn_num,k_u32 display_width, k_u32 display_height);
void vdec_bind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn);
void vdec_unbind_vo(k_s32 vicap_dev, k_s32 vicap_chn, k_s32 vo_chn);
void vdec_read_data(k_s32 chn_num,char* venc_file);
void venc_write_data_to_cache(k_s32 chn_num,pkg_cache_t *cache,int is_save_file);
void vdec_read_data_live(k_s32 chn_num,char* venc_file);
void vdec_process_data(k_s32 chn_num,k_u32 pool_id,uint8_t * data,int data_len,int is_end);
int vdec_output(k_s32 chn_num);
void vdec_stop_stream(k_s32 vdec_chn);
/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*CAP_VIO_H*/

