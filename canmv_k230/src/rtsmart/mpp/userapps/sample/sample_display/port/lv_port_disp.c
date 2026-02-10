/**
 * @file lv_port_disp_templ.c
 *
 */

/*Copy this file as "lv_port_disp.c" and set this value to "1" to enable content*/
#if 1

/*********************
 *      INCLUDES
 *********************/
#include "lv_port_disp.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <stdbool.h>
#include "k_module.h"
#include "k_type.h"
#include "k_vb_comm.h"
#include "k_video_comm.h"
#include "k_sys_comm.h"
#include "mpi_vb_api.h"
#include "mpi_vvi_api.h"
#include "mpi_sys_api.h"
#include "k_video_comm.h"
#include "mpi_vo_api.h"
#include "k_vo_comm.h"
#include "k_connector_comm.h"
#include "mpi_connector_api.h"
#include "../cap_vio.h"

/*********************
 *      DEFINES
 *********************/
#define MY_DISP_HOR_RES    568
#define MY_DISP_VER_RES    1232
 extern k_u8 *pic_addr;
 extern k_video_frame_info vf_info;
 extern k_vo_osd osd_vo;

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void disp_init(void);

static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//        const lv_area_t * fill_area, lv_color_t color);
void lcd_show_color(const lv_area_t * area, void * px_map_color);
/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void lv_port_disp_init(void)
{
    /*-------------------------
     * Initialize your display
     * -----------------------*/
    disp_init();

    /*-----------------------------
     * Create a buffer for drawing
     *----------------------------*/

    /**
     * LVGL requires a buffer where it internally draws the widgets.
     * Later this buffer will passed to your display driver's `flush_cb` to copy its content to your display.
     * The buffer has to be greater than 1 display row
     *
     * There are 3 buffering configurations:
     * 1. Create ONE buffer:
     *      LVGL will draw the display's content here and writes it to your display
     *
     * 2. Create TWO buffer:
     *      LVGL will draw the display's content to a buffer and writes it your display.
     *      You should use DMA to write the buffer's content to the display.
     *      It will enable LVGL to draw the next part of the screen to the other buffer while
     *      the data is being sent form the first buffer. It makes rendering and flushing parallel.
     *
     * 3. Double buffering
     *      Set 2 screens sized buffers and set disp_drv.full_refresh = 1.
     *      This way LVGL will always provide the whole rendered screen in `flush_cb`
     *      and you only need to change the frame buffer's address.
     */

    /* Example for 1) */
    //static lv_disp_draw_buf_t draw_buf_dsc_1;
    //static lv_color_t buf_1[MY_DISP_HOR_RES * 10];                          /*A buffer for 10 rows*/
    //lv_disp_draw_buf_init(&draw_buf_dsc_1, buf_1, NULL, MY_DISP_HOR_RES * 10);   /*Initialize the display buffer*/

    /* Example for 2) */
    //static lv_disp_draw_buf_t draw_buf_dsc_2;
    //static lv_color_t buf_2_1[MY_DISP_HOR_RES * 10];                        /*A buffer for 10 rows*/
   // static lv_color_t buf_2_2[MY_DISP_HOR_RES * 10];                        /*An other buffer for 10 rows*/
    //lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, MY_DISP_HOR_RES * 10);   /*Initialize the display buffer*/

    /* Example for 3) also set disp_drv.full_refresh = 1 below*/
    static lv_disp_draw_buf_t draw_buf_dsc_3;
    static lv_color_t buf_3_1[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*A screen sized buffer*/
    static lv_color_t buf_3_2[MY_DISP_HOR_RES * MY_DISP_VER_RES];            /*Another screen sized buffer*/
    lv_disp_draw_buf_init(&draw_buf_dsc_3, buf_3_1, buf_3_2,MY_DISP_HOR_RES * MY_DISP_VER_RES);   /*Initialize the display buffer*/
    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = MY_DISP_HOR_RES;
    disp_drv.ver_res = MY_DISP_VER_RES;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = disp_flush;

    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_3;

    /*Required for Example 3)*/
    disp_drv.full_refresh = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/*Initialize your display and the required peripherals.*/
static void disp_init(void)
{
    /*You code here*/
     k_connector_type connector_type=RM69A10_MIPI_2LAN_568X1232_60FPS;
    vo_display_init(connector_type);
    k_u32 pool_id;
    vio_vb_config(&pool_id);
    osd_vo=K_VO_OSD3;
    osd_info osd_msg;
    k_u32 display_width=MY_DISP_HOR_RES;
    k_u32 display_height=MY_DISP_VER_RES;
    k_pixel_format pix_format=PIXEL_FORMAT_ARGB_8888;//PIXEL_FORMAT_RGB_888;//PIXEL_FORMAT_ARGB_8888
    vo_osd_config(osd_vo,&osd_msg,display_width,display_height,pix_format);
    memset(&vf_info, 0, sizeof(vf_info));
    vo_config_manage(pool_id,&vf_info,&pic_addr,osd_msg); 
    printf("disp_init,pic_addr is %p \n", pic_addr); 
    printf("pool_id:%d\n",pool_id);  
}

volatile bool disp_flush_enabled = true;

/* Enable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_enable_update(void)
{
    disp_flush_enabled = true;
}

/* Disable updating the screen (the flushing process) when disp_flush() is called by LVGL
 */
void disp_disable_update(void)
{
    disp_flush_enabled = false;
}

void lcd_show_color(const lv_area_t * area, void * px_map_color)
{
//printf("lcd_show_color,x1:%d,y1:%d,x2:%d,y2:%d\n",area->x1,area->y1,area->x2,area->y2);
vo_area sc_area;
sc_area.x1=area->x1;
sc_area.x2=area->x2;
sc_area.y1=area->y1;
sc_area.y2=area->y2;
vo_draw_color(osd_vo,&vf_info,&pic_addr,sc_area,px_map_color);
}





/*Flush the content of the internal buffer the specific area on the display
 *You can use DMA or any hardware acceleration to do this operation in the background but
 *'lv_disp_flush_ready()' has to be called when finished.*/
static void disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
//printf("disp_flush,%d\n",disp_flush_enabled==true?1:0);
    if(disp_flush_enabled) {
        /*The most simple case (but also the slowest) to put all pixels to the screen one-by-one*/

       //printf("disp_flush,x1:%d,y1:%d,x2:%d,y2:%d\n",area->x1,area->y1,area->x2,area->y2);
        //uint32_t * px_map_color = (uint32_t*)color_p; /*Let's say it's a 16 bit (RGB565) display*/
        //lcd_show_color(area,px_map_color);  
        lcd_show_color(area,color_p);
    }
    else
    {
    printf("disp_flush_enabled,false\n");
    }

    /*IMPORTANT!!!
     *Inform the graphics library that you are ready with the flushing*/
    lv_disp_flush_ready(disp_drv);
}

/*OPTIONAL: GPU INTERFACE*/

/*If your MCU has hardware accelerator (GPU) then you can use it to fill a memory with a color*/
//static void gpu_fill(lv_disp_drv_t * disp_drv, lv_color_t * dest_buf, lv_coord_t dest_width,
//                    const lv_area_t * fill_area, lv_color_t color)
//{
//    /*It's an example code which should be done by your GPU*/
//    int32_t x, y;
//    dest_buf += dest_width * fill_area->y1; /*Go to the first line*/
//
//    for(y = fill_area->y1; y <= fill_area->y2; y++) {
//        for(x = fill_area->x1; x <= fill_area->x2; x++) {
//            dest_buf[x] = color;
//        }
//        dest_buf+=dest_width;    /*Go to the next line*/
//    }
//}

#else /*Enable this file at the top*/

/*This dummy typedef exists purely to silence -Wpedantic.*/
typedef int keep_pedantic_happy;
#endif
