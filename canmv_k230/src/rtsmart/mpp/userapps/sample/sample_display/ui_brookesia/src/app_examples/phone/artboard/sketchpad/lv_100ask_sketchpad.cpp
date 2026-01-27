/**
 * @file lv_100ask_sketchpad.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "lv_100ask_sketchpad.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include "sys/ioctl.h"
//#if LV_USE_100ASK_SKETCHPAD != 0

/*********************
 *      DEFINES
 *********************/
#define MY_CLASS &lv_100ask_sketchpad_class

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_100ask_sketchpad_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_100ask_sketchpad_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_100ask_sketchpad_toolbar_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_100ask_sketchpad_toolbar_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj);
static void lv_100ask_sketchpad_event(const lv_obj_class_t * class_p, lv_event_t * e);
static void lv_100ask_sketchpad_toolbar_event(const lv_obj_class_t * class_p, lv_event_t * e);
static void sketchpad_toolbar_event_cb(lv_event_t * e);
static void toolbar_set_event_cb(lv_event_t * e);

/**********************
 *  STATIC VARIABLES
 **********************/
const _lv_obj_class_t lv_100ask_sketchpad_class = {
    .base_class = &lv_img_class,
    .constructor_cb = lv_100ask_sketchpad_constructor,
    .destructor_cb = lv_100ask_sketchpad_destructor,
    .event_cb = lv_100ask_sketchpad_event,
    .instance_size = sizeof(lv_100ask_sketchpad_t),
    
    //.name = "sketchpad",
};

const _lv_obj_class_t lv_100ask_sketchpad_toolbar_class = {
    .base_class = &lv_obj_class,
    .constructor_cb = lv_100ask_sketchpad_toolbar_constructor,
    .destructor_cb = lv_100ask_sketchpad_toolbar_destructor,
    .event_cb = lv_100ask_sketchpad_toolbar_event,
    .width_def = LV_SIZE_CONTENT,
    .height_def = LV_SIZE_CONTENT,
    
    //.name = "sketchpad_toolbar",
};
int fd_touch_draw=0;
 struct rt_touch_data_draw {
    uint8_t event; /* The touch event of the data */
    uint8_t track_id; /* Track id of point */
    uint8_t width; /* Point of width */
    uint16_t x_coordinate; /* Point of x coordinate */
    uint16_t y_coordinate; /* Point of y coordinate */
    uint32_t timestamp; /* The timestamp when the data was received */
};
/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

lv_obj_t * lv_100ask_sketchpad_create(lv_obj_t * parent)
{
    LV_LOG_INFO("begin");
    lv_obj_t * obj = lv_obj_class_create_obj(MY_CLASS, parent);
    lv_obj_class_init_obj(obj);
    int index=1;
    char dev_name[16] = "/dev/touch0";
    dev_name[10] = '0' + index;
    if(fd_touch_draw==0)
    {
    fd_touch_draw = open(dev_name, O_RDWR);
    }

    return obj;
}


/*=====================
 * Other functions
 *====================*/

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void lv_100ask_sketchpad_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");

    lv_100ask_sketchpad_t * sketchpad = (lv_100ask_sketchpad_t *)obj;

    lv_draw_line_dsc_init(&sketchpad->line_rect_dsc);
    sketchpad->line_rect_dsc.width = 4;
    sketchpad->line_rect_dsc.round_start = true;
    sketchpad->line_rect_dsc.round_end = true;
    sketchpad->line_rect_dsc.color = lv_palette_main(LV_PALETTE_RED);
    //sketchpad->line_rect_dsc.opa = LV_OPA_COVER;

    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);

    /*toolbar*/
    lv_obj_t * toolbar = lv_obj_class_create_obj(&lv_100ask_sketchpad_toolbar_class, obj); //TODO
    lv_obj_class_init_obj(toolbar);

    LV_TRACE_OBJ_CREATE("finished");
}


static void lv_100ask_sketchpad_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");

    //lv_canvas_t * canvas = (lv_canvas_t *)obj;
    //if(canvas->draw_buf == NULL) return;

    //lv_image_cache_drop(&canvas->draw_buf);
}


static void lv_100ask_sketchpad_event(const lv_obj_class_t * class_p, lv_event_t * e)
{
    LV_UNUSED(class_p);
    lv_res_t res;

    /*Call the ancestor's event handler*/
    res = lv_obj_event_base(MY_CLASS, e);
    if(res != LV_RES_OK) return;
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e); 
    lv_100ask_sketchpad_t * sketchpad = (lv_100ask_sketchpad_t *)obj;
    //printf("code:%d\n",code);
    static lv_coord_t last_x = -32768, last_y = -32768;

    if (code == LV_EVENT_PRESSING)
    {
    printf("LV_EVENT_PRESSING....\n");
    lv_point_t point;
    int point_number =1;
 struct rt_touch_data_draw touch_data[point_number];
    point_number = read(fd_touch_draw, touch_data, sizeof(touch_data));
    point_number /= sizeof(struct rt_touch_data_draw);
  if(point_number)
  {  
  point.x=touch_data[0].x_coordinate;
  point.y=touch_data[0].y_coordinate;
  }
  else
  {
  printf("------point_number:0\n");
  return;
  }
  
    //printf("touch:(%d,%d)\n",point.x,point.y);
    //printf("sketchpad:(%d,%d)\n",sketchpad->pos.x,sketchpad->pos.y);
     point.x = point.x - sketchpad->pos.x;
     point.y = point.y - sketchpad->pos.y;

     //lv_canvas_set_px(obj, point.x, point.y, lv_color_hex(0x0000FF));

     //printf("curr_point:(%d,%d)->(%d,%d)\n",last_x,last_y,point.x,point.y);
        /*Release or first use*/
        if ((last_x != -32768) && (last_y != -32768))
        {
        
        
        
	lv_point_t linePoint[2];
	linePoint[0].x=last_x;
	linePoint[0].y=last_y;
	linePoint[1].x=point.x;
	linePoint[1].y=point.y;
	
	lv_canvas_draw_line(obj, linePoint, 2, &sketchpad->line_rect_dsc); 
        lv_event_send(obj, LV_EVENT_VALUE_CHANGED, NULL);
           
        }

        last_x = point.x;
        last_y = point.y;
    
    #if 0
        lv_indev_t * indev = lv_indev_active();
        if(indev == NULL)  return;

        lv_point_t point;

        lv_indev_get_point(indev, &point);

        point.x = point.x - sketchpad->pos.x;
        point.y = point.y - sketchpad->pos.y;

        /*Release or first use*/
        if ((last_x != -32768) && (last_y != -32768))
        {
            lv_layer_t layer;
            lv_canvas_init_layer(obj, &layer);

            sketchpad->line_rect_dsc.p1.x = last_x;
            sketchpad->line_rect_dsc.p1.y = last_y;
            sketchpad->line_rect_dsc.p2.x = point.x;
            sketchpad->line_rect_dsc.p2.y = point.y;

            lv_draw_line(&layer, &sketchpad->line_rect_dsc);

            lv_canvas_finish_layer(obj, &layer);

            lv_obj_send_event(obj, LV_EVENT_VALUE_CHANGED, NULL);
        }

        last_x = point.x;
        last_y = point.y;
        #endif
    }

    /*Loosen the brush*/
    else if(code == LV_EVENT_RELEASED)
    {
        printf("LV_EVENT_RELEASED....\n");
        last_x = -32768;
        last_y = -32768;
    }
    else if((code == LV_EVENT_STYLE_CHANGED) || (code == LV_EVENT_SIZE_CHANGED))
    {
        printf("LV_EVENT_STYLE_CHANGED,LV_EVENT_SIZE_CHANGED....\n");
        lv_obj_refr_pos(obj);
        sketchpad->pos.x = lv_obj_get_x(obj);
        sketchpad->pos.y = lv_obj_get_y(obj);     
    }
}



/*toolbar*/
static void lv_100ask_sketchpad_toolbar_constructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{
    LV_UNUSED(class_p);
    LV_TRACE_OBJ_CREATE("begin");

    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(obj, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_grow(obj, 1);
    lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_ROW);

    static lv_coord_t sketchpad_toolbar_cw = LV_100ASK_SKETCHPAD_TOOLBAR_OPT_CW;
    lv_obj_t * color = lv_label_create(obj);
    lv_obj_set_size(color, 100, 50);
    lv_label_set_text(color, LV_SYMBOL_EDIT);
    lv_obj_add_flag(color, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(color, sketchpad_toolbar_event_cb, LV_EVENT_ALL, &sketchpad_toolbar_cw); //TODO

    static lv_coord_t sketchpad_toolbar_width = LV_100ASK_SKETCHPAD_TOOLBAR_OPT_WIDTH;
    lv_obj_t * size = lv_label_create(obj);
    lv_obj_set_size(size, 100, 50);
    lv_label_set_text(size, LV_SYMBOL_EJECT);
    lv_obj_add_flag(size, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(size, sketchpad_toolbar_event_cb, LV_EVENT_ALL, &sketchpad_toolbar_width); //TODO

    LV_TRACE_OBJ_CREATE("finished");
}


static void lv_100ask_sketchpad_toolbar_destructor(const lv_obj_class_t * class_p, lv_obj_t * obj)
{

}


static void lv_100ask_sketchpad_toolbar_event(const lv_obj_class_t * class_p, lv_event_t * e)
{
    LV_UNUSED(class_p);

    lv_res_t res;

    /*Call the ancestor's event handler*/
    res = lv_obj_event_base(&lv_100ask_sketchpad_toolbar_class, e);
    if(res != LV_RES_OK) return;

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);

    if (code == LV_EVENT_PRESSING)
    { 
    printf("toolbar  EVENT_PRESSING\n");
     lv_point_t point;
    int point_number =1;
 struct rt_touch_data_draw touch_data[point_number];
    point_number = read(fd_touch_draw, touch_data, sizeof(touch_data));
    point_number /= sizeof(struct rt_touch_data_draw);
  if(point_number)
  {  
  point.x=touch_data[0].x_coordinate;
  point.y=touch_data[0].y_coordinate;
  }
  else
  {
  printf("------point_number:0\n");
  return;
  }
  lv_obj_set_pos(obj, point.x, point.y);
  
  
        /*lv_indev_t * indev = lv_indev_get_act();
        if(indev == NULL)  return;

        lv_point_t vect;
        lv_indev_get_vect(indev, &vect);

        lv_coord_t x = lv_obj_get_x(obj) + vect.x;
        lv_coord_t y = lv_obj_get_y(obj) + vect.y;
        lv_obj_set_pos(obj, x, y);*/
    }
}

static void sketchpad_toolbar_event_cb(lv_event_t * e)
{
    lv_coord_t *toolbar_opt = (lv_coord_t *)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    lv_obj_t * toolbar= lv_obj_get_parent(obj);
    lv_obj_t * sketchpad= lv_obj_get_parent(toolbar);
    lv_100ask_sketchpad_t * sketchpad_t = (lv_100ask_sketchpad_t *)sketchpad;


    if (code == LV_EVENT_CLICKED)
    {
        if ((*toolbar_opt) == LV_100ASK_SKETCHPAD_TOOLBAR_OPT_CW)
        {
            static lv_coord_t sketchpad_toolbar_cw = LV_100ASK_SKETCHPAD_TOOLBAR_OPT_CW;
            lv_obj_t * cw = lv_colorwheel_create(sketchpad, true); //TODO
            lv_obj_align_to(cw, obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
            lv_obj_add_event_cb(cw, toolbar_set_event_cb, LV_EVENT_RELEASED, &sketchpad_toolbar_cw);
        }
        else if((*toolbar_opt) == LV_100ASK_SKETCHPAD_TOOLBAR_OPT_WIDTH)
        {
            static lv_coord_t sketchpad_toolbar_width = LV_100ASK_SKETCHPAD_TOOLBAR_OPT_WIDTH;
            lv_obj_t * slider = lv_slider_create(sketchpad);
            lv_slider_set_value(slider, (int32_t)(sketchpad_t->line_rect_dsc.width), LV_ANIM_OFF);
            lv_obj_align_to(slider, obj, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
            lv_obj_add_event_cb(slider, toolbar_set_event_cb, LV_EVENT_ALL, &sketchpad_toolbar_width);
        }
    }
}



static void toolbar_set_event_cb(lv_event_t * e)
{
    lv_coord_t *toolbar_opt = (lv_coord_t *)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    lv_100ask_sketchpad_t * sketchpad = (lv_100ask_sketchpad_t *)lv_obj_get_parent(obj);

    if (code == LV_EVENT_RELEASED)
    {
        if ((*toolbar_opt) == LV_100ASK_SKETCHPAD_TOOLBAR_OPT_CW)
        {
            sketchpad->line_rect_dsc.color = lv_colorwheel_get_rgb(obj);  //TODO
            lv_obj_del(obj);
        }
        else if (*(toolbar_opt) == LV_100ASK_SKETCHPAD_TOOLBAR_OPT_WIDTH)
        {
            lv_obj_del(obj);
        }
    }
    else if (code == LV_EVENT_VALUE_CHANGED)
    {
        if((*toolbar_opt) == LV_100ASK_SKETCHPAD_TOOLBAR_OPT_WIDTH)
        {
            sketchpad->line_rect_dsc.width = (lv_coord_t)lv_slider_get_value(obj);
        }
    }
}


//#endif  /*LV_USE_100ASK_SKETCHPAD*/
