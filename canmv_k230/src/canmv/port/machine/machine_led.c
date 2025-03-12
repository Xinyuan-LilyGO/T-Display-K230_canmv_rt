#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "sys/ioctl.h"
#include "py/runtime.h"
#include "py/obj.h"

#define IOCTRL_WS2812_SET_RGB_VALUE     0x00

typedef union
{
    uint32_t value;
    struct 
    {
        uint8_t b;
        uint8_t r;
        uint8_t g;
        uint8_t none;
    };

} ws2812_value;

static uint8_t led_is_open = 0;
static ws2812_value led_state = {.value = 0};
int led_fd;

typedef enum
{
    LED_NONE = 0,
    LED_TYPE_RED,
    LED_TYPE_GREEN,
    LED_TYPE_BLUE
}led_type;

typedef struct 
{
    mp_obj_base_t base;
    led_type type;
}mp_led_t;

STATIC mp_obj_t machine_led_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args)
{
    led_type led_type = LED_NONE;
    if (n_args + n_kw < 1)
    {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("must be one parameter"));
    }

    if (mp_obj_is_int(args[0]))
    {
        int value = mp_obj_get_int(args[0]);
        if (value > LED_TYPE_BLUE && value < LED_TYPE_RED)
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("value > 0 && value < 4"));
        led_type = mp_obj_get_int(args[0]);
    }
    else if (mp_obj_is_str(args[0]))
    {
        if (strcmp(mp_obj_str_get_str(args[0]),"LED_RED") == 0)
            led_type = LED_TYPE_RED;
        else if (strcmp(mp_obj_str_get_str(args[0]),"LED_GREEN") == 0)
            led_type = LED_TYPE_GREEN;
        else if (strcmp(mp_obj_str_get_str(args[0]),"LED_BLUE") == 0)
            led_type = LED_TYPE_BLUE;
        else
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("value error"));
    }

    if (led_is_open == 0)
    {
        led_fd = open("/dev/ws2812",O_RDWR);
        if (led_fd < 0) {
            mp_raise_OSError_with_filename(errno, "/dev/ws2812");
        }
        led_is_open = 1;
    }

    mp_led_t *self =  m_new_obj(mp_led_t);
    if (self == NULL) 
    {
        mp_raise_msg(&mp_type_MemoryError, MP_ERROR_TEXT("obj new failed"));
        return mp_const_none;
    }

    self->type = led_type;
    self->base.type = type;
    return self;
}

static mp_obj_t _led_set_value(led_type type,uint8_t value)
{
    int ret;

    switch (type)
    {
    case LED_TYPE_RED:
        led_state.r = value;
        ret = ioctl(led_fd,IOCTRL_WS2812_SET_RGB_VALUE,&led_state);
        if (ret) {
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("ioctl error"));
        }
        break;
    case LED_TYPE_GREEN:
        led_state.g = value;
        ret = ioctl(led_fd,IOCTRL_WS2812_SET_RGB_VALUE,&led_state);
        if (ret)
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("ioctl error"));
        break; 
    case LED_TYPE_BLUE:
        led_state.b = value;
        ret = ioctl(led_fd,IOCTRL_WS2812_SET_RGB_VALUE,&led_state);
        if (ret)
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("ioctl error"));
        break;
    default:
        break;
    }

    return mp_const_true;
}

STATIC mp_obj_t machine_led_on(mp_obj_t self_in) 
{
    mp_led_t *self = MP_OBJ_TO_PTR(self_in);
    return _led_set_value(self->type,255);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_led_on_obj, machine_led_on);

STATIC mp_obj_t machine_led_off(mp_obj_t self_in) 
{
    mp_led_t *self = MP_OBJ_TO_PTR(self_in);
    return _led_set_value(self->type,0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_led_off_obj, machine_led_off);

STATIC mp_obj_t machine_led_value(mp_obj_t self_in,mp_obj_t value_in) 
{
    mp_led_t *self = MP_OBJ_TO_PTR(self_in);
    int value = mp_obj_get_int(value_in);

    if (value > 255)
        value = 255;
    else if (value < 0)
        value = 0;

    return _led_set_value(self->type,value);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(machine_led_value_obj, machine_led_value);

STATIC const mp_rom_map_elem_t machine_led_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_on), MP_ROM_PTR(&machine_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_off), MP_ROM_PTR(&machine_led_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_value), MP_ROM_PTR(&machine_led_value_obj) },

    { MP_ROM_QSTR(MP_QSTR_LED_RED), MP_ROM_INT(LED_TYPE_RED) },
    { MP_ROM_QSTR(MP_QSTR_LED_GREEN), MP_ROM_INT(LED_TYPE_GREEN) },
    { MP_ROM_QSTR(MP_QSTR_LED_BLUE), MP_ROM_INT(LED_TYPE_BLUE) },
};
STATIC MP_DEFINE_CONST_DICT(machine_led_locals_dict, machine_led_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_led_type,
    MP_QSTR_LED,
    MP_TYPE_FLAG_NONE,
    make_new, machine_led_make_new,
    locals_dict, &machine_led_locals_dict
    );
