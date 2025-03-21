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
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include "py/runtime.h"
#include "py/obj.h"
#include "mpi_ai_api.h"

#include <fcntl.h>
#include <sys/ioctl.h>

#define FUNC_IMPL
#define FUNC_FILE "ai_func_def.h"
#include "func_def.h"

STATIC mp_obj_t ai_set_vol(mp_obj_t chn_obj, mp_obj_t vol_obj) {
    int fd = -1;
    bool succ = true;
    float vol_value = 30.0f - 50.0f;
    mp_int_t vol = mp_obj_get_int(vol_obj);
    mp_int_t chn = mp_obj_get_int(chn_obj);

    if(0 > (fd = open("/dev/acodec_device", O_RDWR))) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("could not open acodec_device device"));
    }

    vol_value += ((float)vol) * 0.5f;

    if(vol_value > 30.0f) {
        vol_value = 30.0f;
    }

    if((-97.0f) > vol_value) {
        vol_value = -97.0f;
    }

    // LEFT
    if(0x01 == (chn & 0x01)) {
        if(0x00 != ioctl(fd, 12, &vol_value)) {
            succ = false;
            mp_printf(&mp_plat_print, "set mic[l] vol failed\n");
        }
    }

    // RIGHT
    if(0x02 == (chn & 0x02)) {
        if(0x00 != ioctl(fd, 13, &vol_value)) {
            succ = false;
            mp_printf(&mp_plat_print, "set mic[r] vol failed\n");
        }
    }

    close(fd);

    return mp_obj_new_bool(succ);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(ai_set_vol_obj, ai_set_vol);

STATIC mp_obj_t ai_get_vol(void) {
    int fd = -1;
    float vol_value;
    int vol = 0;
    const float volume_zero_value = 30.0f - 50.0f;

    if(0 > (fd = open("/dev/acodec_device", O_RDWR))) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("could not open acodec_device device"));
    }

    mp_obj_t tuple_o = mp_obj_new_tuple(2, MP_OBJ_NULL);
    mp_obj_tuple_t *tuple = MP_OBJ_TO_PTR(tuple_o);

    // get left channel
    {
        if(0x00 != ioctl(fd, 26, &vol_value)) {
            mp_printf(&mp_plat_print, "get mic[l] left vol failed\n");
        }

        if((volume_zero_value) > vol_value) {
            vol = 0;
        } else if(30.0f < vol_value) {
            vol = 100;
        } else {
            vol = (int)((vol_value - volume_zero_value) / 0.5f);
        }

        tuple->items[0] = mp_obj_new_int(vol);
    }

    // get right channel
    {
        if(0x00 != ioctl(fd, 27, &vol_value)) {
            mp_printf(&mp_plat_print, "get mic[r] vol failed\n");
        }

        if((volume_zero_value) > vol_value) {
            vol = 0;
        } else if(30.0f < vol_value) {
            vol = 100;
        } else {
            vol = (int)((vol_value - volume_zero_value) / 0.5f);
        }

        tuple->items[1] = mp_obj_new_int(vol);
    }

    close(fd);

    return tuple_o;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(ai_get_vol_obj, ai_get_vol);

STATIC const mp_rom_map_elem_t ai_api_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ai_api) },
    { MP_ROM_QSTR(MP_QSTR_ai_set_vol),  MP_ROM_PTR(&ai_set_vol_obj) },
    { MP_ROM_QSTR(MP_QSTR_ai_get_vol),  MP_ROM_PTR(&ai_get_vol_obj) },
#define FUNC_ADD
#define FUNC_FILE "ai_func_def.h"
#include "func_def.h"
};
STATIC MP_DEFINE_CONST_DICT(ai_api_locals_dict, ai_api_locals_dict_table);

const mp_obj_module_t mp_module_ai_api = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&ai_api_locals_dict,
};
