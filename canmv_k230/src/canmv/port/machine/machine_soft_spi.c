/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "py/runtime.h"
#include "machine_soft_spi.h"
#include "mpprint.h"
#include "sys/ioctl.h"
#include "py/obj.h"

struct rt_spi_priv_data {
    const void *send_buf;
    size_t send_length;
    void *recv_buf;
    size_t recv_length;
};

/*struct rt_spi_configuration {
    uint8_t mode;
    uint8_t data_width;
    uint16_t reserved;
    uint32_t max_hz;
};*/
/*
enum {
    DWENUM_SPI_TXRX = 0,
    DWENUM_SPI_TX = 1,
    DWENUM_SPI_RX = 2,
    DWENUM_SPI_EEPROM = 3
};
*/
enum {
    MP_SPI_IOCTL_INIT,
    MP_SPI_IOCTL_DEINIT,
    MP_SPI_IOCTL_TRANSFER,
};
/*
typedef struct {
    uint32_t baud;
    uint8_t polarity;
    uint8_t phase;
    uint8_t bits;
    uint8_t reserver;
} mp_soft_spi_obj_t;

typedef struct {
    mp_obj_base_t base;
    mp_soft_spi_obj_t spi;
    int fd;
    uint8_t index;
    uint8_t status;
} machine_spi_obj_t;
*/



// SPI protocol
typedef struct _mp_machine_soft_spi_p_t {
    void (*init)(mp_obj_base_t *obj, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);
    void (*deinit)(mp_obj_base_t *obj); // can be NULL
    void (*transfer)(mp_obj_base_t *obj, size_t len, const void *src, void *dest);
} mp_machine_soft_spi_p_t;

typedef struct _mp_soft_spi_obj_t {
    uint32_t delay_half; // microsecond delay for half SCK period
    uint8_t polarity;
    uint8_t phase;
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
} mp_soft_spi_obj_t;



typedef struct {
    mp_obj_base_t base;
    mp_soft_spi_obj_t spi;
	int fd;
    uint8_t index;
    uint8_t status;
} machine_soft_spi_obj_t;

static bool soft_spi_used[3];
 STATIC void mp_machine_soft_spi_transfer(mp_obj_base_t *self_in, size_t len, const void *src, void *dest);

STATIC mp_obj_t mp_machine_soft_spi_read(size_t n_args, const mp_obj_t *args) {
    vstr_t vstr;
    vstr_init_len(&vstr, mp_obj_get_int(args[1]));
    memset(vstr.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, vstr.len);
    mp_machine_soft_spi_transfer(args[0], vstr.len, vstr.buf,vstr.buf);
    return mp_obj_new_bytes_from_vstr(&vstr);
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_soft_spi_read_obj, 2, 3, mp_machine_soft_spi_read);

STATIC mp_obj_t mp_machine_soft_spi_readinto(size_t n_args, const mp_obj_t *args) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[1], &bufinfo, MP_BUFFER_WRITE);
    memset(bufinfo.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, bufinfo.len);
    mp_machine_soft_spi_transfer(args[0], bufinfo.len, bufinfo.buf, bufinfo.buf);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_machine_soft_spi_readinto_obj, 2, 3, mp_machine_soft_spi_readinto);

STATIC mp_obj_t mp_machine_soft_spi_write(mp_obj_t self, mp_obj_t wr_buf) {
    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);
    mp_machine_soft_spi_transfer(self, src.len, (const uint8_t *)src.buf, NULL);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(mp_machine_soft_spi_write_obj, mp_machine_soft_spi_write);

STATIC mp_obj_t mp_machine_soft_spi_write_readinto(mp_obj_t self, mp_obj_t wr_buf, mp_obj_t rd_buf) {
    mp_buffer_info_t src;
    mp_get_buffer_raise(wr_buf, &src, MP_BUFFER_READ);
    mp_buffer_info_t dest;
    mp_get_buffer_raise(rd_buf, &dest, MP_BUFFER_WRITE);
     if (src.len != dest.len) {
         mp_raise_ValueError(MP_ERROR_TEXT("buffers must be the same length"));
     }
    mp_machine_soft_spi_transfer(self, src.len, src.buf, dest.buf);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_3(mp_machine_soft_spi_write_readinto_obj, mp_machine_soft_spi_write_readinto);


/******************************************************************************/
// MicroPython bindings for generic machine.SoftSPI



STATIC uint32_t baudrate_from_delay_half(uint32_t delay_half) {

   return 500000 / delay_half;
}

STATIC uint32_t baudrate_to_delay_half(uint32_t baudrate) {
   	uint32_t delay_half = 500000 / baudrate;
		  // round delay_half up so that: actual_baudrate <= requested_baudrate
		  if (500000 % baudrate != 0) {
			  delay_half += 1;
		  }
		  return delay_half;
}

STATIC void machine_soft_spi_obj_check(machine_soft_spi_obj_t *self) {
    if (self->status == 0) {
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("The Soft SPI object has been deleted"));
    }
}

STATIC void machine_soft_spi_init_helper(machine_soft_spi_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
	enum { ARG_baudrate, ARG_polarity, ARG_phase, ARG_bits, ARG_firstbit, ARG_sck, ARG_mosi, ARG_miso };
		   static const mp_arg_t allowed_args[] = {
			   { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = 500000} },
			   { MP_QSTR_polarity, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
			   { MP_QSTR_phase,    MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
			   { MP_QSTR_bits,	   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 8} },
			   { MP_QSTR_firstbit, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
			   //{ MP_QSTR_sck,    MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
			  // { MP_QSTR_mosi,	   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
			   //{ MP_QSTR_miso,	   MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
			   { MP_QSTR_sck,	   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
			   { MP_QSTR_mosi,	   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
			   { MP_QSTR_miso,	   MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
		   };
		  // mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
		   //mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
		  mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
		  mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
		   // create new object
		   //machine_soft_spi_obj_t *self = mp_obj_malloc(machine_soft_spi_obj_t, &machine_soft_spi_type);
		
		   // set parameters
		   self->spi.delay_half = baudrate_to_delay_half(args[ARG_baudrate].u_int);
		   self->spi.polarity = args[ARG_polarity].u_int;
		   self->spi.phase = args[ARG_phase].u_int;
		   if (args[ARG_bits].u_int != 8) {
			   mp_raise_ValueError(MP_ERROR_TEXT("bits must be 8"));
		   }
		   if (args[ARG_firstbit].u_int != 0) {
			   mp_raise_ValueError(MP_ERROR_TEXT("firstbit must be MSB"));
		   }
		  /* if (args[ARG_sck].u_obj == MP_OBJ_NULL
			   || args[ARG_mosi].u_obj == MP_OBJ_NULL
			   || args[ARG_miso].u_obj == MP_OBJ_NULL) {
			   mp_raise_ValueError(MP_ERROR_TEXT("must specify all of sck/mosi/miso"));
		   }
		   self->spi.sck = mp_hal_get_pin_obj(args[ARG_sck].u_obj);
		   self->spi.mosi = mp_hal_get_pin_obj(args[ARG_mosi].u_obj);
		   self->spi.miso = mp_hal_get_pin_obj(args[ARG_miso].u_obj);*/
		   self->spi.sck = args[ARG_sck].u_int;
		   self->spi.mosi = args[ARG_mosi].u_int;
		   self->spi.miso = args[ARG_miso].u_int;
		   ioctl(self->fd,MP_SPI_IOCTL_INIT,&(self->spi));
        /*mp_soft_spi_obj_t* soft_spi_obj_data=mp_local_alloc(sizeof(mp_soft_spi_obj_t));
		soft_spi_obj_data->delay_half=self->spi.delay_half;
		soft_spi_obj_data->phase=self->spi.phase;
		soft_spi_obj_data->polarity=self->spi.polarity;
		soft_spi_obj_data->sck=self->spi.sck;
		soft_spi_obj_data->mosi=self->spi.mosi;
		soft_spi_obj_data->miso=self->spi.miso;
		ioctl(self->fd,MP_SPI_IOCTL_INIT,soft_spi_obj_data);
		mp_local_free(soft_spi_obj_data);  */
}

STATIC mp_obj_t machine_soft_spi_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    machine_soft_spi_obj_check(args[0]);
    machine_soft_spi_init_helper(args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_soft_spi_init_obj, 1, machine_soft_spi_init);





STATIC void machine_soft_spi_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_soft_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    machine_soft_spi_obj_check(self);
	mp_printf(print, "Soft SPI %u", self->index);
   mp_printf(print, "SoftSPI(baudrate=%u, polarity=%u, phase=%u,sck=%u,mosi=%u,miso=%u)",
        baudrate_from_delay_half(self->spi.delay_half), self->spi.polarity, self->spi.phase,
        self->spi.sck, self->spi.mosi, self->spi.miso);
   //mp_hal_pin_name(self->spi.miso)
}

STATIC mp_obj_t machine_soft_spi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
	
	   // configure bus
	   //mp_soft_spi_ioctl(&self->spi, MP_SPI_IOCTL_INIT);//暂时无实现

	   mp_arg_check_num(n_args, n_kw, 1, 8, true);
	     int index = mp_obj_get_int(args[0]);
    	if (index < 0 || index > 2) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid spi number"));
   		 }
   		 if (soft_spi_used[index]) {
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT(" soft SPI %u busy"), index);
    	}
	   char dev_name[16] = "/dev/soft_spi0";
	   dev_name[13] = '0' + index;
	   int fd = open(dev_name, O_RDWR);
	   if (fd < 0) {
			mp_raise_OSError_with_filename(errno, dev_name);
	     }
		   
	    machine_soft_spi_obj_t *self = m_new_obj_with_finaliser(machine_soft_spi_obj_t);
		self->base.type = &machine_soft_spi_type;
		self->index = index;
		self->fd = fd;
		self->status = 1;
		   
		mp_map_t kw_args;
		mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
		machine_soft_spi_init_helper(self, n_args - 1, args + 1, &kw_args);
		self->status = 2;
		soft_spi_used[index] = 1;
		return MP_OBJ_FROM_PTR(self);
}



STATIC mp_obj_t machine_soft_spi_deinit(mp_obj_t self_in) {
    machine_soft_spi_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->status == 0)
        return mp_const_none;
    close(self->fd);
    if (self->status == 2)
        soft_spi_used[self->index] = 0;
    self->status = 0;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_soft_spi_deinit_obj, machine_soft_spi_deinit);

STATIC const mp_rom_map_elem_t machine_soft_spi_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&machine_soft_spi_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_soft_spi_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&machine_soft_spi_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_machine_soft_spi_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_machine_soft_spi_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_machine_soft_spi_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_write_readinto), MP_ROM_PTR(&mp_machine_soft_spi_write_readinto_obj) },
};
MP_DEFINE_CONST_DICT(mp_machine_soft_spi_locals_dict, machine_soft_spi_locals_dict_table);


/*
int mp_machine_i2c_read(mp_obj_base_t *self_in, uint8_t *dest, size_t len, bool nack) {
    machine_i2c_obj_t *self = (machine_i2c_obj_t *)self_in;
    machine_i2c_obj_check(self);

    int ret = read(self->fd, dest, len);

    return ret;
}

int mp_machine_i2c_write(mp_obj_base_t *self_in, const uint8_t *src, size_t len) {
    machine_i2c_obj_t *self = (machine_i2c_obj_t *)self_in;
    machine_i2c_obj_check(self);

    int ret = write(self->fd, src, len);

    return ret;
}

*/
	STATIC void mp_machine_soft_spi_init(mp_obj_base_t *self_in, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
		machine_soft_spi_obj_t *self = (machine_soft_spi_obj_t *)self_in;
		machine_soft_spi_obj_check(self);
		ioctl(self->fd,MP_SPI_IOCTL_INIT,&(self->spi));
		/*mp_soft_spi_obj_t* soft_spi_obj_data=mp_local_alloc(sizeof(mp_soft_spi_obj_t));
				soft_spi_obj_data->delay_half=self->spi.delay_half;
				soft_spi_obj_data->phase=self->spi.phase;
				soft_spi_obj_data->polarity=self->spi.polarity;
				soft_spi_obj_data->sck=self->spi.sck;
				soft_spi_obj_data->mosi=self->spi.mosi;
				soft_spi_obj_data->miso=self->spi.miso;
				ioctl(self->fd,MP_SPI_IOCTL_INIT,soft_spi_obj_data);
				mp_local_free(soft_spi_obj_data);*/	
	}

	STATIC void mp_machine_soft_spi_transfer(mp_obj_base_t *self_in, size_t len, const void *src, void *dest) {
		machine_soft_spi_obj_t *self = (machine_soft_spi_obj_t *)self_in;
		machine_soft_spi_obj_check(self);
		struct rt_spi_priv_data *priv_data = mp_local_alloc(sizeof(struct rt_spi_priv_data));
		priv_data->send_buf = src;
		priv_data->send_length = len;
		priv_data->recv_buf = dest;
		//priv_data->recv_length = dest_len;
		ioctl(self->fd, MP_SPI_IOCTL_TRANSFER, priv_data);
		mp_local_free(priv_data);   
	}

STATIC const mp_machine_soft_spi_p_t mp_machine_soft_spi_p = {
    .init = mp_machine_soft_spi_init,
    .deinit = NULL,
    .transfer = mp_machine_soft_spi_transfer,
};

MP_DEFINE_CONST_OBJ_TYPE(
    machine_soft_spi_type,
    MP_QSTR_SoftSPI,
    MP_TYPE_FLAG_NONE,
    make_new, machine_soft_spi_make_new,
    print, machine_soft_spi_print,
    protocol, &mp_machine_soft_spi_p,
    locals_dict, &mp_machine_soft_spi_locals_dict);

