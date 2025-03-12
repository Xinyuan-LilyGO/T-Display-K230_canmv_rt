#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "mpconfig.h"
#include "mpprint.h"
#include "sys/ioctl.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "py/obj.h"

#define I2C_SLAVE_IOCTL_SET_BUFFER_SIZE               0
#define I2C_SLAVE_IOCTL_SET_ADDR                      1  

extern const mp_obj_type_t machine_i2c_slave_type;

typedef struct {
    mp_obj_base_t base;
    int fd;
    uint32_t mem_size;
    uint8_t addr;
} machine_i2c_slave_obj_t;


STATIC mp_obj_t machine_i2c_slave_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) 
{
    mp_arg_check_num(n_args, n_kw, 1, 3, true);
    mp_int_t i2c_number = mp_obj_get_int(args[0]);

    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c%ld_slave",i2c_number);

    int fd = open(path, O_RDWR);
    if (fd < 0) 
    {
        mp_raise_msg(&mp_type_AssertionError, MP_ERROR_TEXT("device open failed"));
        return mp_const_none;
    }

    machine_i2c_slave_obj_t *self = m_new_obj_with_finaliser(machine_i2c_slave_obj_t);
    self->base.type = &machine_i2c_slave_type;
    self->fd = fd;    

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    mp_map_elem_t *elem = mp_map_lookup(&kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_addr), MP_MAP_LOOKUP);
    self->addr = elem ? mp_obj_get_int(elem->value) : 0x01;

    elem = mp_map_lookup(&kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_mem_size), MP_MAP_LOOKUP);
    self->mem_size = elem ? mp_obj_get_int(elem->value) : 10;

    ioctl(fd, I2C_SLAVE_IOCTL_SET_ADDR,&self->addr);
    ioctl(fd, I2C_SLAVE_IOCTL_SET_BUFFER_SIZE,&self->mem_size);
    mp_printf(&mp_plat_print, "i2c_slave_addr: 0x%02x, mem_size: %d\r\n",self->addr,self->mem_size);
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t machine_i2c_slave_list(void)
{
    char path[32];
    mp_obj_t dev_list = mp_obj_new_list(0, NULL);

    for (size_t i = 0; i <= 4; i++)
    {
        snprintf(path, sizeof(path), "/dev/i2c%ld_slave",i);
        int fd = open(path, O_RDWR);
        if (fd >= 0)
        {
            close(fd);
            mp_obj_list_append(dev_list, mp_obj_new_int(i));
        }
    }

    return dev_list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(machine_i2c_list_obj, machine_i2c_slave_list);
STATIC MP_DEFINE_CONST_STATICMETHOD_OBJ(machine_i2c_list_staticmethod_obj,MP_ROM_PTR(&machine_i2c_list_obj));

STATIC mp_obj_t machine_i2c_slave_readfrom_mem(mp_obj_t self_in, mp_obj_t mem_addr_obj,mp_obj_t size_obj)
{
    machine_i2c_slave_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_int_t mem_addr = mp_obj_get_int(mem_addr_obj);
    mp_int_t size = mp_obj_get_int(size_obj);

    if ((size+mem_addr) > self->mem_size)
    {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Read size exceeds memory size"));
        return mp_const_none;
    }

    uint8_t data[size];
    lseek(self->fd, mem_addr,SEEK_SET);
    ssize_t ret = read(self->fd, data, size);
    if (ret < 0)
    {
        mp_raise_msg(&mp_type_AssertionError, MP_ERROR_TEXT("device read failed"));
        return mp_const_none;
    }

    return mp_obj_new_bytearray(ret, data);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(machine_i2c_slave_readfrom_mem_obj, machine_i2c_slave_readfrom_mem);

STATIC mp_obj_t machine_i2c_slave_writeto_mem(size_t n_args, const mp_obj_t *args)
{
    machine_i2c_slave_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_int_t mem_addr = mp_obj_get_int(args[1]);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[2],&bufinfo,MP_BUFFER_READ);

    if ((bufinfo.len+mem_addr) > self->mem_size)
    {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Write size exceeds memory size"));
        return mp_const_none;
    }

    lseek(self->fd, mem_addr ,SEEK_SET);
    ssize_t ret = write(self->fd, bufinfo.buf, bufinfo.len);
    if (ret < 0)
    {
        mp_raise_msg(&mp_type_AssertionError, MP_ERROR_TEXT("device write failed"));
        return mp_const_none;
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_i2c_slave_writeto_mem_obj, 3, 3, machine_i2c_slave_writeto_mem);

STATIC mp_obj_t machine_i2c_slave_deinit(mp_obj_t self_in) {
    machine_i2c_slave_obj_t *self = MP_OBJ_TO_PTR(self_in);
    close(self->fd);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_i2c_slave_deinit_obj, machine_i2c_slave_deinit);


STATIC const mp_rom_map_elem_t machine_i2c_slave_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__), MP_ROM_PTR(&machine_i2c_slave_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_list), MP_ROM_PTR(&machine_i2c_list_staticmethod_obj) },
    { MP_ROM_QSTR(MP_QSTR_readfrom_mem), MP_ROM_PTR(&machine_i2c_slave_readfrom_mem_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeto_mem), MP_ROM_PTR(&machine_i2c_slave_writeto_mem_obj) },
};
MP_DEFINE_CONST_DICT(mp_machine_i2c_slave_locals_dict, machine_i2c_slave_locals_dict_table);


MP_DEFINE_CONST_OBJ_TYPE(
    machine_i2c_slave_type,
    MP_QSTR_I2C_Slave,
    MP_TYPE_FLAG_NONE,
    make_new, machine_i2c_slave_make_new,
    locals_dict, &mp_machine_i2c_slave_locals_dict);
