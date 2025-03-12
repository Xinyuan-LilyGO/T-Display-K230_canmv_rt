#ifndef __DRV_LORA__
#define __DRV_LORA__

#include <rtdef.h>

rt_err_t lora_dev_open(rt_device_t dev, rt_uint16_t oflag);
rt_size_t lora_dev_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size);
rt_size_t lora_dev_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size);
rt_err_t lora_dev_close(rt_device_t dev);

#endif

