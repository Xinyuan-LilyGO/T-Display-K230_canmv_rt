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

#include "rtdef.h"
#include <rtthread.h>
#include <touch.h>
#include <lwp_user_mm.h>

#define DBG_TAG          "gt9895"
#ifdef RT_DEBUG
#define DBG_LVL          DBG_LOG
#else
#define DBG_LVL          DBG_WARNING
#endif
#define DBG_COLOR
#include <rtdbg.h>

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED	0x03
#define TOUCH_RESOLUTION_X 1024
#define TOUCH_RESOLUTION_Y 2400
struct gt9895_dev {
    const char *i2c_name;
    rt_uint16_t i2c_addr;
    struct rt_i2c_bus_device *bus;
};

static int gt9895_read_reg(struct gt9895_dev *dev, rt_uint8_t* addr,
        rt_uint8_t *buffer, rt_size_t length)
{
    int ret;
    // todo fix
    struct rt_i2c_msg msgs[2] =
    {
        {
            .addr   = dev->i2c_addr,
            .flags  = RT_I2C_WR,
            .buf    = addr,
            .len    = 4,
        },
        {
            .addr   = dev->i2c_addr,
            .flags  = RT_I2C_RD,
            //.len    = 1,
            .buf=buffer,
            .len    = length,
        },
    }; 
    ret=RT_EOK;
      if (rt_i2c_transfer(dev->bus, msgs, 2) != 2)
      {
        ret= RT_ERROR;
      }
    /*for (int i = 0; i < length; i++) {
        msgs[1].buf    = buffer + i,
        ret = rt_i2c_transfer(dev->bus, msgs, 2);
        addr++;
    }*/      
    return ret;
}

static rt_size_t gt9895_read_point(struct rt_touch_device *touch, void *buf, rt_size_t read_num)
{
	//rt_kprintf("gt9895_read_point!!!\n");
    int ret, read_point, valid_point, offset;
					uint16_t tmp;
					struct rt_touch_data *point;
					uint8_t rdbuf[touch->info.point_num * 9+0];
					//rt_kprintf("touch->info.point_num:%d\n",touch->info.point_num);
					//rt_kprintf("read_num:%d\n",read_num);	
					read_point = read_num / sizeof(struct rt_touch_data);
					if (!read_point)
						return -RT_EINVAL;
					//rt_kprintf("read_point:%d\n",read_point);
				        rt_uint8_t buffer_addr[4] =  { 0x00, 0x01, 0x03, 0x08};
					ret = gt9895_read_reg(touch->config.user_data, buffer_addr, rdbuf, sizeof(rdbuf));
					//rt_kprintf("gt9895_read_reg ret:%d\n",ret);
					if (ret < 0)
						return ret;
					/*rt_kprintf("---------\n");	
					for(int i=0;i<50;i++)	
					{
				           rt_kprintf("rdbuf[%d]:%d ",i,rdbuf[i]);  
				        }
				        rt_kprintf("-------\n");*/				       	
					valid_point = rdbuf[2] & 0xf;
					read_point = read_point > valid_point ? valid_point : read_point;
					//rt_kprintf("read_point:%d\n",read_point);
					point = (struct rt_touch_data *)buf;
					offset = 8;
					//uint8_t *ptr = &rdbuf[8];
					for (int i = 0; i < read_point; i++) {
						//tmp = rdbuf[offset] >> 6;
						//point[i].event = tmp == TOUCH_EVENT_DOWN ?
							//RT_TOUCH_EVENT_DOWN : tmp == TOUCH_EVENT_UP ?
							//RT_TOUCH_EVENT_UP : tmp == TOUCH_EVENT_ON ?
							//RT_TOUCH_EVENT_MOVE : RT_TOUCH_EVENT_NONE;

						point[i].event = tmp == TOUCH_EVENT_DOWN;	
						tmp = (rdbuf[offset]>>4) & 0x0f;
						point[i].track_id = tmp;
						/*point[i].x_coordinate = *((uint16_t *)(ptr + 2));
						point[i].y_coordinate = *((uint16_t *)(ptr + 4));
						point[i].width = *((uint16_t *)(ptr + 6));*/

					        tmp = rdbuf[offset+2]| (rdbuf[offset + 3]<<8);
					        point[i].x_coordinate = tmp;
						tmp = rdbuf[offset+4] | (rdbuf[offset + 5]<<8);
						point[i].y_coordinate = tmp;
						point[i].width = rdbuf[offset + 6];//rdbuf[offset + 7]
						point[i].timestamp = 0;
						offset += 8;
						//ptr += 8;
						
						//scaler
						point[i].x_coordinate=point[i].x_coordinate*touch->info.range_x/TOUCH_RESOLUTION_X;
						point[i].y_coordinate=point[i].y_coordinate*touch->info.range_y/TOUCH_RESOLUTION_Y;
						
							
					}
				
					return sizeof(struct rt_touch_data) * read_point;
}

static rt_err_t gt9895_control(struct rt_touch_device *touch, int cmd, void *arg)
{
//rt_kprintf("gt9895_control!!!\n");
    if(RT_TOUCH_CTRL_GET_INFO == cmd) {
        struct rt_touch_info *info = (struct rt_touch_info*)arg;
        lwp_put_to_user(info, &touch->info, sizeof(*info));
        return RT_EOK;
    }

    return -RT_ENOSYS;
}

static struct rt_touch_ops touch_ops = {
    .touch_readpoint = gt9895_read_point,
    .touch_control = gt9895_control,
};

static struct gt9895_dev gt9895_dev0 = {
    .i2c_name = TOUCH_GT9895_I2C_DEV,
    .i2c_addr = TOUCH_GT9895_I2C_ADDR,
};

static int gt9895_register(struct rt_touch_config *cfg)
{
    int ret;
    rt_touch_t touch_device;
    struct gt9895_dev *dev;
    //rt_kprintf("gt9895_register!!!\n");

    touch_device = (rt_touch_t)rt_calloc(1, sizeof(struct rt_touch_device));
    rt_memcpy(&touch_device->config, cfg, sizeof(struct rt_touch_config));
    dev = (struct gt9895_dev *)(cfg->user_data);
    dev->bus = (struct rt_i2c_bus_device *)rt_device_find(dev->i2c_name);
    if (dev->bus == RT_NULL) {
        LOG_E("Can't find %s device", dev->i2c_name);
        return -RT_ERROR;
    }

    ret = rt_device_open((rt_device_t)dev->bus, RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK) {
        LOG_E("open %s device failed: %d", dev->i2c_name, ret);
        return -RT_ERROR;
    }

    touch_device->info.type = RT_TOUCH_TYPE_CAPACITANCE;
    touch_device->info.vendor = RT_TOUCH_VENDOR_GT;
    touch_device->info.point_num = 10;
    touch_device->info.range_x = 568;
    touch_device->info.range_y = 1232;
    touch_device->ops = &touch_ops;

    ret= rt_hw_touch_register(touch_device, cfg->dev_name, 0, RT_NULL);

    return ret;
}

int gt9895_init(void)
{
//rt_kprintf("gt9895_init!!!\n");
    int ret;
    struct rt_touch_config cfg;

    cfg.dev_name = "touch1";
    cfg.user_data = &gt9895_dev0;

    ret = gt9895_register(&cfg);

    return ret;
}
INIT_DEVICE_EXPORT(gt9895_init);

