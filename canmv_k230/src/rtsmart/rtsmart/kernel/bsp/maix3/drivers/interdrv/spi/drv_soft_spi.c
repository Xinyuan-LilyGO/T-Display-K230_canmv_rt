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

#include <rtthread.h>
#include <rtdevice.h>

#include "drv_spi.h"
#include <drivers/spi.h>
#include <string.h>
#include "riscv_io.h"
#include "board.h"
#include "ioremap.h"
#include "sysctl_rst.h"
#include "sysctl_clk.h"
#include "../gpio/drv_gpio.h"
/*
void kd_pin_write(rt_base_t pin, rt_base_t value);
void kd_pin_mode(rt_base_t pin, rt_base_t mode);
int kd_pin_read(rt_base_t pin);
*/
#define TIMER_CLK_FREQ  (24000000)

	static uint64_t spi_perf_get_times(void)
	{
		uint64_t cnt;
		__asm__ __volatile__(
			"rdtime %0"
			: "=r"(cnt));
		return cnt;
	}
	
	static void spi_delay_us(uint64_t us)
	{
		uint64_t delay = (TIMER_CLK_FREQ / 1000000) * us;
		volatile uint64_t cur_time = spi_perf_get_times();
		while(1)
		{
			if((spi_perf_get_times() - cur_time ) >= delay)
				break;
		}
	}

enum {
    MP_SPI_IOCTL_INIT,
    MP_SPI_IOCTL_DEINIT,
    MP_SPI_IOCTL_TRANSFER,
};
typedef struct _soft_spi_cfg_t {
    uint32_t delay_half; // microsecond delay for half SCK period
    uint8_t polarity;
    uint8_t phase;
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
} soft_spi_cfg_t;
soft_spi_cfg_t g_soft_spi_cfg;
struct rt_device soft_spi_dev;
typedef struct _soft_spi_priv_data_t {
				 const void *send_buf;
				 size_t send_length;
				 void *recv_buf;
				 size_t recv_length;
			 }  soft_spi_priv_data_t;


void soft_spi_transfer(size_t len, const uint8_t *src, uint8_t *dest)
{
	uint32_t delay_half =g_soft_spi_cfg.delay_half;
for (size_t i = 0; i < len; ++i) {
        uint8_t data_out = src[i];
        uint8_t data_in = 0;
        for (int j = 0; j < 8; ++j, data_out <<= 1) {
            kd_pin_write(g_soft_spi_cfg.mosi, (data_out >> 7) & 1);
            if (g_soft_spi_cfg.phase == 0) {
                spi_delay_us(delay_half);
                kd_pin_write(g_soft_spi_cfg.sck, 1 - g_soft_spi_cfg.polarity);
            } else {
                kd_pin_write(g_soft_spi_cfg.sck, 1 - g_soft_spi_cfg.polarity);
                spi_delay_us(delay_half);
            }
            data_in = (data_in << 1) | kd_pin_read(g_soft_spi_cfg.miso);
            if (g_soft_spi_cfg.phase == 0) {
                spi_delay_us(delay_half);
                kd_pin_write(g_soft_spi_cfg.sck, g_soft_spi_cfg.polarity);
            } else {
                kd_pin_write(g_soft_spi_cfg.sck, g_soft_spi_cfg.polarity);
                spi_delay_us(delay_half);
            }
        }
        if (dest != NULL) {
            dest[i] = data_in;
        }
    }


}




static rt_err_t soft_spi_dev_control(rt_device_t dev, int cmd, void *args)
{  
    rt_err_t ret;
	switch (cmd) {
		   case MP_SPI_IOCTL_INIT:
			   soft_spi_cfg_t spi_cfg;
			   lwp_get_from_user(&spi_cfg,args,sizeof(spi_cfg));
		       g_soft_spi_cfg.delay_half=spi_cfg.delay_half;
			   g_soft_spi_cfg.phase=spi_cfg.phase;
			   g_soft_spi_cfg.polarity=spi_cfg.polarity;
			   g_soft_spi_cfg.sck=spi_cfg.sck;
			   g_soft_spi_cfg.mosi=spi_cfg.mosi;
			   g_soft_spi_cfg.miso=spi_cfg.miso;
			   kd_pin_mode(g_soft_spi_cfg.sck,GPIO_DM_OUTPUT);
			   kd_pin_write(g_soft_spi_cfg.sck, g_soft_spi_cfg.polarity);
			   kd_pin_mode(g_soft_spi_cfg.mosi,GPIO_DM_OUTPUT);
			   kd_pin_mode(g_soft_spi_cfg.miso,GPIO_DM_INPUT);
			   ret=RT_EOK;
			   break;
		  case MP_SPI_IOCTL_TRANSFER: 
		  		soft_spi_priv_data_t spi_data;
		  		lwp_get_from_user(&spi_data,args,sizeof(soft_spi_priv_data_t));
			    soft_spi_transfer(spi_data.send_length,spi_data.send_buf,spi_data.recv_buf);
			    ret=RT_EOK;
	           break;
		   case MP_SPI_IOCTL_DEINIT:
			   break;
	   }
	
    return ret;
}



const static struct rt_device_ops soft_spi_dev_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    soft_spi_dev_control
};

int soft_spi_init(void)
{
    rt_err_t ret;
    soft_spi_dev.ops = &soft_spi_dev_ops;
    soft_spi_dev.user_data = NULL;
    soft_spi_dev.type = RT_Device_Class_Char;
    RT_ASSERT(rt_device_register(&soft_spi_dev,"soft_spi0",RT_DEVICE_FLAG_RDWR) == RT_EOK);
    return 0;
}
INIT_DEVICE_EXPORT(soft_spi_init);

