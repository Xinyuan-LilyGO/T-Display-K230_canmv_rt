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

#include <string.h>
#ifdef RT_USING_POSIX
#include <dfs_posix.h>
#include <dfs_poll.h>
#include <posix_termios.h>
#endif

#include "sensor_dev.h"
#include "k_sensor_comm.h"

#include "k_autoconf_comm.h"

struct sensor_driver_dev g_sensor_drv[3];

static sensor_probe_impl sensor_probes[] = {
#ifdef CONFIG_MPP_ENABLE_SENSOR_GC2093
    sensor_gc2093_probe,
#endif // CONFIG_MPP_ENABLE_SENSOR_GC2093
#ifdef CONFIG_MPP_ENABLE_SENSOR_OV5647
    sensor_ov5647_probe,
#endif // CONFIG_MPP_ENABLE_SENSOR_OV5647
#if defined (CONFIG_MPP_ENABLE_SENSOR_IMX335)
    sensor_imx335_probe,
#endif // CONFIG_MPP_ENABLE_SENSOR_IMX335
    0, // end
};

static k_s32 sensor_dev_open(struct dfs_fd *file)
{
    struct rt_device *device;
	struct sensor_driver_dev *pdriver_dev;
	k_s32 ret = 0;

    //rt_kprintf("%s enter, device(%p)\n", __func__, file->fnode->data);

    device = file->fnode->data;
	if (device == NULL) {
		rt_kprintf("%s: device is null\n", __func__);
		return -ENOMEM;
	}
    pdriver_dev = (struct sensor_driver_dev *)device;

	//rt_kprintf("%s exit\n", __func__);

	return 0;
}

static k_s32 sensor_dev_close(struct dfs_fd *file)
{
    struct rt_device *device;
	struct sensor_driver_dev *pdriver_dev;
	k_s32 ret = 0;

    //rt_kprintf("%s enter, device(%p)\n", __func__, file->fnode->data);

    device = file->fnode->data;
	if (device == NULL) {
		rt_kprintf("%s: device is null\n", __func__);
		return -ENOMEM;
	}
    pdriver_dev = (struct sensor_driver_dev *)device;

	//rt_kprintf("%s exit\n", __func__);

	return 0;
}

static k_s32 sensor_dev_ioctl(struct dfs_fd *file, k_s32 cmd, void *args)
{
    struct rt_device *device;
	struct sensor_driver_dev *pdriver_dev;
	k_s32 ret = 0;

    device = file->fnode->data;
	if (device == NULL) {
		rt_kprintf("%s: device is null\n", __func__);
		return -ENOMEM;
	}

    pdriver_dev = (struct sensor_driver_dev *)device;

	rt_mutex_take(&pdriver_dev->sensor_mutex, RT_WAITING_FOREVER);
	ret = sensor_priv_ioctl(pdriver_dev, cmd, args);
    rt_mutex_release(&pdriver_dev->sensor_mutex);

    return ret;
}

static const struct dfs_file_ops sensor_dev_fops = {
    .open = sensor_dev_open,
    .close = sensor_dev_close,
    .ioctl = sensor_dev_ioctl,
};

k_s32 sensor_drv_dev_init(struct sensor_driver_dev *pdriver_dev)
{
    struct rt_device *device;
    char dev_name[32];
    k_s32 ret = 0;

    RT_ASSERT(pdriver_dev != NULL)

    device = &pdriver_dev->parent;
    rt_snprintf(dev_name, sizeof(dev_name), "sensor_%s", pdriver_dev->sensor_name);

    rt_mutex_init(&pdriver_dev->sensor_mutex, "sensor_mutex", RT_IPC_FLAG_PRIO);

    ret = rt_device_register(device, dev_name, RT_DEVICE_FLAG_RDWR);
    if (ret) {
        rt_kprintf("sensor device register fail\n");
        return ret;
    }

    device->fops = &sensor_dev_fops;
    device->user_data = pdriver_dev;

    return 0;
}

#if !defined (CONFIG_MPP_ENABLE_CSI_DEV_0) && !defined (CONFIG_MPP_ENABLE_CSI_DEV_1) && !(defined (CONFIG_MPP_ENABLE_CSI_DEV_2))
#error "Must Enable one csi dev"
#endif

k_s32 sensor_device_init(void) {
  struct k_sensor_probe_cfg probe_cfg[] = {
#ifdef CONFIG_MPP_ENABLE_CSI_DEV_0
      {
          .csi_num = 0,
          .i2c_name = CONFIG_MPP_CSI_DEV0_I2C_DEV,
          .pwd_gpio = CONFIG_MPP_CSI_DEV0_POWER,
          .reset_gpio = CONFIG_MPP_CSI_DEV0_RESET,
      },
#endif // CONFIG_MPP_ENABLE_CSI_DEV_0
#ifdef CONFIG_MPP_ENABLE_CSI_DEV_1
      {
          .csi_num = 1,
          .i2c_name = CONFIG_MPP_CSI_DEV1_I2C_DEV,
          .pwd_gpio = CONFIG_MPP_CSI_DEV1_POWER,
          .reset_gpio = CONFIG_MPP_CSI_DEV1_RESET,
      },
#endif // CONFIG_MPP_ENABLE_CSI_DEV_1
#ifdef CONFIG_MPP_ENABLE_CSI_DEV_2
      {
          .csi_num = 2,
          .i2c_name = CONFIG_MPP_CSI_DEV2_I2C_DEV,
          .pwd_gpio = CONFIG_MPP_CSI_DEV2_POWER,
          .reset_gpio = CONFIG_MPP_CSI_DEV2_RESET,
      },
#endif // CONFIG_MPP_ENABLE_CSI_DEV_2

    // MAX 3 configure, because we just have 3 csi device
  };

  memset(&g_sensor_drv[0], 0, sizeof(g_sensor_drv));

  for (size_t i = 0; i < (sizeof(probe_cfg) / sizeof(probe_cfg[0])); i++) {
    struct k_sensor_probe_cfg *cfg = &probe_cfg[i];

    struct sensor_driver_dev *dev = &g_sensor_drv[cfg->csi_num];

    for(int j = 0; sensor_probes[j]; j++) {
        if(0x00 == sensor_probes[j](cfg, dev)) {
            sensor_drv_dev_init(dev);
            rt_kprintf("find sensor(%s) on csi%d\n", dev->sensor_name, cfg->csi_num);
            break;
        }
    }

    if(NULL == dev->sensor_func.sensor_init) {
        rt_kprintf("Can't find sensor on csi%d\n", cfg->csi_num);
    }
  }

  return 0;
}

/* sensor type helper */
#if !defined (CONFIG_SDK_ENABLE_CANMV)

#define SENSOR_TYPE_NAME(x) { .type = x, .name = #x }

struct sensor_type_name {
    k_vicap_sensor_type type;
    const char *name;
};

static const struct sensor_type_name sth_table[] = {
#if defined (CONFIG_MPP_ENABLE_SENSOR_IMX335)
    SENSOR_TYPE_NAME(IMX335_MIPI_CSI0_2LANE_1920X1080_30FPS_12BIT_LINEAR),
    SENSOR_TYPE_NAME(IMX335_MIPI_CSI0_2LANE_2592X1944_30FPS_12BIT_LINEAR),
    SENSOR_TYPE_NAME(IMX335_MIPI_CSI0_4LANE_2592X1944_30FPS_12BIT_LINEAR),
#endif // CONFIG_MPP_ENABLE_SENSOR_IMX335

#if defined (CONFIG_MPP_ENABLE_SENSOR_OV5647)
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI0_2592x1944_10FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI0_1280X960_45FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI0_1280X720_60FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI0_640x480_90FPS_10BIT_LINEAR),

    SENSOR_TYPE_NAME(OV5647_MIPI_CSI1_2592x1944_10FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI1_1280X960_45FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI1_1280X720_60FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI1_640x480_90FPS_10BIT_LINEAR),

    SENSOR_TYPE_NAME(OV5647_MIPI_CSI2_2592x1944_10FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI2_1280X960_45FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI2_1280X720_60FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(OV5647_MIPI_CSI2_640x480_90FPS_10BIT_LINEAR),
#endif // CONFIG_MPP_ENABLE_SENSOR_OV5647

#if defined (CONFIG_MPP_ENABLE_SENSOR_GC2093)
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI0_1920X1080_60FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI0_1280X960_90FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI0_1280X720_90FPS_10BIT_LINEAR),

    SENSOR_TYPE_NAME(GC2093_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI1_1920X1080_60FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI1_1280X960_90FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI1_1280X720_90FPS_10BIT_LINEAR),

    SENSOR_TYPE_NAME(GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI2_1920X1080_60FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI2_1280X960_90FPS_10BIT_LINEAR),
    SENSOR_TYPE_NAME(GC2093_MIPI_CSI2_1280X720_90FPS_10BIT_LINEAR),
#endif // CONFIG_MPP_ENABLE_SENSOR_GC2093

    /* last type set to U32 MAX */
    {__UINT32_MAX__, "UNKNOWN"},
};

static inline __attribute__((always_inline)) k_u32 shash(const char * s)
{
	uint32_t v = 5381;
	if(s)
	{
		while(*s)
			v = (v << 5) + v + (*s++);
	}
	return v;
}

static void sth(k_s32 argc, char** argv)
{
    k_u32 hash1, hash2;
    const struct sensor_type_name *tn = NULL;

    if(2 > argc) {
        printf("uasge: %s sensor_type_string\n");
        return;
    }
    hash1 = shash(argv[1]);

    for(size_t i = 0; i < sizeof(sth_table) / sizeof(sth_table[0]); i++) {
        tn = &sth_table[i];

        hash2 = shash(tn->name);
        if(hash1 == hash2) {
            break;
        }
    }

    if ((NULL != tn) && (__UINT32_MAX__ != tn->type)) {
        printf("Sensor %s type is %d\n", argv[1], tn->type);
    } else {
        printf("Can not recognize sensor %s\n", argv[1]);
    }

    return;
}

MSH_CMD_EXPORT(sth, sensor type helper)

#endif
