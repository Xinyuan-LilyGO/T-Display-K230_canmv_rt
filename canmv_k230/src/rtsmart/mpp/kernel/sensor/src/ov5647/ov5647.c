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

#include "rtthread.h"
#include "sensor_dev.h"
#include "io.h"
#include "drv_gpio.h"
#include <stdio.h>

#define pr_info(...) //rt_kprintf(__VA_ARGS__)
#define pr_debug(...) //rt_kprintf(__VA_ARGS__)
#define pr_warn(...)    //rt_kprintf(__VA_ARGS__)
#define pr_err(...)    rt_kprintf(__VA_ARGS__)

/* Sensor private ************************************************************/
#define OV5647_CHIP_ID                                      (0x5647)

#define DELAY_MS_SENSOR_DEFAULT                             (100)

#define OV5647_REG_CHIP_ID_H                                (0x300a)
#define OV5647_REG_CHIP_ID_L                                (0x300b)
#define OV5647_REG_MIPI_CTRL00                              (0x4800)
#define OV5647_REG_FRAME_OFF_NUMBER                         (0x4202)
#define OV5647_REG_PAD_OUT                                  (0x300d)

#define OV5647_REG_VTS_H                                    (0x380e)
#define OV5647_REG_VTS_L                                    (0x380f)

#define OV5647_REG_MIPI_CTRL14                              (0x4814)

#define OV5647_SW_STANDBY                                   (0x0100)

#define OV5647_REG_LONG_AGAIN_H                             (0x350a)
#define OV5647_REG_LONG_AGAIN_L                             (0x350b)

#define OV5647_REG_LONG_EXP_TIME_H                          (0x3501)
#define OV5647_REG_LONG_EXP_TIME_L                          (0x3502)

#define OV5647_MIN_GAIN_STEP                                (1.0f/16.0f)
#define OV5647_SW_RESET                                     (0x0103)
#define MIPI_CTRL00_CLOCK_LANE_GATE                         (1 << 5)
#define MIPI_CTRL00_LINE_SYNC_ENABLE                        (1 << 4)
#define MIPI_CTRL00_BUS_IDLE                                (1 << 1)
#define MIPI_CTRL00_CLOCK_LANE_DISABLE                      (1 << 0)

/* include sensor register configure */
#include "sensor_reg_table.c"

#if defined (CONFIG_MPP_ENABLE_CSI_DEV_0)
    #include "sensor_csi0_mode_list.c"
#endif // CONFIG_MPP_ENABLE_CSI_DEV_0

#if defined (CONFIG_MPP_ENABLE_CSI_DEV_1)
    #include "sensor_csi1_mode_list.c"
#endif // CONFIG_MPP_ENABLE_CSI_DEV_1

#if defined (CONFIG_MPP_ENABLE_CSI_DEV_2)
    #include "sensor_csi2_mode_list.c"
#endif // CONFIG_MPP_ENABLE_CSI_DEV_2

static k_s32 _sensor_read_chip_id_r(struct sensor_driver_dev *dev, k_u32 *chip_id)
{
    k_s32 ret = 0;
    k_u16 id_high = 0;
    k_u16 id_low = 0;

    const k_s32 pwd_gpio = dev->pwd_gpio;
    const k_s32 reset_gpio = dev->reset_gpio;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    if(NULL == dev->i2c_info.i2c_bus) {
        pr_err("%s no i2c bus\n", dev->i2c_info.i2c_bus);
        return -1;
    }

    if(0 <= reset_gpio) {
        kd_pin_mode(reset_gpio, GPIO_DM_OUTPUT);
        kd_pin_write(reset_gpio, GPIO_PV_HIGH);
    }

    ret = sensor_reg_read(&dev->i2c_info, OV5647_REG_CHIP_ID_H, &id_high);
    rt_kprintf("ov5647 read chip id high ,ret:%d\n", ret);
    ret |= sensor_reg_read(&dev->i2c_info, OV5647_REG_CHIP_ID_L, &id_low);
    rt_kprintf("ov5647 read chip id low ,ret:%d\n", ret);
    if(chip_id) {
        *chip_id = (id_high << 8) | id_low;
        pr_info("%s chip id 0x%x\n", __func__, *chip_id);
    }

    return ret;
}

/* Sensor functions **********************************************************/
static int _sensor_power_state_set(struct sensor_driver_dev *dev, k_s32 on, k_u32 delay)
{
    const k_s32 pwd_gpio = dev->pwd_gpio;
    const k_s32 reset_gpio = dev->reset_gpio;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    if(0x00 > reset_gpio) {
        return 0;
    }

    if(-1 != pwd_gpio) {
        kd_pin_mode(pwd_gpio, GPIO_DM_OUTPUT);
        kd_pin_write(pwd_gpio, GPIO_PV_LOW);
    }

    kd_pin_mode(reset_gpio, GPIO_DM_OUTPUT);

    if (on) {
        kd_pin_write(reset_gpio, GPIO_PV_HIGH);
        rt_thread_mdelay(delay);
        kd_pin_write(reset_gpio, GPIO_PV_LOW);
        rt_thread_mdelay(delay);
        kd_pin_write(reset_gpio, GPIO_PV_HIGH);
    } else {
        kd_pin_write(reset_gpio, GPIO_PV_LOW);
    }
    rt_thread_mdelay(1);

    return 0;
}

static k_s32 sensor_power_impl(void *ctx, k_s32 on)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    if (K_FALSE == on) {
        ret = sensor_reg_write(&dev->i2c_info, 0x3018, 0xff);
        ret |= sensor_reg_write(&dev->i2c_info, OV5647_SW_STANDBY, 0x00);
    }

    _sensor_power_state_set(dev, on, 100);
    dev->init_flag = on;

    return ret;
}

static k_s32 sensor_init_impl(void *ctx, k_sensor_mode mode)
{
    k_s32 i = 0;
    k_s32 ret = 0;

    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    const k_vicap_sensor_type type = mode.sensor_type;

    pr_info("%s enter, sensor_type:%d %s\n", __func__, type, dev->sensor_name);

    memset(current_mode, 0, sizeof(k_sensor_mode));

    for(k_u32 i = 0; i < dev->mode_count; i++) {
        if(type == dev->sensor_mode_list[i].sensor_type) {
            memcpy(current_mode, &dev->sensor_mode_list[i], sizeof(k_sensor_mode));
            memcpy(&current_mode->ae_info, current_mode->sensor_ae_info, sizeof(k_sensor_ae_info));
            break;
        }
    }

    if (NULL == current_mode->reg_list) {
        pr_err("%s, can not init for sensor type %d\n", __func__, type);
        return -1;
    }
	
    k_u16 channel_id=100;
    k_s32 ret_channel_id;
    _sensor_power_state_set(dev, 1, 1);
    rt_kprintf("ov5647_sensor_init dev->pwd_gpio: %d ,dev->reset_gpio:%d\n",dev->pwd_gpio,dev->reset_gpio);
     //if(NULL == (dev->i2c_info.i2c_bus = rt_i2c_bus_device_find("ov5647_csi1"))) {
        //rt_kprintf("Can't find XXXXXXXXXXXXXXXX\n", );
    //}
    ret_channel_id=sensor_reg_read(&dev->i2c_info, OV5647_REG_MIPI_CTRL14, &channel_id);
    pr_info("ov5647_sensor_init OV5647_REG_MIPI_CTRL14 is %d \n", channel_id);
    rt_kprintf("ov5647_sensor_init OV5647_REG_MIPI_CTRL14 is %d ,ret_channel_id:%d\n",channel_id,ret_channel_id);
    channel_id &= ~(3 << 6);
    ret = sensor_reg_write(&dev->i2c_info, OV5647_REG_MIPI_CTRL14, channel_id | (0 << 6));

    // set mirror
    int set_mirror_reg = 1;

    k_sensor_reg sensor_mirror_reg[] = {
        {0x3820, 0x00},
        {0x3821, 0x00},
        {REG_NULL, 0x00},
    };

    switch(dev->mirror_setting.mirror) {
        case VICAP_MIRROR_NONE: {
            set_mirror_reg = 0;
        } break;
        case VICAP_MIRROR_HOR: {
            sensor_mirror_reg[0].val = 0x0;
            sensor_mirror_reg[1].val = 0x0;
            current_mode->bayer_pattern = BAYER_PAT_BGGR;
        } break;
        case VICAP_MIRROR_VER: {
            sensor_mirror_reg[0].val = 0x2;
            sensor_mirror_reg[1].val = 0x2;
            current_mode->bayer_pattern = BAYER_PAT_RGGB;
        } break;
        case VICAP_MIRROR_BOTH: {
            sensor_mirror_reg[0].val = 0x2;
            sensor_mirror_reg[1].val = 0x0;
            current_mode->bayer_pattern = BAYER_PAT_GRBG;
        } break;
        default: {
            pr_err("%s, not support mirror setting %d\n", __func__, dev->mirror_setting.mirror);
        } break;
    }

    // write sensor reg 
    ret = sensor_reg_list_write(&dev->i2c_info, current_mode->reg_list);
    rt_kprintf("ov5647 sensor_reg_list_write ,ret:%d\n", ret);
    rt_kprintf("ov5647 ,slave_addr:%d,reg_addr_size:%d,reg_val_size:%d,i2c_name:%s\n", dev->i2c_info.slave_addr,dev->i2c_info.reg_addr_size,dev->i2c_info.reg_val_size,dev->i2c_info.i2c_name);
    k_u32 chip_id = 0x1111;
    k_s32 ret_chip_id;
    ret_chip_id=_sensor_read_chip_id_r(dev, &chip_id);
    rt_kprintf("ov5647 _sensor_read_chip_id_r ,ret_chip_id:%d\n", ret_chip_id);
	
	
    // write mirror reg
    if(set_mirror_reg) {
        k_u16 flip = 0;
        k_u16 mirror_reg = 0;
        k_u32 width = current_mode->size.width;
        k_u32 height = current_mode->size.height;

        if(((1920 == width) || (2592 == width)) && ((1080 == height) || (1944 == height))) {
            /* do nothing. */
            rt_kprintf("ov5647 sset_mirror_reg do nothing\n");
        } else {
            ret = sensor_reg_read(&dev->i2c_info, 0x3820, &flip);
            ret = sensor_reg_read(&dev->i2c_info, 0x3821, &mirror_reg);

            sensor_mirror_reg[0].val = sensor_mirror_reg[0].val | (flip & 0x1);
            sensor_mirror_reg[1].val = sensor_mirror_reg[1].val | (mirror_reg & 0x1);
        }
        ret |= sensor_reg_list_write(&dev->i2c_info, sensor_mirror_reg);
        rt_kprintf("ov5647 sensor_reg_list_write sensor_mirror_reg,ret:%d\n", ret);
    }

    current_mode->sensor_again = 0;
    current_mode->et_line = 0;

    k_u16 again_h, again_l;
    k_u16 exp_time_h, exp_time_l;
    k_u16 exp_time;
    float again = 0, dgain = 0;

    k_s32 ret_again_h = sensor_reg_read(&dev->i2c_info, OV5647_REG_LONG_AGAIN_H, &again_h);
    k_s32 ret_again_l= sensor_reg_read(&dev->i2c_info, OV5647_REG_LONG_AGAIN_L, &again_l);
    rt_kprintf("ov5647 ssensor_reg_read again,ret_again_h:%d,ret_again_l:%d\n", ret_again_h,ret_again_l);
    //ret |= sensor_reg_read(&dev->i2c_info, OV5647_REG_LONG_AGAIN_H, &again_h);
    //ret |= sensor_reg_read(&dev->i2c_info, OV5647_REG_LONG_AGAIN_L, &again_l);
    again = (float)(((again_h & 0x03) << 8) + again_l) / 16.0f;
  rt_kprintf("ov5647 ssensor_reg_read again,ret:%d\n", ret);
    dgain = 1.0;
    current_mode->ae_info.cur_gain = again * dgain;
    current_mode->ae_info.cur_long_gain = current_mode->ae_info.cur_gain;
    current_mode->ae_info.cur_vs_gain = current_mode->ae_info.cur_gain;

    ret |= sensor_reg_read(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_H, &exp_time_h);
    ret |= sensor_reg_read(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_L, &exp_time_l);
    exp_time = (exp_time_h << 4) + ((exp_time_l >> 4) & 0x0F);
rt_kprintf("ov5647 ssensor_reg_read exp_time,ret:%d\n", ret);
    current_mode->ae_info.cur_integration_time = exp_time * current_mode->ae_info.one_line_exp_time;

    dev->init_flag = K_TRUE;
	rt_kprintf("ov5647 sensor_init_impl ,ret:%d\n", ret);
    return ret;
}

static k_s32 sensor_get_chip_id_impl(void *ctx, k_u32 *chip_id)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    ret = _sensor_read_chip_id_r(dev, chip_id);

    if(chip_id && (*chip_id != OV5647_CHIP_ID)) {
        ret = -1;
        pr_err("%s, iic read chip id err \n", __func__);
    }

    return ret;
}

static k_s32 sensor_get_mode_impl(void *ctx, k_sensor_mode *mode)
{
    struct sensor_driver_dev *dev = ctx;
    const k_vicap_sensor_type type = mode->sensor_type;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, sensor_type(%d) %s\n", __func__, mode->sensor_type, dev->sensor_name);

    if(type == current_mode->sensor_type) {
        memcpy(mode, current_mode, sizeof(k_sensor_mode));
        return 0;
    }

    for(k_u32 i = 0; i < dev->mode_count; i++) {
        if(type == dev->sensor_mode_list[i].sensor_type) {
            memcpy(current_mode, &dev->sensor_mode_list[i], sizeof(k_sensor_mode));
            memcpy(&current_mode->ae_info, current_mode->sensor_ae_info, sizeof(k_sensor_ae_info));

            memcpy(mode, current_mode, sizeof(k_sensor_mode));
            return 0;
        }
    }

    pr_info("%s, the mode not exist.\n", __func__);

    return -1;
}

static k_s32 sensor_set_mode_impl(void *ctx, k_sensor_mode mode)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    return ret;
}

static k_s32 sensor_enum_mode_impl(void *ctx, k_sensor_enum_mode *enum_mode)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    memset(enum_mode, 0, sizeof(k_sensor_enum_mode));

    return ret;
}

static k_s32 sensor_get_caps_impl(void *ctx, k_sensor_caps *caps)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    memset(caps, 0, sizeof(k_sensor_caps));

    caps->bit_width = current_mode->bit_width;
    caps->bayer_pattern = current_mode->bayer_pattern;
    caps->resolution.width = current_mode->size.width;
    caps->resolution.height = current_mode->size.height;

    return ret;
}

static k_s32 sensor_conn_check_impl(void *ctx, k_s32 *conn)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    *conn = 1;

    return ret;
}

static k_s32 sensor_set_stream_impl(void *ctx, k_s32 enable)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;

    pr_info("%s enter, enable(%d) %s\n", __func__, enable, dev->sensor_name);

    if (enable) {
        ret = sensor_reg_write(&dev->i2c_info, OV5647_SW_STANDBY, 0x01);
    } else {
        ret = sensor_reg_write(&dev->i2c_info, 0x3018, 0xff);
        ret |= sensor_reg_write(&dev->i2c_info, OV5647_SW_STANDBY, 0x00);
    }
    pr_info("%s exit, ret(%d)\n", __func__, ret);

    return ret;
}

static k_s32 sensor_get_again_impl(void *ctx, k_sensor_gain *gain)
{
    k_s32 ret = 0;

    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    if (current_mode->hdr_mode == SENSOR_MODE_LINEAR) {
        gain->gain[SENSOR_LINEAR_PARAS] = current_mode->ae_info.cur_again;
    } else if (current_mode->hdr_mode == SENSOR_MODE_HDR_STITCH) {
        gain->gain[SENSOR_DUAL_EXP_L_PARAS] = current_mode->ae_info.cur_again;
        gain->gain[SENSOR_DUAL_EXP_S_PARAS] = current_mode->ae_info.cur_vs_again;
    } else {
        pr_err("%s, unsupport exposure frame.\n", __func__);
        return -1;
    }

    return ret;
}

static k_s32 sensor_set_again_impl(void *ctx, k_sensor_gain gain)
{
    k_s32 ret = 0;
    k_u32 again, dgain, total;
    k_u8 i;

    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    if (current_mode->hdr_mode == SENSOR_MODE_LINEAR) {
        again = (k_u16)(gain.gain[SENSOR_LINEAR_PARAS] * 16 + 0.5);
        if(current_mode->sensor_again !=again)
        {
            ret = sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_H,(again & 0x0300)>>8);
            ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_L,(again & 0xff));
            current_mode->sensor_again = again;
        }
        current_mode->ae_info.cur_again = (float)current_mode->sensor_again/16.0f;
    } else if (current_mode->hdr_mode == SENSOR_MODE_HDR_STITCH) {
        again = (k_u16)(gain.gain[SENSOR_DUAL_EXP_L_PARAS]  * 16 + 0.5);
         ret = sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_H,(again & 0x0300)>>8);
         ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_L,(again & 0xff));
        current_mode->ae_info.cur_again = (float)again/16.0f;

        again = (k_u16)(gain.gain[SENSOR_DUAL_EXP_S_PARAS] * 16 + 0.5);
        //TODO
        current_mode->ae_info.cur_vs_again = (float)again/16.0f;
    } else {
        pr_err("%s, unsupport exposure frame.\n", __func__);
        return -1;
    }
    pr_debug("%s, exp_frame_type(%d), cur_again(%u)\n", __func__, current_mode->hdr_mode, (k_u32)(current_mode->ae_info.cur_again*1000) );

    return ret;
}

static k_s32 sensor_get_dgain_impl(void *ctx, k_sensor_gain *gain)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    if (current_mode->hdr_mode == SENSOR_MODE_LINEAR) {
        gain->gain[SENSOR_LINEAR_PARAS] = current_mode->ae_info.cur_dgain;
    } else if (current_mode->hdr_mode == SENSOR_MODE_HDR_STITCH) {
        gain->gain[SENSOR_DUAL_EXP_L_PARAS] = current_mode->ae_info.cur_dgain;
        gain->gain[SENSOR_DUAL_EXP_S_PARAS] = current_mode->ae_info.cur_vs_dgain;
    } else {
        pr_err("%s, unsupport exposure frame.\n", __func__);
        return -1;
    }

    return ret;
}

static k_s32 sensor_set_dgain_impl(void *ctx, k_sensor_gain gain)
{
    k_s32 ret = 0;
    k_u32 dgain;
    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, hdr_mode(%d) %s\n", __func__, current_mode->hdr_mode, dev->sensor_name);

    pr_info("%s enter exp_frame_type(%d)\n", __func__, current_mode->hdr_mode);
    if (current_mode->hdr_mode == SENSOR_MODE_LINEAR) {
        dgain = (k_u32)(gain.gain[SENSOR_LINEAR_PARAS] * 1024);
        //ret = sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_H,(dgain & 0x0300)>>8);
        //ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_L,(dgain & 0xff));
        current_mode->ae_info.cur_dgain = (float)dgain/1024.0f;

    } else if (current_mode->hdr_mode == SENSOR_MODE_HDR_STITCH) {
        dgain = (k_u32)(gain.gain[SENSOR_DUAL_EXP_L_PARAS] * 1024);
        // ret = sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_H,(again & 0x0300)>>8);
        // ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_AGAIN_L,(again & 0xff));
        current_mode->ae_info.cur_dgain = (float)dgain/1024.0f;

        dgain = (k_u32)(gain.gain[SENSOR_DUAL_EXP_S_PARAS] * 1024);
        //TODO wirte vs gain register
        current_mode->ae_info.cur_vs_dgain = (float)dgain/1024.0f;
    } else {
        pr_err("%s, unsupport exposure frame.\n", __func__);
        return -1;
    }
    current_mode->ae_info.cur_gain = current_mode->ae_info.cur_again * current_mode->ae_info.cur_dgain;
    pr_debug("%s,cur_gain(%d)\n", __func__, (k_u32)(current_mode->ae_info.cur_gain*10000));

    return ret;
}

static k_s32 sensor_get_intg_time_impl(void *ctx, k_sensor_intg_time *time)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    if (current_mode->hdr_mode == SENSOR_MODE_LINEAR) {
        time->intg_time[SENSOR_LINEAR_PARAS] = current_mode->ae_info.cur_integration_time;
    } else if (current_mode->hdr_mode == SENSOR_MODE_HDR_STITCH) {
        time->intg_time[SENSOR_DUAL_EXP_L_PARAS] = current_mode->ae_info.cur_integration_time;
        time->intg_time[SENSOR_DUAL_EXP_S_PARAS] = current_mode->ae_info.cur_vs_integration_time;
    } else {
        pr_err("%s, unsupport exposure frame.\n", __func__);
        return -1;
    }

    return ret;
}

static k_s32 sensor_set_intg_time_impl(void *ctx, k_sensor_intg_time time)
{
    k_s32 ret = 0;
    k_u16 exp_line = 0;
    float integraion_time = 0;
    struct sensor_driver_dev *dev = ctx;
    k_sensor_mode *current_mode = &dev->current_sensor_mode;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    k_u32 min_vts;
    k_u16 new_vts;
    k_u32 max_vts = current_mode->ae_info.frame_length;
    k_u32 width = current_mode->size.width;
    k_u32 height = current_mode->size.height;

    if((640 == width) && (480 == height)) {
    	min_vts = 573;
    } else if((1280 == width) && (720 == height)) {
    	min_vts = 851;
    } else if((1280 == width) && (960 == height)) {
    	min_vts = 1093;
    } else {
    	min_vts = max_vts;
    }

    if (current_mode->hdr_mode == SENSOR_MODE_LINEAR) {
        integraion_time = time.intg_time[SENSOR_LINEAR_PARAS];
        //printf("int_time = %f, one_line_time = %f \n", integraion_time, current_mode->ae_info.one_line_exp_time);

        exp_line = integraion_time / current_mode->ae_info.one_line_exp_time;
        exp_line = MIN(current_mode->ae_info.max_integraion_line, MAX(current_mode->ae_info.min_integraion_line, exp_line));
        if (current_mode->et_line != exp_line)
        {
    		if(min_vts == max_vts)
    		{
            	ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_H, ( exp_line >> 4) & 0xff);
            	ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_L, ( exp_line << 4) & 0xff);
    		}
    		else
    		{            
        		new_vts = exp_line + 12;
        		if(new_vts < min_vts) new_vts = min_vts;
        		else if(new_vts > max_vts) new_vts = max_vts;
        		if(current_mode->et_line<exp_line)
        		{
        			ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_VTS_H, ( new_vts >> 8) & 0xff);
            		ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_VTS_L,  new_vts & 0xff);
        			ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_H, ( exp_line >> 4) & 0xff);
            		ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_L, ( exp_line << 4) & 0xff);
        		}
        		else
        		{
        			ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_H, ( exp_line >> 4) & 0xff);
            		ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_LONG_EXP_TIME_L, ( exp_line << 4) & 0xff);
        			ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_VTS_H, ( new_vts >> 8) & 0xff);
            		ret |= sensor_reg_write(&dev->i2c_info, OV5647_REG_VTS_L,  new_vts & 0xff);
        		}
    		}
            current_mode->et_line = exp_line;
        }
        current_mode->ae_info.cur_integration_time = (float)current_mode->et_line * current_mode->ae_info.one_line_exp_time;
    } else if (current_mode->hdr_mode == SENSOR_MODE_HDR_STITCH) {
        integraion_time = time.intg_time[SENSOR_DUAL_EXP_L_PARAS];
        exp_line = integraion_time / current_mode->ae_info.one_line_exp_time;
        exp_line = MIN(current_mode->ae_info.max_integraion_line, MAX(current_mode->ae_info.min_integraion_line, exp_line));

        current_mode->ae_info.cur_integration_time = (float)exp_line * current_mode->ae_info.one_line_exp_time;

        integraion_time = time.intg_time[SENSOR_DUAL_EXP_S_PARAS];
        exp_line = integraion_time / current_mode->ae_info.one_line_exp_time;
        exp_line = MIN(current_mode->ae_info.max_integraion_line, MAX(current_mode->ae_info.min_integraion_line, exp_line));

        current_mode->ae_info.cur_vs_integration_time = (float)exp_line * current_mode->ae_info.one_line_exp_time;
    } else {
        pr_err("%s, unsupport exposure frame.\n", __func__);
        return -1;
    }
    pr_debug("%s exp_frame_type(%d), exp_line(%d), integraion_time(%u)\n",\
        __func__, current_mode->hdr_mode, exp_line, (k_u32)(integraion_time * 1000000000));

    return ret;
}

static k_s32 sensor_get_exp_parm_impl(void *ctx, k_sensor_exposure_param *exp_parm)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    memset(exp_parm, 0, sizeof(k_sensor_exposure_param));

    return ret;
}

static k_s32 sensor_set_exp_parm_impl(void *ctx, k_sensor_exposure_param exp_parm)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    return ret;
}

static k_s32 sensor_get_fps_impl(void *ctx, k_u32 *fps)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    *fps = 30000;

    return ret;
}

static k_s32 sensor_set_fps_impl(void *ctx, k_u32 fps)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    return ret;
}

static k_s32 sensor_get_isp_status_impl(void *ctx, k_sensor_isp_status *staus)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    memset(staus, 0, sizeof(k_sensor_isp_status));

    return ret;
}

static k_s32 sensor_set_blc_impl(void *ctx, k_sensor_blc blc)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    return ret;
}

static k_s32 sensor_set_wb_impl(void *ctx, k_sensor_white_balance wb)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    return ret;
}

static k_s32 sensor_get_tpg_impl(void *ctx, k_sensor_test_pattern *tpg)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    memset(tpg, 0, sizeof(k_sensor_test_pattern));

    return ret;
}

static k_s32 sensor_set_tpg_impl(void *ctx, k_sensor_test_pattern tpg)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);

    return ret;
}

static k_s32 sensor_get_expand_curve_impl(void *ctx, k_sensor_compand_curve *curve)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    memset(curve, 0, sizeof(k_sensor_compand_curve));

    return ret;
}

static k_s32 sensor_get_otp_data_impl(void *ctx, void *data)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;
    (void)dev;

    pr_info("%s enter, %s\n", __func__, dev->sensor_name);
    memset(data, 0, sizeof(void *));

    return ret;
}

static k_s32 sensor_mirror_set_impl(void *ctx, k_vicap_mirror_mode mirror)
{
    k_s32 ret = 0;
    struct sensor_driver_dev *dev = ctx;

    pr_info("mirror is %d , sensor tpye is %d, name is %s\n", mirror.mirror, mirror.sensor_type, dev->sensor_name);

    dev->mirror_setting = mirror;

    return ret;
}

static const k_sensor_function sensor_functions = {
    .sensor_power = sensor_power_impl,
    .sensor_init = sensor_init_impl,
    .sensor_get_chip_id = sensor_get_chip_id_impl,
    .sensor_get_mode = sensor_get_mode_impl,
    .sensor_set_mode = sensor_set_mode_impl,
    .sensor_enum_mode = sensor_enum_mode_impl,
    .sensor_get_caps = sensor_get_caps_impl,
    .sensor_conn_check = sensor_conn_check_impl,
    .sensor_set_stream = sensor_set_stream_impl,
    .sensor_get_again = sensor_get_again_impl,
    .sensor_set_again = sensor_set_again_impl,
    .sensor_get_dgain = sensor_get_dgain_impl,
    .sensor_set_dgain = sensor_set_dgain_impl,
    .sensor_get_intg_time = sensor_get_intg_time_impl,
    .sensor_set_intg_time = sensor_set_intg_time_impl,
    .sensor_get_exp_parm = sensor_get_exp_parm_impl,
    .sensor_set_exp_parm = sensor_set_exp_parm_impl,
    .sensor_get_fps = sensor_get_fps_impl,
    .sensor_set_fps = sensor_set_fps_impl,
    .sensor_get_isp_status = sensor_get_isp_status_impl,
    .sensor_set_blc = sensor_set_blc_impl,
    .sensor_set_wb = sensor_set_wb_impl,
    .sensor_get_tpg = sensor_get_tpg_impl,
    .sensor_set_tpg = sensor_set_tpg_impl,
    .sensor_get_expand_curve = sensor_get_expand_curve_impl,
    .sensor_get_otp_data = sensor_get_otp_data_impl,
    .sensor_mirror_set = sensor_mirror_set_impl,
};
/*****************************************************************************/
k_s32 sensor_ov5647_probe(struct k_sensor_probe_cfg *cfg, struct sensor_driver_dev *dev)
{
    k_s32 ret = 0;
    k_u32 chip_id = 0x1111;
    k_u32 chip_id_2;
    const k_sensor_mode *sensor_mode = NULL;
//rt_kprintf("ov5647 aaaaaaaaaaaaaaaaaaaaaaaa\n");
#if defined (CONFIG_MPP_ENABLE_CSI_DEV_0)
    if(0x00 == cfg->csi_num) {
        dev->mode_count = sizeof(sensor_csi0_mode_list) / sizeof(sensor_csi0_mode_list[0]);
        dev->sensor_mode_list = &sensor_csi0_mode_list[1];
        sensor_mode = &dev->sensor_mode_list[0];
    } else
#endif // CONFIG_MPP_ENABLE_CSI_DEV_0
#if defined (CONFIG_MPP_ENABLE_CSI_DEV_1)
    if(0x01 == cfg->csi_num) {
        dev->mode_count = sizeof(sensor_csi1_mode_list) / sizeof(sensor_csi1_mode_list[0]);
        dev->sensor_mode_list = &sensor_csi1_mode_list[1];
        sensor_mode = &dev->sensor_mode_list[0];
    } else
#endif
#if defined (CONFIG_MPP_ENABLE_CSI_DEV_2)
    if(0x02 == cfg->csi_num) {
        dev->mode_count = sizeof(sensor_csi2_mode_list) / sizeof(sensor_csi2_mode_list[1]);
        dev->sensor_mode_list = &sensor_csi2_mode_list[1];
        sensor_mode = &dev->sensor_mode_list[0];
    }
#endif
//rt_kprintf("ov5647 bbbbbbbbbbbbbbbbbbbbbbbbbb\n");
    if(0x00 == dev->mode_count) {
        goto _on_failed;
    }

    if(NULL == sensor_mode) {
        rt_kprintf("FATAL error, %s\n", __func__);
        goto _on_failed;
    }

    /* update dev */
    dev->pwd_gpio = cfg->pwd_gpio;
    dev->reset_gpio = cfg->reset_gpio;

    if(NULL == (dev->i2c_info.i2c_bus = rt_i2c_bus_device_find(cfg->i2c_name))) {
        rt_kprintf("Can't find %s\n", cfg->i2c_name);
        goto _on_failed;
    }
    strncpy(&dev->i2c_info.i2c_name[0], cfg->i2c_name, sizeof(dev->i2c_info.i2c_name));

    memcpy(&dev->sensor_func, &sensor_functions, sizeof(k_sensor_function));

    /* probe sensor */
    sensor_set_mclk(&sensor_mode->mclk_setting[0]);

    /** NEW SENSOR MODIFY START */
    snprintf(dev->sensor_name, sizeof(dev->sensor_name), "ov5647_csi%d", cfg->csi_num);

    _sensor_power_state_set(dev, 1, 1);

    dev->i2c_info.reg_addr_size = SENSOR_REG_VALUE_16BIT;
    dev->i2c_info.reg_val_size = SENSOR_REG_VALUE_8BIT;
    dev->i2c_info.slave_addr = 0x36;
    if((0x00 != _sensor_read_chip_id_r(dev, &chip_id)) || (OV5647_CHIP_ID != chip_id)) {
        rt_kprintf("ov5647 read chip id failed, 0x%04x\n", chip_id);
        goto _on_failed;
    }
    /** NEW SENSOR MODIFY END */
    //rt_kprintf("ov5647 22222222222222222222222222222\n");
    if((0x00 != _sensor_read_chip_id_r(dev, &chip_id_2)) || (OV5647_CHIP_ID != chip_id_2)) {
        rt_kprintf("ov5647 read chip id failed, 0x%04x\n", chip_id);
        goto _on_failed;
    }





    return 0;

_on_failed:
    memset(dev, 0, sizeof(*dev));

    return -1;
}
