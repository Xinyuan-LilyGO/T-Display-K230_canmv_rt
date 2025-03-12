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

#include "regulator.h"
#include <rtdevice.h>
#include <rtthread.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

struct dummpy_dev {
  struct regulator_dev dev;
  const char *dev_name;
};

static int dummpy_init(struct regulator_dev *pdev) { return RT_EOK; }

static int dummpy_deinit(struct regulator_dev *pdev) { return 0; }

static int dummpy_enable(struct regulator_dev *pdev, bool enable) {
  return RT_EOK;
}

static int dummpy_set_voltage(struct regulator_dev *pdev, int vol_uv) {
  return RT_EOK;
}

static int dummpy_get_voltage(struct regulator_dev *pdev, int *pvol_uv) {
  return RT_EOK;
}

static struct regulator_ops dummy_ops = {
    .init = dummpy_init,
    .deinit = dummpy_deinit,
    .enable = dummpy_enable,
    .set_voltage = dummpy_set_voltage,
    .get_voltage = dummpy_get_voltage,
};

static struct dummpy_dev dev[2] = {
    {
        .dev_name = "regulator_cpu",
    },
    {
        .dev_name = "regulator_kpu",
    },
};

int regulator_dummpy_init(void) {
  int ret;

  for (int i = 0; i < ARRAY_SIZE(dev); i++) {
    ret = regulator_dev_register(&dev[i].dev, &dummy_ops, dev[i].dev_name);
    if (ret) {
      rt_kprintf("dummpy %d register fial: %d\n", i, ret);
      break;
    }
  }

  return ret;
}
INIT_DEVICE_EXPORT(regulator_dummpy_init);
