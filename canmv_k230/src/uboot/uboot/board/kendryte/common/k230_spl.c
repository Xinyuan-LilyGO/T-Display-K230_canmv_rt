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
#include <asm-generic/sections.h>
#include <asm/asm.h>
#include <asm/io.h>
#include <asm/spl.h>
#include <asm/types.h>
#include <asm/cache.h>
#include <command.h>
#include <common.h>
#include <cpu_func.h>
#include <init.h>
#include <linux/kernel.h>
#include <lmb.h>
#include <stdio.h>

#include "board_common.h"

#ifndef CONFIG_UBOOT_SPL_BOOT_IMG_TYPE
  #define CONFIG_UBOOT_SPL_BOOT_IMG_TYPE BOOT_SYS_UBOOT
#endif

void board_boot_order(u32 *spl_boot_list) {
  if(BOOT_MEDIUM_NORFLASH == g_boot_medium) {
    spl_boot_list[0] = BOOT_DEVICE_SPI;
    // spl_boot_list[1] = BOOT_DEVICE_SPI;
  } else if(BOOT_MEDIUM_NANDFLASH == g_boot_medium) {
    spl_boot_list[0] = BOOT_DEVICE_NAND;
    // spl_boot_list[1] = BOOT_DEVICE_SPI;
  } else if(BOOT_MEDIUM_SDIO0 == g_boot_medium) {
    spl_boot_list[0] = BOOT_DEVICE_MMC1;
    // spl_boot_list[1] = BOOT_DEVICE_MMC2;
  } else /* if(BOOT_MEDIUM_SDIO1 == g_boot_medium) */{
    spl_boot_list[0] = BOOT_DEVICE_MMC2;
    // spl_boot_list[1] = BOOT_DEVICE_MMC1;
  }
}

void spl_board_prepare_for_boot(void) {
  cache_flush();
  icache_disable();
  dcache_disable();
  asm volatile(".long 0x0170000b\n" ::: "memory");
}

void spl_device_disable(void) {
  uint32_t value;

  // disable ai power
  if (readl((volatile void __iomem *)0x9110302c) & 0x2)
    writel(0x30001, (volatile void __iomem *)0x91103028);
  // disable vpu power
  if (readl((volatile void __iomem *)0x91103080) & 0x2)
    writel(0x30001, (volatile void __iomem *)0x9110307c);
  // disable dpu power
  if (readl((volatile void __iomem *)0x9110310c) & 0x2)
    writel(0x30001, (volatile void __iomem *)0x91103108);
  // disable disp power
  if (readl((volatile void __iomem *)0x91103040) & 0x2)
    writel(0x30001, (volatile void __iomem *)0x9110303c);
  // check disable status
  value = 1000000;
  while ((!(readl((volatile void __iomem *)0x9110302c) & 0x1) || !(readl((volatile void __iomem *)0x91103080) & 0x1) ||
          !(readl((volatile void __iomem *)0x9110310c) & 0x1) || !(readl((volatile void __iomem *)0x91103040) & 0x1)) &&
         value)
    value--;
  // disable ai clk
  value = readl((volatile void __iomem *)0x91100008);
  value &= ~((1 << 0));
  writel(value, (volatile void __iomem *)0x91100008);
  // disable vpu clk
  value = readl((volatile void __iomem *)0x9110000c);
  value &= ~((1 << 0));
  writel(value, (volatile void __iomem *)0x9110000c);
  // disable dpu clk
  value = readl((volatile void __iomem *)0x91100070);
  value &= ~((1 << 0));
  writel(value, (volatile void __iomem *)0x91100070);
  // disable mclk
  value = readl((volatile void __iomem *)0x9110006c);
  value &= ~((1 << 0) | (1 << 1) | (1 << 2));
  writel(value, (volatile void __iomem *)0x9110006c);
}

int spl_board_init_f(void) {
  int ret = 0;

  spl_device_disable();

  /* init dram */
  ddr_init_training();
  /* Clear the BSS. */
  memset(__bss_start, 0, (ulong)&__bss_end - (ulong)__bss_start);

  ret += k230_img_load_boot_sys(CONFIG_UBOOT_SPL_BOOT_IMG_TYPE);
  if (ret) {
    printf("uboot boot failed %d\n", ret);
  }

  return ret;
}
