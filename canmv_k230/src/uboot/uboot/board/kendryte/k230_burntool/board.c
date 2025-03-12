#include <asm/asm.h>
#include <asm/io.h>
#include <asm/spl.h>
#include <asm/types.h>
#include <command.h>
#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <env_internal.h>
#include <image.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <lmb.h>
#include <stdio.h>

#include <kendryte/k230_platform.h>

#include "board_common.h"

int board_early_init_f(void) {
  return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void) {
#define USB_IDPULLUP0 (1 << 4)
#define USB_DMPULLDOWN0 (1 << 8)
#define USB_DPPULLDOWN0 (1 << 9)

  u32 usb0_test_ctl3 = readl((void *)USB0_TEST_CTL3);
  usb0_test_ctl3 |= USB_IDPULLUP0;
  writel(usb0_test_ctl3, (void *)USB0_TEST_CTL3);

  return 0;
}
#endif
