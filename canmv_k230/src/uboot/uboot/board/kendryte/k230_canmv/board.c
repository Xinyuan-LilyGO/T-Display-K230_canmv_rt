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

int ddr_init_training(void) {
  if (0x00 != (readl((const volatile void __iomem *)0x980001bcULL) & 0x1)) {
    // have init ,not need reinit;
    return 0;
  }

  board_ddr_init();

  return 0;
}

int board_early_init_f(void) {
  /* force set boot medium to sdio1 */
  g_boot_medium = BOOT_MEDIUM_SDIO1;
  return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void) {
  ofnode node;

  node = ofnode_by_compatible(ofnode_null(), "kendryte,k230_canmv");
  if (ofnode_valid(node)) {
#define GPIO_BASE_ADDR0     (0x9140B000U)

    u32 wifi_regon_gpio1_dir = readl((void *)(GPIO_BASE_ADDR0 + 0x4));
    wifi_regon_gpio1_dir |= 1 << 1;
    writel(wifi_regon_gpio1_dir, (void *)(GPIO_BASE_ADDR0 + 0x4));

    // reset gpio1 -> WIFI REGON
    u32 wifi_regon_gpio1_data = readl((void *)(GPIO_BASE_ADDR0 + 0x0));
    wifi_regon_gpio1_data &= ~(1 << 1);
    writel(wifi_regon_gpio1_data, (void *)(GPIO_BASE_ADDR0 + 0x0));
    mdelay(10);
    // reset gpio1 -> WIFI REGON
    wifi_regon_gpio1_data |= 1 << 1;
    writel(wifi_regon_gpio1_data, (void *)(GPIO_BASE_ADDR0 + 0x0));
  }

#define USB_IDPULLUP0 (1 << 4)
#define USB_DMPULLDOWN0 (1 << 8)
#define USB_DPPULLDOWN0 (1 << 9)

  u32 usb0_test_ctl3 = readl((void *)USB0_TEST_CTL3);
  u32 usb1_test_ctl3 = readl((void *)USB1_TEST_CTL3);

  usb0_test_ctl3 |= USB_IDPULLUP0;
  usb1_test_ctl3 |= USB_IDPULLUP0;

  writel(usb0_test_ctl3, (void *)USB0_TEST_CTL3);
  writel(usb1_test_ctl3, (void *)USB1_TEST_CTL3);

  return 0;
}
#endif
