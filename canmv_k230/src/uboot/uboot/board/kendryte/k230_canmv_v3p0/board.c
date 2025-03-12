#include <asm/asm.h>
#include <asm/io.h>
#include <asm/spl.h>
#include <asm/types.h>
#include <command.h>
#include <common.h>
#include <cpu_func.h>
#include <env_internal.h>
#include <gzip.h>
#include <image.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <lmb.h>
#include <stdio.h>
#include <dm.h>

#include "board_common.h"

#include <kendryte/k230_platform.h>

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
/* USB ***********************************************************************/
#define USB_IDPULLUP0 (1 << 4)
#define USB_DMPULLDOWN0 (1 << 8)
#define USB_DPPULLDOWN0 (1 << 9)

  u32 usb0_test_ctl3 = readl((void *)USB0_TEST_CTL3);
  u32 usb1_test_ctl3 = readl((void *)USB1_TEST_CTL3);

  usb0_test_ctl3 |= USB_IDPULLUP0;
  usb1_test_ctl3 |= USB_IDPULLUP0;

  writel(usb0_test_ctl3, (void *)USB0_TEST_CTL3);
  writel(usb1_test_ctl3, (void *)USB1_TEST_CTL3);

/* MMC ***********************************************************************/
//   ofnode node;

//   node = ofnode_by_compatible(ofnode_null(), "kendryte,k230_canmv_v3p0");
//   if (ofnode_valid(node)) {
// #define SDHCI_EMMC_BASE 0x91580000
// #define SDHCI_EMMC_CTRL_R 0x52C
// #define EMMC_RST_N_OE 3
// #define EMMC_RST_N 2
//     u32 wifi_regon_ctrl = readl((void *)(SDHCI_EMMC_BASE + SDHCI_EMMC_CTRL_R));
//     wifi_regon_ctrl |= (1 << EMMC_RST_N_OE);
//     wifi_regon_ctrl &= ~(1 << EMMC_RST_N);
//     mdelay(10);
//     wifi_regon_ctrl |= (1 << EMMC_RST_N);
//   }

  return 0;
}
#endif
