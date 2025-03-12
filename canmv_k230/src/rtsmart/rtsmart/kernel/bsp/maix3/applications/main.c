/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <rthw.h>
#include <rtthread.h>

#include <dfs_fs.h>
#include <ioremap.h>
#include "riscv_mmu.h"

#include <msh.h>

#include "./config.h"
#include "dfs_posix.h"

#ifdef CONFIG_BOARD_K230_CANMV_LCKFB
#include "drv_gpio.h"
#endif // CONFIG_BOARD_K230_CANMV_LCKFB

#ifdef ENABLE_CHERRY_USB

#include "canmv_usb.h"

#ifdef ENABLE_CHERRY_USB_DEVICE
#include "usbd_core.h"
#endif // ENABLE_CHERRY_USB_DEVICE

#ifdef ENABLE_CHERRY_USB_HOST
#include "usbh_core.h"
#include "drv_gpio.h"
#endif // ENABLE_CHERRY_USB_HOST

#if defined(ENABLE_CHERRY_USB_DEVICE) && defined (ENABLE_CHERRY_USB_HOST)
  #if CHERRY_USB_DEVICE_USING_DEV + CHERRY_USB_HOST_USING_DEV != 1
    #error "Can not set same usb device as device and host"
  #endif
#endif

#endif // ENABLE_CHERRY_USB

#ifdef RT_USING_SDIO

#define TO_STRING(x) #x
#define CONCAT(x, y, z) x y z

#define SD_DEV_PART(s, d, p) CONCAT(s, TO_STRING(d), p)
#ifndef RT_SHELL_PATH
#define RT_SHELL_PATH "/bin/init.sh"
#endif
static const struct dfs_mount_tbl custom_mount_table[] = {
  {SD_DEV_PART("sd", SDCARD_ON_SDIO_DEV, "0"), "/bin", "elm", 0, 0},
  {SD_DEV_PART("sd", SDCARD_ON_SDIO_DEV, "1"), "/sdcard", "elm", 0, 0},
  {SD_DEV_PART("sd", SDCARD_ON_SDIO_DEV, "2"), "/data", "elm", 0, 0},
  {0}
};

static void mnt_mount_table(void)
{
    int ret;
    int err = 0;
    int index = 0;
    int fd = -1;
    int mkfs_for_data_partition = 0;

    while (1)
    {
        if (custom_mount_table[index].path == NULL) break;

        if (0x00 != (ret = dfs_mount(custom_mount_table[index].device_name,
                      custom_mount_table[index].path,
                      custom_mount_table[index].filesystemtype,
                      custom_mount_table[index].rwflag,
                      custom_mount_table[index].data)))
        {
            err = errno;
            rt_kprintf("mount fs[%s] on %s failed(%d), error %d.\n", custom_mount_table[index].filesystemtype,
                       custom_mount_table[index].path, ret, err);

          if(0x00 == strcmp("/data", custom_mount_table[index].path)) {
              g_fs_mount_data_succ = false;

#if defined (CONFIG_RT_AUTO_RESIZE_PARTITION)
              if(0 <= (fd = open("/bin/auto_mkfs_data", O_RDONLY))) {
                close(fd);
                unlink("/bin/auto_mkfs_data");
                fd = 0x1234;
              }

              if((0x1234 == fd) && (0x00 == mkfs_for_data_partition)) {
                mkfs_for_data_partition = 1;

                rt_kprintf("\033[31mStart format partition[2] to fat, it will took a long time, DO NOT POWEROFF THE BOARD, PLEASE WAIT IT DONE\033[0m\n");
                dfs_mkfs("elm", custom_mount_table[index].device_name);
                rt_kprintf("\n\n\033[32mformat done.\033[0m\n");

                index--;
              }
#endif

              if((-19) == err) {
                rt_kprintf("Please format the partition[2] to FAT32.\nRefer to https://support.microsoft.com/zh-cn/windows/%E5%88%9B%E5%BB%BA%E5%92%8C%E6%A0%BC%E5%BC%8F%E5%8C%96%E7%A1%AC%E7%9B%98%E5%88%86%E5%8C%BA-bbb8e185-1bda-ecd1-3465-c9728f7d7d2e\n");
              }
            }
        } else {
          if(0x00 == strcmp("/data", custom_mount_table[index].path)) {
            g_fs_mount_data_succ = true;
          }
        }

        index ++;
    }
}
#endif // RT_USING_SDIO

static void check_bank_voltage(void)
{
#define MAP_SIZE    PAGE_SIZE
#define MAP_MASK    (MAP_SIZE - 1)

  const rt_ubase_t target = 0x91213418UL;
  void *map_base = rt_ioremap_nocache((void *)(target & ~MAP_MASK), MAP_SIZE);
  volatile void *virt_addr = map_base + (target & MAP_MASK);
  volatile rt_uint32_t read_result = *((rt_uint32_t *) virt_addr);

  if(0x00 == read_result) {
    rt_kprintf("\n\n\033[31mTHIS BOARD MAYBE NOT CONFIGURE BANK VOLTAGE!!!\n\n\033[0m");
#ifdef CONFIG_BOARD_K230_CANMV_LCKFB
    kd_pin_mode(62, GPIO_DM_OUTPUT);
    kd_pin_write(62, GPIO_PV_LOW);
#endif
  }

  rt_iounmap(map_base);
}

int main(void) {
  check_bank_voltage();

#ifdef CONFIG_SDK_ENABLE_CANMV
  rt_kprintf("CanMV Start\n");
#endif //CONFIG_SDK_ENABLE_CANMV

#ifdef RT_USING_SDIO
  while (mmcsd_wait_cd_changed(100) != MMCSD_HOST_PLUGED) {
  }
  mnt_mount_table();

  excute_sdcard_config();
#endif //RT_USING_SDIO

#ifdef ENABLE_CHERRY_USB
  void *usb_base;
  const void *usb_dev_addr[2] = {(void *)0x91500000UL, (void *)0x91540000UL};

  /* Strange BUG, ​​USB Host must be initialized first */
#if defined (ENABLE_CHERRY_USB_HOST) && defined (ENABLE_CANMV_USB_HOST)
  usb_base = (void *)rt_ioremap((void *)usb_dev_addr[CHERRY_USB_HOST_USING_DEV], 0x10000);

  usbh_initialize(0, (uint32_t)usb_base);
#endif // ENABLE_CHERRY_USB_HOST

#if defined (ENABLE_CHERRY_USB_DEVICE) && defined (ENABLE_CANMV_USB_DEV)
  usb_base = (void *)rt_ioremap((void *)usb_dev_addr[CHERRY_USB_DEVICE_USING_DEV], 0x10000);

  canmv_usb_device_init(usb_base);

#ifdef CANMV_USB_PWR_PIN
  int usb_host_pin = CANMV_USB_PWR_PIN;
  if(0 <= usb_host_pin) {
    kd_pin_mode(usb_host_pin, GPIO_DM_OUTPUT);
    kd_pin_write(usb_host_pin, CANMV_USB_PWR_PIN_VALID_VAL);
  }
#endif

#endif // ENABLE_CHERRY_USB_DEVICE

  rt_thread_delay(10);
#endif //ENABLE_CHERRY_USB

#ifdef CONFIG_SDK_ENABLE_CANMV
  msh_exec("/sdcard/micropython", 32);
#endif //CONFIG_SDK_ENABLE_CANMV
  msh_exec(RT_SHELL_PATH, strlen(RT_SHELL_PATH)+1);
  return 0;
}
