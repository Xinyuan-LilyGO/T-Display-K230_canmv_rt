// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2017 Eddie Cai <eddie.cai.linux@gmail.com>
 */

#include <command.h>
#include <console.h>
#include <g_dnl.h>
#include <usb.h>

#include "kburn.h"

static int do_kburn(struct cmd_tbl *cmdtp, int flag, int argc,
		      char *const argv[])
{
	int ret, controller_index;
	char *usb_controller;

	if (argc < 1)
		return CMD_RET_USAGE;

	usb_controller = argv[1];
	controller_index = simple_strtoul(usb_controller, NULL, 0);

	ret = usb_gadget_initialize(controller_index);
	if (ret) {
		printf("USB init failed: %d\n", ret);
		return CMD_RET_FAILURE;
	}

	g_dnl_clear_detach();
	ret = g_dnl_register("usb_dnl_kburn");
	if (ret)
		return CMD_RET_FAILURE;

	if (!g_dnl_board_usb_cable_connected()) {
		puts("\rUSB cable not detected, Command exit.\n");
		ret = CMD_RET_FAILURE;
		goto exit;
	}

	while (1) {
		if (g_dnl_detach())
			break;
		if (ctrlc())
			break;
		usb_gadget_handle_interrupts(controller_index);
	}
	ret = CMD_RET_SUCCESS;

exit:
	g_dnl_unregister();
	g_dnl_clear_detach();
	usb_gadget_release(controller_index);

	return ret;
}

U_BOOT_CMD(kburn, 2, 1, do_kburn,
	   "Canaan usb burner protocol",
	   "<USB_controller> e.g. kburn 0\n"
);

#if defined (CONFIG_CMD_KBURN_BENCHMARK)
/*****************************************************************************/
#include <common.h>
#include <command.h>
#include <mmc.h>
#include <timer.h>

int test_mmc_write_speed(int dev_num, ulong start_block, ulong block_count)
{
    struct mmc *mmc;
    ulong start_time, end_time;
    ulong write_size = block_count * 512; // Assuming block size is 512 bytes
    char *write_buffer;
    int ret;

    mmc = find_mmc_device(dev_num);
    if (!mmc) {
        printf("MMC device %d not found\n", dev_num);
        return -1;
    }

    ret = mmc_init(mmc);
    if (ret) {
        printf("MMC init failed\n");
        return -1;
    }

    write_buffer = malloc(write_size);
    if (!write_buffer) {
        printf("Memory allocation failed\n");
        return -1;
    }

    memset(write_buffer, 0xAA, write_size); // Fill the buffer with a pattern

    start_time = get_timer(0); // Get start time

	ret = blk_dwrite(mmc_get_blk_desc(mmc), start_block, block_count, write_buffer);

    end_time = get_timer(start_time); // Get end time

    if (ret != block_count) {
        printf("MMC write failed\n");
        free(write_buffer);
        return -1;
    }

    printf("MMC write speed: %lu bytes/sec\n", write_size * 1000 / end_time);

    free(write_buffer);
    return 0;
}

int do_test_mmc_speed(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    int dev_num = 0;
    ulong start_block = 0;
    ulong block_count = 1024; // Default to 1024 blocks

    if (argc > 1)
        dev_num = simple_strtol(argv[1], NULL, 10);
    if (argc > 2)
        start_block = simple_strtol(argv[2], NULL, 10);
    if (argc > 3)
        block_count = simple_strtol(argv[3], NULL, 10);

    return test_mmc_write_speed(dev_num, start_block, block_count);
}

U_BOOT_CMD(
    kburn_bench_mmc, 4, 0, do_test_mmc_speed,
    "Test MMC write speed",
    "[dev_num] [start_block] [block_count] - Test write speed of MMC device"
);

/*****************************************************************************/
#include <common.h>
#include <command.h>
#include <spi.h>
#include <spi_flash.h>
#include <dm.h>
#include <timer.h>

int test_spi_flash_write_speed(int index, ulong offset, ulong size)
{
    struct udevice *dev;
    struct spi_flash *flash;
    ulong start_time, end_time;
    char *write_buffer;
    int ret;

    ret = uclass_get_device(UCLASS_SPI_FLASH, index, &dev);
    if (ret) {
        printf("SPI flash device %d not found\n", index);
        return -1;
    }

    flash = dev_get_uclass_priv(dev);
    if (!flash) {
        printf("Failed to get SPI flash info\n");
        return -1;
    }

    write_buffer = malloc(size);
    if (!write_buffer) {
        printf("Memory allocation failed\n");
        return -1;
    }

    memset(write_buffer, 0xAA, size); // Fill the buffer with a pattern

    start_time = get_timer(0); // Get start time

    ret = spi_flash_erase(flash, offset, size);
    ret += spi_flash_write(flash, offset, size, write_buffer);

    end_time = get_timer(start_time); // Get end time

    if (ret) {
        printf("SPI flash write failed\n");
        free(write_buffer);
        return -1;
    }

    printf("SPI flash write speed: %lu bytes/sec\n", size * 1000 / end_time);

    free(write_buffer);
    return 0;
}

int do_test_spi_flash_speed(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
    int index = 0;
    ulong offset = 0;
    ulong size = 1024 * 1024; // Default to 1MB

    if (argc > 1)
        index = simple_strtol(argv[1], NULL, 10);
    if (argc > 2)
        offset = simple_strtol(argv[2], NULL, 10);
    if (argc > 3)
        size = simple_strtol(argv[3], NULL, 10);

    return test_spi_flash_write_speed(index, offset, size);
}

U_BOOT_CMD(
    kburn_bench_sf, 4, 0, do_test_spi_flash_speed,
    "Test SPI flash write speed",
    "[index] [offset] [size] - Test write speed of SPI flash device"
);
/*****************************************************************************/

#endif // CONFIG_CMD_KBURN_BENCHMARK
