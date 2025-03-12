// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2018, Bin Meng <bmeng.cn@gmail.com>
 */

#include <common.h>
#include <command.h>
#include <hang.h>

#if defined (CONFIG_KENDRYTE_K230)
#include <asm/io.h>
#include <kendryte/k230_platform.h>

int do_reset(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	printf("resetting ...\n");

#ifndef CONFIG_SPL_BUILD
	writel(0x10001, (void*)SYSCTL_BOOT_BASE_ADDR+0x60);
	while(1);
#else
	printf("reset not supported yet\n");
	hang();
#endif

	return 0;
}
#else
int do_reset(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	printf("resetting ...\n");

	printf("reset not supported yet\n");
	hang();

	return 0;
}
#endif
