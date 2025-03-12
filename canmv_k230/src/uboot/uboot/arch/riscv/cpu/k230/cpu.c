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
#include <common.h>
#include <asm/types.h>
#include <asm/asm.h>
#include <asm/csr.h>
#include <common.h>
#include <cpu_func.h>
#include <fdt_support.h>
#include <asm/io.h>
#include <command.h>
#include <spl.h>
#include <asm/cache.h>
#include <linux/delay.h>

#include <kendryte/k230_platform.h>

int g_boot_medium = 0xFF;

static inline void improving_cpu_performance(void)
{
	/* Set cpu regs */
	csr_write(CSR_MCOR, 0x70013);
	csr_write(CSR_MCCR2, 0xe0000009);
	csr_write(CSR_MHCR, 0x11ff);
	csr_write(CSR_MXSTATUS, 0x638000);
	csr_write(CSR_MHINT, 0x6e30c);
}

/*
 * cleanup_before_linux() is called just before we call linux
 * it prepares the processor for linux
 *
 * we disable interrupt and caches.
 */
int cleanup_before_linux(void)
{
	cache_flush();
	icache_disable();
	dcache_disable();

	asm volatile(".long 0x0170000b\n":::"memory");

	improving_cpu_performance();
	csr_write(CSR_SMPEN, 0x1);

	return 0;
}

void harts_early_init(void)
{
	/* read boot from which medium */
	int temp = readl((volatile void __iomem *)(SYSCTL_BOOT_BASE_ADDR + 0x40));
	g_boot_medium = temp & 0x03;

    writel(0x1, (volatile void __iomem *)0x91108020);
	writel(0x1, (volatile void __iomem *)0x91108030);
	writel(0x69, (volatile void __iomem *)0x91108000);

	writel(0x80199805, (volatile void __iomem *)0x91100004);

    writel(0x0, (volatile void __iomem *)(SYSCTL_PWR_BASE_ADDR + 0x158));

	csr_write(pmpaddr0, 0x24484dff);
	csr_write(pmpaddr1, 0x244851ff);
	csr_write(pmpcfg0, 0x9999);

	improving_cpu_performance();
}
