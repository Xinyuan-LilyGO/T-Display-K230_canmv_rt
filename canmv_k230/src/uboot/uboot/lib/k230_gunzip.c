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
#include <asm/io.h>
#include <command.h>
#include <common.h>
#include <cpu_func.h>
#include <linux/delay.h>
#include <malloc.h>
#include <stdio.h>

#define ZIP_LINE_SIZE (128 * 1024)
#define ZIP_RD_CH DMA_CH_0
#define ZIP_WR_CH DMA_CH_1

#define UGZIP_BASE_ADDR (0x80808000ULL)
#define GSDMA_CTRL_ADDR (0x80800000ULL)
#define SRAM_RD_ADDR (0x80280000ULL)
#define SRAM_WR_ADDR (0X80200000ULL)
#define SDMA_CH_CFG (0x80800050ULL)
#define SDMA_CH_LENGTH 0x30

typedef struct sdma_llt {
  uint32_t reserved_0 : 28;
  uint32_t dimension : 1;
  uint32_t pause : 1;
  uint32_t node_intr : 1;
  uint32_t reserved : 1;

  uint32_t src_addr;

  uint32_t line_size;

  uint32_t line_num : 16;
  uint32_t line_space : 16;

  uint32_t dst_addr;

  uint32_t next_llt_addr;
} sdma_llt_t;

typedef enum DIMENSION {
  DIMENSION1,
  DIMENSION2,
} sdma_dimension_e;

struct ugzip_reg {
  uint32_t decomp_start;
  uint32_t gzip_src_size;
  uint32_t dma_out_size;
  uint32_t decomp_intstat;
};
typedef enum dma_ch {
  DMA_CH_0 = 0,
  DMA_CH_1 = 1,
  DMA_CH_2 = 2,
  DMA_CH_3 = 3,
  DMA_CH_4 = 4,
  DMA_CH_5 = 5,
  DMA_CH_6 = 6,
  DMA_CH_7 = 7,
  DMA_CH_MAX,
} dma_ch_t;

typedef enum ugzip_rw {
  UGZIP_RD = 0,
  UGZIP_WR = 1,
} ugzip_rw_e;

typedef struct sdma_ch_cfg {
  uint32_t ch_ctl;
  uint32_t ch_status;
  uint32_t ch_cfg;
  uint32_t ch_usr_data;
  uint32_t ch_llt_saddr;
  uint32_t ch_current_llt;
} sdma_ch_cfg_t;

typedef struct gsdma_ctrl {
  uint32_t dma_ch_en;
  uint32_t dma_int_mask;
  uint32_t dma_int_stat;
  uint32_t dma_cfg;
  uint32_t reserved[11];
  uint32_t dma_weight;
} gsdma_ctrl_t;

sdma_llt_t *g_llt_list_w = NULL;
sdma_llt_t *g_llt_list_r = NULL;
static uint32_t *ugzip_llt_cal(uint8_t *addr, uint32_t length,
                               ugzip_rw_e mode) {
  int i;
  uint32_t list_num;
  sdma_llt_t *llt_list;

  list_num = (length - 1) / ZIP_LINE_SIZE + 1;
  llt_list = (sdma_llt_t *)malloc(sizeof(sdma_llt_t) * list_num);
  if (NULL == llt_list) {
    printf("malloc error =\n");
    return NULL;
  }

  if (mode == UGZIP_RD) {
    g_llt_list_r = llt_list;
  } else {
    g_llt_list_w = llt_list;
  }

  memset(llt_list, 0, sizeof(sdma_llt_t) * list_num);
  for (i = 0; i < list_num; i++) {
    llt_list[i].dimension = DIMENSION1;
    llt_list[i].pause = 0;
    llt_list[i].node_intr = 0;

    if (mode == UGZIP_RD) { /* from memory to sram */
      llt_list[i].src_addr = ((uint32_t)(uint64_t)addr + ZIP_LINE_SIZE * i);
      llt_list[i].dst_addr = SRAM_RD_ADDR + (i % 2) * ZIP_LINE_SIZE;
    } else if (mode == UGZIP_WR) { /* from sram to memory */
      llt_list[i].src_addr = SRAM_WR_ADDR + (i % 4) * ZIP_LINE_SIZE;
      llt_list[i].dst_addr = ((uint32_t)(uint64_t)addr + ZIP_LINE_SIZE * i);
    }

    if (i == list_num - 1) {
      llt_list[i].line_size = ZIP_LINE_SIZE;
      llt_list[i].next_llt_addr = 0;
    } else {
      llt_list[i].line_size = ZIP_LINE_SIZE;
      llt_list[i].next_llt_addr = (uint32_t)(uint64_t)(&llt_list[i + 1]);
    }
  }

  flush_dcache_range((uint64_t)llt_list,
                     (uint64_t)llt_list + sizeof(sdma_llt_t) * list_num);

  return (uint32_t *)llt_list;
}

static int ugzip_sdma_cfg(uint8_t ch, ugzip_rw_e mode, uint8_t *addr,
                          uint32_t length) {
  uint32_t unzip_list_add = 0;
  struct sdma_ch_cfg *ch_cfg = (struct sdma_ch_cfg *)SDMA_CH_CFG;
  struct gsdma_ctrl *gsct = (struct gsdma_ctrl *)GSDMA_CTRL_ADDR;
  uint32_t int_stat;

  writel(0x2,
         (volatile void *)((uint64_t)&ch_cfg->ch_ctl + ch * SDMA_CH_LENGTH));
  while (0x1 & readl((volatile void *)((uint64_t)&ch_cfg->ch_status +
                                       ch * SDMA_CH_LENGTH))) {
  }

  int_stat = readl((volatile void *)&gsct->dma_ch_en);

  writel(int_stat | 1 << ch, (volatile void *)&gsct->dma_ch_en);

  writel(0x111 << ch, (volatile void *)&gsct->dma_int_stat);

  if (ch == ZIP_RD_CH) {
    writel(0x1 << 10,
           (volatile void *)((uint64_t)&ch_cfg->ch_cfg + ch * SDMA_CH_LENGTH));
  }

  unzip_list_add = (uint32_t)(uint64_t)ugzip_llt_cal(addr, length, mode);

  writel(unzip_list_add, (volatile void *)((uint64_t)&ch_cfg->ch_llt_saddr +
                                           ch * SDMA_CH_LENGTH));

  writel(0x1,
         (volatile void *)((uint64_t)&ch_cfg->ch_ctl + ch * SDMA_CH_LENGTH));

  return 0;
}

int k230_priv_unzip(void *dst, int dstlen, unsigned char *src,
                    unsigned long *lenp) {
  struct ugzip_reg *pUgzipReg = (struct ugzip_reg *)UGZIP_BASE_ADDR;
  struct sdma_ch_cfg *ch_cfg = (struct sdma_ch_cfg *)SDMA_CH_CFG;
  struct gsdma_ctrl *gsct = (struct gsdma_ctrl *)GSDMA_CTRL_ADDR;

  int ret = 3;
  uint64_t stime = get_ticks();
  uint64_t etime = stime + 80000000;
  uint32_t int_stat, decomp_intstat;
  volatile uint32_t *p_rest_reg = NULL;

  flush_dcache_range((uint64_t)src, (uint64_t)src + (uint64_t)*lenp);
  writel(0x80000000, (volatile void *)&pUgzipReg->gzip_src_size);
  ugzip_sdma_cfg(ZIP_RD_CH, UGZIP_RD, src, *lenp);
  ugzip_sdma_cfg(ZIP_WR_CH, UGZIP_WR, dst, dstlen);
  writel(0x51f, (volatile void *)0x91302310ULL);
  writel(*lenp | (0x1 << 31), (volatile void *)&pUgzipReg->gzip_src_size);
  writel(dstlen, (volatile void *)&pUgzipReg->dma_out_size);
  writel(0x3, (volatile void *)&pUgzipReg->decomp_start);

  do {
    int_stat = readl((volatile void *)&gsct->dma_int_stat);
    decomp_intstat = readl((volatile void *)&pUgzipReg->decomp_intstat);
    if (int_stat) {

      if (int_stat & 0x111) {
        writel(0x111 << 0, (volatile void *)&gsct->dma_int_stat);
      }
      if (int_stat & 0x222) {
        writel(0x222, (volatile void *)&gsct->dma_int_stat);
      }
      if (int_stat & 0x2) {
        if (((0x1 << 10) & decomp_intstat)) {
          ret = 0;
        } else {
          ret = 1;
          printf("unzip crc error %x  int %x \n", decomp_intstat, int_stat);
        }
        break;
      }
    }
    if (etime < get_ticks()) {
      ret = 2;
      writel(0x111 << 0, (volatile void *)&gsct->dma_int_stat);
      writel(0x111 << 1, (volatile void *)&gsct->dma_int_stat);
      break;
    }
  } while (1);
  if (g_llt_list_r) {
    free(g_llt_list_r);
    g_llt_list_r = NULL;
  }
  if (g_llt_list_w) {
    free(g_llt_list_w);
    g_llt_list_w = NULL;
  }
  invalidate_dcache_range((uint64_t)dst, (uint64_t)dst + dstlen);
  etime = get_ticks();
  if (ret) {
    p_rest_reg = (volatile int *)(0x91101000 + 0x54);
    writel(0x2, p_rest_reg);
    while (0 == (readl(p_rest_reg) & BIT(29))) {
    }
    writel(BIT(29), p_rest_reg);
    p_rest_reg = (volatile int *)(0x91101000 + 0x5c);
    writel(0x1, p_rest_reg);
    while (0 == (readl(p_rest_reg) & BIT(31))) {
    }
    writel(BIT(31), p_rest_reg);
  }
  writel(0, (volatile void *)&pUgzipReg->gzip_src_size);
  writel(0, (volatile void *)((uint64_t)&ch_cfg->ch_cfg));

  return ret;
}
