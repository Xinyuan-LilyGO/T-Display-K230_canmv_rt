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
#include <asm/asm.h>
#include <asm/io.h>
#include <asm/spl.h>
#include <asm/types.h>
#include <command.h>
#include <common.h>
#include <cpu_func.h>
#include <dm/device-internal.h>
#include <gzip.h>
#include <image.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mtd/mtd.h>
#include <lmb.h>
#include <mmc.h>
#include <nand.h>
#include <spi.h>
#include <spi_flash.h>
#include <spl.h>
#include <stdio.h>

#ifdef CONFIG_K230_PUFS
#include "pufs_sm2.h"
#include <pufs_ecp.h>
#include <pufs_hmac.h>
#include <pufs_rt.h>
#include <pufs_sp38a.h>
#include <pufs_sp38d.h>
#endif

#include "board_common.h"

#define SUPPORT_MMC_LOAD_BOOT

#ifndef CONFIG_MEM_BASE_ADDR
#undef CONFIG_MEM_BASE_ADDR
#undef CONFIG_MEM_TOTAL_SIZE

// we assume min ddr size is 128MB, and dts set memory start address is 0.
#define CONFIG_MEM_BASE_ADDR    0x00
#define CONFIG_MEM_TOTAL_SIZE   (128 * 1024 * 1024)

#endif

static int k230_boot_reset_big_hard_and_run(ulong core_run_addr);
static int k230_boot_check_and_get_plain_data(firmware_head_s *pfh,
                                              ulong *pplain_addr);

unsigned long k230_get_encrypted_image_load_addr(void) {
  return CONFIG_MEM_BASE_ADDR + CONFIG_MEM_TOTAL_SIZE -
         (CONFIG_MEM_TOTAL_SIZE / 3);
}

unsigned long k230_get_encrypted_image_decrypt_addr(void) {
  return CONFIG_MEM_BASE_ADDR + CONFIG_MEM_TOTAL_SIZE -
         ((CONFIG_MEM_TOTAL_SIZE / 3) * 2);
}

static int k230_boot_decomp_to_load_addr(image_header_t *pUh, ulong des_len,
                                         ulong data, ulong *plen) {
  int ret = 0;

  int img_compress_algo = image_get_comp(pUh);
  ulong img_load_addr = (ulong)image_get_load(pUh);

  printf("image: %s load to %lx compress =%d src %lx len=%lx \n",
         image_get_name(pUh), img_load_addr, img_compress_algo, data, *plen);

  if (IH_COMP_GZIP == img_compress_algo) {
    if (0x00 !=
        (ret = gunzip((void *)img_load_addr, des_len, (void *)data, plen))) {
      printf("unzip fialed ret =%x\n", ret);
      return -1;
    }
  } else if (IH_COMP_NONE == img_compress_algo) {
    memmove((void *)img_load_addr, (void *)data, *plen);
  } else {
    printf("Error: Unsupport compress algo.\n");
    return -2;
  }

  flush_cache(img_load_addr, *plen);

  return ret;
}

static int k230_boot_rtt_uimage(image_header_t *pUh) {
  int ret = 0;
  ulong len = image_get_size(pUh);
  ulong data = image_get_data(pUh);

  // multi
  image_multi_getimg(pUh, 0, &data, &len);

  if (0x00 ==
      (ret = k230_boot_decomp_to_load_addr(pUh, 0x6000000, data, &len))) {
    k230_boot_reset_big_hard_and_run(image_get_load(pUh));
    while (1) {
      asm volatile("wfi");
    }
  }

  printf("Boot RT-Smart failed. %d\n", ret);

  return ret;
}

static int k230_boot_uboot_uimage(image_header_t *pUh) {
  void (*uboot)(ulong hart, void *dtb);

  int ret = 0;
  ulong len = image_get_data_size(pUh);
  ulong data = image_get_data(pUh);

  if (0x00 ==
      (ret = k230_boot_decomp_to_load_addr(pUh, 0x6000000, data, &len))) {
    icache_disable();
    dcache_disable();

    asm volatile(".long 0x0170000b\n" ::: "memory");
    uboot = (void (*)(ulong, void *))(ulong)image_get_load(pUh);
    uboot(0, 0);
  }

  return ret;
}

__weak ulong get_blk_start_by_boot_firmre_type(en_boot_sys_t sys) {
  ulong blk_s = IMG_PART_NOT_EXIT;

  switch (sys) {
  case BOOT_SYS_RTT:
    blk_s = RTT_SYS_IN_IMG_OFF_SEC;
    break;
  case BOOT_SYS_UBOOT:
    blk_s = UBOOT_SYS_IN_IMG_OFF_SEC;
    break;
  default:
    break;
  }

  return blk_s;
}

static int k230_load_sys_from_mmc_or_sd(en_boot_sys_t sys,
                                        ulong buff) //(ulong offset ,ulong buff)
{
  static struct blk_desc *pblk_desc = NULL;
  ulong blk_s = get_blk_start_by_boot_firmre_type(sys);
  struct mmc *mmc = NULL;
  int ret = 0;
  firmware_head_s *pfh = (firmware_head_s *)buff;
  ulong data_sect = 0;

  if (IMG_PART_NOT_EXIT == blk_s)
    return IMG_PART_NOT_EXIT;

  if (NULL == pblk_desc) {
    if (mmc_init_device(g_boot_medium - BOOT_MEDIUM_SDIO0)) {
      return 1;
    }

    mmc = find_mmc_device(g_boot_medium - BOOT_MEDIUM_SDIO0);

    if (NULL == mmc) {
      return 2;
    }

    if (mmc_init(mmc)) {
      return 3;
    }

    pblk_desc = mmc_get_blk_desc(mmc);
    if (NULL == pblk_desc) {
      return 3;
    }
  }

  ret = blk_dread(pblk_desc, blk_s, HD_BLK_NUM, (char *)buff);
  if (ret != HD_BLK_NUM) {
    return 4;
  }

  if (pfh->magic != K230_IMAGE_MAGIC_NUM) {
    debug("pfh->magic 0x%x != 0x%x blk=0x%lx buff=0x%lx  ", pfh->magic,
          K230_IMAGE_MAGIC_NUM, blk_s, buff);
    return 5;
  }

  data_sect = DIV_ROUND_UP(pfh->length + sizeof(*pfh), BLKSZ) - HD_BLK_NUM;

  ret = blk_dread(pblk_desc, blk_s + HD_BLK_NUM, data_sect,
                  (char *)buff + HD_BLK_NUM * BLKSZ);

  if (ret != data_sect) {
    return 6;
  }

  return 0;
}

static int k230_img_load_sys_from_dev(en_boot_sys_t sys, ulong buff) {
  int ret = 0;

  if ((BOOT_MEDIUM_SDIO0 == g_boot_medium) ||
      (BOOT_MEDIUM_SDIO1 == g_boot_medium)) {
    ret = k230_load_sys_from_mmc_or_sd(sys, buff);
  } else {
    ret = -1;
    printf("Error, Unsupport media type %d\n", sys);
  }

  return ret;
}

static int k230_img_load_boot_sys_auot_boot(en_boot_sys_t sys) {
  int ret = 0;

  if (0x00 != (ret = k230_img_load_boot_sys(BOOT_SYS_RTT))) {
    printf("Error, Autoboot RT-Smart failed. %d\n", ret);
  }

  return ret;
}

int k230_img_boot_sys_bin(firmware_head_s *fhBUff) {
  int ret = 0;

  image_header_t *pUh = NULL;
  const char *image_name = NULL;
  ulong plain_addr = 0;

  ret = k230_boot_check_and_get_plain_data((firmware_head_s *)fhBUff,
                                           &plain_addr);
  if (ret) {
    printf("decrypt image failed.");
    return ret;
  }

  pUh = (image_header_t *)(plain_addr + 4);
  if (!image_check_magic(pUh)) {
    printf("bad magic \n");
    return -3;
  }

  image_name = image_get_name(pUh);

  if (0 == strcmp(image_name, "rtt")) {
    ret = k230_boot_rtt_uimage(pUh);
  } else if (0 == strcmp(image_name, "uboot")) {
    ret = k230_boot_uboot_uimage(pUh);
  } else {
    printf("Error, Unsupport image type %s\n", image_name);
    return -4;
  }

  return ret;
}

int k230_img_load_boot_sys(en_boot_sys_t sys) {
  int ret = 0;

  ulong img_load_addr = k230_get_encrypted_image_load_addr();

  if (sys == BOOT_SYS_AUTO) {
    ret = k230_img_load_boot_sys_auot_boot(sys);
  } else {
    if (0x00 == (ret = k230_img_load_sys_from_dev(sys, img_load_addr))) {
      if (0x00 !=
          (ret = k230_img_boot_sys_bin((firmware_head_s *)img_load_addr))) {
        printf("Error, boot image failed.%d\n", ret);
      }
    } else {
      printf("Error, load image failed.%d\n", ret);
    }
  }

  return ret;
}

static int k230_boot_reset_big_hard_and_run(ulong core_run_addr) {
  printf("Jump to big hart\n");

  writel(core_run_addr,   (void *)0x91102104ULL);
  writel(0x10001000,      (void *)0x9110100cULL);
  writel(0x10001,         (void *)0x9110100cULL);
  writel(0x10000,         (void *)0x9110100cULL);

  return 0;
}

#ifdef CONFIG_K230_PUFS
static int k230_boot_check_and_get_plain_data_securiy(firmware_head_s *pfh,
                                                      ulong *pplain_addr) {
  int ret = 0;
  pufs_dgst_st md;
  unsigned int outlen;
  uint8_t puk_hash_otp[32];
  uint8_t temp32_0[32] = {0};

  ulong plain_data_addr = k230_boot_get_encrypted_image_decrypt_addr();
  char *pplaint = (char *)plain_data_addr;

  const char *gcm_iv = "\x9f\xf1\x85\x63\xb9\x78\xec\x28\x1b\x3f\x27\x94";
  const char *sm4_iv =
      "\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f";
  const char *gcm_key =
      "\x24\x50\x1a\xd3\x84\xe4\x73\x96\x3d\x47\x6e\xdc\xfe\x08\x20\x52\x37\xac"
      "\xfd\x49\xb5\xb8\xf3\x38\x57\xf8\x11\x4e\x86\x3f\xec\x7f";

  pufs_ec_point_st puk;
  pufs_ecdsa_sig_st sig;

  if (pfh->crypto_type == INTERNATIONAL_SECURITY) {
    K230_dbg(" INTERNATIONAL_SECURITY aes \n");
    // 检验头携带的RSA2048/SM2的 public key是否正确，与烧录到OTP里面的PUK
    // HASH值做比对。 直接把public
    // key烧录到OTP也可以，但是需要消耗2Kbit的OTP空间，这里主要是节约otp空间考虑。
    // 把PUK HASH烧录到OTP，可以保证只有经过PRK签名的固件才可以正常启动。
    if (SUCCESS !=
        cb_pufs_read_otp(puk_hash_otp, 32, OTP_BLOCK_RSA_PUK_HASH_ADDR)) {
      printf("otp read puk hash error \n");
      return -6;
    }
    if (0 == memcmp(temp32_0, puk_hash_otp, 32)) {
      printf("you have not input otp key,not support security boot\n");
      return -18;
    }
    if (cb_pufs_hash(&md, (const uint8_t *)&pfh->verify, 256 + 4, SHA_256) !=
        SUCCESS) {
      printf("hash calc error \n");
      return -7;
    }
    //验证公钥是否正确
    if (memcmp(md.dgst, puk_hash_otp, 32)) {
      printf("pubk hash error \n");
      return -8;
    }

    //使用公钥验证mac签名
    char *gcm_tag = (char *)(pfh + 1) + pfh->length - 16;
    ret = pufs_rsa_p1v15_verify(pfh->verify.rsa.signature, RSA2048,
                                pfh->verify.rsa.n, pfh->verify.rsa.e, gcm_tag,
                                16);
    if (ret) {
      printf("rsa verify error \n");
      return -9;
    }

    // gcm解密,同时保证完整性；获取明文
    ret = pufs_dec_gcm((uint8_t *)pplaint, &outlen, (const uint8_t *)(pfh + 1),
                       pfh->length - 16, AES, OTPKEY, OTPKEY_2, 256,
                       (const uint8_t *)gcm_iv, 12, NULL, 0, gcm_tag, 16);
    if (ret) {
      printf("dec gcm error ret=%x \n", ret);
      return -10;
    }

    if (pplain_addr) {
      *pplain_addr = (ulong)pplaint;
    }
  } else if (pfh->crypto_type == CHINESE_SECURITY) {
    K230_dbg("CHINESE_SECURITY sm\n");
    if (SUCCESS !=
        cb_pufs_read_otp(puk_hash_otp, 32, OTP_BLOCK_SM2_PUK_HASH_ADDR)) {
      printf("otp read puk hash error \n");
      return -6;
    }
    if (0 == memcmp(temp32_0, puk_hash_otp, 32)) {
      printf("you have not input otp key,not support security boot\n");
      return -18;
    }
    if (cb_pufs_hash(&md, (const uint8_t *)&pfh->verify, 512 + 4 - 32 - 32,
                     SM3) != SUCCESS) {
      printf("hash calc error \n");
      return -7;
    }
    //验证公钥是否正确
    if (memcmp(md.dgst, puk_hash_otp, 32)) {
      printf("pubk hash error \n");
      return -8;
    }

    // SM2 解密hash验签
    puk.qlen = sig.qlen = 32;
    memcpy(puk.x, pfh->verify.sm2.pukx, puk.qlen);
    memcpy(puk.y, pfh->verify.sm2.puky, puk.qlen);
    memcpy(sig.r, pfh->verify.sm2.r, sig.qlen);
    memcpy(sig.s, pfh->verify.sm2.s, sig.qlen);
    if (cb_pufs_sm2_verify(sig, (const uint8_t *)(pfh + 1), pfh->length,
                           pfh->verify.sm2.id, pfh->verify.sm2.idlen,
                           puk) != SUCCESS) {
      printf("sm verify error\n");
      return -11;
    }

    // SM4 CBC 解密
    if (cb_pufs_dec_cbc((uint8_t *)pplaint, &outlen, (const uint8_t *)(pfh + 1),
                        pfh->length, SM4, OTPKEY, OTPKEY_4, 128,
                        (const uint8_t *)sm4_iv, 0) != SUCCESS) {
      printf("dec cbc  error\n");
      return -12;
    }
    if (pplain_addr)
      *pplain_addr = (ulong)pplaint;
  } else if (pfh->crypto_type == GCM_ONLY) {
    K230_dbg(" POC GCM_ONLY \n");
    char *gcm_tag = (char *)(pfh + 1) + pfh->length - 16;

    // gcm解密,同时保证完整性；获取明文
    ret = pufs_dec_gcm_poc(
        (uint8_t *)pplaint, &outlen, (const uint8_t *)(pfh + 1),
        pfh->length - 16, AES, SSKEY, (const uint8_t *)gcm_key, 256,
        (const uint8_t *)gcm_iv, 12, NULL, 0, (uint8_t *)gcm_tag, 16);
    if (ret) {
      printf("dec gcm error ret=%x \n", ret);
      return -10;
    }

    if (pplain_addr) {
      *pplain_addr = (ulong)pplaint;
    }
  } else {
    return -10;
  }

  return 0;
}
#endif

static int k230_boot_check_and_get_plain_data(firmware_head_s *pfh,
                                              ulong *pplain_addr) {
  int ret = 0;
#ifdef CONFIG_K230_PUFS
  uint32_t otp_msc = 0;
  pufs_dgst_st md;
#else
  uint8_t sha256[SHA256_SUM_LEN];
#endif

  if (pfh->magic != K230_IMAGE_MAGIC_NUM) {
    printf("magic error %x : %x \n", K230_IMAGE_MAGIC_NUM, pfh->magic);
    return CMD_RET_FAILURE;
  }

  if (pfh->crypto_type == NONE_SECURITY) {
#ifdef CONFIG_K230_PUFS
    if (SUCCESS != cb_pufs_read_otp((uint8_t *)&otp_msc,
                                    OTP_BLOCK_PRODUCT_MISC_BYTES,
                                    OTP_BLOCK_PRODUCT_MISC_ADDR)) {
      printf("otp read error \n");
      return -4;
    }

    if (otp_msc & 0x1) {
      printf(" NONE_SECURITY not support  %x \n", pfh->crypto_type);
      return -5;
    }

    cb_pufs_hash(&md, (const uint8_t *)(pfh + 1), pfh->length, SHA_256);
    if (memcmp(md.dgst, pfh->verify.none_sec.signature, SHA256_SUM_LEN)) {
#else
    sha256_csum_wd((const uint8_t *)(pfh + 1), pfh->length, sha256,
                   CHUNKSZ_SHA256);
    if (memcmp(sha256, pfh->verify.none_sec.signature, SHA256_SUM_LEN)) {
#endif
      printf("sha256 error");
      return -3;
    }

    if (pplain_addr) {
      *pplain_addr = (ulong)pfh + sizeof(*pfh);
    }
    ret = 0;
  } else if ((pfh->crypto_type == CHINESE_SECURITY) ||
             (pfh->crypto_type == INTERNATIONAL_SECURITY) ||
             (pfh->crypto_type == GCM_ONLY)) {
#ifdef CONFIG_K230_PUFS
    ret = k230_boot_check_and_get_plain_data_securiy(pfh, pplain_addr);
#else
    printf("error, not enable k230 puf driver\n");
#endif
  } else {
    printf("error crypto type =%x\n", pfh->crypto_type);
    return -9;
  }

  return ret;
}
