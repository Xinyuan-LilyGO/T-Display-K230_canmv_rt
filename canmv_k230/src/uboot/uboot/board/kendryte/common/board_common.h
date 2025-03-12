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
#ifndef __K230_BOARD_H__
#define __K230_BOARD_H__

// Attempt to include the header file
#ifdef __has_include
  #if __has_include("sdk_autoconf.h")
    #include "sdk_autoconf.h"
    #define MYHEADER_EXISTS
  #endif
#endif

// Fallback definition or other code
#ifndef MYHEADER_EXISTS
  // Provide alternative definitions or inform about missing header
  #warning "sdk_autoconf.h not found. Default behavior will be used."
#endif

typedef enum
{
    BOOT_MEDIUM_NORFLASH        = 0,
    BOOT_MEDIUM_NANDFLASH       = 1,
    BOOT_MEDIUM_SDIO0           = 2,
    BOOT_MEDIUM_SDIO1           = 3,
    BOOT_MEDIUM_MAX,
} boot_medium_e;

typedef enum {
  NONE_SECURITY = 0,
  GCM_ONLY,
  CHINESE_SECURITY,
  INTERNATIONAL_SECURITY
} crypto_type_e;

typedef struct __firmware_head_st {
  uint32_t magic;  // 方便升级时快速判断固件是否有效。
  uint32_t length; // 从存储介质读到SRAM的数据量
  crypto_type_e
      crypto_type; // 支持国密或国际加密算法，或支持不加密启动(otp可以控制是否支持)。
  // 设想这样一个场景，如果固件只使用对称加密，在工厂批量生产的时候，解密密钥必然会泄露给工厂。如果使用非对称加密就可以这种问题了，只需要把公钥交给工厂。
  union verify_ {
    struct rsa_ {
      uint8_t n
          [256]; // 非对称加密的验签，防止固件被篡改。同时其HASH值会被烧录到otp。
      uint32_t e;
      uint8_t signature[256];
    } rsa;
    struct sm2_ {
      uint32_t idlen;
      uint8_t id[512 - 32 * 4];
      uint8_t pukx[32];
      uint8_t puky[32];
      uint8_t r[32];
      uint8_t s[32];
    } sm2;
    struct none_sec_ {
      uint8_t signature
          [32]; // 计算HASH保证启动固件的完整性。避免程序异常难以定位原因。
      uint8_t reserved[516 - 32];
    } none_sec;
  } verify;
} __attribute__((packed, aligned(4))) firmware_head_s; //总的512+16 bytes

typedef enum _en___boot_type {
  BOOT_SYS_LINUX,
  BOOT_SYS_RTT,
  BOOT_QUICK_BOOT_CFG,
  BOOT_FACE_DB,
  BOOT_SENSOR_CFG,
  BOOT_AI_MODE,
  BOOT_SPECKLE,
  BOOT_RTAPP,
  BOOT_SYS_UBOOT,
  BOOT_SYS_ADDR,
  BOOT_SYS_AUTO
} en_boot_sys_t;

#define K230_IMAGE_MAGIC_NUM 0x3033324B // "K230"

#define BLKSZ 512
#define HD_BLK_NUM DIV_ROUND_UP(sizeof(firmware_head_s), BLKSZ)

#define UBOOT_SYS_IN_IMG_OFF_SEC (2 * 1024 * 1024 / BLKSZ)
#define RTT_SYS_IN_IMG_OFF_SEC (10 * 1024 * 1024 / BLKSZ)
#define LINUX_SYS_IN_IMG_OFF_SEC (30 * 1024 * 1024 / BLKSZ)

#define UBOOT_SYS_IN_SPI_NOR_OFF 0x80000
#define RTT_SYS_IN_SPI_NOR_OFF CONFIG_SPI_NOR_RTTK_BASE
#define LINUX_SYS_IN_SPI_NOR_OFF CONFIG_MEM_LINUX_SYS_BASE

#define UBOOT_SYS_IN_SPI_NAND_OFF 0x80000
#define LINUX_SYS_IN_SPI_NAND_OFF 0x00a00000
#define RTT_SYS_IN_SPI_NAND_OFF 0x00200000

#define IMG_PART_NOT_EXIT 0XFFFFFFFF

extern int g_boot_medium;

unsigned long k230_get_encrypted_image_load_addr(void);
unsigned long k230_get_encrypted_image_decrypt_addr(void);

int k230_img_boot_sys_bin(firmware_head_s *fhBUff);
int k230_img_load_boot_sys(en_boot_sys_t sys);

void board_ddr_init(void);
int ddr_init_training(void);

int kd_board_init(void);

#ifdef CONFIG_SPL_BUILD
void spl_device_disable(void);
int spl_load_image_type(void);
#endif

#endif
