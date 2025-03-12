#ifndef __KD_BURNER_H__
#define __KD_BURNER_H__

#include <common.h>

#if defined(CONFIG_KBURN_OVER_USB) && (0x00 != CONFIG_KBURN_OVER_USB)

#define KBURN_USB_INTF_CLASS        (0xFF)
#define KBURN_USB_INTF_SUBCLASS     (0x02)
#define KBURN_USB_INTF_PROTOCOL     (0x00)

#define KBURN_USB_EP_BUFFER_SZIE    (128 * 1024)
#define KBURN_USB_BUFFER_SIZE       (KBURN_USB_EP_BUFFER_SZIE * 2)

#endif

enum KBURN_MEDIA_TYPE {
    KBURN_MEDIA_NONE = 0x00,
    KBURN_MEDIA_eMMC = 0x01,
    KBURN_MEDIA_SDCARD = 0x02, // SD Card is eMMC too.
    KBURN_MEDIA_SPI_NAND = 0x03,
    KBURN_MEDIA_SPI_NOR = 0x04,
    KBURN_MEDIA_OTP = 0x05,
};

struct kburn_medium_info {
    u64 capacity;
    u64 blk_size;
    u64 erase_size;
    u64 timeout_ms:32;
    u64 wp:8;
    u64 type:7;
    u64 valid:1;
};

struct kburn {
    enum KBURN_MEDIA_TYPE type;
    struct kburn_medium_info medium_info;
    void *dev_priv;

	int (*get_medium_info)(struct kburn *burn);

	int (*read_medium)(struct kburn *burn,
			u64 offset, void *buf, u64 *len);

	int (*write_medium)(struct kburn *burn,
			u64 offset, const void *buf, u64 *len);

    int (*erase_medium)(struct kburn *burn, u64 offset, u64 *len);

    int (*destory)(struct kburn *burn);
};

struct kburn * kburn_probe_media(enum KBURN_MEDIA_TYPE type);

int kburn_get_medium_info(struct kburn *burn);

int kburn_read_medium(struct kburn *burn, u64 offset, void *buf, u64 *len);

int kburn_write_medium(struct kburn *burn, u64 offset, const void *buf, u64 *len);

int kburn_erase_medium(struct kburn *burn, u64 offset, u64 *len);

void kburn_destory(struct kburn *burn);

#if defined (CONFIG_KBURN_MMC)
struct kburn *kburn_mmc_probe(uint8_t index);
#endif // CONFIG_KBURN_MMC

#if defined (CONFIG_KBURN_SF)
struct kburn *kburn_sf_probe(void);
#endif // CONFIG_KBURN_SF

#if defined (CONFIG_KBURN_MTD)
struct kburn *kburn_mtd_probe(void);
#endif // CONFIG_KBURN_MTD

#endif // __KD_BURNER_H__
