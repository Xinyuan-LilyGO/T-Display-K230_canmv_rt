#include <common.h>
#include <log.h>
#include <malloc.h>

#include "kburn.h"

struct kburn * kburn_probe_media(enum KBURN_MEDIA_TYPE type)
{
    struct kburn * kburn = NULL;

#if defined (CONFIG_KBURN_MMC)
    if(KBURN_MEDIA_eMMC == type) {
        kburn = kburn_mmc_probe(0); // K230 eMMC is on mmc0
    } else if (KBURN_MEDIA_SDCARD == type) {
        kburn = kburn_mmc_probe(1); // K230 SD Card is on mmc1
    } else
#endif // CONFIG_KBURN_MMC
#if defined (CONFIG_KBURN_SF)
    if(KBURN_MEDIA_SPI_NOR == type) {
        kburn = kburn_sf_probe();
    } else
#endif // CONFIG_KBURN_SF
#if defined (CONFIG_KBURN_MTD)
    if(KBURN_MEDIA_SPI_NAND == type) {
        kburn = kburn_mtd_probe();
    } else
#endif // CONFIG_KBURN_MTD
    {
        printf("kburn probe not support type %x\n", type);
    }

    if (NULL == kburn) {
        printf("kburn probe failed, type %x\n", type);
    }

    return kburn;
}

int kburn_get_medium_info(struct kburn *burn)
{
    if((NULL == burn) || (NULL == burn->get_medium_info)) {
        pr_err("invalid arg\n");
        return -1;
    }
    return burn->get_medium_info(burn);
}

int kburn_read_medium(struct kburn *burn, u64 offset, void *buf, u64 *len)
{
    if((NULL == burn) || (NULL == burn->read_medium)) {
        pr_err("invalid arg\n");
        return -1;
    }
    return burn->read_medium(burn, offset, buf, len);
}

int kburn_write_medium(struct kburn *burn, u64 offset, const void *buf, u64 *len)
{
    if((NULL == burn) || (NULL == burn->write_medium)) {
        pr_err("invalid arg\n");
        return -1;
    }
    return burn->write_medium(burn, offset, buf, len);
}

int kburn_erase_medium(struct kburn *burn, u64 offset, u64 *len)
{
    if((NULL == burn) || (NULL == burn->erase_medium)) {
        pr_err("invalid arg\n");
        return -1;
    }
    return burn->erase_medium(burn, offset, len);
}

void kburn_destory(struct kburn *burn)
{
    if((NULL == burn) || (NULL == burn->destory)) {
        pr_err("invalid arg\n");
        return;
    }

    if(0x00 != burn->destory(burn)) {
        pr_err("destory kburn failed.\n");
    }

    free(burn);
}
