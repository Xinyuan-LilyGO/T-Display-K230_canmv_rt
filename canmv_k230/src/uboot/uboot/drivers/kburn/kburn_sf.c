#include <common.h>
#include <div64.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>

#include <spi_flash.h>

#include <kburn.h>

struct kburn_sf_priv {
    struct spi_flash *flash;
    int dev_num;
};

static int sf_get_medium_info(struct kburn *burn)
{
    struct kburn_sf_priv *priv = (struct kburn_sf_priv *)(burn->dev_priv);
    struct spi_flash *flash = priv->flash;

    burn->medium_info.valid = 1;
    burn->medium_info.wp = 0;

    burn->medium_info.capacity = flash->size;
    burn->medium_info.erase_size = flash->erase_size;
    burn->medium_info.blk_size = flash->page_size;
    burn->medium_info.timeout_ms = 5000;
    burn->medium_info.type = KBURN_MEDIA_SPI_NOR;

    pr_info("Device SPI-FLASH %d capacity %lld, erase size %lld, blk_sz %lld\n", \
        priv->dev_num, burn->medium_info.capacity, burn->medium_info.erase_size, burn->medium_info.blk_size);

    return 0;
}

static int sf_read_medium(struct kburn *burn, u64 offset, void *buf, u64 *len)
{
    pr_err("TODO");
    return -1;
}

static bool sf_page_is_empty(const void *buf, u64 size)
{
    if(((0x00 == (((u64)buf) & 0x07))) && (0x00 == (size % 8))) {
        const u64 *pdata = (u64*)buf;
        const u64 *pend = pdata + (size / 8);

        do {
            if(0xFFFFFFFFFFFFFFFFULL != pdata[0]) {
                return false;
            }
        } while(pdata++ < pend);
    } else {
        const u8 *pdata = (u8*)buf;
        const u8 *pend = pdata + size;

        do {
            if(0xFF != pdata[0]) {
                return false;
            }
        } while(pdata++ < pend);
    }

    return true;
}

static bool sf_is_aligned_with_min_io_size(struct spi_flash *flash, u64 size)
{
	return !do_div(size, flash->erase_size);
}

static bool sf_is_aligned_with_block_size(struct spi_flash *flash, u64 size)
{
	return !do_div(size, flash->page_size);
}

static int sf_write_medium(struct kburn *burn, u64 offset, const void *buf, u64 *len)
{
    struct kburn_sf_priv *priv = (struct kburn_sf_priv *)(burn->dev_priv);
    struct spi_flash *flash = priv->flash;

    int rc;
    const void *buffer = buf;
    u64 off = offset;
    u64 size = *len;
    u64 remaining, send_len;

    pr_debug("sf write offset %lld, size %lld\n", offset, size);

	if (!sf_is_aligned_with_min_io_size(flash, off)) {
		pr_err("Offset not aligned with a block (0x%x)\n",
		       flash->erase_size);
        return -1;
	}

	if (!sf_is_aligned_with_block_size(flash, size)) {
		pr_err("Size not a multiple of a block (0x%x)\n",
		       flash->erase_size);
        return -1;
	}

    if(0x00 != (rc = spi_flash_erase(flash, (u32)offset, size))) {
        pr_err("sf write, erase %lld failed, %d\n", offset, rc);
        return -1;
    }

    pr_info("Erasing 0x%08llx ... 0x%08llx\n",
	       off, off + size - 1);

    remaining = size;
    send_len = remaining > 4096 ? 4096 : remaining;

    while(remaining) {
        if(sf_page_is_empty(buffer, send_len)) {
            off += send_len;
            buffer += send_len;
            remaining -= send_len;
            send_len = remaining > 4096 ? 4096 : remaining;
            continue;
        }

        if(0x00 != (rc = spi_flash_write(flash, (u32)off, (size_t)send_len, buffer))) {
            pr_err("sf wrtie failed %llx, err %d\n", off, rc);
            return -1;
        }

        off += send_len;
        buffer += send_len;
        remaining -= send_len;
        send_len = remaining > 4096 ? 4096 : remaining;
    }

    if(0x00 != rc) {
        return -1;
    }

    return 0;
}

static int sf_erase_medium(struct kburn *burn, u64 offset, u64 *len)
{
    struct kburn_sf_priv *priv = (struct kburn_sf_priv *)(burn->dev_priv);
    struct spi_flash *flash = priv->flash;

    pr_info("erase medium start, offset %lld, size %lld\n", offset, *len);

    int rc = spi_flash_erase(flash, (u32)offset, (size_t)*len);

    pr_info("erase medium done, result %d\n", rc);

    return rc;
}

static int sf_destory(struct kburn *burn)
{
    return 0;
}

struct kburn *kburn_sf_probe(void)
{
    struct udevice *ud_sf, *ud_sf_parent;

    struct kburn *burner;
    struct kburn_sf_priv *priv;

    if(0x00 != uclass_first_device_err(UCLASS_SPI_FLASH, &ud_sf)) {
        pr_err("can not found spi flash device");
        return NULL;
    }
    ud_sf_parent = dev_get_parent(ud_sf);

    // we find a sf device, and init it.
    burner = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(*burner) + sizeof(*priv));
    if(NULL == burner) {
        pr_err("memaligin failed\n");
        return NULL;
    }
    memset(burner, 0, sizeof(*burner));

    priv = (struct kburn_sf_priv *)((char *)burner + sizeof(*burner));
    priv->flash = dev_get_uclass_priv(ud_sf);
    priv->dev_num = dev_seq(ud_sf_parent);

    burner->type = KBURN_MEDIA_SPI_NOR;
    burner->dev_priv = (void *)priv;

	burner->get_medium_info = sf_get_medium_info;
	burner->read_medium = sf_read_medium;
	burner->write_medium = sf_write_medium;
    burner->erase_medium = sf_erase_medium;
    burner->destory = sf_destory;

    return burner;
}
