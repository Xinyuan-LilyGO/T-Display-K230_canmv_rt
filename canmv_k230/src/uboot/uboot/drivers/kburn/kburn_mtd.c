#include <common.h>
#include <div64.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <errno.h>
#include <log.h>
#include <malloc.h>

#include <mtd.h>

#include <kburn.h>

struct kburn_mtd_priv {
    struct mtd_info *mtd;
    int dev_num;
};

static int mtd_get_medium_info(struct kburn *burn)
{
    struct kburn_mtd_priv *priv = (struct kburn_mtd_priv *)(burn->dev_priv);
    struct mtd_info *mtd = priv->mtd;

    burn->medium_info.valid = 1;
    burn->medium_info.wp = 0;

    burn->medium_info.capacity = mtd->size;
    burn->medium_info.erase_size = mtd->erasesize;
    burn->medium_info.blk_size = mtd->writesize;
    burn->medium_info.timeout_ms = 5000;
    burn->medium_info.type = KBURN_MEDIA_SPI_NAND;

    pr_info("Device %s %d capacity %lld, erase size %lld, blk_sz %lld\n", \
        mtd->name, priv->dev_num, burn->medium_info.capacity, burn->medium_info.erase_size, burn->medium_info.blk_size);

    return 0;
}

static bool mtd_is_aligned_with_min_io_size(struct mtd_info *mtd, u64 size)
{
	return !do_div(size, mtd->writesize);
}

static bool mtd_is_aligned_with_block_size(struct mtd_info *mtd, u64 size)
{
	return !do_div(size, mtd->erasesize);
}

/* Logic taken from cmd/mtd.c:mtd_oob_write_is_empty() */
static bool mtd_page_is_empty(struct mtd_oob_ops *op)
{
	int i;

	for (i = 0; i < op->len; i++)
		if (op->datbuf[i] != 0xff)
			return false;

	/* oob is not used, with MTD_OPS_AUTO_OOB & ooblen=0 */

	return true;
}

static int mtd_read_medium(struct kburn *burn, u64 offset, void *buf, u64 *len)
{
    pr_err("TODO\n");
    return -1;
}

static int mtd_erase_medium(struct kburn *burn, u64 offset, u64 *len)
{
    struct kburn_mtd_priv *priv = (struct kburn_mtd_priv *)(burn->dev_priv);
    struct mtd_info *mtd = priv->mtd;

	struct erase_info erase_op = {};
	u64 off, size;
	int ret = 0;

    off = offset;
    size = *len;

	if (!mtd_is_aligned_with_block_size(mtd, off)) {
		pr_err("Offset not aligned with a block (0x%x)\n",
		       mtd->erasesize);
        return -1;
	}

	if (!mtd_is_aligned_with_block_size(mtd, size)) {
		pr_err("Size not a multiple of a block (0x%x)\n",
		       mtd->erasesize);
        return -1;
	}

	pr_info("Erasing 0x%08llx ... 0x%08llx (%d eraseblock(s))\n",
	       off, off + size - 1, mtd_div_by_eb(size, mtd));

	erase_op.mtd = mtd;
	erase_op.addr = off;
	erase_op.len = mtd->erasesize;
	erase_op.scrub = false;

    while (size) {
		ret = mtd_erase(mtd, &erase_op);

		if (ret) {
			/* Abort if its not a bad block error */
			if (ret != -EIO)
				break;
			pr_info("Skipping bad block at 0x%08llx\n",
			       erase_op.addr);
		}

		size -= mtd->erasesize;
		erase_op.addr += mtd->erasesize;
	}

	if (ret && ret != -EIO) {
        pr_err("Erase mtd failed. %d\n", ret);
        return -1;
    }

    return 0;
}

static int mtd_write_medium(struct kburn *burn, u64 offset, const void *buf, u64 *len)
{
    struct kburn_mtd_priv *priv = (struct kburn_mtd_priv *)(burn->dev_priv);
    struct mtd_info *mtd = priv->mtd;

	struct mtd_oob_ops io_op = {};

    int ret = -1;
	u64 off = offset, size= *len, remaining = *len;
    u64 lock_ofs = offset, lock_len = *len;

	bool has_pages = mtd->type == MTD_NANDFLASH ||
			 mtd->type == MTD_MLCNANDFLASH;

    if (!mtd_is_aligned_with_min_io_size(mtd, off)) {
		pr_err("Offset not aligned with a page (0x%x)\n",
		       mtd->writesize);
        return -1;
	}

	if (!mtd_is_aligned_with_min_io_size(mtd, size)) {
		pr_err("Size not on a page boundary (0x%x), rounding to 0x%llx\n",
		       mtd->writesize, size);
        return -1;
    }

    pr_debug("Unlocking the mtd device\n");
    ret = mtd_unlock(mtd, lock_ofs, lock_len);
    if (ret && ret != -EOPNOTSUPP) {
        printf("MTD device unlock failed\n");
        return -1;
    }

    if(0x00 != mtd_erase_medium(burn, offset, &size)) {
        printf("MTD device erase failed\n");
		mtd_lock(mtd, lock_ofs, lock_len);
        return -1;
    }

    io_op.mode = MTD_OPS_AUTO_OOB;
	io_op.len = size;
	if (has_pages && io_op.len > mtd->writesize)
		io_op.len = mtd->writesize;
	io_op.ooblen = 0;
	io_op.datbuf = buf;
	io_op.oobbuf = NULL;

	/* Search for the first good block after the given offset */
	while (mtd_block_isbad(mtd, off))
		off += mtd->erasesize;

	/* Loop over the pages to do the actual read/write */
	while (remaining) {
		/* Skip the block if it is bad */
		if (mtd_is_aligned_with_block_size(mtd, off) &&
		    mtd_block_isbad(mtd, off)) {
			off += mtd->erasesize;
			continue;
		}

        if(has_pages && mtd_page_is_empty(&io_op)) {
			ret = 0;
			io_op.retlen = mtd->writesize;
			io_op.oobretlen = mtd->oobsize;
        } else {
            ret = mtd_write_oob(mtd, off, &io_op);
        }

		if (ret) {
			pr_err("Failure while writing at offset 0x%llx\n", off);
			break;
		}

		off += io_op.retlen;
		remaining -= io_op.retlen;
		io_op.datbuf += io_op.retlen;
		io_op.len = remaining;
		if (has_pages && io_op.len > mtd->writesize)
			io_op.len = mtd->writesize;
	}

    mtd_lock(mtd, lock_ofs, lock_len);

	if (ret) {
		printf("mtd write on %s failed with error %d\n", mtd->name, ret);
        return -1;
	}

    return 0;
}

static int mtd_destory(struct kburn *burn)
{
    return 0;
}

struct kburn *kburn_mtd_probe(void)
{
    struct udevice *ud_mtd, *ud_mtd_parent;

    struct kburn *burner;
    struct kburn_mtd_priv *priv;

    if(0x00 != uclass_first_device_err(UCLASS_MTD, &ud_mtd)) {
        pr_err("can not found mtd device");
        return NULL;
    }
    ud_mtd_parent = dev_get_parent(ud_mtd);

    burner = memalign(CONFIG_SYS_CACHELINE_SIZE, sizeof(*burner) + sizeof(*priv));
    if(NULL == burner) {
        pr_err("memaligin failed\n");
        return NULL;
    }
    memset(burner, 0, sizeof(*burner));

    priv = (struct kburn_mtd_priv *)((char *)burner + sizeof(*burner));
    priv->mtd = dev_get_uclass_priv(ud_mtd);
    priv->dev_num = dev_seq(ud_mtd_parent);

    burner->type = KBURN_MEDIA_SPI_NAND;
    burner->dev_priv = (void *)priv;

	burner->get_medium_info = mtd_get_medium_info;
	burner->read_medium = mtd_read_medium;
	burner->write_medium = mtd_write_medium;
    burner->erase_medium = mtd_erase_medium;
    burner->destory = mtd_destory;

    return burner;
}
