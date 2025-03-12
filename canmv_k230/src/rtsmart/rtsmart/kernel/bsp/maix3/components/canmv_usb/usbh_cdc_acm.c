#include "canmv_usb.h"

#include "rtdef.h"

#include "cache.h"

#include <dfs.h>
#include <dfs_fs.h>
#include <dfs_file.h>

#include "rtthread.h"
#include "usb_util.h"
#include "usbh_cdc_acm.h"
#include <stdint.h>
#include <stdio.h>

#define DBG_SECTION_NAME    "USBH_CDC_ACM"
#define DBG_LEVEL           DBG_LOG
#include <rtdbg.h>

struct usbh_cdc_acm_dev_wrap {
    struct rt_device dev;
    struct rt_event evt;
};

static struct usbh_cdc_acm_dev_wrap usbh_cdc_acm_dev[CONFIG_USBHOST_MAX_CDC_ACM_CLASS];

static int usbh_cdc_acm_dev_open(struct dfs_fd *file) {
    return 0;
}

static int usbh_cdc_acm_dev_close(struct dfs_fd *file) {
    return 0;
}

void usbh_cdc_acm_callback(void *arg, int nbytes)
{
    uint32_t set = 0x80000000 | nbytes;

    struct usbh_cdc_acm_dev_wrap *dev_wrap = (struct usbh_cdc_acm_dev_wrap *)arg;

    rt_event_send(&dev_wrap->evt, set);
}

static int usbh_cdc_acm_dev_read(struct dfs_fd *file, void *buf, size_t count)
{
    int ret;
    uint32_t evt_set, recved;

    rt_device_t dev_id;
    struct usbh_cdc_acm_dev_wrap *dev_wrap;

    struct usbh_cdc_acm *cdc_acm_class;

    RT_ASSERT(file != RT_NULL);

    /* get device handler */
    dev_id = (rt_device_t)file->fnode->data;
    dev_wrap = (struct usbh_cdc_acm_dev_wrap *)dev_id;
    RT_ASSERT(dev_id != RT_NULL);

    cdc_acm_class = (struct usbh_cdc_acm *)dev_id->user_data;
    RT_ASSERT(cdc_acm_class != RT_NULL);

    usbh_bulk_urb_fill(&cdc_acm_class->bulkin_urb, cdc_acm_class->hport, cdc_acm_class->bulkin, buf, cdc_acm_class->bulkin->wMaxPacketSize, 3000, usbh_cdc_acm_callback, dev_id);
    ret = usbh_submit_urb(&cdc_acm_class->bulkin_urb);
    if (ret < 0) {
        LOG_E("bulk in error,ret:%d\r\n", ret);
        return ret;
    } else {
        if(RT_EOK != rt_event_recv(&dev_wrap->evt, 0x80000000, RT_EVENT_FLAG_AND, rt_tick_from_millisecond(200), &evt_set)) {
            LOG_W("wait usb read done timeout\n");
            return -1;
        }

        recved = evt_set & 0x7FFFFFFF;
        rt_hw_cpu_dcache_clean((void *)buf, recved);
    }

    if (recved > 0) {
        for (size_t i = 0; i < recved; i++) {
            LOG_RAW("0x%02x ", ((uint8_t *)buf)[i]);
        }
        LOG_RAW("nbytes:%d\r\n", recved);
    }

    return recved;
}

static int usbh_cdc_acm_dev_write(struct dfs_fd *file, const void *buf, size_t count)
{
    int ret;

    rt_device_t dev_id;
    struct usbh_cdc_acm *cdc_acm_class;

    RT_ASSERT(file != RT_NULL);

    /* get device handler */
    dev_id = (rt_device_t)file->fnode->data;
    RT_ASSERT(dev_id != RT_NULL);

    cdc_acm_class = (struct usbh_cdc_acm *)dev_id->user_data;
    RT_ASSERT(cdc_acm_class != RT_NULL);

    rt_hw_cpu_dcache_invalidate((void *)buf, count);

    if (0 > (ret = usbh_cdc_acm_bulk_out_transfer(cdc_acm_class, (uint8_t *)buf, count, 500))) {
        LOG_E("bulk out error,ret:%d\r\n", ret);
        return ret;
    } else {
        LOG_I("send over:%d\r\n", cdc_acm_class->bulkout_urb.actual_length);
    }

    return cdc_acm_class->bulkout_urb.actual_length;
}

static int usbh_cdc_acm_dev_ioctl(struct dfs_fd *file, int cmd, void *args) {
  return -1;
}

static const struct dfs_file_ops usbh_cdc_acm_dev_ops = {
    .open   = usbh_cdc_acm_dev_open,        // int (*open)     (struct dfs_fd *fd);
    .close  = usbh_cdc_acm_dev_close,       // int (*close)    (struct dfs_fd *fd);
    .ioctl  = usbh_cdc_acm_dev_ioctl,       // int (*ioctl)    (struct dfs_fd *fd, int cmd, void *args);
    .read   = usbh_cdc_acm_dev_read,        // int (*read)     (struct dfs_fd *fd, void *buf, size_t count);
    .write  = usbh_cdc_acm_dev_write,       // int (*write)    (struct dfs_fd *fd, const void *buf, size_t count);
    // int (*flush)    (struct dfs_fd *fd);
    // int (*lseek)    (struct dfs_fd *fd, off_t offset);
    // int (*getdents) (struct dfs_fd *fd, struct dirent *dirp, uint32_t count);
    // int (*poll)     (struct dfs_fd *fd, struct rt_pollreq *req);
};

void usbh_cdc_acm_run(struct usbh_cdc_acm *cdc_acm_class)
{
    char name[32];
    struct usbh_cdc_acm_dev_wrap *dev_wrap = NULL;
    struct rt_device *dev = NULL;
    uint8_t minor = cdc_acm_class->minor;

    if(minor > ARRAY_SIZE(usbh_cdc_acm_dev)) {
        LOG_E("cdc acm device id(%d) too large\n", minor);
        return;
    }
    dev_wrap = &usbh_cdc_acm_dev[minor];
    dev = &dev_wrap->dev;

    dev->type = RT_Device_Class_Char;
    dev->user_data = (void *)cdc_acm_class;

    snprintf(name, sizeof(name), "ttyACM%d", minor);

    if(rt_device_find(name)) {
        LOG_E("/dev/%s already exists\n");
        return;
    }

    LOG_I("register /dev/%s\n", name);

    rt_device_register(dev, name, RT_DEVICE_FLAG_RDWR);
    dev->fops = &usbh_cdc_acm_dev_ops;

    rt_event_init(&dev_wrap->evt, "usb_acm", RT_IPC_FLAG_FIFO);
}

void usbh_cdc_acm_stop(struct usbh_cdc_acm *cdc_acm_class)
{
    char name[32];
    struct usbh_cdc_acm_dev_wrap *dev_wrap = NULL;
    uint8_t minor = cdc_acm_class->minor;

    if(minor > ARRAY_SIZE(usbh_cdc_acm_dev)) {
        LOG_E("cdc acm device id(%d) too large\n", minor);
        return;
    }
    dev_wrap = &usbh_cdc_acm_dev[minor];

    snprintf(name, sizeof(name), "ttyACM%d", minor);
    LOG_I("unregister /dev/%s\n", name);

    rt_device_unregister(&dev_wrap->dev);
    rt_event_detach(&dev_wrap->evt);

    memset(dev_wrap, 0, sizeof(*dev_wrap));
}
