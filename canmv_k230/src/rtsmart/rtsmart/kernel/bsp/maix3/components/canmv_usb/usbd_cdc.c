#include "canmv_usb.h"

#include "usbd_cdc.h"

#include "dfs_poll.h"
#include "dfs_file.h"

#include "ipc/waitqueue.h"

#define CDC_MAX_MPS USB_DEVICE_MAX_MPS

/*****************************************************************************/
static int cdc_poll_flag;
static uint32_t actual_read;

static USB_MEM_ALIGNX uint8_t usb_read_buffer[4096];

static struct rt_device cdc_device;
static struct rt_semaphore cdc_read_sem, cdc_write_sem;

static int cdc_open(struct dfs_fd *fd)
{
    return 0;
}

static int cdc_close(struct dfs_fd *fd)
{
    return 0;
}

static int cdc_read(struct dfs_fd *fd, void *buf, size_t count) {
    rt_err_t error = rt_sem_take(&cdc_read_sem, rt_tick_from_millisecond(100));

    if (error == RT_EOK) {
        int tmp = actual_read;
        memcpy(buf, usb_read_buffer, actual_read);
        actual_read = 0;
        usbd_ep_start_read(USB_DEVICE_BUS_ID, CDC_OUT_EP, usb_read_buffer, sizeof(usb_read_buffer));
        return tmp;
    } else if (error == -RT_ETIMEOUT) {
        USB_LOG_WRN("read timeout\n");
        return 0;
    } else {
        return -1;
    }
}

static int cdc_write(struct dfs_fd *fd, const void *buf, size_t count) {
    if (count == 0 || (false == g_usb_device_connected)) {
        return 0;
    }

    rt_err_t error = rt_sem_take(&cdc_write_sem, rt_tick_from_millisecond(10));
    if (error == RT_EOK) {
        usbd_ep_start_write(USB_DEVICE_BUS_ID, CDC_IN_EP, buf, count);
    }

    return error == RT_EOK ? count : -error;
}

static int cdc_poll(struct dfs_fd *fd, struct rt_pollreq *req)
{
    rt_poll_add(&cdc_device.wait_queue, req);
    int tmp = cdc_poll_flag;
    cdc_poll_flag = 0;
    return tmp;
}

static const struct dfs_file_ops cdc_ops = {
    .open = cdc_open,
    .close = cdc_close,
    .read = cdc_read,
    .write = cdc_write,
    .poll = cdc_poll
};

static void cdc_device_init(void)
{
    rt_err_t err = rt_device_register(&cdc_device, "ttyUSB", RT_DEVICE_OFLAG_RDWR);
    if (err != RT_EOK) {
        rt_kprintf("[usb] register device error\n");
        return;
    }
    cdc_device.fops = &cdc_ops;
    rt_sem_init(&cdc_read_sem, "cdc_read", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&cdc_write_sem, "cdc_write", 1, RT_IPC_FLAG_FIFO);
}

/*****************************************************************************/
static void usbd_cdc_acm_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    actual_read = nbytes;
    cdc_poll_flag |= POLLIN;
    rt_wqueue_wakeup(&cdc_device.wait_queue, (void*)POLLIN);
    rt_sem_release(&cdc_read_sem);
}

static void usbd_cdc_acm_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    /* send zlp */
    if (nbytes && (nbytes % CDC_MAX_MPS) == 0)
        usbd_ep_start_write(USB_DEVICE_BUS_ID, CDC_IN_EP, NULL, 0);
    else
        rt_sem_release(&cdc_write_sem);
}

static struct usbd_interface usbd_cdc_intf;

static struct usbd_endpoint cdc_out_ep = {
    .ep_addr = CDC_OUT_EP,
    .ep_cb = usbd_cdc_acm_bulk_out
};

static struct usbd_endpoint cdc_in_ep = {
    .ep_addr = CDC_IN_EP,
    .ep_cb = usbd_cdc_acm_bulk_in
};

void canmv_usb_device_cdc_on_connected(void)
{
    rt_sem_control(&cdc_write_sem, RT_IPC_CMD_RESET, (void *)1);
    usbd_ep_start_read(USB_DEVICE_BUS_ID, CDC_OUT_EP, usb_read_buffer, sizeof(usb_read_buffer));
}

void canmv_usb_device_cdc_init(void)
{
    usbd_cdc_acm_init_intf(USB_DEVICE_BUS_ID, &usbd_cdc_intf);
    usbd_add_interface(USB_DEVICE_BUS_ID, &usbd_cdc_intf);
    usbd_add_endpoint(USB_DEVICE_BUS_ID, &cdc_out_ep);
    usbd_add_endpoint(USB_DEVICE_BUS_ID, &cdc_in_ep);

    cdc_device_init();
}

void usbd_cdc_acm_set_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{

}

void usbd_cdc_acm_get_line_coding(uint8_t busid, uint8_t intf, struct cdc_line_coding *line_coding)
{
    line_coding->dwDTERate = 2000000;
    line_coding->bDataBits = 8;
    line_coding->bParityType = 0;
    line_coding->bCharFormat = 0;
}

void usbd_cdc_acm_set_dtr(uint8_t busid, uint8_t intf, bool dtr)
{
    cdc_poll_flag |= POLLERR;
    rt_wqueue_wakeup(&cdc_device.wait_queue, (void*)POLLERR);
}

void usbd_cdc_acm_set_rts(uint8_t busid, uint8_t intf, bool rts)
{

}

void usbd_cdc_acm_send_break(uint8_t busid, uint8_t intf)
{

}
