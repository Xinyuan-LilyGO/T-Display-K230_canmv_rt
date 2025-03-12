#include "canmv_usb.h"

#include "netif/etharp.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/tcpip.h"
#include "rtdef.h"

#if LWIP_DHCP
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#endif

#include <rtdevice.h>
#include <netif/ethernetif.h>
#include <netdev.h>

#include "usbh_rtl8152.h"

static struct eth_device rtl8152_dev;

static rt_err_t rt_usbh_rtl8152_control(rt_device_t dev, int cmd, void *args)
{
    struct usbh_rtl8152 *rtl8152_class = (struct usbh_rtl8152 *)dev->user_data;

    switch (cmd) {
        case NIOCTL_GADDR:

            /* get mac address */
            if (args)
                rt_memcpy(args, rtl8152_class->mac, 6);
            else
                return -RT_ERROR;

            break;
        case 0x1000: // set mac
            // can we write mac address?

            break;
        case 0x1001: // get connect_status
            *(bool *)args = rtl8152_class->connect_status;
            break;

        default:
            break;
    }

    return RT_EOK;
}

const static struct rt_device_ops rtl8152_device_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    rt_usbh_rtl8152_control
};

static rt_err_t rt_usbh_rtl8152_eth_tx(rt_device_t dev, struct pbuf *p)
{
    return usbh_rtl8152_linkoutput(NULL, p);
}

void usbh_rtl8152_run(struct usbh_rtl8152 *rtl8152_class)
{
    struct netdev *netdev;

    usb_memset(&rtl8152_dev, 0, sizeof(struct eth_device));

    rtl8152_dev.parent.ops         = &rtl8152_device_ops;

    rtl8152_dev.eth_rx = NULL;
    rtl8152_dev.eth_tx = rt_usbh_rtl8152_eth_tx;
    rtl8152_dev.parent.user_data = rtl8152_class;

    eth_device_init(&rtl8152_dev, CANMV_USB_HOST_NET_RTL8152_DEV_NAME);
    // eth_device_linkchange(&rtl8152_dev, RT_TRUE);

    usb_osal_thread_create("usbh_rtl8152_rx", 4096, 15, usbh_rtl8152_rx_thread, rtl8152_dev.netif);
}

void usbh_rtl8152_stop(struct usbh_rtl8152 *rtl8152_class)
{
    eth_device_deinit(&rtl8152_dev);
}

void usbh_rtl8152_link_changed(struct usbh_rtl8152 *rtl8152_class, int state)
{
    if(0x00 == state) {
        eth_device_linkchange(&rtl8152_dev, RT_FALSE);
    } else {
        eth_device_linkchange(&rtl8152_dev, RT_TRUE);
    }
}
