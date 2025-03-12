#pragma once

#include <stdbool.h>

#include "rtthread.h"

#if defined (ENABLE_CHERRY_USB_DEVICE)
    #include "usbd_core.h"
    #include "usb_cdc.h"
    #include "usb_mtp.h"
#endif

#if defined (ENABLE_CHERRY_USB_HOST)
    #include "usbh_core.h"
#endif

#include "usb_osal.h"

/*!< device id */
#define USB_DEVICE_BUS_ID 0x00

/*!< endpoint address */
#define CDC_IN_EP  0x81
#define CDC_OUT_EP 0x01
#define CDC_INT_EP 0x82

/*!< endpoint address */
#define MTP_IN_EP  0x83
#define MTP_OUT_EP 0x03
#define MTP_INT_EP 0x84

// OpenMV Cam
#define USBD_VID           0x1209
#define USBD_PID           0xABD1
#define USBD_MAX_POWER     500
#define USBD_LANGID_STRING 1033

#ifdef CONFIG_USB_HS
#define USB_DEVICE_MAX_MPS 512
#else
#define USB_DEVICE_MAX_MPS 64
#endif

/*!< config descriptor size */
#if defined (CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM) && defined (CHERRY_USB_DEVICE_ENABLE_CLASS_MTP)
    #define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN + MTP_DESCRIPTOR_LEN)
#elif defined (CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM)
    #define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN)
#else
#error "Must enable CDC or CDC and MTP"
#endif

extern bool g_fs_mount_data_succ;
extern bool g_usb_device_connected;

extern void canmv_usb_device_cdc_on_connected(void);
extern void canmv_usb_device_cdc_init(void);

extern void canmv_usb_device_mtp_init(void);

extern void canmv_usb_device_init(void *usb_base);
