#include "canmv_usb.h"
#include "rtthread.h"

/*!< global descriptor */
static const uint8_t canmv_usb_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),

#if defined (CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM) && defined (CHERRY_USB_DEVICE_ENABLE_CLASS_MTP)
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x03, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
#elif defined (CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM)
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x02, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
#endif

#if defined (CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM)
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, USB_DEVICE_MAX_MPS, 0x02),
#endif

#if defined (CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM) && defined (CHERRY_USB_DEVICE_ENABLE_CLASS_MTP)
    MTP_DESCRIPTOR_INIT(0x02, MTP_OUT_EP, MTP_IN_EP, MTP_INT_EP, 0x00),
#endif
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x12,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'K', 0x00,                  /* wcChar0 */
    'e', 0x00,                  /* wcChar1 */
    'n', 0x00,                  /* wcChar2 */
    'd', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    't', 0x00,                  /* wcChar6 */
    'e', 0x00,                  /* wcChar7 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x0C,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'a', 0x00,                  /* wcChar1 */
    'n', 0x00,                  /* wcChar2 */
    'M', 0x00,                  /* wcChar3 */
    'V', 0x00,                  /* wcChar4 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '0', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '1', 0x00,                  /* wcChar2 */
    '0', 0x00,                  /* wcChar3 */
    '0', 0x00,                  /* wcChar4 */
    '0', 0x00,                  /* wcChar5 */
    '0', 0x00,                  /* wcChar6 */
    '0', 0x00,                  /* wcChar7 */
    '0', 0x00,                  /* wcChar8 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0xEF,
    0x02,
    0x01,
    0x40,
    0x00,
    0x00,
#endif
    0x00
};

bool g_fs_mount_data_succ = false;
bool g_usb_device_connected = false;

static void event_handler(uint8_t busid, uint8_t event)
{
    if (event == USBD_EVENT_RESET) {
        g_usb_device_connected = false;
    } else if (event == USBD_EVENT_CONFIGURED) {
        g_usb_device_connected = true;

        canmv_usb_device_cdc_on_connected();
    }
}

/*****************************************************************************/
void canmv_usb_device_init(void *usb_base)
{
    usbd_desc_register(USB_DEVICE_BUS_ID, canmv_usb_descriptor);

#if defined (CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM)
    canmv_usb_device_cdc_init();
#endif // CHERRY_USB_DEVICE_ENABLE_CLASS_CDC_ACM

#if defined(CHERRY_USB_DEVICE_ENABLE_CLASS_MTP)
    canmv_usb_device_mtp_init();
#endif // CHERRY_USB_DEVICE_ENABLE_CLASS_MTP

    usbd_initialize(USB_DEVICE_BUS_ID, (uint32_t)usb_base, event_handler);
}
