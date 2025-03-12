/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <rthw.h>
#include "usb_config.h"
#include "usb_log.h"

#ifdef ENABLE_CHERRY_USB
#define DEFAULT_USB_HCLK_FREQ_MHZ 200

uint32_t SystemCoreClock = (DEFAULT_USB_HCLK_FREQ_MHZ * 1000 * 1000);
uintptr_t g_usb_otg0_base = (uintptr_t)0x91500000UL;
uintptr_t g_usb_otg1_base = (uintptr_t)0x91540000UL;

static void sysctl_reset_hw_done(volatile uint32_t *reset_reg, uint8_t reset_bit, uint8_t done_bit)
{
    *reset_reg |= (1 << done_bit);      /* clear done bit */
    rt_thread_mdelay(1);
    
    *reset_reg |= (1 << reset_bit);     /* set reset bit */
    rt_thread_mdelay(1);
    /* check done bit */
    while(*reset_reg & (1 << done_bit) == 0);
}

#define USB_IDPULLUP0 		(1<<4)
#define USB_DMPULLDOWN0 	(1<<8)
#define USB_DPPULLDOWN0 	(1<<9)

// USB Host
#ifdef ENABLE_CHERRY_USB_HOST
static void usb_hc_interrupt_cb(int irq, void *arg_pv)
{
    extern void USBH_IRQHandler(uint8_t busid);
    USBH_IRQHandler(0);
}

uint32_t usbh_get_dwc2_gccfg_conf(uint32_t reg_base)
{
    return 0;
}
#if defined (CHERRY_USB_HOST_USING_DEV0)
void usb_hc_low_level_init(void)
{
    sysctl_reset_hw_done((volatile uint32_t *)0x9110103c, 0, 28);

    uint32_t *hs_reg = (uint32_t *)rt_ioremap((void *)(0x91585000 + 0x7C), 0x1000);
    uint32_t usb_ctl3 = *hs_reg | USB_IDPULLUP0;

    *hs_reg = usb_ctl3 | (USB_DMPULLDOWN0 | USB_DPPULLDOWN0);
    
    rt_iounmap(hs_reg);

    rt_hw_interrupt_install(173, usb_hc_interrupt_cb, NULL, "usbh");
    rt_hw_interrupt_umask(173);
}

void usb_hc_low_level_deinit(void)
{
    rt_hw_interrupt_mask(173);
}
#elif defined (CHERRY_USB_HOST_USING_DEV1)
void usb_hc_low_level_init(void)
{
    sysctl_reset_hw_done((volatile uint32_t *)0x9110103c, 1, 29);

    uint32_t *hs_reg = (uint32_t *)rt_ioremap((void *)(0x91585000 + 0x9C), 0x1000);
    uint32_t usb_ctl3 = *hs_reg | USB_IDPULLUP0;

    *hs_reg = usb_ctl3 | (USB_DMPULLDOWN0 | USB_DPPULLDOWN0);

    rt_iounmap(hs_reg);

    rt_hw_interrupt_install(174, usb_hc_interrupt_cb, NULL, "usbh");
    rt_hw_interrupt_umask(174);
}

void usb_hc_low_level_deinit(void)
{
    rt_hw_interrupt_mask(174);
}
#else
#error "Usb device select error"
#endif

#endif // ENABLE_CHERRY_USB_HOST

// USB Device
#if defined (ENABLE_CHERRY_USB_DEVICE) 
static void usb_dc_interrupt_cb(int irq, void *arg_pv)
{
    extern void USBD_IRQHandler(uint8_t busid);
    USBD_IRQHandler(0);
}
uint32_t usbd_get_dwc2_gccfg_conf(uint32_t reg_base)
{
    return 0;
}
#if defined (CHERRY_USB_DEVICE_USING_DEV0)
void usb_dc_low_level_init(void)
{
    sysctl_reset_hw_done((volatile uint32_t *)0x9110103c, 0, 28);
    uint32_t *hs_reg = (uint32_t *)rt_ioremap((void *)(0x91585000 + 0x7C), 0x1000);
    *hs_reg = 0x37;
    rt_iounmap(hs_reg);

    rt_hw_interrupt_install(173, usb_dc_interrupt_cb, NULL, "usbd");
    rt_hw_interrupt_umask(173);
}

void usb_dc_low_level_deinit(void)
{
    rt_hw_interrupt_mask(173);
}
#elif defined(CHERRY_USB_DEVICE_USING_DEV1)
void usb_dc_low_level_init(void)
{
    sysctl_reset_hw_done((volatile uint32_t *)0x9110103c, 1, 29);
    uint32_t *hs_reg = (uint32_t *)rt_ioremap((void *)(0x91585000 + 0x9C), 0x1000);
    *hs_reg = 0x37;
    rt_iounmap(hs_reg);

    rt_hw_interrupt_install(174, usb_dc_interrupt_cb, NULL, "usbd");
    rt_hw_interrupt_umask(174);
}

void usb_dc_low_level_deinit(void)
{
    rt_hw_interrupt_mask(174);
}
#else
#error "Usb device select error"
#endif 

#endif // ENABLE_CHERRY_USB_DEVICE

#endif // ENABLE_CHERRY_USB
