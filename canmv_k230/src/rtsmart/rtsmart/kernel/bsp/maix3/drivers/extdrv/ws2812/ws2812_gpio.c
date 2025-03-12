#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>

#include <stdint.h>

#include <riscv_io.h>
#include "drv_gpio.h"
#include "board.h"
#include "ioremap.h"
#include <lwp_user_mm.h>

#if defined (WS2812_USE_DRV_GPIO)
/**
 * 因为使用gpio驱动，很多代码考虑极致的性能场景，无法实现配置通用
 */
#define WS2812_GPIO_PIN             (WS2812_GPIO_PIN_NUM)

#if WS2812_GPIO_PIN_NUM > 32
    #undef WS2812_GPIO_PIN
    #define WS2812_GPIO_PIN             (WS2812_GPIO_PIN_NUM - 32) /// >32则-32
#endif

#define IOCTRL_WS2812_SET_RGB_VALUE     0x00

typedef union
{
    uint32_t value;
    struct
    {
        uint8_t b;
        uint8_t r;
        uint8_t g;
        uint8_t none;
    };
} ws2812_value;

static char *gpio1_reg = NULL;
struct rt_device ws2812_dev;

static inline uint64_t get_ticks()
{
    static volatile uint64_t time_elapsed = 0;
    __asm__ __volatile__(
        "rdtime %0"
        : "=r"(time_elapsed));
    return time_elapsed;
}

static inline void ws2812_out0(void)
{
    volatile uint64_t tick;
    uint32_t gpio_val;

    gpio_val = readl(gpio1_reg);

    gpio_val |= (1 << WS2812_GPIO_PIN);
    writel(gpio_val, gpio1_reg);

    tick = get_ticks();
    while (get_ticks() - tick < 7);

    gpio_val &= ~(1 << WS2812_GPIO_PIN);
    writel(gpio_val, gpio1_reg);

    tick = get_ticks();
    while (get_ticks() - tick < 17);
}

static inline  void ws2812_out1(void)
{
    volatile uint64_t tick;
    uint32_t gpio_val;

    gpio_val = readl(gpio1_reg);

    gpio_val |= (1 << WS2812_GPIO_PIN);
    writel(gpio_val, gpio1_reg);

    tick = get_ticks();
    while (get_ticks() - tick < 17);

    gpio_val &= ~(1 << WS2812_GPIO_PIN);
    writel(gpio_val, gpio1_reg);

    tick = get_ticks();
    while (get_ticks() - tick < 17);
}

void ws2812_set_value(ws2812_value data)
{
    data.value &= 0x00ffffff;
    data.value = data.value << 8;

    rt_enter_critical();
    rt_base_t it = rt_hw_interrupt_disable();

    for (size_t i = 0; i < 24; i++)
    {
        if (data.value & 0x80000000)
        {
            ws2812_out1();
        }
        else
        {
            ws2812_out0();
        }
        data.value = data.value << 1;
    }

    rt_hw_interrupt_enable(it);
    rt_exit_critical();
}

static rt_err_t _ws2812_dev_control(rt_device_t dev, int cmd, void *args)
{
    if (cmd == IOCTRL_WS2812_SET_RGB_VALUE)
    {
        ws2812_value data;
        lwp_get_from_user(&data,args,sizeof(ws2812_value));
        ws2812_set_value(data);
        return RT_EOK;
    }
    return RT_EINVAL;
}

const static struct rt_device_ops ws2812_dev_ops =
{
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    _ws2812_dev_control
};

int ws2812_regulator_init(void)
{
    rt_err_t ret;
    gpio1_reg = rt_ioremap((void *)GPIO1_BASE_ADDR, GPIO1_IO_SIZE);
    kd_pin_mode(WS2812_GPIO_PIN_NUM, GPIO_DM_OUTPUT);
    ws2812_out0();

    ws2812_dev.ops = &ws2812_dev_ops;
    ws2812_dev.user_data = NULL;
    ws2812_dev.type = RT_Device_Class_Char;
    RT_ASSERT(rt_device_register(&ws2812_dev,"ws2812",RT_DEVICE_FLAG_RDWR) == RT_EOK);
    return 0;
}
INIT_DEVICE_EXPORT(ws2812_regulator_init);

#endif // WS2812_USE_DRV_GPIO
