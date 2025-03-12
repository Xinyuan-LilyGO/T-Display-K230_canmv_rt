

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "sys/ioctl.h"
#include "rt_fpioa.h"
#include <sys/mman.h>
#define MAX_PIN_NUM 64
#define PIN_FUNC_NUM 5
#define FUNC_ALT_PIN_NUM 4
#define TEMP_STR_LEN 128
#define IOMUX_REG_ADD 0X91105000



#pragma pack (1)

struct st_func_describe {
    enum en_func_def func;
    char *name;
    uint32_t default_cfg;
};

static const struct st_func_describe g_func_describ_array[] = {
    { BOOT0, "BOOT0", 0x101 },
    { BOOT1, "BOOT1", 0x101 },
    { CI0, "RESV", 0x101 },
    { CI1, "RESV", 0x101 },
    { CI2, "RESV", 0x101 },
    { CI3, "RESV", 0x101 },
    { CO0, "RESV", 0x08E },
    { CO1, "RESV", 0x08E },
    { CO2, "RESV", 0x08E },
    { CO3, "RESV", 0x08E },
    { DI0, "RESV", 0x101 },
    { DI1, "RESV", 0x101 },
    { DI2, "RESV", 0x101 },
    { DI3, "RESV", 0x101 },
    { DO0, "RESV", 0x08E },
    { DO1, "RESV", 0x08E },
    { DO2, "RESV", 0x08E },
    { DO3, "RESV", 0x08E },
    { HSYNC0, "HSYNC0", 0x08E },
    { HSYNC1, "HSYNC1", 0x08E },
    { IIC0_SCL, "IIC0_SCL", 0x18F },
    { IIC0_SDA, "IIC0_SDA", 0x18F },
    { IIC1_SCL, "IIC1_SCL", 0x18F },
    { IIC1_SDA, "IIC1_SDA", 0x18F },
    { IIC2_SCL, "IIC2_SCL", 0x18F },
    { IIC2_SDA, "IIC2_SDA", 0x18F },
    { IIC3_SCL, "IIC3_SCL", 0x18F },
    { IIC3_SDA, "IIC3_SDA", 0x18F },
    { IIC4_SCL, "IIC4_SCL", 0x18F },
    { IIC4_SDA, "IIC4_SDA", 0x18F },
    { IIS_CLK, "IIS_CLK", 0x08E },
    { IIS_D_IN0_PDM_IN3, "IIS_D_IN0_PDM_IN3", 0x10F },
    { IIS_D_IN1_PDM_IN2, "IIS_D_IN1_PDM_IN2", 0x10F },
    { IIS_D_OUT0_PDM_IN1, "IIS_D_OUT0_PDM_IN1", 0x08E },
    { IIS_D_OUT1_PDM_IN0, "IIS_D_OUT1_PDM_IN0", 0x08E },
    { IIS_WS, "IIS_WS", 0x08E },
    { JTAG_RST, "JTAG_RST", 0x141 },
    { JTAG_TCK, "JTAG_TCK", 0x121 },
    { JTAG_TDI, "JTAG_TDI", 0x121 },
    { JTAG_TDO, "JTAG_TDO", 0x08E },
    { JTAG_TMS, "JTAG_TMS", 0x121 },
    { M_CLK1, "M_CLK1", 0x08E },
    { M_CLK2, "M_CLK2", 0x08E },
    { M_CLK3, "M_CLK3", 0x08E },
    { MMC1_CLK, "MMC1_CLK", 0x08E },
    { MMC1_CMD, "MMC1_CMD", 0x1CF },
    { MMC1_D0, "MMC1_D0", 0x1CF },
    { MMC1_D1, "MMC1_D1", 0x1CF },
    { MMC1_D2, "MMC1_D2", 0x1CF },
    { MMC1_D3, "MMC1_D3", 0x1CF },
    { OSPI_CLK, "OSPI_CLK", 0x08E },
    { OSPI_CS, "OSPI_CS", 0x0CE },
    { OSPI_D0, "OSPI_D0", 0x18F },
    { OSPI_D1, "OSPI_D1", 0x18F },
    { OSPI_D2, "OSPI_D2", 0x18F },
    { OSPI_D3, "OSPI_D3", 0x18F },
    { OSPI_D4, "OSPI_D4", 0x18F },
    { OSPI_D5, "OSPI_D5", 0x18F },
    { OSPI_D6, "OSPI_D6", 0x18F },
    { OSPI_D7, "OSPI_D7", 0x18F },
    { OSPI_DQS, "OSPI_DQS", 0x101 },
    { PDM_IN0, "PDM_IN0", 0x101 },
    { PDM_IN1, "PDM_IN1", 0x101 },
    { PDM_IN2, "PDM_IN2", 0x101 },
    { PDM_IN3, "PDM_IN3", 0x101 },
    { PULSE_CNTR0, "PULSE_CNTR0", 0x101 },
    { PULSE_CNTR1, "PULSE_CNTR1", 0x101 },
    { PULSE_CNTR2, "PULSE_CNTR2", 0x101 },
    { PULSE_CNTR3, "PULSE_CNTR3", 0x101 },
    { PULSE_CNTR4, "PULSE_CNTR4", 0x101 },
    { PULSE_CNTR5, "PULSE_CNTR5", 0x101 },
    { PWM0, "PWM0", 0x08E },
    { PWM1, "PWM1", 0x08E },
    { PWM2, "PWM2", 0x08E },
    { PWM3, "PWM3", 0x08E },
    { PWM4, "PWM4", 0x08E },
    { PWM5, "PWM5", 0x08E },
    { QSPI0_CLK, "QSPI0_CLK", 0x08E },
    { QSPI0_CS0, "QSPI0_CS0", 0x0CE },
    { QSPI0_CS1, "QSPI0_CS1", 0x0CE },
    { QSPI0_CS2, "QSPI0_CS2", 0x0CE },
    { QSPI0_CS3, "QSPI0_CS3", 0x0CE },
    { QSPI0_CS4, "QSPI0_CS4", 0x0CE },
    { QSPI0_D0, "QSPI0_D0", 0x18F },
    { QSPI0_D1, "QSPI0_D1", 0x18F },
    { QSPI0_D2, "QSPI0_D2", 0x18F },
    { QSPI0_D3, "QSPI0_D3", 0x18F },
    { QSPI1_CLK, "QSPI1_CLK", 0x08E },
    { QSPI1_CS0, "QSPI1_CS0", 0x0CE },
    { QSPI1_CS1, "QSPI1_CS1", 0x0CE },
    { QSPI1_CS2, "QSPI1_CS2", 0x0CE },
    { QSPI1_CS3, "QSPI1_CS3", 0x0CE },
    { QSPI1_CS4, "QSPI1_CS4", 0x0CE },
    { QSPI1_D0, "QSPI1_D0", 0x18F },
    { QSPI1_D1, "QSPI1_D1", 0x18F },
    { QSPI1_D2, "QSPI1_D2", 0x18F },
    { QSPI1_D3, "QSPI1_D3", 0x18F },
    { SPI2AXI_CK, "SPI2AXI_CK", 0x101 },
    { SPI2AXI_CS, "SPI2AXI_CS", 0x141 },
    { SPI2AXI_DI, "SPI2AXI_DI", 0x101 },
    { SPI2AXI_DO, "SPI2AXI_DO", 0x08E },
    { UART0_RXD, "UART0_RXD", 0x101 },
    { UART0_TXD, "UART0_TXD", 0x08E },
    { UART1_CTS, "UART1_CTS", 0x101 },
    { UART1_RTS, "UART1_RTS", 0x08E },
    { UART1_RXD, "UART1_RXD", 0x101 },
    { UART1_TXD, "UART1_TXD", 0x08E },
    { UART2_CTS, "UART2_CTS", 0x101 },
    { UART2_RTS, "UART2_RTS", 0x08E },
    { UART2_RXD, "UART2_RXD", 0x101 },
    { UART2_TXD, "UART2_TXD", 0x08E },
    { UART3_CTS, "UART3_CTS", 0x101 },
    { UART3_DE, "UART3_DE", 0x08E },
    { UART3_RE, "UART3_RE", 0x08E },
    { UART3_RTS, "UART3_RTS", 0x08E },
    { UART3_RXD, "UART3_RXD", 0x101 },
    { UART3_TXD, "UART3_TXD", 0x08E },
    { UART4_RXD, "UART4_RXD", 0x101 },
    { UART4_TXD, "UART4_TXD", 0x08E },
    { PDM_CLK, "PDM_CLK", 0x08E },
    { VSYNC0, "VSYNC0", 0x08E },
    { VSYNC1, "VSYNC1", 0x08E },
    { CTRL_IN_3D, "CTRL_IN_3D", 0x101 },
    { CTRL_O1_3D, "CTRL_O1_3D", 0x08E },
    { CTRL_O2_3D, "CTRL_O2_3D", 0x08E },
};

static const uint8_t g_pin_func_array[][PIN_FUNC_NUM] = {
    { GPIO0, BOOT0, TEST_PIN0, FUNC_MAX, FUNC_MAX },
    { GPIO1, BOOT1, TEST_PIN1, FUNC_MAX, FUNC_MAX, },
    { GPIO2, JTAG_TCK, PULSE_CNTR0, TEST_PIN2, FUNC_MAX, },
    { GPIO3, JTAG_TDI, PULSE_CNTR1, UART1_TXD, TEST_PIN0, },
    { GPIO4, JTAG_TDO, PULSE_CNTR2, UART1_RXD, TEST_PIN1, },
    { GPIO5, JTAG_TMS, PULSE_CNTR3, UART2_TXD, TEST_PIN2, },
    { GPIO6, JTAG_RST, PULSE_CNTR4, UART2_RXD, TEST_PIN3, },
    { GPIO7, PWM2, IIC4_SCL, TEST_PIN3, DI0, },
    { GPIO8, PWM3, IIC4_SDA, TEST_PIN4, DI1, },
    { GPIO9, PWM4, UART1_TXD, IIC1_SCL, DI2, },
    { GPIO10, CTRL_IN_3D, UART1_RXD, IIC1_SDA, DI3, },
    { GPIO11, CTRL_O1_3D, UART2_TXD, IIC2_SCL, DO0, },
    { GPIO12, CTRL_O2_3D, UART2_RXD, IIC2_SDA, DO1, },
    { GPIO13, M_CLK1, DO2, FUNC_MAX, FUNC_MAX, },
    { GPIO14, OSPI_CS, TEST_PIN5, QSPI0_CS0, DO3, },
    { GPIO15, OSPI_CLK, TEST_PIN6, QSPI0_CLK, CO3, },
    { GPIO16, OSPI_D0, QSPI1_CS4, QSPI0_D0, CO2, },
    { GPIO17, OSPI_D1, QSPI1_CS3, QSPI0_D1, CO1, },
    { GPIO18, OSPI_D2, QSPI1_CS2, QSPI0_D2, CO0, },
    { GPIO19, OSPI_D3, QSPI1_CS1, QSPI0_D3, TEST_PIN4, },
    { GPIO20, OSPI_D4, QSPI1_CS0, PULSE_CNTR0, TEST_PIN5, },
    { GPIO21, OSPI_D5, QSPI1_CLK, PULSE_CNTR1, TEST_PIN6, },
    { GPIO22, OSPI_D6, QSPI1_D0, PULSE_CNTR2, TEST_PIN7, },
    { GPIO23, OSPI_D7, QSPI1_D1, PULSE_CNTR3, TEST_PIN8, },
    { GPIO24, OSPI_DQS, QSPI1_D2, PULSE_CNTR4, TEST_PIN9, },
    { GPIO25, PWM5, QSPI1_D3, PULSE_CNTR5, TEST_PIN10, },
    { GPIO26, MMC1_CLK, TEST_PIN7, PDM_CLK, FUNC_MAX, },
    { GPIO27, MMC1_CMD, PULSE_CNTR5, PDM_IN0, CI0, },
    { GPIO28, MMC1_D0, UART3_TXD, PDM_IN1, CI1, },
    { GPIO29, MMC1_D1, UART3_RXD, CTRL_IN_3D, CI2, },
    { GPIO30, MMC1_D2, UART3_RTS, CTRL_O1_3D, CI3, },
    { GPIO31, MMC1_D3, UART3_CTS, CTRL_O2_3D, TEST_PIN11, },
    { GPIO32, IIC0_SCL, IIS_CLK, UART3_TXD, TEST_PIN12, },
    { GPIO33, IIC0_SDA, IIS_WS, UART3_RXD, TEST_PIN13, },
    { GPIO34, IIC1_SCL, IIS_D_IN0_PDM_IN3,  UART3_RTS, FUNC_MAX},
    { GPIO35, IIC1_SDA, IIS_D_OUT0_PDM_IN1,  UART3_CTS, FUNC_MAX},
    { GPIO36, IIC3_SCL, IIS_D_IN1_PDM_IN2,  UART4_TXD,  FUNC_MAX},
    { GPIO37, IIC3_SDA, IIS_D_OUT1_PDM_IN0,  UART4_RXD, FUNC_MAX},
    { GPIO38, UART0_TXD, TEST_PIN8, QSPI1_CS0, HSYNC0, },
    { GPIO39, UART0_RXD, TEST_PIN9, QSPI1_CLK, VSYNC0, },
    { GPIO40, UART1_TXD, IIC1_SCL, QSPI1_D0, TEST_PIN18, },
    { GPIO41, UART1_RXD, IIC1_SDA, QSPI1_D1, TEST_PIN19, },
    { GPIO42, UART1_RTS, PWM0, QSPI1_D2, TEST_PIN20, },
    { GPIO43, UART1_CTS, PWM1, QSPI1_D3, TEST_PIN21, },
    { GPIO44, UART2_TXD, IIC3_SCL, TEST_PIN10, SPI2AXI_CK, },
    { GPIO45, UART2_RXD, IIC3_SDA, TEST_PIN11, SPI2AXI_CS, },
    { GPIO46, UART2_RTS, PWM2, IIC4_SCL, TEST_PIN22, },
    { GPIO47, UART2_CTS, PWM3, IIC4_SDA, TEST_PIN23, },
    { GPIO48, UART4_TXD, TEST_PIN12, IIC0_SCL, SPI2AXI_DI, },
    { GPIO49, UART4_RXD, TEST_PIN13, IIC0_SDA, SPI2AXI_DO, },
    { GPIO50, UART3_TXD, IIC2_SCL, QSPI0_CS4, TEST_PIN24, },
    { GPIO51, UART3_RXD, IIC2_SDA, QSPI0_CS3, TEST_PIN25, },
    { GPIO52, UART3_RTS, PWM4, IIC3_SCL, TEST_PIN26, },
    { GPIO53, UART3_CTS, PWM5, IIC3_SDA, FUNC_MAX, },
    { GPIO54, QSPI0_CS0, MMC1_CMD, PWM0, TEST_PIN27, },
    { GPIO55, QSPI0_CLK, MMC1_CLK, PWM1, TEST_PIN28, },
    { GPIO56, QSPI0_D0, MMC1_D0, PWM2, TEST_PIN29, },
    { GPIO57, QSPI0_D1, MMC1_D1, PWM3, TEST_PIN30, },
    { GPIO58, QSPI0_D2, MMC1_D2, PWM4, TEST_PIN31, },
    { GPIO59, QSPI0_D3, MMC1_D3, PWM5, FUNC_MAX, },
    { GPIO60, PWM0, IIC0_SCL, QSPI0_CS2, HSYNC1, },
    { GPIO61, PWM1, IIC0_SDA, QSPI0_CS1, VSYNC1, },
    { GPIO62, M_CLK2, UART3_DE, TEST_PIN14, FUNC_MAX, },
    { GPIO63, M_CLK3, UART3_RE, TEST_PIN15, FUNC_MAX, },
};

#pragma pack () 




struct st_iomux_reg {
    union {
        struct {
            uint32_t st : 1;// bit 0 输入施密特触发器控制使能
            uint32_t ds : 4; // bit 4-1 驱动电流控制：
            uint32_t pd : 1;// bit 5 下拉；
            uint32_t pu : 1;// bit 6
            uint32_t oe : 1; // bit 7 out
            uint32_t ie : 1; // bit 8 input
            uint32_t msc : 1;// bit 9 电压选择；
            uint32_t sl : 1;// bit 10  输出翻转率控制使能（双电压PAD无此控制端
            uint32_t io_sel : 3; // bit 13-11 func 选择
            uint32_t rsv : 17; // bit 30-14
            uint32_t di : 1; // bit 31 当前PAD输入到芯片内部的数据(即PAD的C端)
        } bit;
        uint32_t value;
    } u;
};




static int fpioa_drv_reg_get_or_set(uint32_t pin, uint32_t *value, int set_flage) {
    static volatile uint32_t *reg_base = NULL;

    if (reg_base == NULL) {
        int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd < 0) {
            return -1;
        }
        reg_base = (uint32_t *)mmap(NULL, 4 * MAX_PIN_NUM, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, IOMUX_REG_ADD);
        close(mem_fd);
        if (reg_base == NULL) {
            return -2;
        }
    }

    if (set_flage) {
        *(reg_base + pin) = (*(reg_base + pin) & 0x200) | *value;
    } else {
        *value = *(reg_base + pin);
    }

    return 0;
}




int fpioa_drv_reg_set(uint32_t pin, uint32_t value) {
    return fpioa_drv_reg_get_or_set(pin, &value, 1);
}

int fpioa_drv_reg_get(uint32_t pin, uint32_t *value) {
    return fpioa_drv_reg_get_or_set(pin, value, 0);
}
static int fpioa_drv_func_2_pin_io_sel(uint32_t func, uint32_t pin) {
    for (int i = 0; i < PIN_FUNC_NUM; i++) {
        if (func == g_pin_func_array[pin][i]) {
            return i;
        }
    }

    return -1;
}

static int fpioa_drv_get_pin_from_func(uint32_t func) {
    struct st_iomux_reg reg_value;

    for (int i = 0; i < MAX_PIN_NUM; i++) {
        int io_sel = fpioa_drv_func_2_pin_io_sel(func, i);
        if (io_sel < 0) {
            continue;
        }
        if (fpioa_drv_reg_get(i, (uint32_t *)&reg_value)) {
            return -1;
        }
        if (reg_value.u.bit.io_sel == io_sel) {
            return i;
        }
    }

    return -1;
}

static int fpioa_drv_get_pins_from_func(uint32_t func, uint8_t *pins) {
    int count = 0;

    if (func >= GPIO0 && func <= GPIO63) {
        pins[0] = func;
        return 1;
    }

    for (int i = 0; i < MAX_PIN_NUM; i++) {
        int io_sel = fpioa_drv_func_2_pin_io_sel(func, i);
        if (io_sel < 0) {
            continue;
        }
        pins[count] = i;
        count++;
    }

    return count;
}

static int fpioa_drv_get_func_from_pin(uint32_t pin) {
    struct st_iomux_reg reg_value;

    if (fpioa_drv_reg_get(pin, (uint32_t *)&reg_value)) {
        return -1;
    }

    return g_pin_func_array[pin][reg_value.u.bit.io_sel % PIN_FUNC_NUM];
}

static int fpioa_drv_get_func_name_str(uint32_t func, char *str, uint32_t len) {
    if (func >= FUNC_MAX) {
        return 0;
    }
    if (func <= GPIO63) {
        snprintf(str, len - 2, "GPIO%d", func - GPIO0);
    } else if (func < TEST_PIN0) {
        strncpy(str, g_func_describ_array[func - BOOT0].name, len - 2);
    } else if (func < FUNC_MAX) {
        snprintf(str, len - 2, "RESV");
    }
    strncat(str, "/", len - 1);

    return strlen(str);
}

static char *fpioa_drv_get_pin_funcs_str(uint32_t pin, char *str, uint32_t len) {
    uint32_t cur_pos = 0;

    for (int i = 0; i < PIN_FUNC_NUM; i++) {
        cur_pos += fpioa_drv_get_func_name_str(g_pin_func_array[pin][i], str + cur_pos, len - cur_pos);
    }

    return str;
}

static char *fpioa_drv_get_pin_func_str(uint32_t pin, char *str, uint32_t len, int detail_flage) {
    struct st_iomux_reg reg_value;
    int cur_pos = 0;

    if (fpioa_drv_reg_get(pin, (uint32_t *)&reg_value)) {
        return str;
    }

    cur_pos = fpioa_drv_get_func_name_str(g_pin_func_array[pin][reg_value.u.bit.io_sel % PIN_FUNC_NUM], str, len);
    if (cur_pos == 0) {
        return str;
    }

    str[cur_pos - 1] = 0;

    if (detail_flage) {
        str[cur_pos - 1] = ','; // gpio0,ie:,oe:,
        snprintf(str + cur_pos, len - cur_pos,
            "ie:%d,oe:%d,pd:%d,pu:%d,msc:%s,ds:%d,st:%d,sl:%d,di:%d",
            reg_value.u.bit.ie, reg_value.u.bit.oe, reg_value.u.bit.pd, reg_value.u.bit.pu,
            ((reg_value.u.bit.msc) ? "1-1.8v" : "0-3.3v"), reg_value.u.bit.ds, reg_value.u.bit.st,
            reg_value.u.bit.sl, reg_value.u.bit.di);
    }

    return str;
}

static uint32_t fpioa_drv_extract_cfg(int pin,int func,int ie, int oe, int pu, int pd, int ds, int st, int sl) {
    int sel, tmp;
    struct st_iomux_reg cfg;    
    if (pin < 0 || pin >= MAX_PIN_NUM) {
        //mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("pin number is invalid"));
    }
    if (func < 0 || func >= FUNC_MAX) {
        //mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("function number is invalid"));
    }
    sel = fpioa_drv_func_2_pin_io_sel(func, pin);
    if (sel == -1) {
        //mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("pin not have this function"));
    }
    if (func <= GPIO63) {
        cfg.u.value = 0x18F;
    } else if (func < TEST_PIN0) {
        cfg.u.value = g_func_describ_array[func - BOOT0].default_cfg;
    } else if (func < FUNC_MAX) {
        cfg.u.value = 0;
    }
    cfg.u.bit.io_sel = sel;
    tmp = ie;
    if (tmp != -1) {
        cfg.u.bit.ie = tmp > 0 ? 1 : 0;
    }
    tmp = oe;
    if (tmp != -1) {
        cfg.u.bit.oe = tmp > 0 ? 1 : 0;
    }
    tmp = pu;
    if (tmp != -1) {
        cfg.u.bit.pu = tmp > 0 ? 1 : 0;
    }
    tmp = pd;
    if (tmp != -1) {
        cfg.u.bit.pd = tmp > 0 ? 1 : 0;
    }
    tmp = st;
    if (tmp != -1) {
        cfg.u.bit.st = tmp > 0 ? 1 : 0;
    }
    tmp = sl;
    if (tmp != -1) {
        cfg.u.bit.sl = tmp > 0 ? 1 : 0;
    }
    tmp = ds;
    if (tmp != -1) {
        cfg.u.bit.ds = tmp < 0 ? 0 : tmp > 0xF ? 0xF : tmp;
    }
    return cfg.u.value;
}


void fpioa_help_print_pin_func(int pin_num, int detail_flag) {
    char str_tmp[TEMP_STR_LEN];
    char str_tmp1[TEMP_STR_LEN];

    if (pin_num == -1) {
        for (int i = 0; i < MAX_PIN_NUM; i++) {
            fpioa_help_print_pin_func(i, detail_flag);
        }
        return;
    }
    memset(str_tmp, 0, sizeof(str_tmp));
    memset(str_tmp1, 0, sizeof(str_tmp1));
    fpioa_drv_get_pin_funcs_str(pin_num, str_tmp, sizeof(str_tmp));
    fpioa_drv_get_pin_func_str(pin_num, str_tmp1, sizeof(str_tmp1), detail_flag);

    if (detail_flag) {
        //printf(&mp_plat_print, "|%-17s|%-60s|\r\n", "current config", str_tmp1);
        //printf(&mp_plat_print, "|%-17s|%-60s|\r\n", "can be function", str_tmp);
    } else {
        //printf(&mp_plat_print, "| %-2d   | %-10s | %-56s|\r\n", pin_num, str_tmp1, str_tmp);
    }
}



int fpioa_set_function(int pin,int func,int ie, int oe, int pu, int pd, int ds, int st, int sl) {

    uint32_t cfg = fpioa_drv_extract_cfg(pin,func,ie,oe, pu, pd, ds, st,sl);
    uint8_t pins[FUNC_ALT_PIN_NUM];
    int pin_count = fpioa_drv_get_pins_from_func(func, pins);
    for (int i = 0; i < pin_count; i++) {
        if (pins[i] == pin || fpioa_drv_get_func_from_pin(pins[i]) != func) {
            continue;
        }
        fpioa_drv_reg_set(pins[i], 0);
    }

    fpioa_drv_reg_set(pin, cfg);

    return 0;
}
int fpioa_get_pin_num(int func)
{
     int pin;
    if (func < 0 || func >= FUNC_MAX) {
       return -1; 
    }

    pin = fpioa_drv_get_pin_from_func(func);
    return pin;

}
int fpioa_get_pin_func(int pin) {
    int func;
    if (pin < 0 || pin >= MAX_PIN_NUM) {
     return -1;   
    }

    func = fpioa_drv_get_func_from_pin(pin);
    return func;
}


int fpioa_help(int pin) {
    /*enum { ARG_num, ARG_func };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_num, MP_ARG_INT, { .u_int = -1 } },
        { MP_QSTR_func, MP_ARG_BOOL | MP_ARG_KW_ONLY, { .u_bool = false } },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    if (args[ARG_func].u_bool == false) { // pin mode
        int pin = args[ARG_num].u_int;
        if (pin != -1) {
            if ((pin >= MAX_PIN_NUM) || (pin < 0)) {
                //mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("pin number is invalid"));
            }
            printf(&mp_plat_print, "|%-17s|%-60d|\r\n", "pin num ", pin);
            fpioa_help_print_pin_func(pin, 1);
        } else {
            printf(&mp_plat_print, "| pin  | cur func   |                can be func                              |\r\n");
            printf(&mp_plat_print, "| ---- |------------|---------------------------------------------------------|\r\n");
            fpioa_help_print_pin_func(-1, 0);
        }
    } else { // func mode
        int func = args[ARG_num].u_int;
        if (func < 0 || func >= FUNC_MAX) {
            //mp_raise_msg_varg(&mp_type_ValueError, MP_ERROR_TEXT("function number is invalid"));
        }
        uint8_t pins[FUNC_ALT_PIN_NUM];
        int count;
        char str_tmp[TEMP_STR_LEN];
        int len;
        count = fpioa_drv_get_pins_from_func(func, pins);
        len = fpioa_drv_get_func_name_str(func, str_tmp, TEMP_STR_LEN);
        str_tmp[len - 1] = 0;
        printf(&mp_plat_print, "%s function can be set to ", str_tmp);
        for (int i = 0; i < count; i++) {
            printf(&mp_plat_print, "PIN%d%s", pins[i], i + 1 == count ? "" : ", ");
        }
        printf(&mp_plat_print, "\r\n");
    }*/
    return 0;
}
