#include <unistd.h>
#include <stdio.h>
#include <rtdbg.h>
#include <stdbool.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <encoding.h>
#include <io.h>
#include "drv_gpio.h"
#include "sx1276.h"
#include <riscv_io.h>
#define GPIO_BASE_ADDR0     (0x9140B000U)
#define GPIO_BASE_ADDR1     (0x9140C000U)

//static struct rt_device g_eeprom_device[EEPROM_CNT] = {0};

//static struct rt_device g_lora_device;


/********* device *********/
#define DDEVICE_NAME		"sx1276"		/* 名字 */
//#define DDEVICE_CNT			1				/* 设备号个数 */

#define DDEVICE_SPEED_STD		0			/* 标准速度还是高速，标准速度为1, 高速为0 */
#define DDEVICE_RW_RETRIES		5			/* 读写最大尝试次数, 最小为1 */

#define EEPROM_MEM_START	0x00
#define EEPROM_MEM_SIZE		0x80
#define EEPROM_ID_START		0x00	
#define EEPROM_ID_SIZE		0x20
#define EEPROM_MEM_ADDR		EEPROM_MEM_START	
#define EEPROM_ID_ADDR		(EEPROM_MEM_ADDR + EEPROM_MEM_SIZE)
#define EEPROM_ADDR 		EEPROM_MEM_ADDR
#define EEPROM_SIZE			(EEPROM_MEM_SIZE + EEPROM_ID_SIZE)

/* 设备结构体 */
typedef struct lora_dev {
	int io;					/* GPIO编号 */
	unsigned int gpio_reg;
	unsigned int gpio_bit;
	struct rt_mutex mutex;
	uint8_t rom_id[8];
} lora_device_t, sx1276_device_t;

//static struct eeprom_dev eeprom_devp[EEPROM_CNT];

//static struct lora_dev lora_devp;


static inline unsigned long get_cycles(void)
{
	unsigned long result;

    __asm volatile("rdcycle %0" : "=r"(result));
    return (result);
}

void nsdelay(unsigned long nsecs)
{
    unsigned long start = get_cycles();
    double over_time = nsecs*1.6;
    while ((get_cycles() - start) < over_time)
    ;
}
void usdelay(unsigned long usecs)
{
    nsdelay(usecs*1000);
}

static void gpio_pin_enable_output(rt_uint32_t reg, int bit)
{
	rt_uint32_t dir=readl((void*)(reg + 0x4));
    writel(dir | (1 << bit), (void*)(reg + 0x4));
}

static void gpio_pin_set_low(rt_uint32_t reg, int bit)
{
	writel(readl((void*)(reg)) & ~(1 << bit), (void*)(reg));
}

static void gpio_pin_enable_input(rt_uint32_t reg, int bit)
{
	rt_uint32_t dir=readl((void*)(reg + 0x4));
    writel(dir & ~(1 << bit), (void*)(reg + 0x4));
}

static int gpio_pin_get_value(rt_uint32_t reg, int bit)
{
	return (readl((void*)(reg + 0x50))>>bit) & 0x1;
}
#define CLEAR_BIT(reg, bit) gpio_pin_enable_input(reg, bit)
#define SET_BIT(reg, bit) ({gpio_pin_enable_output(reg, bit); gpio_pin_set_low(reg, bit);})
#define GET_BIT(reg, bit) ({gpio_pin_enable_input(reg, bit); gpio_pin_get_value(reg, bit);})

/********* device *********/

/********* sx1276 *********/
// at21cs01 address
#define DEVICE_ADDRESS              0x00
// available opcodes for the at21cs01
#define EEPROM_ACCESS               (0xA0 | DEVICE_ADDRESS)
#define SECURITY_REGISTER_ACCESS    (0xB0 | DEVICE_ADDRESS)
#define LOCK_SECURITY_REGISTER      (0x20 | DEVICE_ADDRESS)
#define ROM_ZONE_REGISTER_ACCESS    (0x70 | DEVICE_ADDRESS)
#define FREEZE_ROM_ZONE_STATE       (0x10 | DEVICE_ADDRESS)
#define MANUFACTURER_ID_READ        (0xC0 | DEVICE_ADDRESS)
#define STANDARD_SPEED_MODE         (0xD0 | DEVICE_ADDRESS)
#define HIGH_SPEED_MODE             (0xE0 | DEVICE_ADDRESS)
// ack no_ack
#define ACK                         0
#define NO_ACK                      1
// memory address range
#define USER_MEMORY_SIZE            0x80
#define MEMORY_PAGE_SIZE            0x08
// error code
#define NO_DEVICE                   -1
#define CRC_ERROR                   -2
#define WRITE_PROTECTED             -3
#define DEVICE_NO_ACK               -4
#define INVALID_PARAMETER           -5

#define DEVICE_NO_ACK_STEP_CMD      1         
#define DEVICE_NO_ACK_STEP_REG      2         
#define DEVICE_NO_ACK_STEP_CMD2     3         

static inline void lora_delay_us(uint64_t us)
{
	usdelay(us);
}

static inline void start(sx1276_device_t *device)
{
	CLEAR_BIT(device->gpio_reg, device->gpio_bit);

#if DDEVICE_SPEED_STD
	usdelay(650);
#else
    usdelay(200);
#endif
}

static inline void stop(sx1276_device_t *device)
{
	CLEAR_BIT(device->gpio_reg, device->gpio_bit);

#if DDEVICE_SPEED_STD
	usdelay(650);
#else
    usdelay(200);
#endif
}



static uint8_t write_byte_hs(sx1276_device_t *device, uint8_t data)
{
/*
    uint8_t ack;
	int i;

    for (i = 0; i < 8; i++) {
        SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
        // eeprom_delay_us(1);
        nsdelay(500);
        if (data & 0x80) // first send msb
            CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
        eeprom_delay_us(10);
        CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
        eeprom_delay_us(4);
        data <<= 1;
    }
    // get ack bit
    SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
    // eeprom_delay_us(1);
	nsdelay(500);
    CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    // eeprom_delay_us(1);
	nsdelay(200);
    ack = GET_BIT(device->gpio_reg, device->gpio_bit); // get data line input
    eeprom_delay_us(14);

    return ack;
    */
    return 0;
}

#if DDEVICE_SPEED_STD
static uint8_t write_byte(sx1276_device_t *device, uint8_t data)
{
   /* uint8_t ack;
	int i;

    for (i = 0; i < 8; i++) {
        SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
        eeprom_delay_us(4);
        if (data & 0x80) // first send msb
            CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
        eeprom_delay_us(20);
        CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
        eeprom_delay_us(18);
        data <<= 1;
    }
    // get ack bit
    SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
    eeprom_delay_us(4);
    CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    eeprom_delay_us(1);
    ack = GET_BIT(device->gpio_reg, device->gpio_bit); // get data line input
    eeprom_delay_us(36);

    return ack;*/
    return 0;
}

static uint8_t read_byte(sx1276_device_t *device, uint8_t ack)
{/*
    uint8_t data;
	int i;

    for (i = 0; i < 8; i++) {
        data <<= 1;
        SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
        eeprom_delay_us(4);
        CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
        eeprom_delay_us(1);
        if (GET_BIT(device->gpio_reg, device->gpio_bit)) // get data line input
            data++; // first receive msb
        eeprom_delay_us(36);
    }
    // send ack bit
    SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
    eeprom_delay_us(4);
    if (ack)
        CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    eeprom_delay_us(20);
    CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    eeprom_delay_us(18);

    return data;
    */
    return 0;
}

static bool sx1276_set_std_speed(sx1276_device_t *device)
{
/*
	bool retb = true;
	rt_base_t level;

	level = rt_hw_interrupt_disable();
	start(device);
    if (write_byte_hs(device, STANDARD_SPEED_MODE) != ACK){
		retb = false;
	}

	start(device);
    if (write_byte(device, STANDARD_SPEED_MODE | 1) != ACK){
		retb = false;
	}
    stop(device);
	rt_hw_interrupt_enable(level);

	return retb;*/
	return 0;
}

#else  // high speed

static inline uint8_t write_byte(sx1276_device_t *device, uint8_t data)
{
	return write_byte_hs(device, data);
}

static uint8_t read_byte(sx1276_device_t *device, uint8_t ack)
{
/*
    uint8_t data;
	int i;

    for (i = 0; i < 8; i++) {
        data <<= 1;
        SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
        // eeprom_delay_us(1);
		nsdelay(500);
        CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
        // eeprom_delay_us(1);
		nsdelay(200);
        if (GET_BIT(device->gpio_reg, device->gpio_bit)) // get data line input
            data++; // first receive msb
        eeprom_delay_us(14);
    }
    // send ack bit
    SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
    // eeprom_delay_us(1);
	nsdelay(500);
    if (ack)
        CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    eeprom_delay_us(10);
    CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    eeprom_delay_us(4);

    return data;*/
    return 0;
}
#endif

int sx1276_initialization(sx1276_device_t *device)
{
/*
    int result;
	rt_base_t level;

	level = rt_hw_interrupt_disable();

    // 1.begin the reset sequence
    SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
    eeprom_delay_us(500);
    CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    eeprom_delay_us(10);
    // 2.begin the discovery response sequence
    SET_BIT(device->gpio_reg, device->gpio_bit); // drive data line output low
    eeprom_delay_us(1);
	// nsdelay(500);
    CLEAR_BIT(device->gpio_reg, device->gpio_bit); // release the data line
    eeprom_delay_us(2);
    result = GET_BIT(device->gpio_reg, device->gpio_bit); // wait response
    eeprom_delay_us(26);

	rt_hw_interrupt_enable(level);

    return result == 0 ? 0 : NO_DEVICE;
    */
    return 0;
}

static int write_data(sx1276_device_t *device, uint8_t cmd, uint8_t reg, uint8_t *data, uint8_t length)
{
/*
    int result;
	int i;
	rt_base_t level;

	level = rt_hw_interrupt_disable();
    result = DEVICE_NO_ACK;
    start(device);
    // send cmd
    if (write_byte(device, cmd) != ACK){
		rt_kprintf("%s: %s cmd(0x%x) no ack\n", DDEVICE_NAME, __FUNCTION__, cmd);
        goto error;
	}
    // send reg
    if (write_byte(device, reg) != ACK){
		rt_kprintf("%s: %s reg(0x%x) no ack\n", DDEVICE_NAME, __FUNCTION__, reg);
        goto error;
	}
    // send data
    for (i = 0; i < length; i++) {
        if (write_byte(device, *data++) != ACK){
			rt_kprintf("%s: %s data(%d:0x%x) no ack\n", DDEVICE_NAME, __FUNCTION__, i, *(--data));
            goto error;
		}
    }
    stop(device);
    result = 0;
error:
	rt_hw_interrupt_enable(level);
    if (result == 0)
        eeprom_delay_us(5000);

    return result;
    */
    return 0;
}

static int read_data(sx1276_device_t *device, uint8_t cmd, uint8_t reg, uint8_t *data, uint8_t length)
{
/*
	int i;
    int result;
	rt_base_t level;

	level = rt_hw_interrupt_disable();
    result = DEVICE_NO_ACK;
    start(device);
    // send cmd
    if (write_byte(device, cmd) != ACK){
		rt_kprintf("%s: %s cmd(0x%x) no ack!\n", DDEVICE_NAME, __FUNCTION__, cmd);
        goto error;
	}
    // send reg
    if (write_byte(device, reg) != ACK){
		rt_kprintf("%s: %s reg(0x%x) no ack!\n", DDEVICE_NAME, __FUNCTION__, reg);
        goto error;
	}
    start(device);
    // send cmd
    if (write_byte(device, cmd | 1) != ACK){
		rt_kprintf("%s: %s cmd2(0x%x) no ack!\n", DDEVICE_NAME, __FUNCTION__, cmd);
        goto error;
	}
    // receive data
    for (i = 1; i < length; i++)
        *data++ = read_byte(device, ACK);
    *data++ = read_byte(device, NO_ACK);
    stop(device);
    result = 0;
error:
	rt_hw_interrupt_enable(level);

    return result;
    */
    return 0;
}

bool sx1276_is_high_speed(sx1276_device_t *device)
{
/*
	bool retb = true;

	rt_base_t level;

	level = rt_hw_interrupt_disable();
    start(device);
    // send cmd
    if (write_byte(device, HIGH_SPEED_MODE | 1) != ACK){
		retb = false;
	}
    stop(device);
	rt_hw_interrupt_enable(level);

	return retb;
	*/
	return 0;
}


int sx1276_read_romid(sx1276_device_t *device)
{
   /* int result;

    result = read_data(device, SECURITY_REGISTER_ACCESS, 0, device->rom_id, 8);
    if ((result == 0) && eeprom_crc8(device->rom_id, 8) != 0)
        result = CRC_ERROR;

    return result;*/
    return 0;
}

int sx1276_write(sx1276_device_t *device, uint32_t cmd_addr, uint32_t addr, uint8_t *data, uint32_t length)
{
    /*int result;
	uint8_t retry;

    while (length) {
        uint32_t page_remain_space = MEMORY_PAGE_SIZE - (addr & (MEMORY_PAGE_SIZE - 1)); // remain space on the current page
        uint32_t write_length = length > page_remain_space ? page_remain_space : length;

		retry = DDEVICE_RW_RETRIES;
		while(retry > 0){
			result = write_data(device, cmd_addr, addr, data, write_length);
			if(result == 0){
				break;
			}

			rt_kprintf("%s: write data(addr: %d, first: 0x%x) no ack, and retries %d!!!\n",
				DDEVICE_NAME, addr, *data, DDEVICE_RW_RETRIES - retry);

			retry--;
		}
		if(retry <= 0){
			rt_kprintf("%s: write data(addr: %d, first: 0x%x) no ack, and retry more times failed!!!\n", 
				DDEVICE_NAME, addr, *data);
			break;
		}
		
        length -= write_length;
        addr += write_length;
        data += write_length;
    }

    return result;*/
    return 0;
}

int sx1276_read(sx1276_device_t *device, uint32_t cmd_addr, uint32_t addr, uint8_t *data, uint32_t length)
{
    /*int result;
	uint8_t retry;

	retry = DDEVICE_RW_RETRIES;
	while(retry > 0){
		result = read_data(device, cmd_addr, addr, data, length);
		if(result == 0){
			break;
		}

		rt_kprintf("%s: read data(addr: %d) no ack, and retries %d!!!\n",
				DDEVICE_NAME, addr, DDEVICE_RW_RETRIES - retry);

		retry--;
	}
	if(retry <= 0){
		rt_kprintf("%s: read data(addr: %d) no ack, and retry more times failed!!!\n", 
			DDEVICE_NAME, addr);
	}

    return result;*/

	return 0;
}
/********* sx1276 function*********/

rt_err_t lora_dev_open(rt_device_t dev, rt_uint16_t oflag)
{
/*
    struct lora_dev *lora;
    int ret, retry;
    //bool retb;
   // uint8_t *p;

    RT_ASSERT(dev != RT_NULL);
    lora = (struct lora_dev *) dev->user_data;
    
    rt_mutex_take(&lora->mutex, RT_WAITING_FOREVER);
    
    ret = sx1276_initialization(lora);
	if(!ret){
		rt_kprintf("%s: sx1276 initialize failed(%d)!\n", DDEVICE_NAME, ret);
        rt_mutex_release(&lora->mutex);
		return -1;
	}

    
    return RT_EOK;
    */
    return 0;
}

rt_err_t lora_dev_close(rt_device_t dev)
{
    /*struct eeprom_dev *lora;
	RT_ASSERT(dev != RT_NULL);
    lora = (struct lora_dev *) dev->user_data;

	rt_mutex_release(&lora->mutex);*/
	return 0;
}

rt_size_t lora_dev_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
/*
    int ret;
    struct eeprom_dev *eeprom;
    RT_ASSERT(dev != RT_NULL);
    eeprom = (struct eeprom_dev *) dev->user_data;
    unsigned int count = size; 
    uint32_t cmd_addr = 0;
    bool flag_rd_romid = false;
    
    if(pos >= (EEPROM_ADDR + EEPROM_SIZE)){
		return 0;
	}
    if(pos < EEPROM_ID_ADDR){
		count = (pos + count) <= (EEPROM_ID_ADDR) ? count: (EEPROM_ID_ADDR - pos);
		cmd_addr = EEPROM_ACCESS;
	} else if((pos == EEPROM_ID_ADDR) && (count == 8)){ // romid
		pos = 0;
		cmd_addr = SECURITY_REGISTER_ACCESS;
		flag_rd_romid = true;
	} else {
		pos = pos - EEPROM_ID_ADDR;
		count = (count <= EEPROM_ID_SIZE)? count:EEPROM_ID_SIZE;
		cmd_addr = SECURITY_REGISTER_ACCESS;
	}
    ret = at21cs01_read(eeprom, cmd_addr, pos, buffer, count);
	if(ret){
		return -1;
	}
    // check romid
	if(flag_rd_romid){
		if(eeprom_crc8(buffer, 8) != 0){
			return -1;
		}
	}

	return count;
	*/
	return 0;
}

rt_size_t lora_dev_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
/*
    int ret;
    struct eeprom_dev *eeprom;
    RT_ASSERT(dev != RT_NULL);
    eeprom = (struct eeprom_dev *) dev->user_data;
    unsigned int count = size; 

	if(pos >= (EEPROM_ID_ADDR)){
		return 0;
	}
	
	count = (pos + count) <= (EEPROM_ID_ADDR) ? count: (EEPROM_ID_ADDR - pos);

	ret = at21cs01_write(eeprom, EEPROM_ACCESS, pos & 0xFF, buffer, count);
	if(ret){
		return -1;
	}
	return count;
	*/
	return 0;
}

const static struct rt_device_ops lora_ops =
{
    RT_NULL,
    lora_dev_open,
    RT_NULL,
    lora_dev_read,
    lora_dev_write,
    RT_NULL
};

int sx1276_init(void)
{
	rt_err_t ret = RT_EOK;
	rt_device_t lora_device_sx1276;
	lora_device_sx1276= (rt_device_t)rt_calloc(1, sizeof(struct rt_device));
	struct lora_dev lora_dev_sx1276;
	char name[8];
	
	sprintf(name, "lora%d", 0);
	lora_device_sx1276->ops = &lora_ops;
	/*struct lora_dev {
		int io; 				
		unsigned int gpio_reg;
		unsigned int gpio_bit;
		struct rt_mutex mutex;
		uint8_t rom_id[8];
	} lora_device_t, sx1276_device_t;*/
	lora_dev_sx1276.io=0;
	lora_dev_sx1276.gpio_reg=1;
	lora_dev_sx1276.gpio_bit=2;
		   lora_device_sx1276->user_data = &lora_dev_sx1276;
		   ret = rt_device_register(lora_device_sx1276, name, RT_DEVICE_FLAG_RDWR);
		   if(!ret)
		   	{
			   rt_kprintf("canaan %s driver register OK\n", name);
		   	}
		   else
		   	{
			   rt_kprintf("canaan %s driver register OK\n", name);

		    }
		   
	return ret;	  
		   
/*
    rt_err_t ret = RT_EOK;
    rt_device_t lora_device;
    char name[8];
    char *ptr;

    //ptr = strtok(eeprom_gpio_str, ",");
    sprintf(name, "lora%d", 0);
        //lora_device = &g_eeprom_device[i];
        ret = rt_device_register(lora_device, name, RT_DEVICE_FLAG_RDWR);
        if(!ret)
            rt_kprintf("canaan %s driver register OK\n", name);
        
        lora_device->ops = &lora_ops;

        eeprom_devp[i].io = atoi(ptr);
        ptr = strtok(NULL, ",");
        rt_kprintf("lora_devp[%d].io=%d\n", i, eeprom_devp[i].io);
        eeprom_devp[i].gpio_bit = eeprom_devp[i].io % 32;
        if(eeprom_devp[i].io >= 32) {
            eeprom_devp[i].gpio_reg = GPIO_BASE_ADDR1;
        }
        else {
            eeprom_devp[i].gpio_reg = GPIO_BASE_ADDR0;
        }
        rt_mutex_init(&eeprom_devp[i].mutex, name, RT_IPC_FLAG_PRIO);
        lora_device->user_data = &lora_devp[i];
        CLEAR_BIT(eeprom_devp[i].gpio_reg, eeprom_devp[i].gpio_bit);
 

    return ret;
    */
}

INIT_DEVICE_EXPORT(sx1276_init);

