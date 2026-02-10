#include "k230Hal.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "cconfig.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "../../../../../../fpioa/rt_fpioa.h"
#define USE_SPI_HW
//#define USE_SPI_SOFT
#define TIMER_CLK_FREQ 27000000
/* ioctl */

#define	GPIO_DM_OUTPUT           _IOW('G', 0, int)
#define	GPIO_DM_INPUT            _IOW('G', 1, int)
#define	GPIO_DM_INPUT_PULL_UP    _IOW('G', 2, int)
#define	GPIO_DM_INPUT_PULL_DOWN  _IOW('G', 3, int)
#define	GPIO_WRITE_LOW           _IOW('G', 4, int)
#define	GPIO_WRITE_HIGH          _IOW('G', 5, int)

#define	GPIO_PE_RISING           _IOW('G', 7, int)
#define	GPIO_PE_FALLING          _IOW('G', 8, int)
#define	GPIO_PE_BOTH             _IOW('G', 9, int)
#define	GPIO_PE_HIGH             _IOW('G', 10, int)
#define	GPIO_PE_LOW              _IOW('G', 11, int)

#define GPIO_READ_VALUE       	_IOW('G', 12, int)

#define RT_SPI_DEV_CTRL_CONFIG (0xc00 + 0x01)
#define RT_SPI_DEV_CTRL_RW (0xc00 + 0x02)
#define RT_SPI_DEV_CTRL_CLK (0xc00 + 0x03)
struct rt_spi_configuration {
    uint8_t mode;
    uint8_t data_width;
    uint16_t reserved;
    uint32_t max_hz;
};

enum {
    DWENUM_SPI_TXRX = 0,
    DWENUM_SPI_TX = 1,
    DWENUM_SPI_RX = 2,
    DWENUM_SPI_EEPROM = 3
};







struct rt_spi_priv_data {
    const void *send_buf;
    size_t send_length;
    void *recv_buf;
    size_t recv_length;
};
enum {
    MP_SPI_IOCTL_INIT,
    MP_SPI_IOCTL_DEINIT,
    MP_SPI_IOCTL_TRANSFER,
};
typedef struct _soft_spi_obj_t {
    uint32_t delay_half; // microsecond delay for half SCK period
    uint8_t polarity;
    uint8_t phase;
    uint8_t sck;
    uint8_t mosi;
    uint8_t miso;
} soft_spi_obj_t;


void machine_soft_spi_init(int fd,uint8_t polarity,uint8_t phase,uint8_t sck,uint8_t mosi,uint8_t miso) {		
    soft_spi_obj_t soft_spi;
    soft_spi.delay_half=0;//1
    soft_spi.polarity=polarity;
    soft_spi.phase=phase;
    soft_spi.sck=sck;
    soft_spi.mosi=mosi;
    soft_spi.miso=miso;
    ioctl(fd,MP_SPI_IOCTL_INIT,&soft_spi);
    }

void machine_soft_spi_transfer(int fd, size_t len, const void *src, void *dest) {

		struct rt_spi_priv_data *priv_data = (struct rt_spi_priv_data *)malloc(sizeof(struct rt_spi_priv_data));
		priv_data->send_buf = src;
		priv_data->send_length = len;
		priv_data->recv_buf = dest;
		ioctl(fd, MP_SPI_IOCTL_TRANSFER, priv_data);	
		free(priv_data);   
	}





static uint64_t perf_get_times(void)
{
    uint64_t cnt;
    __asm__ __volatile__(
        "rdtime %0"
        : "=r"(cnt));
    return cnt;
}

void k230_delay_us(uint64_t us)
{
    uint64_t delay = (TIMER_CLK_FREQ / 1000000) * us;
    volatile uint64_t cur_time = perf_get_times();
    while (1) {
        if ((perf_get_times() - cur_time) >= delay)
            break;
    }
}

typedef struct hw_spi_obj_t {
    uint32_t baud;
    uint8_t polarity;
    uint8_t phase;
    uint8_t bits;
    uint8_t reserver;

} hw_spi_obj_t;
hw_spi_obj_t hw_spi;
void machine_hw_spi_init(int fd,uint8_t polarity,uint8_t phase,uint32_t baud) {		 
    hw_spi.polarity=polarity;
    hw_spi.phase=phase;
    hw_spi.baud=baud;
    hw_spi.bits=8;
    }

void machine_hw_spi_transfer(int fd, size_t len, const void *src, void *dest) {

		struct rt_spi_priv_data *priv_data = (struct rt_spi_priv_data *)malloc(sizeof(struct rt_spi_priv_data));
		priv_data->send_buf = src;
		priv_data->send_length = len;
		priv_data->recv_buf = dest;
		priv_data->recv_length=len;
		 if (dest == NULL) {
        struct rt_spi_configuration cfg = {
            .mode = DWENUM_SPI_TX,
            .data_width = hw_spi.bits,
            .max_hz = hw_spi.baud,
        };
        ioctl(fd, RT_SPI_DEV_CTRL_CONFIG, &cfg);
        write(fd, src, len);
    } else if ((src == NULL) && (dest != NULL)) {
        struct rt_spi_configuration cfg = {
            .mode = DWENUM_SPI_RX,
            .data_width = hw_spi.bits,
            .max_hz = hw_spi.baud,
        };
        ioctl(fd, RT_SPI_DEV_CTRL_CONFIG, &cfg);
        read(fd, dest,len);
    }else {
        struct rt_spi_configuration cfg = {
            .mode = DWENUM_SPI_EEPROM,
            .data_width = hw_spi.bits,
            .max_hz = hw_spi.baud,
        };
        ioctl(fd, RT_SPI_DEV_CTRL_CONFIG, &cfg);      
        ioctl(fd, RT_SPI_DEV_CTRL_RW, priv_data);
    }
    //printf("fd:%d,dest[0]:%02X,dest[1]:%02X",fd,(((uint8_t*)(priv_data->recv_buf))[0]),(((uint8_t*)(priv_data->recv_buf))[1]));
    //printf("fd:%d,dest[0]:%02X,dest[1]:%02X",fd,((uint8_t*)dest)[0],((uint8_t*)dest)[1]);
		free(priv_data);   
	}

void machine_hw_spi_write_readinto(int fd, char* send,int send_len,char*recv,int recv_len) {
    machine_hw_spi_transfer(fd, send_len, send, recv);
    return;
}


//ArduinoHal::ArduinoHal(): RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING), spi(&RADIOLIB_DEFAULT_SPI), initInterface(true) {}

//ArduinoHal::ArduinoHal(SPIClass& spi, SPISettings spiSettings): RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING), spi(&spi), spiSettings(spiSettings) {}

//ArduinoHal::ArduinoHal(): RadioLibHal(1, 3, 0, 1, 1, 2), initInterface(true) {}

k230Hal::k230Hal():RadioLibHal(GPIO_DM_INPUT, GPIO_DM_OUTPUT, GPIO_WRITE_LOW, GPIO_WRITE_HIGH, GPIO_PE_RISING, GPIO_PE_FALLING) {}

void k230Hal::init() {
  
fpioa_set_function(5,GPIO5,-1,-1,-1,-1,-1,-1,-1);
#if defined(USE_SPI_SOFT)
fpioa_set_function(14,GPIO14,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(15,GPIO15,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(16,GPIO16,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(17,GPIO17,-1,-1,-1,-1,-1,-1,-1);
#endif

#if defined(USE_SPI_HW)
fpioa_set_function(14,QSPI0_CS0,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(15,QSPI0_CLK,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(16,QSPI0_D0,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(17,QSPI0_D1,-1,-1,-1,-1,-1,-1,-1);
#endif



fpioa_set_function(19,GPIO19,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(20,GPIO20,-1,-1,-1,-1,-1,-1,-1);
 
  gpio_fd = open("/dev/gpio", O_RDWR);
    if (gpio_fd < 0)
    {
        perror("open /dev/pin err\n");
        return;
    }
 
  pin_gpio_t pin_19;
  pin_19.pin = 19;
  ioctl(gpio_fd, GPIO_DM_INPUT, &pin_19);  //pin3 output
  
  
  //rst
   pin_gpio_t pin_5;
  pin_19.pin = 5;
  ioctl(gpio_fd, GPIO_DM_INPUT, &pin_5);  //pin3 output
 ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_5);
 usleep(10);
 ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_5);
 usleep(10);
 ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_5);
    #if defined(USE_SPI_SOFT)
    int index=0; 
    char dev_name[16] = "/dev/soft_spi0";
    //dev_name[8] = '0' + index;
    dev_name[13] = '0' + index;
    fd_soft_spi = open(dev_name, O_RDWR);
    if (fd_soft_spi < 0) {
        perror("open /dev/soft_spi0");
        return ;
    }  
   machine_soft_spi_init(fd_soft_spi,0,0,15,16,17);
  #endif
  
  #if defined(USE_SPI_HW)
   int index=1; 
    char dev_name[16] = "/dev/spi0";
    dev_name[8] = '0' + index;
    fd_hw_spi = open(dev_name, O_RDWR);
    if (fd_hw_spi < 0) {
        perror("open /dev/spi0");
        return;
    }  
    //printf("open success\n");
   machine_hw_spi_init(fd_hw_spi,0,0,5000000);
  #endif
  
  
  //test
  
    /*char cmd_read_version0[2]={0x01,0x1C};
    char data_version0[2]={0xff,0xff};
    
    machine_hw_spi_transfer(fd_hw_spi, 2, cmd_read_version0, data_version0);
    
    printf("data_version0:%02x,%02x\n", data_version0[0],data_version0[1]);
    
     char cmd_read_version1[4]={0x00,0x00,0x00,0x00};
    char data_version1[4]={0xff,0xff,0xff,0xff};   
    machine_hw_spi_transfer(fd_hw_spi,4,cmd_read_version1,data_version1);
    printf("data_version1:%02x,%02x,%02x,%02x\n", data_version1[0],data_version1[1],data_version1[2],data_version1[3]);*/
  
  
  
  
   
}

void k230Hal::term() {
  //if(initInterface) {
    //spiEnd();
  //}
}

void inline k230Hal::pinMode(uint32_t pin, uint32_t mode) {
    pin_gpio_t pin_opt;
    pin_opt.pin=pin;
    ioctl(gpio_fd, mode, &pin_opt);
}

void inline k230Hal::digitalWrite(uint32_t pin, uint32_t value) {
    pin_gpio_t pin_opt;
    pin_opt.pin=pin;
    if(value==GPIO_WRITE_HIGH)
    {
    ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_opt);
    }
    else if(value==GPIO_WRITE_LOW)
    {
    ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_opt);
    }
}

uint32_t inline k230Hal::digitalRead(uint32_t pin) {
    pin_gpio_t pin_opt;
    pin_opt.pin=pin;
    ioctl(gpio_fd, GPIO_READ_VALUE, &pin_opt);
    return pin_opt.val;
}

void inline k230Hal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) {
 //empty
}

void inline k230Hal::detachInterrupt(uint32_t interruptNum) {
 //empty
}

void inline k230Hal::delay(RadioLibTime_t ms) {
//usleep(ms*1000);//1ms
k230_delay_us(ms*1000);
//rt_thread_mdelay(ms);
//sleep(ms);
//empty
}

void inline k230Hal::delayMicroseconds(RadioLibTime_t us) {
//usleep(us);//1us
k230_delay_us(us);
//empty
}

RadioLibTime_t inline k230Hal::millis() {
//empty
volatile uint64_t cur_time = perf_get_times();
return (cur_time*1000)/TIMER_CLK_FREQ;
}

RadioLibTime_t inline k230Hal::micros() {
//empty
volatile uint64_t cur_time = perf_get_times();
return (cur_time*1000*1000)/TIMER_CLK_FREQ;
}

long inline k230Hal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) {
 //empty
  return 0;
}

void inline k230Hal::spiBegin() {

  //spi->begin();
}

void inline k230Hal::spiBeginTransaction() {
  //spi->beginTransaction(spiSettings);
}
void k230Hal::spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
 
 
  #if defined(USE_SPI_SOFT)
 //machine_soft_spi_transfer(fd_soft_spi, send_len, send, recv);
 machine_soft_spi_transfer(fd_soft_spi,len,out,in);
 #endif
 
  #if defined(USE_SPI_HW) 
  machine_hw_spi_transfer(fd_hw_spi,len,out,in);
  #endif 
}

void inline k230Hal::spiEndTransaction() {
  //spi->endTransaction();
}

void inline k230Hal::spiEnd() {
  //spi->end();
}

void inline k230Hal::tone(uint32_t pin, unsigned int frequency, RadioLibTime_t duration) {
  
}

void inline k230Hal::noTone(uint32_t pin) {
 
}

void inline k230Hal::yield() {
  #if !defined(RADIOLIB_YIELD_UNSUPPORTED)
  //::yield();
  #endif
  //rt_thread_yield();
  k230_delay_us(1000);
}

uint32_t inline k230Hal::pinToInterrupt(uint32_t pin) {
  //return(digitalPinToInterrupt(pin));
  return 0;
}
