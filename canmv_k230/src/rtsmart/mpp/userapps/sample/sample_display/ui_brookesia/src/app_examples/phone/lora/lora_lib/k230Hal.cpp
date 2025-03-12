#include "k230Hal.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
//#include "rt_fpioa.h"
#include "../../../../../../fpioa/rt_fpioa.h"

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
    soft_spi.delay_half=1;
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
















//ArduinoHal::ArduinoHal(): RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING), spi(&RADIOLIB_DEFAULT_SPI), initInterface(true) {}

//ArduinoHal::ArduinoHal(SPIClass& spi, SPISettings spiSettings): RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING), spi(&spi), spiSettings(spiSettings) {}

//ArduinoHal::ArduinoHal(): RadioLibHal(1, 3, 0, 1, 1, 2), initInterface(true) {}

k230Hal::k230Hal():RadioLibHal(GPIO_DM_INPUT, GPIO_DM_OUTPUT, GPIO_WRITE_LOW, GPIO_WRITE_HIGH, GPIO_PE_RISING, GPIO_PE_FALLING) {}

void k230Hal::init() {
  //if(initInterface) {
    //spiBegin();
  //}
 
fpioa_set_function(3,GPIO3,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(4,GPIO4,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(5,GPIO5,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(14,GPIO14,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(15,GPIO15,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(16,GPIO16,-1,-1,-1,-1,-1,-1,-1);
fpioa_set_function(17,GPIO17,-1,-1,-1,-1,-1,-1,-1);
 
  gpio_fd = open("/dev/gpio", O_RDWR);
    if (gpio_fd < 0)
    {
        perror("open /dev/pin err\n");
        return;
    }
  
pin_gpio_t pin_3;
//pin_gpio_t pin_5;
//pin_gpio_t pin_14;
//pin_gpio_t pin_15;
pin_3.pin = 3;
ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_3);  //pin3 output
 // pin_5.pin = 5;//rst
//ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_5);  //pin5 output
 // pin_14.pin = 14;//cs
//ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_14);  //pin14 output
  //pin_15.pin = 15;
//ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_15);  //pin15 output
 
 
 
 //rst
 /*ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_5);
 
 ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_5);
  
 ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_5);
 */
 
 //tcxo_en
  ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_3);
    
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
usleep(ms*1000);//1ms
//rt_thread_mdelay(ms);
//sleep(ms);
//empty
}

void inline k230Hal::delayMicroseconds(RadioLibTime_t us) {
usleep(us);//1us
//empty
}

RadioLibTime_t inline k230Hal::millis() {
//empty
return NULL;
}

RadioLibTime_t inline k230Hal::micros() {
//empty
return NULL;

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
 
 
 
 
 //machine_soft_spi_transfer(fd_soft_spi, send_len, send, recv);
 machine_soft_spi_transfer(fd_soft_spi,len,out,in);
 
 
 
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
}

uint32_t inline k230Hal::pinToInterrupt(uint32_t pin) {
  //return(digitalPinToInterrupt(pin));
  return 0;
}
