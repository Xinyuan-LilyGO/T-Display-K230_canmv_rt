#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include "rt_fpioa.h"
#include "sx1276.h"


#define KD_GPIO_HIGH     1
#define KD_GPIO_LOW      0

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


typedef struct kd_pin_mode
{
    unsigned short pin;     /* pin number, from 0 to 63 */
    unsigned short mode;    /* pin level status, 0 low level, 1 high level */
} pin_mode_t;











int gpio_fd;
int fd_soft_spi;
//pin_mode_t pin_14;
#define RT_SPI_DEV_CTRL_CONFIG (0xc00 + 0x01)
#define RT_SPI_DEV_CTRL_RW (0xc00 + 0x02)
#define RT_SPI_DEV_CTRL_CLK (0xc00 + 0x03)
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
	

/*
void machine_soft_spi_read(int fd,) {
    vstr_t vstr;
    vstr_init_len(&vstr, mp_obj_get_int(args[1]));
    memset(vstr.buf, n_args == 3 ? mp_obj_get_int(args[2]) : 0, vstr.len);
    machine_soft_spi_transfer(fd, vstr.len, vstr.buf,vstr.buf);
    return;
}

void  machine_soft_spi_readinto(int fd, char* send,int send_len) {
    machine_soft_spi_transfer(fd, send_len, send, send);    
    return;
}
*/
void machine_soft_spi_write_byte(int fd, uint8_t send) {
    pin_mode_t pin_14;
    pin_14.pin=14;
    ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_14);   
    machine_soft_spi_transfer(fd, 1, &send, NULL); 
    ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_14);   
    return ;
}

void machine_soft_spi_write(int fd, uint8_t* send,int send_len) {
    pin_mode_t pin_14;
    pin_14.pin=14;
    ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_14);
    machine_soft_spi_transfer(fd, send_len, send, NULL);
    ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_14);
    return ;
}

void machine_soft_spi_write_readinto(int fd, uint8_t* send,int send_len,uint8_t*recv,int recv_len) {
    pin_mode_t pin_14;
    pin_14.pin=14;
    ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_14);
    machine_soft_spi_transfer(fd, send_len, send, recv);
    ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_14);   
    return;
}	


void init()
{

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
 
 pin_mode_t pin_3;
 pin_mode_t pin_5;
 pin_mode_t pin_14;
 pin_mode_t pin_15;
 pin_3.pin = 3;
ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_3);  //pin3 output
  pin_5.pin = 5;
ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_5);  //pin5 output
  pin_14.pin = 14;
ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_14);  //pin14 output
  pin_15.pin = 15;
ioctl(gpio_fd, GPIO_DM_OUTPUT, &pin_15);  //pin15 output
 
 
 
 //rst
 ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_5);
 
 ioctl(gpio_fd, GPIO_WRITE_LOW, &pin_5);
  
 ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_5);
 
 
 //tcxo_en
  ioctl(gpio_fd, GPIO_WRITE_HIGH, &pin_3);
    
}

void soft_spi_init()
{
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


uint8_t get_version()
{    
    uint8_t cmd[2]={0x42,0xff};
    uint8_t data[2]={0xff,0xff};
    
    machine_soft_spi_write_readinto(fd_soft_spi,cmd,2,data,2);
    return data[1];
}

uint8_t get_mode()
{
    uint8_t cmd[2]={0x01,0xff};
    uint8_t data[2]={0xff,0xff};
    machine_soft_spi_write_readinto(fd_soft_spi,cmd,2,data,2);
    
    return data[1];
}
void set_mode(uint8_t _cmd,uint8_t _data)
{

  /*uint8_t currentValue = SPIreadRegister(reg);
  uint8_t mask = ~((0b11111111 << (msb + 1)) | (0b11111111 >> (8 - lsb)));
  uint8_t newValue = (currentValue & ~mask) | (value & mask);
  SPIwriteRegister(reg, newValue);*/
  
  machine_soft_spi_write_byte(fd_soft_spi,_cmd|(1<<7));
  machine_soft_spi_write_byte(fd_soft_spi,_data);
}

void set_frequency(float newFreq) {
  

  // calculate register values
  //uint32_t FRF = (newFreq * (uint32_t(1) << 14)) / RADIOLIB_SX127X_CRYSTAL_FREQ;
   uint32_t FRF = (newFreq * ((uint32_t)(1<< 14))) / 1000000;
  // write registers
  machine_soft_spi_write_byte(fd_soft_spi,0x06);
  machine_soft_spi_write_byte(fd_soft_spi,(uint8_t)((FRF & 0xFF0000) >> 16));
  
  
  machine_soft_spi_write_byte(fd_soft_spi,0x07);
  machine_soft_spi_write_byte(fd_soft_spi,(uint8_t)((FRF & 0x00FF00) >> 8));
  
  
  machine_soft_spi_write_byte(fd_soft_spi,0x08);
  machine_soft_spi_write_byte(fd_soft_spi,(uint8_t)(FRF & 0x0000FF));
  
}

void set_sf(float new_sf)
{












}






























