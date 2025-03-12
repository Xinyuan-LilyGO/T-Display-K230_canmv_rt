#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>


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




 




int main(int argc, char* argv[]) {       
    int fd = open("/dev/ws2812", O_RDWR);
    if (fd < 0) {
        perror("open /dev/ws2812");
        return -1;
    }
     //ws2812_value led_rgb;
   //led_rgb.value=0x00ff00ff;
    ws2812_value led_rgb;
    //uint8_t green;
    //uint8_t red;
    //uint8_t blue;  
  while(1)
  {
  for(uint8_t r=0;r<255;r++)
    {
     for(uint8_t g=0;g<255;g++)
     {
      for(uint8_t b=0;b<255;b++)
      {
      led_rgb.value=(g<<16|r<<8|b);
      ioctl(fd,0x00,&led_rgb);
      sleep(1);
      }
    }
  }  
}
    close(fd);
    return 0;
}
