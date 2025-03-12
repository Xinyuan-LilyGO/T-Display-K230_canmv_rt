#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
//#include "rt_fpioa.h"
#include "sx1276.h"

int main(int argc, char* argv[]) { 
      init();
      soft_spi_init();
   char version=get_version();  
   printf("data_version:%02x\n", version);
    
    return 0;
}
