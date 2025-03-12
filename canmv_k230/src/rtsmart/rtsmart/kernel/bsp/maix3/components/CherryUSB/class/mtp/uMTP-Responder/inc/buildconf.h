#define _LARGEFILE64_SOURCE
#define _FILE_OFFSET_BITS 64

#define CONFIG_READ_FILE_BUFFER_SIZE  (256*1024)  // Must be a 2^x value.
#define CONFIG_MAX_TX_USB_BUFFER_SIZE (1024)    // Must be a multiple of 512 and be less than CONFIG_READ_FILE_BUFFER_SIZE
#define CONFIG_MAX_RX_USB_BUFFER_SIZE (1024)    // Must be a multiple of 512

#include "custom_buildconf.h"
