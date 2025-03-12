#ifndef __RT_FPIOA_H__
#define __RT_FPIOA_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif
// SX127x physical layer properties
#define RADIOLIB_SX127X_FREQUENCY_STEP_SIZE                     61.03515625
#define RADIOLIB_SX127X_MAX_PACKET_LENGTH                       255
#define RADIOLIB_SX127X_MAX_PACKET_LENGTH_FSK                   64
#define RADIOLIB_SX127X_CRYSTAL_FREQ                            32.0
#define RADIOLIB_SX127X_DIV_EXPONENT                            19

void init();
void soft_spi_init();
uint8_t get_version();





#ifdef __cplusplus
} /*extern "C"*/
#endif
#endif
