#include <stdio.h>

#include "k_sensor_comm.h"

#define MIRROR  (1)
#define FLIP    (2)

struct sensor_type_mirror_t {
  k_vicap_sensor_type type;
  k_u32 mirror;
};

#if defined(CONFIG_BOARD_K230_CANMV)
static struct sensor_type_mirror_t type_mirror_tbl[] = {
#if defined (CONFIG_MPP_ENABLE_SENSOR_OV5647)
    {.type = OV5647_MIPI_CSI0_2592x1944_10FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_1280X960_45FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_1280X720_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_640x480_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = OV5647_MIPI_CSI1_2592x1944_10FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_1280X960_45FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_1280X720_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_640x480_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = OV5647_MIPI_CSI2_2592x1944_10FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_1280X960_45FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_1280X720_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_640x480_90FPS_10BIT_LINEAR, .mirror = 0},
#endif // CONFIG_MPP_ENABLE_SENSOR_OV5647

#if defined (CONFIG_MPP_ENABLE_SENSOR_GC2093)
    {.type = GC2093_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI0_1920X1080_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI0_1280X960_90FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI0_1280X720_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = GC2093_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI1_1920X1080_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI1_1280X960_90FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI1_1280X720_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI2_1920X1080_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI2_1280X960_90FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI2_1280X720_90FPS_10BIT_LINEAR, .mirror = 0},
#endif // CONFIG_MPP_ENABLE_SENSOR_GC2093
};
#elif defined(CONFIG_BOARD_K230_CANMV_V2)
static struct sensor_type_mirror_t type_mirror_tbl[] = {
    // {.type =, .mirror = },
};
#elif defined(CONFIG_BOARD_K230D_CANMV)
static struct sensor_type_mirror_t type_mirror_tbl[] = {
    // {.type =, .mirror = },
};
#elif defined(CONFIG_BOARD_K230_CANMV_01STUDIO)
static struct sensor_type_mirror_t type_mirror_tbl[] = {
#if defined (CONFIG_MPP_ENABLE_SENSOR_OV5647)
    {.type = OV5647_MIPI_CSI0_2592x1944_10FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_1280X960_45FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_1280X720_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI0_640x480_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = OV5647_MIPI_CSI1_2592x1944_10FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_1280X960_45FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_1280X720_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI1_640x480_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = OV5647_MIPI_CSI2_2592x1944_10FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_1280X960_45FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_1280X720_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = OV5647_MIPI_CSI2_640x480_90FPS_10BIT_LINEAR, .mirror = 0},
#endif // CONFIG_MPP_ENABLE_SENSOR_OV5647

#if defined (CONFIG_MPP_ENABLE_SENSOR_GC2093)
    {.type = GC2093_MIPI_CSI0_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI0_1920X1080_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI0_1280X960_90FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI0_1280X720_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = GC2093_MIPI_CSI1_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI1_1920X1080_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI1_1280X960_90FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI1_1280X720_90FPS_10BIT_LINEAR, .mirror = 0},

    {.type = GC2093_MIPI_CSI2_1920X1080_30FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI2_1920X1080_60FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI2_1280X960_90FPS_10BIT_LINEAR, .mirror = 0},
    {.type = GC2093_MIPI_CSI2_1280X720_90FPS_10BIT_LINEAR, .mirror = 0},
#endif // CONFIG_MPP_ENABLE_SENSOR_GC2093
};
#elif defined(CONFIG_BOARD_K230_CANMV_DONGSHANPI)
static struct sensor_type_mirror_t type_mirror_tbl[] = {
    // {.type =, .mirror = },
};
#elif defined(CONFIG_BOARD_K230D_CANMV_BPI_ZERO)
static struct sensor_type_mirror_t type_mirror_tbl[] = {
    // {.type =, .mirror = },
};
#endif

#if defined(CONFIG_BOARD_K230_CANMV) || defined(CONFIG_BOARD_K230_CANMV_V2) || \
    defined(CONFIG_BOARD_K230D_CANMV) ||                                       \
    defined(CONFIG_BOARD_K230_CANMV_01STUDIO) ||                               \
    defined(CONFIG_BOARD_K230_CANMV_DONGSHANPI) || \
    defined(CONFIG_BOARD_K230D_CANMV_BPI_ZERO)

k_u32 get_mirror_by_sensor_type(k_vicap_sensor_type type) {
  k_u32 mirror = 0;

  struct sensor_type_mirror_t *p = NULL;

  size_t count = (sizeof(type_mirror_tbl) / sizeof(type_mirror_tbl[0]));

  for (size_t i = 0; i < count; i++) {
    p = &type_mirror_tbl[i];

    if (type == p->type) {
      mirror = p->mirror;
      break;
    }
  }

  return mirror;
}
#else
k_u32 get_mirror_by_sensor_type(k_vicap_sensor_type type) {
  printf("unsupport board type %s\n", CONFIG_BOARD_NAME);

  return 0;
}
#endif
