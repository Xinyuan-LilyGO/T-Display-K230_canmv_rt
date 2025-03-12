
include $(MPP_SRC_DIR)/kernel/mpp.mk
include $(MPP_SRC_DIR)/kernel/rt-smart.mk

include $(SDK_SRC_ROOT_DIR)/.config

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
LIB = $(MPP_SRC_DIR)/kernel/lib/lib$(CURRECT_DIR_NAME).a

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)/src/ \
			   -I$(RTSMART_SRC_DIR)/kernel/bsp/maix3/board/interdrv/gpio

obj-y := src/sensor_comm.o src/sensor_dev.o
obj-$(CONFIG_MPP_ENABLE_SENSOR_GC2093) += src/gc2093/gc2093.o
obj-$(CONFIG_MPP_ENABLE_SENSOR_OV5647) += src/ov5647/ov5647.o
obj-$(CONFIG_MPP_ENABLE_SENSOR_IMX335) += src/imx335/imx335.o

OBJS = $(obj-y)

all: $(LIB)
	@-rm -f $(OBJS)
	echo "${PWD}/Makefile all"

$(OBJS): %.o : %.c
	@$(CC) $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_DRIVER_CFLGAS) -c $< -o $@

$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

clean:
	echo "${PWD}/Makefile clean"
	-rm -rf $(LIB)
	-rm -f $(OBJS)

.PHONY: all clean
