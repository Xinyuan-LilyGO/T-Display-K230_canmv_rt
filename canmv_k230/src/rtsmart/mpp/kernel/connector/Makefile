
include $(MPP_SRC_DIR)/kernel/mpp.mk
include $(MPP_SRC_DIR)/kernel/rt-smart.mk

include $(SDK_SRC_ROOT_DIR)/.config

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
LIB = $(MPP_SRC_DIR)/kernel/lib/lib$(CURRECT_DIR_NAME).a

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)/src/ \
			   -I$(RTSMART_SRC_DIR)/kernel/bsp/maix3/board/interdrv/gpio

obj-y := src/connector_comm.o src/connector_dev.o

obj-$(CONFIG_MPP_DSI_ENABLE_VIRT) += src/virtdev.o

obj-$(CONFIG_MPP_DSI_ENABLE_HDMI_LT9611) += src/lt9611.o
obj-$(CONFIG_MPP_DSI_ENABLE_HDMI_HX8399) += src/hx8399.o
obj-$(CONFIG_MPP_DSI_ENABLE_LCD_RM69A10) += src/rm69a10.o
obj-$(CONFIG_MPP_DSI_ENABLE_LCD_ST7701) += src/st7701.o
obj-$(CONFIG_MPP_DSI_ENABLE_LCD_ILI9806) += src/ili9806.o

OBJS = $(obj-y)

all: $(LIB)
	@-rm -f $(OBJS)
	echo "${PWD}/Makefile all"

$(OBJS): %.o : %.c
	@$(CC) $(CC_CFLAGS) -Wno-error $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_DRIVER_CFLGAS) -c $< -o $@

$(LIB): $(OBJS)
	$(AR) $(ARFLAGS) $@ $(OBJS)

clean:
	echo "${PWD}/Makefile clean"
	-rm -rf $(LIB)
	-rm -f $(OBJS)

.PHONY: all clean


