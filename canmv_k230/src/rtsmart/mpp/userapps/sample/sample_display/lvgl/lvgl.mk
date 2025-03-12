LVGL_PATH ?= ${shell pwd}/lvgl

#ASRCS += $(shell find $(LVGL_PATH)/src -type f -name '*.S')
LVGL_CSRCS += $(shell find $(LVGL_PATH)/src -type f -name '*.c')
#LVGL_CSRCS += $(shell find $(LVGL_PATH)/demos -type f -name '*.c')
#LVGL_CSRCS += $(shell find $(LVGL_PATH)/examples -type f -name '*.c')
#CXXEXT := .cpp
#CXXSRCS += $(shell find $(LVGL_PATH)/src/libs/thorvg -type f -name '*${CXXEXT}')

#AFLAGS += "-I$(LVGL_PATH)"
#LVGL_CFLAGS += "-I$(LVGL_PATH)"
#CXXFLAGS += "-I$(LVGL_PATH)"
LVGL_CFLAGS += -I$(LVGL_PATH)
