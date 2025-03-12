BROOKESIA_PATH ?= ${shell pwd}/ui_brookesia

BROOKESIA_CSRCS += $(shell find $(BROOKESIA_PATH)/src -type f -name '*.c')
#BROOKESIA_CSRCS += $(shell find $(LVGL_PATH)/demos -type f -name '*.c')
#BROOKESIA_CSRCS += $(shell find $(LVGL_PATH)/examples -type f -name '*.c')
CXXEXT := .cpp
BROOKESIA_CXXSRCS += $(shell find $(BROOKESIA_PATH)/src -type f -name '*${CXXEXT}')

BROOKESIA_CFLAGS += -I$(BROOKESIA_PATH)
BROOKESIA_CFLAGS += -I${shell pwd}/ui_brookesia/src
BROOKESIA_CXXFLAGS += -I$(BROOKESIA_PATH)
BROOKESIA_CXXFLAGS += -I${shell pwd}/ui_brookesia/src

