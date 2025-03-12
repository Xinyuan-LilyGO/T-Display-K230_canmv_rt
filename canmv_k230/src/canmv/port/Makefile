include ${SDK_SRC_ROOT_DIR}/.config

CROSS_COMPILE=$(SDK_TOOLCHAIN_DIR)/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin/riscv64-unknown-linux-musl-

BOARD_DIR = boards/${CONFIG_BOARD}
BUILD = ${SDK_CANMV_BUILD_DIR}

include ../micropython/py/mkenv.mk

# Use the default frozen manifest, variants may override this.
FROZEN_MANIFEST ?= ${BOARD_DIR}/manifest.py
include boards/mpconfigport.mk
-include ${BOARD_DIR}/mpconfigport.mk

# This should be configured by the mpconfigvariant.mk
PROG ?= micropython

USER_C_MODULES = 3rd-party/ulab

# include py core make definitions
include $(TOP)/py/py.mk 
include $(TOP)/extmod/extmod.mk

include $(SDK_RTSMART_SRC_DIR)/librtsmart.mk
include $(SDK_RTSMART_SRC_DIR)/libmpp.mk
include $(SDK_RTSMART_SRC_DIR)/libopencv.mk
include $(SDK_RTSMART_SRC_DIR)/libnncase.mk

-include $(BUILD)/CANMV_VER
CANMV_VER = $(shell git describe --tags --always `git rev-list --tags --max-count=1`)-$(shell git rev-parse --short HEAD)
ifneq ($(CANMV_VER),$(CANMV_VER_OLD))
$(shell echo CANMV_VER_OLD=$(CANMV_VER) > $(BUILD)/CANMV_VER)
$(shell touch core/main.c)
endif
CDEF = -DCANMV_VER="\"$(CANMV_VER)\""

INC += -I.
INC += -I$(TOP)
INC += -I$(TOP)/py
INC += -I$(TOP)/shared/readline
INC += -I$(TOP)/lib/crypto-algorithms
INC += -I$(BUILD)
INC += -I$(SDK_SRC_ROOT_DIR)/include
INC += -I$(BOARD_DIR)
INC += -Iinclude/ai_cube
INC += -Iinclude/ai_demo
INC += -Iinclude/ai_demo/kws
INC += -Iinclude/ai_demo/tts_zh
INC += -Iinclude/core
INC += -Iinclude/kpu
INC += -Iinclude/machine
INC += -Iinclude/maix
INC += -Iinclude/mpp
INC += -Iinclude/multi_media
INC += -Iinclude/omv
INC += -Iomv/alloc
INC += -Iomv/boards/canmv
INC += -Iomv/common
INC += -Iomv/imlib
INC += -Iomv/modules
INC += -I3rd-party/ulab
INC += -I3rd-party/wrap
INC += -I3rd-party/lv_bindings
INC += -I$(SDK_CANMV_BUILD_DIR)/freetype/include
INC += -I$(SDK_RTSMART_SRC_DIR)/mpp/middleware/src/rtsp_server/include
INC += -I$(SDK_RTSMART_SRC_DIR)/mpp/middleware/src/mp4_format/include
INC += $(SDK_MPP_USER_INC) $(SDK_RTSMART_INC) $(NNCASE_INC) $(OPENCV_INC)

# compiler settings
# Debugging/Optimization
ifdef DEBUG
COPT ?= -Og
else
COPT ?= -Os
COPT += -DNDEBUG
endif

# Remove unused sections.
COPT += -fdata-sections -ffunction-sections

# Always enable symbols -- They're occasionally useful, and don't make it into the
# final .bin/.hex/.dfu so the extra size doesn't matter.
CFLAGS += -g -gdwarf-2

CWARN = -Wall -Werror
CWARN += -Wno-int-conversion
CFLAGS += -fopenmp -march=rv64imafdcv -mabi=lp64d -mcmodel=medany
CFLAGS += $(INC) -std=gnu99 $(COPT) $(CFLAGS_EXTRA) $(CDEF)

# Workaround gcc 12 bug.
CFLAGS += -Wno-array-bounds

ifeq ($(MICROPY_PY_TERMIOS),1)
CFLAGS += -DMICROPY_PY_TERMIOS=1
endif
CFLAGS += -DHAVE_CCONFIG_H

CXXFLAGS += -fopenmp -march=rv64imafdcv -mabi=lp64d -mcmodel=medany
CXXFLAGS += $(INC) $(COPT) $(NNCASEINC) -std=c++20

# source files
SRC_C += \
	core/main.c \
	core/gccollect.c \
	core/mphalport.c \
	core/input.c \
	core/mpthreadport.c \
	core/mpp_vb_mgmt.c \
	$(wildcard $(BOARD_DIR)/*.c)

SHARED_SRC_C += $(addprefix shared/,\
	runtime/gchelper_generic.c \
	timeutils/timeutils.c \
	readline/readline.c \
	runtime/pyexec.c \
	netutils/netutils.c \
	)

MACHINE_SRC_C = $(wildcard machine/*.c)
MAIX_SRC_C = $(wildcard maix/*.c)
MPP_SRC_C = $(wildcard builtin_py/mpp_binding/*.c)
KPU_SRC_C = $(wildcard kpu/*.c)
AICUBE_SRC_C = $(wildcard ai_cube/*.c)
AIDEMO_SRC_C = $(wildcard ai_demo/*.c)
MODULES_SRC_C = $(wildcard modules/*.c)
MODULES_SRC_C += $(wildcard modules/*/*.c)
OMV_SRC_C = $(wildcard omv/*.c)
OMV_SRC_C += $(wildcard omv/*/*.c)
MULTIMEDIA_SRC_C = $(wildcard multi_media/*.c)

NETWORK_SRC_C :=
ifneq ($(filter y,$(CONFIG_ENABLE_NETWORK_RT_WLAN) $(CONFIG_ENABLE_NETWORK_RT_LAN)),)
    NETWORK_SRC_C += $(wildcard network/*.c)
endif

# LVGL build rules
LVGL_BINDING_DIR = 3rd-party/lv_bindings
LVGL_DIR = $(LVGL_BINDING_DIR)/lvgl
ALL_LVGL_SRC = $(shell find $(LVGL_DIR) -type f -name '*.h') $(LVGL_BINDING_DIR)/lv_conf.h
LVGL_PP = $(BUILD)/lvgl/lvgl.pp.c
LVGL_MPY = $(BUILD)/lvgl/lv_mpy.c
LVGL_MPY_METADATA = $(BUILD)/lvgl/lv_mpy.json

$(LVGL_MPY): $(ALL_LVGL_SRC) $(LVGL_BINDING_DIR)/gen_mpy.py
	$(ECHO) "LVGL-GEN $@"
	$(Q)mkdir -p $(dir $@)
	$(Q)$(CPP) $(LV_CFLAGS) -DPYCPARSER -x c -I $(LVGL_BINDING_DIR)/pycparser/utils/fake_libc_include $(INC) $(LVGL_DIR)/lvgl.h > $(LVGL_PP)
	$(Q)python $(LVGL_BINDING_DIR)/gen_mpy.py -M lvgl -MP lv -MD $(LVGL_MPY_METADATA) -E $(LVGL_PP) $(LVGL_DIR)/lvgl.h > $@

LVGL_SRC_C_ALL = $(subst $(TOP)/,,$(shell find $(LVGL_DIR)/src -type f -name "*.c") $(LVGL_MPY))
LVGL_SRC_C_EXCLUDE = \
	$(addprefix $(LVGL_DIR)/src/,\
		hal/lv_hal_tick.c \
		libs/png/lodepng.c \
		libs/png/lv_png.c \
	)
LVGL_SRC_C := $(filter-out $(LVGL_SRC_C_EXCLUDE),$(LVGL_SRC_C_ALL))
LVGL_SRC_C += $(LVGL_BINDING_DIR)/src/hal/lv_hal_tick.c
LVGL_SRC_C += $(LVGL_BINDING_DIR)/src/libs/png/lodepng.c
LVGL_SRC_C += $(LVGL_BINDING_DIR)/src/libs/png/lv_png.c

# freetype build
$(SDK_CANMV_BUILD_DIR)/_ft.c:
	@$(MAKE) -C 3rd-party/freetype
	@echo "// place holder" > $(SDK_CANMV_BUILD_DIR)/_ft.c

FREETYPE_C = $(SDK_CANMV_BUILD_DIR)/_ft.c
CLEAN_EXTRA += $(FREETYPE_C) $(LVGL_PP) $(LVGL_MPY)

SRC_C += \
	${MACHINE_SRC_C} \
	${MAIX_SRC_C} \
	${MPP_SRC_C} \
	${KPU_SRC_C} \
	${NETWORK_SRC_C} \
	${OMV_SRC_C} \
	${AICUBE_SRC_C} \
	${AIDEMO_SRC_C} \
	${MODULES_SRC_C} \
	${LVGL_SRC_C} \
	${FREETYPE_C} \
	${MULTIMEDIA_SRC_C}

KPU_SRC_CXX = $(wildcard kpu/*.cpp)
AICUBE_SRC_CXX = $(wildcard ai_cube/*.cpp)
AIDEMO_SRC_CXX = $(wildcard ai_demo/*.cpp)
AIDEMO_KWS_SRC_CXX = $(wildcard ai_demo/kws/*.cpp)
AIDEMO_TTS_ZH_SRC_CXX = $(wildcard ai_demo/tts_zh/*.cpp)
MODULES_SRC_CXX = $(wildcard modules/*.cpp)
MODULES_SRC_CXX += $(wildcard modules/*/*.cpp)
MULTIMEDIA_SRC_CXX = $(wildcard multi_media/*.cpp)

SRC_CXX += \
	${KPU_SRC_CXX} \
	${AICUBE_SRC_CXX} \
	${AIDEMO_SRC_CXX} \
	${AIDEMO_KWS_SRC_CXX} \
	${AIDEMO_TTS_ZH_SRC_CXX} \
	${MODULES_SRC_CXX} \
	${MULTIMEDIA_SRC_CXX}

OBJ = $(PY_O)
OBJ += $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_CXX:.cpp=.o))
OBJ += $(addprefix $(BUILD)/, $(SHARED_SRC_C:.c=.o))

# List of sources for qstr extraction
SRC_QSTR += $(SRC_C) $(SRC_CXX) $(SHARED_SRC_C)
# Append any auto-generated sources that are needed by sources listed in
# SRC_QSTR
SRC_QSTR_AUTO_DEPS +=

CXXFLAGS += $(filter-out -Wmissing-prototypes -Wold-style-definition -std=gnu99, $(CXXFLAGS_MOD))
CFLAGS += $(CWARN)

# Use gcc syntax for map file
LDFLAGS_ARCH = -Wl,-Map=$@.map,--cref -Wl,--gc-sections
LDFLAGS += -T core/link.lds --static

# rtsp server
RTSP_SERVER_LIBS := 
RTSP_SERVER_LIBS += -L$(SDK_RTSMART_SRC_DIR)/mpp/middleware/lib -lrtsp_server -lliveMedia -lgroupsock -lBasicUsageEnvironment -lUsageEnvironment

LDFLAGS_LIBS := $(NNCASE_LIB_DIR) $(OPENCV_LIB_DIR) $(SDK_MPP_USER_LIB_DIR) $(SDK_MPP_MIDDLEWARE_LIB_DIR) -L$(SDK_CANMV_BUILD_DIR)/freetype/
LDFLAGS_LIBS += -Wl,--start-group -lm -lpthread -lfreetype -lmp4 $(NNCASE_LIBS) $(OPENCV_LIBS) $(SDK_MPP_USER_LIBS) $(RTSP_SERVER_LIBS) -Wl,--end-group

LDFLAGS += $(LDFLAGS_MOD) $(LDFLAGS_ARCH) $(LDFLAGS_LIBS) $(LDFLAGS_EXTRA)

include $(TOP)/py/mkrules.mk
