include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk

CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
BIN = $(MPP_SRC_DIR)/userapps/sample/elf/$(CURRECT_DIR_NAME).elf
LIBPATH = $(MPP_LIB_PATH)
LIBS = $(MPP_LIBS)

LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)

SRCS = $(wildcard $(LOCAL_SRC_DIR)/*.c)

OBJS = $(patsubst %.c,%.o,$(SRCS))

CPP_SRCS += $(wildcard $(LOCAL_SRC_DIR)/*.cpp)
CPP_SRCS += $(wildcard $(LOCAL_SRC_DIR)/modules/SX127x/*.cpp)
CPP_SRCS += $(wildcard $(LOCAL_SRC_DIR)/protocols/PhysicalLayer/*.cpp)
CPP_SRCS += $(wildcard $(LOCAL_SRC_DIR)/utils/*.cpp)

CPP_OBJS = $(patsubst %.cpp,%.o,$(CPP_SRCS))

all: $(BIN)

$(OBJS): %.o : %.c
	@echo CC $@
	@$(CC) $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -c $< -o $@

$(CPP_OBJS): %.o : %.cpp
	@echo CXX $@
	@$(CC) $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) -c $< -o $@

$(BIN): $(OBJS) $(CPP_OBJS)
	@echo LD $@
	@$(CC) -o $(BIN) $(LINKFLAG)  -Wl,--whole-archive -Wl,--no-whole-archive -n --static $^ -L$(LIBPATH) -Wl,--start-group $(LIBS) -lstdc++ -Wl,--end-group

clean:
	-rm -rf $(BIN) **/*.o

.PHONY: all clean
