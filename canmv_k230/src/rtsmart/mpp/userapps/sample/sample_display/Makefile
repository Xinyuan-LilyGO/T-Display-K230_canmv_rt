include $(MPP_SRC_DIR)/userapps/sample/mpp.mk
include $(MPP_SRC_DIR)/userapps/sample/rt-smart.mk
include $(MPP_SRC_DIR)/../libopencv.mk
CURRECT_DIR_NAME=$(shell basename `pwd`)
LOCAL_SRC_DIR = $(shell pwd)
BIN = $(MPP_SRC_DIR)/userapps/sample/elf/$(CURRECT_DIR_NAME).elf
LIBPATH = $(MPP_LIB_PATH)
LIBS = $(MPP_LIBS)
AI_LIBPATH=$(MPP_SRC_DIR)/../libs/nncase
KMODEL_LIBPATH=$(MPP_SRC_DIR)/userapps/sample/kmodel

WERROR_FLAGS = -Wno-register -Wno-multichar -Wno-pessimizing-move -Wno-deprecated-declarations -Wno-unused-result -Wno-unused-variable -Wno-format -Wno-return-type -Wno-sign-compare -Wno-unused-label

RISCV64_NN_INCLUDE = \
                        -I$(AI_LIBPATH)/riscv64 \
                        -I$(AI_LIBPATH)/riscv64/nncase/include \
                        -I$(AI_LIBPATH)/riscv64/nncase/include/nncase \
                        -I$(AI_LIBPATH)/riscv64/nncase/include/nncase/runtime \
                        -I$(AI_LIBPATH)/riscv64/rvvlib/include \

RISCV64_NN_LIBS = \
                        -L$(AI_LIBPATH)/riscv64/rvvlib/ -lrvv \
                        -L$(AI_LIBPATH)/riscv64/nncase/lib/ -lNncase.Runtime.Native -lnncase.rt_modules.k230 -lfunctional_k230

CXX_FLAGS_AI = -std=c++20 \
                        $(WERROR_FLAGS) \
                        $(RISCV64_NN_INCLUDE) \
                        $(RISCV64_NN_LIBS) \

include $(LOCAL_SRC_DIR)/lvgl/lvgl.mk
include $(LOCAL_SRC_DIR)/ui_brookesia/ui_brookesia.mk
LOCAL_CFLAGS = -I$(LOCAL_SRC_DIR)
SRCS = $(wildcard $(LOCAL_SRC_DIR)/*.c)
#SRCS+=$(LVGL_CSRCS)
#SRCS+= $(wildcard $(LOCAL_SRC_DIR)/port/*.c)
#SRCS+=$(BROOKESIA_CSRCS)
#SRCS+=$(BROOKESIA_CXXSRCS)
SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/*.c)
#SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/lvg/*.c)
SRCS_C += $(LVGL_CSRCS)
SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/port/*.c)
SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/audio/*.c)
SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/network/*.c)
SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/fpioa/*.c)
SRCS_C += $(BROOKESIA_CSRCS)

SRCS_CXX += $(wildcard $(LOCAL_SRC_DIR)/*.cpp)
SRCS_CXX += $(BROOKESIA_CXXSRCS)



OBJS_C = $(patsubst %.c,%.o,$(SRCS_C))
OBJS_CXX = $(patsubst %.cpp,%.o,$(SRCS_CXX))

aaa=$(MPP_SRC_DIR)/include
bbb=$(MPP_SRC_DIR)/include/comm
ccc=$(MPP_SRC_DIR)/userapps/api

#OPENCV_INC
#export PATH=$PATH:$aaa:$bbb:$ccc

all:$(BIN)
	@-rm -f $(OBJS)
	echo "${PWD}/Makefile all"
$(BIN): $(OBJS_C) $(OBJS_CXX)
	$(CC) -o $(BIN) $(LINKFLAG) -Wl,--whole-archive -Wl,--no-whole-archive -n --static $^ -L$(LIBPATH) $(CXX_FLAGS_AI) $(OPENCV_LIB_DIR) -Wl,--start-group $(LIBS) $(OPENCV_LIBS) -lstdc++ -Wl,--end-group
	$(STRIP) $(BIN)
	cp $(KMODEL_LIBPATH)/mobile_retinaface/test.kmodel $(MPP_SRC_DIR)/userapps/sample/elf/
#%.o:%.c %.cpp
#	$(CC)  $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LVGL_CFLAGS) $(BEOOKESIA_CXXFLAGS) -c $< -o $@
$(OBJS_C):%.o:%.c
	$(CC)  $(CC_CFLAGS) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LVGL_CFLAGS) $(OPENCV_INC) $(BROOKESIA_CFLAGS)  -c $< -o $@
$(OBJS_CXX):%.o:%.cpp
	$(CC)  $(CC_CFLAGS) $(CXX_FLAGS_AI) $(LOCAL_CFLAGS) $(BSP_CFLGAS) $(RTSMART_CFLAGS) $(MPP_USER_CFLGAS) $(LVGL_CFLAGS) $(OPENCV_INC) $(BROOKESIA_CXXFLAGS)  -c $< -o $@
clean:
	echo "${PWD}/Makefile clean"
	-rm -rf $(BIN)
	-rm -f $(OBJS_C)
	-rm -f $(OBJS_CXX)

.PHONY: all clean
