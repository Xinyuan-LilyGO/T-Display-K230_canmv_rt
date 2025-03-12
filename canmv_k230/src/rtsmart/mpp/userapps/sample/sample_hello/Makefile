RT_PRE=/home/lvzhenhao/pro/k230
RT_CCE=/home/lvzhenhao/pro/k230/k230_sdk/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin/riscv64-unknown-linux-musl-gcc



LINKFLAG=-T /home/lvzhenhao/pro/k230/init_factory/canmv_k230/src/rtsmart/mpp/userapps/sample/linker_scripts/riscv64/link.lds


LOCAL_SRC_DIR = $(shell pwd)
CURRECT_DIR_NAME=$(shell basename `pwd`)
BIN = ./$(CURRECT_DIR_NAME).elf
#OBJS += $(patsubst %.cpp,%.o,sample_hello.cpp)


#SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/*.c)
#LVG_SRCS_C += $(wildcard $(LOCAL_SRC_DIR)/lvg/*.c)
SRCS_C +=  $(shell find $(LOCAL_SRC_DIR) -type f -name '*.c')
SRCS_CXX +=  $(shell find $(LOCAL_SRC_DIR) -type f -name '*.cpp')

#SRCS_CXX += $(wildcard $(LOCAL_SRC_DIR)/*.cpp)
OBJS_C += $(patsubst %.c,%.o,$(SRCS_C))
#LVG_OBJS_C += $(patsubst %.c,%.o,$(LVG_SRCS_C))

OBJS_CXX += $(patsubst %.cpp,%.o,$(SRCS_CXX))
LINKFLAG=-T /home/lvzhenhao/pro/k230/init_factory/canmv_k230/src/rtsmart/mpp/userapps/sample/linker_scripts/riscv64/link.lds


LIBPATH = -L$(RT_PRE)/k230_sdk/src/big/rt-smart/userapps/sdk/lib/risc-v/rv64 -L$(RT_PRE)/k230_sdk/src/big/rt-smart/userapps/sdk/rt-thread/lib/risc-v/rv64
LIBS =-lrtthread

all:$(OBJS_C) $(OBJS_CXX)
	$(RT_CCE) -o $(BIN) -mcmodel=medany -march=rv64imafdcv -mabi=lp64d $(LINKFLAG)  -Wl,--whole-archive -Wl,--no-whole-archive -n --static $^ $(LIBPATH)  -Wl,--start-group $(LIBS) -Wl,--end-group
%.o:%.c %.cpp
	$(RT_CCE)  -mcmodel=medany -march=rv64imafdcv -mabi=lp64d -I. -I$(RT_PRE) -c $< -o $@
#$(LVG_OBJS_C):%.o:%.c
#	$(RT_CC)  -mcmodel=medany -march=rv64imafdcv -mabi=lp64d -c $< -o $@
#$(OBJS_CXX):$(SRCS_CXX)
#	$(RT_CC)  -mcmodel=medany -march=rv64imafdcv -mabi=lp64d -c $<  -o $@
clean:
	rm -rf $(BIN)
	rm -f $(OBJS_C) $(OBJS_CXX)



