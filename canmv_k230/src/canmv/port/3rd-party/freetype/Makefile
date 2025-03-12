include ${SDK_SRC_ROOT_DIR}/.config
include $(SDK_TOOLS_DIR)/toolchain_rtsmart.mk

FREETYPE_DIR = $(dir $(shell pwd)/freetype/)
BUILD = ${SDK_CANMV_BUILD_DIR}/freetype/

RM = rm
ECHO = @echo
CP = cp
MKDIR = mkdir
SED = sed
CAT = cat
TOUCH = touch
PYTHON = python3
ZIP = zip

AS = $(CROSS_COMPILE)as
CC = $(CROSS_COMPILE)gcc
CPP = $(CC) -E
CXX = $(CROSS_COMPILE)g++
GDB = $(CROSS_COMPILE)gdb
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
STRIP = $(CROSS_COMPILE)strip
AR = $(CROSS_COMPILE)ar

SRC_DIRS :=

SFILES :=
SFILES_EXCLUDE :=
SFLAGS :=

CFILES :=
CFILES_EXCLUDE :=
CFLAGS :=

CFILES += \
      freetype/src/base/ftsystem.c \
      freetype/src/base/ftinit.c \
      freetype/src/base/ftdebug.c \
      freetype/src/base/ftbase.c \
      freetype/src/base/ftbbox.c \
      freetype/src/base/ftglyph.c \
      freetype/src/base/ftbitmap.c \
      freetype/src/base/ftmm.c \
      # freetype/src/base/ftbdf.c \
      freetype/src/base/ftcid.c \
      freetype/src/base/ftfstype.c \
      freetype/src/base/ftgasp.c \
      freetype/src/base/ftgxval.c \
      freetype/src/base/ftotval.c \
      freetype/src/base/ftpatent.c \
      freetype/src/base/ftpfr.c \
      freetype/src/base/ftstroke.c \
      freetype/src/base/ftsynth.c \
      freetype/src/base/fttype1.c \
      freetype/src/base/ftwinfnt.c \

CFILES += \
      freetype/src/sfnt/sfnt.c \
      freetype/src/truetype/truetype.c

CFILES += \
      freetype/src/smooth/smooth.c \
      # freetype/src/autofit/autofit.c \
      freetype/src/raster/raster.c \
      freetype/src/sdf/sdf.c \

CFILES += \
      freetype/src/cache/ftcache.c

CFLAGS += -I. -Ifreetype/include
CFLAGS += -DFT2_BUILD_LIBRARY
CFLAGS += -DFT_CONFIG_MODULES_H="\"canmv_ftmodule.h\""
CFLAGS += -DFT_CONFIG_OPTIONS_H="\"canmv_ftoption.h\""

CWARN := 
# CWARN += -Wall -Werror
# CWARN += -Wno-int-conversion

CFLAGS += -std=gnu99 -fopenmp -march=rv64imafdcv -mabi=lp64d -mcmodel=medany $(CWARN)

SFILES_ALL = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.S))
CFILES_ALL = $(foreach dir, $(SRC_DIRS), $(wildcard $(dir)/*.c))

SFILES += $(filter-out $(SFILES_EXCLUDE), $(SFILES_ALL))
CFILES += $(filter-out $(CFILES_EXCLUDE), $(CFILES_ALL))

SDEPS	:= $(patsubst %, $(BUILD)/%, $(SFILES:.S=.o.d))
CDEPS	:= $(patsubst %, $(BUILD)/%, $(CFILES:.c=.o.d))
DEPS	:= $(SDEPS) $(CDEPS)

SOBJS	:= $(patsubst %, $(BUILD)/%, $(SFILES:.S=.o))
COBJS	:= $(patsubst %, $(BUILD)/%, $(CFILES:.c=.o))
OBJS	:= $(SOBJS) $(COBJS)

LIB_FREETYPE = libfreetype.a

.PHONY: all lib prepare

all: prepare $(BUILD)/$(LIB_FREETYPE)

$(BUILD)/$(LIB_FREETYPE): $(OBJS) 
	@$(ECHO) "Link $(LIB_FREETYPE)"
	@$(LD) -r -o $@ $^

prepare:
	@set -e; \
	if [ ! -f $(BUILD)/include/ft2build.h ]; then \
		$(MKDIR) -p $(BUILD); \
		ln -sf $(FREETYPE_DIR)/include $(BUILD); \
      fi;

clean: $(OBJS) $(DEPS)
	@$(RM) $^

$(SOBJS) : $(BUILD)/%.o : %.S
	@echo [AS] $<
	@$(AS) $(SFLAGS) -MD -MP -MF $@.d -c $< -o $@

$(COBJS) : $(BUILD)/%.o : %.c
	@echo [CC] $<
	@$(CC) $(CFLAGS) -MD -MP -MF $@.d -c $< -o $@

# $(sort $(var)) removes duplicates
#
# The net effect of this, is it causes the objects to depend on the
# object directories (but only for existence), and the object directories
# will be created if they don't exist.
OBJ_DIRS = $(sort $(dir $(OBJS)))
$(OBJS): | $(OBJ_DIRS)
$(OBJ_DIRS):
	@$(MKDIR) -p $@

sinclude $(DEPS)
