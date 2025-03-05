#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (c) 2019 Western Digital Corporation or its affiliates.
#
# Authors:
#   Damien Le Moal <damien.lemoal@wdc.com>
#

-include $(SDK_SRC_ROOT_DIR)/.config

SDK_FW_TEXT_START=$(CONFIG_MEM_RTSMART_BASE)
SDK_FW_JUMP_ADDR=$(shell printf "0x%X" $$(($(CONFIG_MEM_RTSMART_BASE) + 0x20000)))

# Compiler flags
platform-cppflags-y =
platform-cflags-y = -g -I$(SDK_SRC_ROOT_DIR)/include
platform-asflags-y = -g
platform-ldflags-y = -g

# Blobs to build
PLATFORM_RISCV_ISA=rv64gcxthead
FW_TEXT_START?=$(SDK_FW_TEXT_START)
FW_PAYLOAD=y
FW_PAYLOAD_OFFSET=0x20000
FW_JUMP_ADDR?=$(SDK_FW_JUMP_ADDR)
