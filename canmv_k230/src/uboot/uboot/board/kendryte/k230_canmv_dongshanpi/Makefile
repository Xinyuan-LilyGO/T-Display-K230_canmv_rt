# /* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions are met:
#  * 1. Redistributions of source code must retain the above copyright
#  * notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright
#  * notice, this list of conditions and the following disclaimer in the
#  * documentation and/or other materials provided with the distribution.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#  * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#  * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#  * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
#  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#  */

BOARD_COMMON_DIR = $(srctree)/board/$(VENDOR)/common/
EXTRA_CFLAGS += -I$(BOARD_COMMON_DIR)

obj-y += board.o

obj-$(CONFIG_CANMV_DONGSHANPI_LPDDR3_2133) += gen_canmv_ddr_2133.lex.o

$(obj)/gen_canmv_ddr_2133.lex.c: $(src)/canmv_ddr_init_2133.c   $(BOARD_COMMON_DIR)/ddr.sh
	$(Q)($(BOARD_COMMON_DIR)/ddr.sh $< $@ 0x50000 0x53fff 0x54000 0x54359 || exit 1)

obj-$(CONFIG_CANMV_DONGSHANPI_LPDDR3_1866) += gen_canmv_ddr_1866.lex.o

$(obj)/gen_canmv_ddr_1866.lex.c: $(src)/canmv_ddr_init_1866.c   $(BOARD_COMMON_DIR)/ddr.sh
	$(Q)($(BOARD_COMMON_DIR)/ddr.sh $< $@ 0x50000 0x53fff 0x54000 0x54359 || exit 1)
