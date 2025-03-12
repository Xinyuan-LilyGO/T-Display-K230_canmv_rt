BOARD_COMMON_DIR = $(srctree)/board/$(VENDOR)/common

obj-$(CONFIG_K230D_SIP_LPDDR4_1600) += gen_k230d_sip_lpddr4_32_swap_1600.lex.o
obj-$(CONFIG_K230D_SIP_LPDDR4_2667) += gen_k230d_sip_lpddr4_32_swap_2667.lex.o
obj-$(CONFIG_K230D_SIP_LPDDR4_3200_ALLODT) += gen_k230d_sip_lpddr4_32_swap_3200_with_allodt.lex.o
obj-$(CONFIG_K230D_SIP_LPDDR4_3200_HWODT) += gen_k230d_sip_lpddr4_32_swap_3200_with_wodt.lex.o

$(obj)/gen_k230d_sip_lpddr4_32_swap_1600.lex.c: $(src)/k230d_sip_lpddr4_init_32_swap_1600.c
	($(BOARD_COMMON_DIR)/ddr.sh $< $@ 0x50000 0x53fff 0x54000 0x5433d  || exit 1)

$(obj)/gen_k230d_sip_lpddr4_32_swap_2667.lex.c: $(src)/k230d_sip_lpddr4_init_32_swap_2667.c
	($(BOARD_COMMON_DIR)/ddr.sh $< $@ 0x50000 0x53fff 0x54000 0x5433d  || exit 1)

$(obj)/gen_k230d_sip_lpddr4_32_swap_3200_with_allodt.lex.c: $(src)/k230d_sip_lpddr4_init_32_swap_3200_with_allodt.c
	($(BOARD_COMMON_DIR)/ddr_2d.sh $< $@ 0x50000 0x53fff 0x54000 0x5433d  0x542bd || exit 1)

$(obj)/gen_k230d_sip_lpddr4_32_swap_3200_with_wodt.lex.c: $(src)/k230d_sip_lpddr4_init_32_swap_3200_with_wodt.c   
	($(BOARD_COMMON_DIR)/ddr_2d.sh $< $@ 0x50000 0x53fff 0x54000 0x5433d  0x542bd || exit 1)
