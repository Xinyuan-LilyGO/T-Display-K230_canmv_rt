ifndef CONFIG_SPL_BUILD
	ifndef  CONFIG_TARGET_K230_FPGA
		ifdef CONFIG_SPL
			INPUTS-y += add_firmware_head
		endif
	endif  
endif

add_firmware_head: u-boot.bin spl/u-boot-spl.bin
	@echo "Add header to u-boot-spl.bin"
	@cp spl/u-boot-spl.bin t.bin; python $(srctree)/tools/k230_image.py -i t.bin -o k230-u-boot-spl.bin -n; rm -rf t.bin

	@echo "Make uboot.img"
	@$(srctree)/tools/k230_priv_gzip -k -f u-boot.bin
	@$(objtree)/tools/mkimage -A riscv -O u-boot -C gzip -T firmware -a ${CONFIG_SYS_TEXT_BASE} -e ${CONFIG_SYS_TEXT_BASE} -n uboot -d u-boot.bin.gz u-boot.img

	@echo "Add header to u-boot.img"
	@cp u-boot.img t.bin;python $(srctree)/tools/k230_image.py -i t.bin -o k230-u-boot.img -n; rm -rf t.bin

	@echo "Generate k230_uboot_sd.img"
	@dd if=k230-u-boot-spl.bin of=k230_uboot_sd.img bs=512 seek=$$((0x100000/512))
	@dd if=k230-u-boot.img of=k230_uboot_sd.img bs=512 seek=$$((0x200000/512))
