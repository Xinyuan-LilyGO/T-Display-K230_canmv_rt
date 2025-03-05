deps_config := \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/applications/test/Kconfig \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//output/k230_canmv_v3p0/Kconfig.app \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/applications/Kconfig \
	Kconfig.canmv \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/opensbi/Kconfig \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/rtsmart/mpp/Kconfig \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/rtsmart/Kconfig \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/uboot/Kconfig \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//boards/k230_canmv_v3p0/Kconfig \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//boards/Kconfig \
	/home/lvzhenhao/pro/k230/init_factory/canmv_k230//Kconfig \

include/config/auto.conf: $(deps_config)

ifneq "$(SDK_BOARDS_DIR)" "/home/lvzhenhao/pro/k230/init_factory/canmv_k230//boards"
include/config/auto.conf: FORCE
endif
ifneq "$(SDK_BOARD_DIR)" "/home/lvzhenhao/pro/k230/init_factory/canmv_k230//boards/k230_canmv_v3p0"
include/config/auto.conf: FORCE
endif
ifneq "$(SDK_UBOOT_SRC_DIR)" "/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/uboot"
include/config/auto.conf: FORCE
endif
ifneq "$(SDK_RTSMART_SRC_DIR)" "/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/rtsmart"
include/config/auto.conf: FORCE
endif
ifneq "$(SDK_OPENSBI_SRC_DIR)" "/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/opensbi"
include/config/auto.conf: FORCE
endif
ifneq "$(SDK_APPS_SRC_DIR)" "/home/lvzhenhao/pro/k230/init_factory/canmv_k230//src/applications"
include/config/auto.conf: FORCE
endif
ifneq "$(SDK_BUILD_DIR)" "/home/lvzhenhao/pro/k230/init_factory/canmv_k230//output/k230_canmv_v3p0"
include/config/auto.conf: FORCE
endif

$(deps_config): ;
