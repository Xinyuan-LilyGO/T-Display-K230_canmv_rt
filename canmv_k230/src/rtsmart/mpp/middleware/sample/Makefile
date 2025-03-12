include $(MPP_SRC_DIR)/middleware/mpp.mk
include $(MPP_SRC_DIR)/middleware/rt-smart.mk

include $(SDK_SRC_ROOT_DIR)/.config

$(shell if [ ! -e $(MPP_MIDDLEWARE_LIB_INSTALL_PATH) ];then mkdir -p $(MPP_MIDDLEWARE_LIB_INSTALL_PATH); fi)
$(shell if [ ! -e $(MPP_MIDDLEWARE_ELF_INSTALL_PATH) ];then mkdir -p $(MPP_MIDDLEWARE_ELF_INSTALL_PATH); fi)

subdirs-y :=

subdirs-$(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES_DEMUXER)		+= sample_demuxer
subdirs-$(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES_MUXER)		+= sample_muxer
subdirs-$(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES_PLAYER)		+= sample_player
subdirs-$(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES_RTSP_CLIENT)	+= sample_rtspclient
subdirs-$(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES_RTSP_PUSHER)	+= sample_rtsppusher
subdirs-$(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES_RTSP_SERVER)	+= sample_rtspserver

.PHONY: all clean

all:
ifeq ($(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES),y)
	@$(foreach dir,$(subdirs-y),make -C $(dir) all;)
endif
	@echo "Make MPP Middleware samples done."

clean:
ifeq ($(CONFIG_MPP_ENABLE_MIDDLEWARE_SAMPLES),y)
	@$(foreach dir,$(subdirs-y),make -C $(dir) clean;)
endif
	@rm -rf $(MPP_MIDDLEWARE_ELF_INSTALL_PATH)/*.*
	@echo "Clean MPP Middleware samples done."
