
ifneq ($(shell [ -d ${SDK_BUILD_IMAGES_DIR}/sdcard/ ] && echo 1 || echo 0),1)
$(shell mkdir -p ${SDK_BUILD_IMAGES_DIR}/sdcard/)
endif

.PHONY: all clean distclean

.PHONY: copy_freetype_fonts
copy_freetype_fonts:
	@echo "Copy freetype resources"
	@if [ ! -d ${SDK_BUILD_IMAGES_DIR}/sdcard/res/font ]; then \
		mkdir -p ${SDK_BUILD_IMAGES_DIR}/sdcard/res/font/; \
	fi; \
	rsync -aq --delete $(SDK_CANMV_SRC_DIR)/resources/font/ ${SDK_BUILD_IMAGES_DIR}/sdcard/res/font/

.PHONY: copy_libs
copy_libs:
	@echo "Copy libs"
	@if [ ! -d ${SDK_BUILD_IMAGES_DIR}/sdcard/libs ]; then \
		mkdir -p ${SDK_BUILD_IMAGES_DIR}/sdcard/libs/; \
	fi; \
	rsync -aq --delete $(SDK_CANMV_SRC_DIR)/resources/libs/ ${SDK_BUILD_IMAGES_DIR}/sdcard/libs/

.PHONY: copy_examples
copy_examples:
	@echo "Copy examples"
	@if [ ! -d ${SDK_BUILD_IMAGES_DIR}/sdcard/examples ]; then \
		mkdir -p ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/; \
	fi; \
	if [ -f $(SDK_CANMV_SRC_DIR)/resources/main.py ]; then \
		cp -f $(SDK_CANMV_SRC_DIR)/resources/main.py ${SDK_BUILD_IMAGES_DIR}/sdcard/main.py; \
	else \
		rm -rf ${SDK_BUILD_IMAGES_DIR}/sdcard/main.py; \
	fi;\
	rsync -aq --delete --exclude='.git' $(SDK_CANMV_SRC_DIR)/resources/examples/ ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/

.PHONY: copy_kmodels
copy_kmodels:
	@echo "Copy kmodels"
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_detection_320.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/18-NNCase/face_detection/
	@if [ ! -d ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel ]; then \
		mkdir -p ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel; \
	fi
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_recognition.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_detection_320.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/yolov8n_320.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/yolov8n_seg_320.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/LPD_640.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/ocr_det_int16.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/hand_det.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_landmark.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_pose.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_parse.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/LPD_640.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/licence_reco.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/handkp_det.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/ocr_rec_int16.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/hand_reco.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/person_detect_yolov5n.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/yolov8n-pose.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/kws.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_alignment.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/face_alignment_post.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/eye_gaze.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/yolov5n-falldown.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/cropped_test127.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/nanotrack_backbone_sim.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/nanotracker_head_calib_k230.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/gesture.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/recognition.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/hifigan.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/zh_fastspeech_2.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/
	@cp -r ${SDK_RTSMART_SRC_DIR}/libs/kmodel/ai_poc/kmodel/zh_fastspeech_1_f32.kmodel ${SDK_BUILD_IMAGES_DIR}/sdcard/examples/kmodel/

.PHONY: copy_micropython
copy_micropython:
	@echo "Copy micropython"
	@if [ ! -e $(SDK_CANMV_BUILD_DIR)/micropython ]; then \
		echo "micropython not exists." && exit 1; \
	fi; \
	cp -rf $(SDK_CANMV_BUILD_DIR)/micropython ${SDK_BUILD_IMAGES_DIR}/sdcard/

.PHONY: build
build:
	@$(MAKE) -j$(NCPUS) -C port || exit $?;

.PHONY: gen_image
gen_image: build copy_freetype_fonts copy_libs copy_examples copy_kmodels copy_micropython

all: gen_image
	@echo "Make canmv done."

clean:
	@rm -rf ${SDK_BUILD_IMAGES_DIR}/sdcard/
	@make -C port clean

distclean: clean
