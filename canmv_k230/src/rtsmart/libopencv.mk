opencv_lib_dir := 
opencv_lib_dir += $(RTSMART_SRC_DIR)/../libs/opencv/lib
opencv_lib_dir += $(RTSMART_SRC_DIR)/../libs/opencv/lib/opencv4/3rdparty

opencv_libs := $(addprefix -l,$(subst lib, ,$(basename $(notdir $(foreach dir, $(opencv_lib_dir), $(wildcard $(dir)/*))))))

OPENCV_INC = -I$(RTSMART_SRC_DIR)/../libs/opencv/include/opencv4
# OPENCV_LD_FLAGS := $(addprefix -L, $(opencv_lib_dir)) -Wl,--start-group $(mpp_user_libs) -Wl,--end-group

OPENCV_LIBS = -lstdc++ -lopencv_core -lopencv_imgcodecs -lopencv_imgproc -lopencv_highgui -lopencv_videoio -lzlib -llibjpeg-turbo -llibopenjp2 -llibpng -llibtiff -llibwebp -lcsi_cv -latomic
OPENCV_LIB_DIR = $(addprefix -L, $(opencv_lib_dir))
