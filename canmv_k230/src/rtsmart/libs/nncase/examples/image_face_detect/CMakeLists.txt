cmake_minimum_required(VERSION 3.2)
project(image_face_detect C CXX)

set(src main.cc model.cc mobile_retinaface.cc anchors_320.cc util.cc)
set(bin image_face_detect.elf)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${nncase_sdk_root}/riscv64/rvvlib/include)
include_directories(${k230_sdk}/src/big/mpp/userapps/api/)
include_directories(${k230_sdk}/src/big/mpp/include)
include_directories(${k230_sdk}/src/big/mpp/include/comm)

link_directories(${nncase_sdk_root}/riscv64/rvvlib/)

add_executable(${bin} ${src})
target_link_libraries(${bin} rvv nncase.rt_modules.k230 Nncase.Runtime.Native functional_k230 sys)
target_link_libraries(${bin} opencv_imgcodecs opencv_imgproc opencv_core libjpeg-turbo libopenjp2 libpng libtiff libwebp zlib csi_cv)
install(TARGETS ${bin} DESTINATION bin)