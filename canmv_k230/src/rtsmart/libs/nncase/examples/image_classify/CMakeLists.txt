set(src main.cc)
set(bin image_classify.elf)

add_executable(${bin} ${src})
target_link_libraries(${bin} nncase.rt_modules.k230 Nncase.Runtime.Native functional_k230 sys)
target_link_libraries(${bin} opencv_imgcodecs opencv_imgproc opencv_core libjpeg-turbo libopenjp2 libpng libtiff libwebp zlib csi_cv)

install(TARGETS ${bin} DESTINATION bin)