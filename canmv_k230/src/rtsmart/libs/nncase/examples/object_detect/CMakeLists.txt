set(src main.cc utils.cc object_detect.cc)
set(bin object_detect.elf)

add_executable(${bin} ${src})
target_link_libraries(${bin} nncase.rt_modules.k230 Nncase.Runtime.Native functional_k230 sys)
target_link_libraries(${bin} opencv_imgproc opencv_imgcodecs opencv_core zlib libjpeg-turbo libopenjp2 libpng libtiff libwebp csi_cv)
install(TARGETS ${bin} DESTINATION bin)