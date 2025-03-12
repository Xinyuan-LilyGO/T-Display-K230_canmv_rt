#!/bin/bash
set -x

# set cross build toolchain
export PATH=$PATH:/opt/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin/

clear
rm -rf out
mkdir out
pushd out
cmake -DCMAKE_BUILD_TYPE=Release                 \
      -DCMAKE_INSTALL_PREFIX=`pwd`               \
      -DCMAKE_TOOLCHAIN_FILE=cmake/Riscv64.cmake \
      ..

make -j && make install
popd

# assemble all test cases
k230_bin=`pwd`/k230_bin
mkdir -p ${k230_bin}
if [ -f out/bin/image_classify.elf ]; then
      image_classify=${k230_bin}/image_classify
      rm -rf ${image_classify}
      cp -a image_classify/data/ ${image_classify}
      cp out/bin/image_classify.elf ${image_classify}
      cp tmp/mbv2_tflite/test.kmodel ${image_classify}
fi

if [ -f out/bin/object_detect.elf ]; then
      object_detect=${k230_bin}/object_detect
      rm -rf ${object_detect}
      cp -a object_detect/data/ ${object_detect}
      rm ${object_detect}/*.bin
      cp out/bin/object_detect.elf ${object_detect}
      cp tmp/yolov5s_onnx/test.kmodel ${object_detect}
fi

if [ -f out/bin/image_face_detect.elf ]; then
      image_face_detect=${k230_bin}/image_face_detect
      rm -rf ${image_face_detect}
      cp -a image_face_detect/data/ ${image_face_detect}
      cp out/bin/image_face_detect.elf ${image_face_detect}
      cp tmp/mobile_retinaface/test.kmodel ${image_face_detect}/
fi