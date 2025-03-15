# T-Display-K230_canmv_rt

base platform:
      ubuntu20.04

参考BUILD，可使用Dokcer或本地环境编译，编译速度更快

或者参考K230 CanMV 自定义固件

烧录
linux下直接使用dd命令进行烧录，windows下使用烧录工具进行烧录，可参考K230 CanMV 如何烧录固件
Operator:
compile app:
change to current dir anmv_k230
      cd anmv_k230/src/rtsmart/mpp
      source build.sh
      cd /userapps/sample/sample_display
      make
#in the sample/elf generate   sample_display.elf

#default app: sample_display


compile app: 
##change to current dir anmv_k230
cd anmv_k230/src/rtsmart/mpp
source build.sh 
cd /userapps/sample/sample_display
make
in the dir  sample/elf 
generate sample_display.elf
      

