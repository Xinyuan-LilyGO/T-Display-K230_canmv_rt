# T-Display-K230_canmv_rt

base platform:
      ubuntu20.04

参考BUILD，可使用Dokcer或本地环境编译，编译速度更快

或者参考K230 CanMV 自定义固件

烧录
linux下直接使用dd命令进行烧录，windows下使用烧录工具进行烧录，可参考K230 CanMV 如何烧录固件
Operator:

编译固件：


      cd anmv_k230
      time make log
     
canmv_k230/output/k230_canmv_v3p0    生成CanMV-K230-V3P0_rtsmart_localnncase_v2.9.0.img   可烧录至sd卡



compile app:
change to current dir anmv_k230

      
      cd anmv_k230/src/rtsmart/mpp
      source build.sh
      cd /userapps/sample/sample_display
      make
in the dir sample/elf     generate sample_display.elf

default app: sample_display


将sample_display.elf 改名为app.elf   拷到sd卡 sdcard盘中重新上电，默认启动运行
      
