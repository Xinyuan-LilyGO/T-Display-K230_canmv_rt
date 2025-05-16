# T-Display-K230_canmv_rt
## K230芯片简介

K230芯片是嘉楠科技 Kendryte®系列AIoT芯片中的最新一代SoC产品。该芯片采用全新的多异构单元加速计算架构，集成了2个RISC-V高能效计算核心，内置新一代KPU（Knowledge Process Unit）智能计算单元，具备多精度AI算力，广泛支持通用的AI计算框架，部分典型网络的利用率超过了70%。

该芯片同时具备丰富多样的外设接口，以及2D、2.5D等多个标量、向量、图形等专用硬件加速单元，可以对多种图像、视频、音频、AI等多样化计算任务进行全流程计算加速，具备低延迟、高性能、低功耗、快速启动、高安全性等多项特性。






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
      
