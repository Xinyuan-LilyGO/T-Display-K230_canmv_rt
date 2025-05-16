# T-Display-K230_canmv_rt
## K230芯片简介

K230芯片是嘉楠科技 Kendryte®系列AIoT芯片中的最新一代SoC产品。该芯片采用全新的多异构单元加速计算架构，集成了2个RISC-V高能效计算核心，内置新一代KPU（Knowledge Process Unit）智能计算单元，具备多精度AI算力，广泛支持通用的AI计算框架，部分典型网络的利用率超过了70%。

该芯片同时具备丰富多样的外设接口，以及2D、2.5D等多个标量、向量、图形等专用硬件加速单元，可以对多种图像、视频、音频、AI等多样化计算任务进行全流程计算加速，具备低延迟、高性能、低功耗、快速启动、高安全性等多项特性。
## 硬件开发板简介

### K230 EVB 硬件开发板简介

K230-USIP-LP3-EVB是针对AI芯片K230-USIP开发，以LPDDR3为内存而设计的测评板;意在通过全面的接口覆盖K230所支持的各项特色功能的同时，为客户提供参考设计以完成自主研发。

![k230_board](./.github/images/k230_unsip_board.png)

| 序号 | 说明             |
|------------|------------------|
| 1      | CKLINK调试 |
| 2      | LCD扩展, 4lane MIPI DSI|
| 3      | LPDDR3 512MB|
| 4      | K230 SoC |
| 5      | 32Mbit QSPI NOR Flash|
| 6      | Sensor转接卡 (4Lane MIPI CSI)|
| 7      | 4GB eMMC|
| 8      | USB转串口FT2232|
| 9      | SD卡座|
| 10     | USB转二路串口|
| 11     | USB0可接USB转以太网|
| 12     | 复位按键|
| 13     | 电源开关|
| 14     | Boot启动开关|
| 15     | USB 5V电源|

### CanMV-K230 硬件开发板简介

CanMV-K230针对AI芯片K230-USIP开发，以LPDDR3为内存而设计的测评板,集成了常规开发板所需的HDMI和网络、WIFI/BT等接口。

![k230_Canmv-k230_board](./.github/images/k230_unsip_canmv_board.png)

| 序号 | 说明             |
|------------|------------------|
| 1      | HDMI |
| 2      | 网口|
| 3      | 耳机|
| 4      | 复位按键 |
| 5      | 电源&串口（接电脑USB口） |
| 6      | USB |
| 7      | 按键 |
| 8      | Sensor转接口（OV5647） |
| 9      | IO扩展口 |

## K230 SDK软件开发快速指南

### K230 SDK结构简介

| 一级目录| 二级目录 | 说明  |
|--------|--------|------|
| configs| NA  | 资源配置 (内存分配规划) |
| output | NA  | SDK编译产物|
| src    | big | 大核RTSmart代码|
| src    | common | 大小核公共代码|
| src    | little | 小核Linux代码|
| tools  | docker | dockerfile |
| tools  | doxygen|doxygen脚本和配置文件 |
| tools  | kconfig| |
| tools  | gen_image.sh | 生成可烧写镜像的脚本 |
| tools  | gen_image_cfg| 镜像分区配置文件|
| tools  | tuning-tool-client| PC端图像调试工具 |

K230 SDK 是面向K230 开发板的软件开发包，包含了基于Linux&RT-smart 双核异构系统开发需要用到的源代码，工具链和其他相关资源。

![k230_software_arch](./.github/images/software_arch.png)

### 配置软件开发环境

K230 SDK需要在Linux环境下编译，推荐使用Ubuntu Liunx 20.04。
> 如需使用windows环境编译，建议使用WSL2 + Docker环境。

#### 使用docker编译环境

- 获取docker编译镜像
  推荐在docker环境中编译K230 SDK，可直接使用如下docker镜像：

  ```shell
  docker pull ghcr.io/kendryte/k230_sdk
  ```

  可使用如下命令确认docker镜像拉取成功：

  ```shell
  docker images | grep k230_sdk
  ```

  > 说明： docker镜像中默认不包含toolchain，下载源码后，使用命令'make prepare_sourcecode'命令会自动下载toolchain至当前编译目录中。

  如果不使用docker编译环境，而是希望使用原生Linux进行编译，可参考`tools/docker/Dockerfile`，安装相应的工具至您的Linux系统中即可。

  如下载速度较慢或无法成功，可使用`tools/docker/Dockerfile`自行编译docker image，详情请参考[K230 SDK使用说明](https://github.com/kendryte/k230_docs/blob/main/zh/01_software/board/K230_SDK_使用说明.md#43-编译-sdk)

### 编译K230 SDK

#### 下载K230 SDK源码

```shell
git clone https://github.com/kendryte/k230_sdk
cd k230_sdk
make prepare_sourcecode
```

> `make prepare_sourcecode` 会自动下载Linux和RT-Smart toolchain, buildroot package, AI package等. 请确保该命令执行成功并没有Error产生，下载时间和速度以实际网速为准。

#### 开始编译K230 SDK

以docker镜像编译为例：

  1. 确认当前目录为`k230_sdk`源码根目录，
  2. 使用如下命令进入docker

  ```shell
  docker run -u root -it -v $(pwd):$(pwd) -v $(pwd)/toolchain:/opt/toolchain -w $(pwd) ghcr.io/kendryte/k230_sdk /bin/bash
  ```

  1. 根据不同开发板或软件功能，选择不同的配置config进行编译，编译命令格式：`make CONF=xxx`，如：
 > - 编译K230-USIP-LP3-EVB板子镜像，执行`make CONF=k230_evb_defconfig`  命令开始编译
 > - 编译CanMV-K230板子的镜像，执行 `make CONF=k230_canmv_defconfig`  命令开始编译

> - 外部目录中自动下载的toolchain会映射至docker镜像中的`/opt/toolchain/`目录下。
> - 默认参数`-u root`指定docker以root用户执行，k230_sdk无需root权限即可编译
> - docker镜像请使用`ghcr.io/kendryte/k230_sdk`完整路径，如自行编译的本地docker镜像，请修改相应名称

#### 编译产物简介
以make CONF=k230_evb_defconfig 编译生成的产物为例：
``` shell
k230_evb_defconfig/images
├── big-core
├── env.env
├── jffs2.env
├── little-core
├── sysimage-sdcard.img    # SD和emmc非安全启动镜像
├── sysimage-sdcard.img.gz # SD和emmc的非安全启动镜像压缩包
├── sysimage-spinor32m.img # norflash非安全启动镜像
├── sysimage-spinor32m.img.gz    # norflash非安全启动镜像压缩包
└── sysimage-spinor32m_jffs2.img # norflash jffs2非安全启动镜像
```

> TF卡和eMMC均可使用`sysimage-sdcard.img`镜像,或使用`sysimage-sdcard.img.gz`解压缩得到该文件。

### 预编译镜像下载

如果不希望自行编译镜像，可下载预编译镜像，直接烧录使用
1. **[main branch](https://github.com/kendryte/k230_sdk/tree/main)**: Github默认分支，作为release分支，编译release镜像自动发布至[Release](https://github.com/kendryte/k230_sdk/releases)页面.(从`v1.4`版本开始支持)
2. 预编译release镜像：请访问[嘉楠开发者社区](https://developer.canaan-creative.com/resource), 然后在`K230/Images`分类中，下载所需的镜像文件，`evb`设备下载`k230_evb*.img.gz`, `canmv`设备下载`k230_canmv*.img.gz`。

> 下载的镜像默认为`.gz`压缩格式，需先解压缩，然后再烧录。
> [K230 micropython](https://github.com/kendryte/k230_canmv/)镜像所支持的功能与K230 SDK并不相同

### 烧录镜像文件

#### 烧录TF卡

如使用Linux烧录TF卡,需要先确认TF卡在系统中的名称`/dev/sdx`, 并替换如下命令中的`/dev/sdx`

``` shell
sudo dd if=sysimage-sdcard.img of=/dev/sdx bs=1M oflag=sync
```

如使用Windows烧录, 建议使用[rufus](http://rufus.ie/downloads/)工具

其它更详细的烧录方法，请参考K230 SDK文档

### 上电启动

#### K230 EVB开发板上电启动

K230 EVB支持SDCard、eMMC、norflash等多种启动方式，用户可以通过改变开板上启动拔码开关的设置，来[切换不同启动模式](https://github.com/kendryte/k230_docs/blob/main/zh/00_hardware/K230_DEMO_BOARD资源使用指南.md#电源区开机上电方式)。
为方便开发，建议您准备一张TF卡，并将**拔码开关切换至SD卡启动模式**，后续可考虑将镜像文件固化至emmc中。

1. 请先**确认启动开关SW1选择在SD卡启动模式**下（详情可参考[开机上电方式](https://github.com/kendryte/k230_docs/blob/main/zh/00_hardware/K230_DEMO_BOARD资源使用指南.md#电源区开机上电方式)）
1. 将烧录完成的TF卡插入开发板TF卡槽中
1. 开发板接上电源
1. **将电源开关K1拔到ON位置**，系统可上电启动
1. 如果您有接好串口，可在串口中看到启动日志输出。

#### CanMV-K230开发板上电启动

K230 CanMV-K230开发板支持SDCard启动方式、HDMI输出显示，因此，需要准备一张TF卡，此外建议准备一个HDMI显示器。

1. 将烧录完成的TF卡插入开发板TF卡槽中
2. 开发板上电，此时，系统可上电启动

系统上电后，默认会有**两个串口设备**，可分别用于访问小核Linux和大核RTSmart

小核Linux默认用户名root，密码为空。大核RTSmart系统中开机会自动启动一个应用程序，可按`q`键退出至命令提示符终端。








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
      
