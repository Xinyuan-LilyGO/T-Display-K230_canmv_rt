[TOC]

## 1. 环境搭建

### 1.1 启动docker

```shell
$ cd /path/to/k230_sdk
$ docker run -u root -it --rm -v $(pwd):/mnt -v $(pwd)/toolchain:/opt/toolchain -w /mnt ghcr.io/kendryte/k230_sdk:latest /bin/bash
```

默认挂载了两个目录

- 将k230_sdk目录挂载到docker的/mnt
- 将交叉编译工具链k230_sdk/toolchain目录挂载到docker的/opt/toolchain



### 1.2 安装wheel包

- **linux平台**支持在线安装nncase和nncase-kpu
- **windows平台**只支持nncase在线安装， nncase-kpu需要到[nncase github release](https://github.com/kendryte/nncase/releases)单独下载并安装

```shell
root@c08eb1760d50:/mnt# pip install -i https://pypi.org/simple nncase==2.9.0 nncase-kpu==2.9.0
```



查看安装的nncase版本信息

```shell
root@c08eb1760d50:/mnt/# pip list|grep nncase
nncase            2.9.0  
nncase-kpu        2.9.0  

root@c08eb1760d50:/mnt/# python3
Python 3.8.10 (default, May 26 2023, 14:05:08) 
[GCC 9.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> import _nncase
>>> print(_nncase.__version__)
2.9.0
>>> quit()sss
```



## 2. 编译模型

### 2.1 编译

```shell
root@c08eb1760d50:/mnt/# cd src/big/nncase/examples/
root@c08eb1760d50:/mnt/src/big/nncase/examples# ./build_model.sh
```



### 2.2 查看编译结果

```shell
root@c08eb1760d50:/mnt/src/big/nncase/examples# ls -l tmp/
total 12
drwxr-xr-x 11 root root 4096 Jul 31 10:55 mbv2_tflite
drwxr-xr-x 11 root root 4096 Jul 31 10:57 mobile_retinaface
drwxr-xr-x 11 root root 4096 Jul 31 10:56 yolov5s_onnx
```



## 3. 编译App

###  3.1 App介绍

| App               | 备注                                                         |
| ----------------- | ------------------------------------------------------------ |
| image_classify    | 图片分类demo, 输入是RGB图片, 推理结果打印到串口              |
| object_detect     | 目标检测demo, 输入是RGB图片, 推理结果打印到串口              |
| image_face_detect | 人脸检测demo, 输入是RGB图片, 推理结果会输出到画了人脸box和landmark的图片 |



### 3.2 编译

```shell
root@c08eb1760d50:/mnt/src/big/nncase/examples# ./build_app.sh
```



### 3.3 查看编译结果

编译app结束后, 默认会将demo及其运行所需文件拷贝到当前目录下的k230_bin子目录

```shell
root@8c6c27a92c6c:/mnt/src/big/nncase/examples# ls -l k230_bin/
total 12
drwxrwxr-x 2 1000 1000 4096 Jul 31 10:58 image_classify
drwxrwxr-x 2 1000 1000 4096 Jul 31 10:58 image_face_detect
drwxrwxr-x 2 1000 1000 4096 Jul 31 10:58 object_detect
```



### 3.4 传送demo

- 将k230_bin目录拷贝到本地PC的nfsroot目录并重命名为nncase_k230_v2.9.0
- 在小核串口搭建sharefs环境, 并将本地PC的nfsroot目录mount到/sharefs
- 在大核串口下进入/sharefs执行相应demo



## 4. 上板运行

**前置条件**

- gnne频率: 800MHZ

- noc限速: 3.2GB/s


### 4.1 图片分类demo

```shell
msh /sharefs/k230/nncase_k230_v2.9.0>cd image_classify/
msh /sharefs/k230/nncase_k230_v2.9.0/image_classify>./cpp.sh
case ./image_classify.elf built at Jul 31 2024 10:58:34
interp.run() took: 2.19233 ms
image classify result: tabby(0.453891)
```



### 4.2 目标检测demo

```shell
msh /sharefs/k230/nncase_k230_v2.9.0>cd object_detect/
msh /sharefs/k230/nncase_k230_v2.9.0/object_detect>./cpp.sh
case ./object_detect.elf built at Jul 31 2024 10:58:34
od set_input took 0.191222 ms
od run took 17.1841 ms
od get output took 12.8689 ms
post process took 57.7554 ms
text = truck:0.300000
text = dog:0.250000
text = bicycle:0.230000
draw result took 30.9608 ms
```

推理结果(画框)会生成到od_result.jpg



### 4.3 人脸检测demo

```shell
msh /sharefs/k230/nncase_k230_v2.9.0>cd image_face_detect/
msh /sharefs/k230/nncase_k230_v2.9.0/image_face_detect>./cpp.sh
case ./image_face_detect.elf built at Jul 31 2024 10:58:34
Press 'q + enter' to exit!!!
```

推理结果(人脸box和landmark)会生成到face_500x500_result_x.jpg
