<!--
 * @Description: None
 * @version: V1.0.0
 * @Author: LILYGO_L
 * @Date: 2023-09-11 16:13:14
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2025-03-05 18:07:02
 * @License: GPL 3.0
-->
<h1 align = "center">T-Display-K230_canmv_rt</h1>

<p align="center" width="100%">
    <img src="image/14.jpg" alt="">
</p>

## **English | [中文](./README_CN.md)**

## Version iteration:
| Version                              | Update date                       |Update description|
| :-------------------------------: | :-------------------------------: | :-------------------------------: |
| T-Display-K230_canmv_rt_V1.0     | 2025-02-20      |Original version      |
| T-Display-K230_canmv_rt_V1.1     | 2025-03-25       |  ********************   |

## PurchaseLink

| Product                     | SOC           |  FLASH  |  LPDDR   | Link                   |
| :------------------------: | :-----------: |:-------: | :---------: | :------------------: |
| T-Display-K230_canmv_rt| k230 |   SD Card   | 1GiB | [LILYGO Mall](https://lilygo.cc/products/t-display-k230?_pos=2&_psq=k230&_ss=e&_v=1.0&variant=45197449953461)  |

## Directory
- [Describe](#describe)
- [Preview](#preview)
- [Module](#module)
- [QuickStart](#quickstart)
- [PinOverview](#pinoverview)
- [RelatedTests](#RelatedTests)
- [FAQ](#faq)
- [Project](#project)
- [Information](#information)
- [DependentLibraries](#dependentlibraries)

## Describe

T-Display-K230 is a development board featuring a high-definition AMOLED display, based on the k230, designed for standalone battery connectivity.

## Preview

### Actual Product Image

<p align="center" width="100%">
    <img src="image/14.jpg" alt="">
</p>

## Module

### 1.MCU

* Chip: k230
* For more details, please visit[Espressif ESP32-S3 Datashee](https://www.espressif.com.cn/sites/default/files/documentation/esp32-s3_datasheet_en.pdf)

### 2. Screen

* Size: 4.1-inch AMOLED  screen
* Resolution: 568x1232px
* Screen type: AMOLED
* Driver chip: RM69A10

### 3. Touch

* Chip: GT9895
* Bus communication protocol: IIC
<!--
### 4. Charging chip

* Chip: SY6970
* Bus communication protocol: IIC
* Other: The output waveform of the chip will be highly unstable when powered by a 5V supply without a battery connected. To stabilize the situation, it is necessary to either connect a battery or use software to disable the battery channel. By doing so, the instability will be alleviated.

### 5. RTC

* Chip: PCF8563
* Bus communication protocol: IIC
-->
## QuickStart

### Examples Support



| Firmware | Description | Picture |
| ------  | ------  | ------ |
#### k230
| Setting                               | Value                                 |
| :-------------------------------: | :-------------------------------: |
| Board                                 | ESP32S3 Dev Module           |
| Upload Speed                     | 921600                               |
| USB Mode                           | Hardware CDC and JTAG     |
| USB CDC On Boot                | Enabled                              |
| USB Firmware MSC On Boot | Disabled                             |
| USB DFU On Boot                | Disabled                             |
| CPU Frequency                   | 240MHz (WiFi)                    |
| Flash Mode                         | QIO 80MHz                         |
| Flash Size                           | 16MB (128Mb)                    |
| Core Debug Level                | None                                 |
| Partition Scheme                | 16M Flash (3MB APP/9.9MB FATFS) |
| PSRAM                                | OPI PSRAM                         |
| Arduino Runs On                  | Core 1                               |
| Events Run On                     | Core 1                               |           

6. Select the correct port.

7. Click "<kbd>[√](image/8.png)</kbd>" in the upper right corner to compile,If the compilation is correct, connect the microcontroller to the computer,Click "<kbd>[→](image/9.png)</kbd>" in the upper right corner to download.

### firmware download
1. Open the project file "tools" and locate the ESP32 burning tool. Open it.

2. Select the correct burning chip and burning method, then click "OK." As shown in the picture, follow steps 1->2->3->4->5 to burn the program. If the burning is not successful, press and hold the "BOOT-0" button and then download and burn again.

3. Burn the file in the root directory of the project file "[firmware](./firmware/)" file,There is a description of the firmware file version inside, just choose the appropriate version to download.

<p align="center" width="100%">
    <img src="image/10.png" alt="example">
    <img src="image/11.png" alt="example">
</p>


## PinOverview

| AMOLED Screen Pin  | k230 Pin|
| :------------------: | :------------------:|
| SDIO0         | IO11       |
| SDIO1         | IO13       |
| SDIO2         | IO14       |
| SDIO3         | IO15       |
| SCLK         | IO12       |
| CS         | IO10       |
| RST         | IO17       |
| EN         | IO16       |

| Touch Chip Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| INT         | IO9       |
| SDA         | IO7       |
| SCL         | IO6       |

| Power Chip Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| SDA         | IO7       |
| SCL         | IO6       |

| Battery Measurement Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| BATTERY_VOLTAGE_ADC_DATA         | IO4       |

| SD Card Pin  | ESP32S3 Pin|
| :------------------: | :------------------:|
| CS         | IO4       |
| MOSI        | IO39       |
| MISO         | IO40       |
| SCLK         | IO41       |

## RelatedTests

### Power Dissipation
| Firmware | Program| Description | Picture |
| ------  | ------  | ------ | ------ | 
| `[T-Display-S3-AMOLED-1.43_V1.0][Light_Sleep_Wake_Up]_firmware_V1.0.0.bin` | `Light Sleep Wake Up` | Power dissipation: 1282.8uA | <p align="center" width="10%"> <img src="image/13.jpg" alt="example" width="50%"> </p> |
| `[T-Display-S3-AMOLED-1.43_V1.0][Deep_Sleep_Wake_Up]_firmware_V1.0.0.bin` | `Deep Sleep Wake Up` | Power dissipation: 174.2uA |<p align="center" width="10%"> <img src="image/12.jpg" alt="example" width="50%"> </p> |

## FAQ

* Q. After reading the above tutorials, I still don't know how to build a programming environment. What should I do?
* A. If you still don't understand how to build an environment after reading the above tutorials, you can refer to the [LilyGo-Document](https://github.com/Xinyuan-LilyGO/LilyGo-Document) document instructions to build it.

<br />

* Q. Why does Arduino IDE prompt me to update library files when I open it? Should I update them or not?
* A. Choose not to update library files. Different versions of library files may not be mutually compatible, so it is not recommended to update library files.

<br />

* Q. Why is there no serial data output on the "Uart" interface on my board? Is it defective and unusable?
* A. The default project configuration uses the USB interface as Uart0 serial output for debugging purposes. The "Uart" interface is connected to Uart0, so it won't output any data without configuration.<br />For PlatformIO users, please open the project file "platformio.ini" and modify the option under "build_flags = xxx" from "-D ARDUINO_USB_CDC_ON_BOOT=true" to "-D ARDUINO_USB_CDC_ON_BOOT=false" to enable external "Uart" interface.<br />For Arduino users, open the "Tools" menu and select "USB CDC On Boot: Disabled" to enable the external "Uart" interface.

<br />

* Q. Why is my board continuously failing to download the program?
* A. Please hold down the "BOOT-0" button and try downloading the program again.

## Project
* [T-Display-S3-AMOLED-1.43-1.75_V1.0](./project/T-Display-S3-AMOLED-1.43-1.75_V1.0.pdf)

## Information
* [FT3168](./information/FT3168.pdf)
* [PCF8563](./information/PCF8563.pdf)
* [SH8601](./information/SH8601Z.pdf)
* [DO0143FAT01](./information/SPEC-DO0143FAT01-20230830.pdf)
* [AN_SY6970](./information/AN_SY6970.pdf)
* [EVB_SY6970](./information/EVB_SY6970.pdf)

## DependentLibraries
* [Arduino_GFX-1.3.7](https://github.com/moononournation/Arduino_GFX)
* [Arduino_DriveBus-1.1.12](https://github.com/Xk-w/Arduino_DriveBus)
* [JPEGDEC-1.2.8](https://github.com/bitbank2/JPEGDEC)
* [lvgl-8.3.5](https://lvgl.io)
* [MiniTV](https://github.com/moononournation/MiniTV)
* [SensorLib](https://github.com/lewisxhe/SensorsLib)

