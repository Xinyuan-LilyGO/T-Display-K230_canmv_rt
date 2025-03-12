from machine import Pin
from machine import FPIOA
from machine import I2C_Slave
import time,os


#初始化gpio用于通知主机有数据更新
def gpio_int_init():
    # 实例化FPIOA
    fpioa = FPIOA()
    # 设置Pin5为GPIO5
    fpioa.set_function(5, FPIOA.GPIO5)
    # 实例化Pin5为输出
    pin = Pin(5, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
    # 设置输出为高
    pin.value(1)
    return pin

pin = gpio_int_init()
#i2c slave设备id列表
device_id = I2C_Slave.list()
print("Find i2c slave device:",device_id)

#构造I2C Slave，设备地址为0x01，模拟eeprom映射的内存大小为20byte
i2c_slave = I2C_Slave(device_id[0],addr=0x10,mem_size=20)

#等待master发送数据，并读取映射内存中的值
last_dat = i2c_slave.readfrom_mem(0,20)
dat = last_dat
while dat == last_dat:
    dat = i2c_slave.readfrom_mem(0,20)
    time.sleep_ms(100)
    os.exitpoint()
print(dat)
    
#修改映射内存中的值
for i in range(len(dat)):
    dat[i] = i
i2c_slave.writeto_mem(0,dat)
#通知主机有数据更新
pin.value(0)
time.sleep_ms(1)
pin.value(1)