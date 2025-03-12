import network
import os

def sta_test():
    sta=network.WLAN(0)
    #sta连接ap
    sta.connect("Canaan","Canaan314")
    #查看sta状态
    print(sta.status())
    
    while sta.ifconfig()[0] == '0.0.0.0':
        os.exitpoint()
    
    #查看ip配置
    print(sta.ifconfig())
    #查看是否连接
    print(sta.isconnected())
    #断开连接
    sta.disconnect()
    #查看sta状态
    print(sta.status())
    #连接ap
    sta.connect("Canaan","Canaan314")
    #查看状态
    print(sta.status())

sta_test()

