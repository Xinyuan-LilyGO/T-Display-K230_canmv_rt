import network
import socket
import os,time

def network_use_wlan(is_wlan=True):
    if is_wlan:
        sta=network.WLAN(0)
        sta.connect("Canaan","Canaan314")
        print(sta.status())
        while sta.ifconfig()[0] == '0.0.0.0':
            os.exitpoint()
        print(sta.ifconfig())
        ip = sta.ifconfig()[0]
        return ip
    else:
        a=network.LAN()
        if not a.active():
            raise RuntimeError("LAN interface is not active.")
        a.ifconfig("dhcp")
        print(a.ifconfig())
        ip = a.ifconfig()[0]
        return ip

def main(use_stream=True):
    
    #获取lan接口
    network_use_wlan(True)
    #创建socket
    s = socket.socket()
    #获取地址及端口号 对应地址
    ai = []
        
    for attempt in range(0, 3):
        try:
            #获取地址及端口号 对应地址
            ai = socket.getaddrinfo("www.baidu.com", 80)
            break
        except:
            print("getaddrinfo again");
    
    if(ai == []):
        print("connet error")
        s.close()
        return

    print("Address infos:", ai)
    addr = ai[0][-1]
    
    print("Connect address:", addr)
    #连接
    s.connect(addr)

    if use_stream:
        # MicroPython socket objects support stream (aka file) interface
        # directly, but the line below is needed for CPython.
        s = s.makefile("rwb", 0)
        #发送http请求
        s.write(b"GET /index.html HTTP/1.0\r\n\r\n")
        #打印请求内容
        print(s.read())
    else:
        #发送http请求
        s.send(b"GET /index.html HTTP/1.0\r\n\r\n")
        #打印请求内容
        print(s.recv(4096))
        #print(s.read())
    #关闭socket
    s.close()


#main()
main(use_stream=True)
main(use_stream=False)