#配置 tcp/udp socket调试工具
import socket
import time,os
import network

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

def udpserver():
    #获取lan接口
    ip = network_use_wlan(True)
        
    #获取地址及端口号对应地址
    ai = socket.getaddrinfo(ip, 8080)
    #ai = socket.getaddrinfo("10.10.1.94", 60000)
    print("Address infos:", ai)
    addr = ai[0][-1]

    print("udp server %s port:%d\n" % ((ip),8080))
    #建立socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #设置属性
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #绑定
    s.bind(addr)
    #延时
    time.sleep(1)
    
    counter=0
    while True :
        os.exitpoint()
        data, addr = s.recvfrom(800)
        if data == b"":
            continue
        print("recv %d" % counter,data,addr)
        #回复内容
        s.sendto(b"%s have recv count=%d " % (data,counter), addr)
        counter = counter+1
        if counter > 10 :
            break
    #关闭
    s.close()
    print("udp server exit!!")




#main()
udpserver()
