#配置 tcp/udp socket调试工具
import socket
import os,time
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


def udpclient():
    #获取lan接口
    network_use_wlan(True)
    
    #获取地址和端口号 对应地址
    ai = socket.getaddrinfo('192.168.1.110', 8080)
    #ai = socket.getaddrinfo("10.10.1.94", 60000)
    print("Address infos:", ai)
    addr = ai[0][-1]

    print("Connect address:", addr)
    #建立socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for i in range(10):
        str="K230 udp client send test {0} \r\n".format(i)
        print(str)
        #发送字符串
        print(s.sendto(str,addr))
        #延时
        time.sleep(0.2)
        #time.sleep(1)
        #print(s.recv(4096))
        #print(s.read())
    #延时
    time.sleep(1)
    #关闭
    s.close()
    print("end")



#main()
udpclient()