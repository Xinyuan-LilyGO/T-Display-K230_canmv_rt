#配置 tcp/udp socket调试工具
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

def client():
    #获取lan接口
    network_use_wlan(True)
    
    #建立socket
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM, 0)
    #获取地址及端口号 对应地址
    ai = socket.getaddrinfo("192.168.1.110", 8080)
    #ai = socket.getaddrinfo("10.10.1.94", PORT)
    print("Address infos:", ai)
    addr = ai[0][-1]

    print("Connect address:", addr)
    #连接地址
    if(s.connect(addr) == False):
        s.close()
        print("conner err")
        return

    for i in range(10):
        str="K230 tcp client send test {0} \r\n".format(i)
        print(str)
        #print(s.send(str))
        #发送字符串
        print(s.write(str))
        time.sleep(0.2)
        #time.sleep(1)
        #print(s.recv(4096))
        #print(s.read())
    #延时1秒
    time.sleep(1)
    #关闭socket
    s.close()
    print("end")



#main()
client()


