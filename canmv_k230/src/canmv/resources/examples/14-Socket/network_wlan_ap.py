import network

def ap_test():
    ap=network.WLAN(network.AP_IF)
    #配置并创建ap
    ap.config(ssid='k230_ap_wjx', key='12345678')
    #查看ap信息
    print(ap.info())
    #查看ap的状态
    print(ap.status())

ap_test()

