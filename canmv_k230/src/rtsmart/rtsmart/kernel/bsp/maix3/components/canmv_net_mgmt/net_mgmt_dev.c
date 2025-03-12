#include "rtdef.h"
#include <stdbool.h>

#include <rthw.h>
#include <rtthread.h>

#include <lwp.h>
#include <lwp_user_mm.h>

#include <netdev_ipaddr.h>
#include <netdev.h>
#include <netdb.h>
#define CONFIG_ENABLE_NETWORK_RT_WLAN
#define CONFIG_ENABLE_NETWORK_RT_LAN
#ifdef CONFIG_ENABLE_NETWORK_RT_WLAN
    #include <wlan_mgnt.h>
#endif // CONFIG_ENABLE_NETWORK_RT_WLAN

#define DBG_TAG "NetMgmtDev"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef CONFIG_ENABLE_NETWORK_RT_WLAN
    // wlan basic
    #define IOCTRL_WM_GET_AUTO_RECONNECT 0x00
    #define IOCTRL_WM_SET_AUTO_RECONNECT 0x01
    #define IOCTRL_WM_GET_ISACTIVE      0x02

    // wlan sta
    #define IOCTRL_WM_STA_CONNECT 0x10
    #define IOCTRL_WM_STA_DISCONNECT 0x11
    #define IOCTRL_WM_STA_IS_CONNECTED 0x12
    #define IOCTRL_WM_STA_GET_MAC 0x13
    #define IOCTRL_WM_STA_SET_MAC 0x14
    #define IOCTRL_WM_STA_GET_AP_INFO 0x15
    #define IOCTRL_WM_STA_GET_RSSI 0x16
    #define IOCTRL_WM_STA_SCAN 0x17

    // wlan ap
    #define IOCTRL_WM_AP_START  0x20
    #define IOCTRL_WM_AP_STOP  0x21
    #define IOCTRL_WM_AP_IS_ACTIVE  0x22
    #define IOCTRL_WM_AP_GET_INFO  0x23
    #define IOCTRL_WM_AP_GET_STA_INFO  0x24
    #define IOCTRL_WM_AP_DEAUTH_STA  0x25
    #define IOCTRL_WM_AP_GET_COUNTRY  0x26
    #define IOCTRL_WM_AP_SET_COUNTRY  0x27
#endif // CONFIG_ENABLE_NETWORK_RT_WLAN

#ifdef CONFIG_ENABLE_NETWORK_RT_LAN
    // lan
    #define IOCTRL_LAN_GET_ISCONNECTED 0x80
    #define IOCTRL_LAN_GET_ISACTIVE   0x81
    #define IOCTRL_LAN_GET_STATUS   0x82
    #define IOCTRL_LAN_GET_MAC   0x83
    #define IOCTRL_LAN_SET_MAC   0x84
#endif // CONFIG_ENABLE_NETWORK_RT_LAN

// network util
#define IOCTRL_NET_IFCONFIG 0x100
#define IOCTRL_NET_GETHOSTBYNAME 0x101

struct rt_net_mgmt_device
{
    struct rt_device device;
    // struct rt_mutex lock;
};
static struct rt_net_mgmt_device net_mgmt_device;

/* WLAN CMD HANDLE ***********************************************************/
#ifdef CONFIG_ENABLE_NETWORK_RT_WLAN
struct rt_wlan_connect_config {
    int use_info;
    union {
        rt_wlan_ssid_t ssid;
        struct rt_wlan_info info;
    };
    rt_wlan_key_t key;
};

static rt_err_t _wlan_mgmt_cmd_basic_get_auto_reconnect(void *mgmt_dev, void *args)
{
    int auto_reconnect = (int)rt_wlan_get_autoreconnect_mode();

    lwp_put_to_user(args, &auto_reconnect, sizeof(int));

    return RT_EOK;
}

static rt_err_t _wlan_mgmt_cmd_basic_set_auto_reconnect(void *mgmt_dev, void *args)
{
    int auto_reconnect = 1;

    lwp_get_from_user(&auto_reconnect, args, sizeof(int));

    rt_wlan_config_autoreconnect((rt_bool_t)(auto_reconnect & 0x01));

    return RT_EOK;
}

static rt_err_t _wlan_mgmt_cmd_sta_connect(void *mgmt_dev, void *args)
{
    rt_err_t err = RT_EOK;
    struct rt_wlan_connect_config config;

    lwp_get_from_user(&config, args, sizeof(struct rt_wlan_connect_config));

    if(config.use_info) {
        err = rt_wlan_connect_adv(&config.info, (char *)config.key.val);
    } else {
        err = rt_wlan_connect((char *)config.ssid.val, (char *)config.key.val);
    }

    return err;
}

static rt_err_t _wlan_mgmt_cmd_sta_disconnect(void *mgmt_dev, void *args)
{
    return rt_wlan_disconnect();
}

static rt_err_t _wlan_mgmt_cmd_sta_isconnected(void *mgmt_dev, void *args)
{
    int status = (int)rt_wlan_is_connected();

    lwp_put_to_user(args, &status, sizeof(int));

    return RT_EOK;
}

static rt_err_t _wlan_mgmt_cmd_sta_get_mac(void *mgmt_dev, void *args)
{
    rt_err_t err = RT_EOK;
    uint8_t mac[6];

    err = rt_wlan_get_mac(mac);
    lwp_put_to_user(args, mac, sizeof(mac));

    return err;
}

static rt_err_t _wlan_mgmt_cmd_sta_set_mac(void *mgmt_dev, void *args)
{
    uint8_t mac[6];

    lwp_get_from_user(&mac[0], args, sizeof(mac));

    return rt_wlan_set_mac(mac);
}

static rt_err_t _wlan_mgmt_cmd_sta_get_ap_info(void *mgmt_dev, void *args)
{
    rt_err_t err = RT_EOK;
    struct rt_wlan_info info;

    err = rt_wlan_get_info(&info);

    lwp_put_to_user(args, &info, sizeof(struct rt_wlan_info));

    return err;
}

static rt_err_t _wlan_mgmt_cmd_sta_get_rssi(void *mgmt_dev, void *args)
{
    int rssi = rt_wlan_get_rssi();

    lwp_put_to_user(args, &rssi, sizeof(int));

    return RT_EOK;
}

static rt_err_t _wlan_mgmt_cmd_sta_scan(void *mgmt_dev, void *args)
{
    struct rt_wlan_scan_result *result = (struct rt_wlan_scan_result *)args;
    struct rt_wlan_scan_result *scan_result = RT_NULL;

    int32_t result_num;
    struct rt_wlan_info info;

    lwp_get_from_user(&result_num, &result->num, sizeof(int32_t));
    lwp_get_from_user(&info, &result->info[0], sizeof(struct rt_wlan_info));

    /* clean scan result */
    rt_wlan_scan_result_clean();
    /* scan ap info */
    scan_result = rt_wlan_scan_with_info(&info);

    if(scan_result) {
        if(scan_result->num > result_num) {
            LOG_W("Scan result count(%d) bigger than user requset(%d)\n", scan_result->num, result_num);
            scan_result->num = result_num;
        }

        lwp_put_to_user(&result->num, &scan_result->num, sizeof(int32_t));

        for(int i = 0; i< scan_result->num; i++) {
            lwp_put_to_user(&result->info[i], &scan_result->info[i], sizeof(struct rt_wlan_info));
        }
        rt_wlan_scan_result_clean();

        return RT_EOK;
    }

    LOG_E("Can't scan ap\n");

    return -1;
}

static rt_err_t _wlan_mgmt_cmd_ap_start(void *mgmt_dev, void *args)
{
    rt_err_t err = RT_EOK;
    struct rt_wlan_connect_config config;

    lwp_get_from_user(&config, args, sizeof(struct rt_wlan_connect_config));

    if(config.use_info) {
        err = rt_wlan_start_ap_adv(&config.info, (char *)config.key.val);
    } else {
        err = rt_wlan_start_ap((char *)config.ssid.val, (char *)config.key.val);
    }

    return err;
}

static rt_err_t _wlan_mgmt_cmd_ap_stop(void *mgmt_dev, void *args)
{
    return rt_wlan_ap_stop();
}

static rt_err_t _wlan_mgmt_cmd_ap_isactive(void *mgmt_dev, void *args)
{
    int active = (int)rt_wlan_ap_is_active();
    lwp_put_to_user(args, &active, sizeof(int));

    return RT_EOK;
}

static rt_err_t _wlan_mgmt_cmd_ap_get_info(void *mgmt_dev, void *args)
{
    rt_err_t err = RT_EOK;
    struct rt_wlan_info info;

    err = rt_wlan_ap_get_info(&info);

    lwp_put_to_user(args, &info, sizeof(struct rt_wlan_info));

    return err;
}

static rt_err_t _wlan_mgmt_cmd_ap_get_stations(void *mgmt_dev, void *args)
{
    struct rt_wlan_scan_result *result = (struct rt_wlan_scan_result *)args;
    int32_t result_num;

    int station_num = 0;
    struct rt_wlan_info *info = NULL;

    station_num = rt_wlan_ap_get_sta_num();
    lwp_get_from_user(&result_num, &result->num, sizeof(int32_t));

    if(station_num > result_num) {
        LOG_W("Station count(%d) bigger than user requset(%d)\n", station_num, result_num);
        station_num = result_num;
    }

    info = malloc(sizeof(struct rt_wlan_info) * station_num);
    if(NULL == info) {
        LOG_E("No memory\n");
        return -1;
    }

    rt_wlan_ap_get_sta_info(info, station_num);

    lwp_put_to_user(&result->num, &station_num, sizeof(int32_t));
    lwp_put_to_user(result->info, info, sizeof(struct rt_wlan_info) * station_num);

    free(info);

    return RT_EOK;
}

static rt_err_t _wlan_mgmt_cmd_ap_deauth_sta(void *mgmt_dev, void *args)
{
    uint8_t mac[6];

    lwp_get_from_user(mac, args, sizeof(mac));

    return rt_wlan_ap_deauth_sta(mac);
}

static rt_err_t _wlan_mgmt_cmd_ap_get_country(void *mgmt_dev, void *args)
{
    int country = (int)rt_wlan_ap_get_country();

    lwp_put_to_user(args, &country, sizeof(int));

    return RT_EOK;
}

static rt_err_t _wlan_mgmt_cmd_ap_set_country(void *mgmt_dev, void *args)
{
    int country = 0;

    lwp_get_from_user(&country, args, sizeof(int));

    return rt_wlan_ap_set_country((rt_country_code_t)country);
}
#endif // CONFIG_ENABLE_NETWORK_RT_WLAN

#ifdef CONFIG_ENABLE_NETWORK_RT_LAN
/* LAN CMD HANDLE ************************************************************/
static rt_err_t _lan_mgmt_cmd_get_isconnected(void *mgmt_dev, void *args)
{
    int isconnected = 0;
    bool connect_status;

    ip_addr_t ip;

    rt_device_t eth_dev;
    struct netdev *netdev = NULL;

    if(NULL == (eth_dev = rt_device_find(CANMV_USB_HOST_NET_RTL8152_DEV_NAME))) {
        goto _failed;
    }

    if(RT_EOK != rt_device_control(eth_dev, 0x1001 /* get connect status */, &connect_status) || (false == connect_status)) {
        goto _failed;
    }

    /* eth rtl8152 device and netdev name is same */
    netdev = netdev_get_by_name(CANMV_USB_HOST_NET_RTL8152_DEV_NAME);
    if(NULL == netdev) {
        goto _failed;
    }

    memcpy(&ip, &netdev->ip_addr, sizeof(ip_addr_t));
    if(0x00 == ip.addr) {
        goto _failed;
    }

    isconnected = 1;
    lwp_put_to_user(args, &isconnected, sizeof(isconnected));

    return RT_EOK;

_failed:
    isconnected = 0;
    lwp_put_to_user(args, &isconnected, sizeof(isconnected));

    return RT_EOK;
}

static rt_err_t _lan_mgmt_cmd_get_status(void *mgmt_dev, void *args)
{
    int status = 0;
    bool connect_status;

    rt_device_t eth_dev;

    if(NULL == (eth_dev = rt_device_find(CANMV_USB_HOST_NET_RTL8152_DEV_NAME))) {
        status = 0;
        goto _exit;
    }

    if(RT_EOK != rt_device_control(eth_dev, 0x1001 /* get connect status */, &connect_status)) {
        status = 0;
        goto _exit;
    }

    if(connect_status) {
        status = 1;
    } else {
        status = 2;
    }

_exit:
    lwp_put_to_user(args, &status, sizeof(status));

    return RT_EOK;
}

static rt_err_t _lan_mgmt_cmd_get_mac(void *mgmt_dev, void *args)
{
    uint8_t mac[6];

    rt_device_t eth_dev;

    if(NULL == (eth_dev = rt_device_find(CANMV_USB_HOST_NET_RTL8152_DEV_NAME))) {
        goto _failed;
    }
    if(RT_EOK != rt_device_control(eth_dev, 0x01 /* NIOCTL_GADDR */, &mac[0])) {
        goto _failed;
    }

    lwp_put_to_user(args, &mac[0], sizeof(mac));

    return RT_EOK;

_failed:

    return RT_ERROR;
}

static rt_err_t _lan_mgmt_cmd_set_mac(void *mgmt_dev, void *args)
{
    uint8_t mac[6];

    rt_device_t eth_dev;

    if(sizeof(mac) != lwp_get_from_user(&mac[0], args, sizeof(mac))) {
        rt_kprintf("get mac failed\n");

        return RT_ERROR;
    }

    if(NULL == (eth_dev = rt_device_find(CANMV_USB_HOST_NET_RTL8152_DEV_NAME))) {
        goto _failed;
    }

    if(RT_EOK != rt_device_control(eth_dev, 0x1000 /* set mac */, &mac[0])) {
        goto _failed;
    }

    return RT_EOK;

_failed:

    return RT_ERROR;
}
#endif // CONFIG_ENABLE_NETWORK_RT_LAN

/* UTIL CMD HANDLE ***********************************************************/
static rt_err_t _net_mgmt_cmd_get_isactive(void *mgmt_dev, void *args)
{
    int itf = -1;
    int isactive = 0;
    rt_device_t net_dev;
    const char *dev_name;

    lwp_get_from_user(&itf, args, sizeof(itf));

    if(0x00 == itf) { /* wlan station */
        net_dev = rt_device_find(RT_WLAN_DEVICE_STA_NAME);
    } else if(0x01 == itf) { /* wlan ap */
        net_dev = rt_device_find(RT_WLAN_DEVICE_AP_NAME);
    } else if(0x02 == itf) { /* eth usb */
        net_dev = rt_device_find(CANMV_USB_HOST_NET_RTL8152_DEV_NAME);
    }

    isactive = (NULL == net_dev) ? 0 : 1;
    lwp_put_to_user(args, &isactive, sizeof(isactive));

    return RT_EOK;
}

static rt_err_t _net_mgmt_dev_cmd_net_ifconfig(void *mgmt_dev, void *args)
{
    struct ifconfig {
        uint16_t net_if;            /* 0: sta, 1: ap, 2:... */
        uint16_t set;               /* 0: get ip info, 1: disable dhcp, set static ip, 2: enable dhcp */
        ip_addr_t ip;               /* IP address */
        ip_addr_t gw;               /* gateway */
        ip_addr_t netmask;          /* subnet mask */
        ip_addr_t dns;              /* DNS server */
    };
    struct ifconfig ifconfig;
    struct ifconfig *out = (struct ifconfig *)args;

    struct rt_wlan_device *wlan_dev = NULL;
    struct netdev *netdev = NULL;

    lwp_get_from_user(&ifconfig, args, sizeof(struct ifconfig));

    if(0x00 == ifconfig.net_if) { /* wlan station */
        wlan_dev = (struct rt_wlan_device *)rt_device_find(RT_WLAN_DEVICE_STA_NAME);
        if(NULL == wlan_dev) {
            LOG_E("Can't find netif %s\n", RT_WLAN_DEVICE_STA_NAME);
            return -RT_ERROR;
        }
        netdev = wlan_dev->netdev;
    } else if(0x01 == ifconfig.net_if) { /* wlan ap */
        wlan_dev = (struct rt_wlan_device *)rt_device_find(RT_WLAN_DEVICE_AP_NAME);
        if(NULL == wlan_dev) {
            LOG_E("Can't find netif %s\n", RT_WLAN_DEVICE_STA_NAME);
            return -RT_ERROR;
        }
        netdev = wlan_dev->netdev;
    } else if(0x02 == ifconfig.net_if) { /* eth usb */
        /* eth rtl8152 device and netdev name is same */
        netdev = netdev_get_by_name(CANMV_USB_HOST_NET_RTL8152_DEV_NAME);
        if(NULL == netdev) {
            LOG_E("Can't find netif %s\n", CANMV_USB_HOST_NET_RTL8152_DEV_NAME);
            return -RT_ERROR;
        }
    } else {
        LOG_E("Unsupport netif %d\n", ifconfig.net_if);
        return -RT_ERROR;
    }

    if(NULL == netdev) {
        LOG_E("Can't find netif %d\n", ifconfig.net_if);
        return -RT_ERROR;
    }

    if(0x00 == ifconfig.set) { /* get */
        lwp_put_to_user(&out->ip, &netdev->ip_addr, sizeof(ip_addr_t));
        lwp_put_to_user(&out->gw, &netdev->gw, sizeof(ip_addr_t));
        lwp_put_to_user(&out->netmask, &netdev->netmask, sizeof(ip_addr_t));
        lwp_put_to_user(&out->dns, &netdev->dns_servers[0], sizeof(ip_addr_t));
    } else if (0x01 == ifconfig.set){ /* set static ip */
        if(0x00 != netdev_dhcp_enabled(netdev, RT_FALSE)) {
            LOG_E("Set itf %s disable dhcp failed\n", netdev->name);
            return -RT_ERROR;
        }

        if(0x00 != netdev_set_ipaddr(netdev, &ifconfig.ip)) {
            LOG_E("Set itf %s ip failed\n", netdev->name);
            return -RT_ERROR;
        }
        if(0x00 != netdev_set_gw(netdev, &ifconfig.gw)) {
            LOG_E("Set itf %s gw failed\n", netdev->name);
            return -RT_ERROR;
        }
        if(0x00 != netdev_set_netmask(netdev, &ifconfig.netmask)) {
            LOG_E("Set itf %s netmask failed\n", netdev->name);
            return -RT_ERROR;
        }
        if(0x00 != netdev_set_dns_server(netdev, 1, &ifconfig.dns)) {
            LOG_E("Set itf %s dns failed\n", netdev->name);
            return -RT_ERROR;
        }
    } else if(0x02 == ifconfig.set) {
        if(0x00 != netdev_dhcp_enabled(netdev, RT_TRUE)) {
            LOG_E("Set itf %s enable dhcp failed\n", netdev->name);
            return -RT_ERROR;
        }
    }

    return RT_EOK;
}

static rt_err_t _net_mgmt_dev_cmd_net_gethostbyname(void *mgmt_dev, void *args)
{
    struct result {
        uint8_t ip[4];
        int name_len; // max 256
        char name[0];
    };

    uint64_t buffer[(sizeof(struct result) + 256) / sizeof(uint64_t) + 1];
    struct result *request = (struct result *)args;
    struct result *result = (struct result *)buffer;
    struct hostent *host = NULL;

    lwp_get_from_user(&result->name_len, &request->name_len, sizeof(int));
    if(256 < result->name_len) {
        LOG_E("Host name too long\n");
        return -1;
    }

    lwp_get_from_user(&result->name[0], &request->name[0], result->name_len);
    result->name[result->name_len] = '\0';
    if(NULL == (host = gethostbyname(result->name))) {
        LOG_W("get host failed1, %s\n", result->name);
        return -2;
    }

    if(NULL == host->h_addr_list) {
        LOG_W("get host failed2, %s\n", result->name);
        return -2;
    }

    lwp_put_to_user(request->ip, host->h_addr_list[0], sizeof(request->ip));

    return 0;
}

struct rt_net_mgmt_device_cmd_handle {
    int cmd;
    rt_err_t (*func)(void *mgmt_dev, void *args);
};

static struct rt_net_mgmt_device_cmd_handle cmd_handles[] = {
#ifdef CONFIG_ENABLE_NETWORK_RT_WLAN
    // wlan basic
    {
        .cmd = IOCTRL_WM_GET_AUTO_RECONNECT,
        .func = _wlan_mgmt_cmd_basic_get_auto_reconnect,
    },
    {
        .cmd = IOCTRL_WM_SET_AUTO_RECONNECT,
        .func = _wlan_mgmt_cmd_basic_set_auto_reconnect,
    },
    {
        .cmd = IOCTRL_WM_GET_ISACTIVE,
        .func = _net_mgmt_cmd_get_isactive,
    },

    // station
    {
        .cmd = IOCTRL_WM_STA_CONNECT,
        .func = _wlan_mgmt_cmd_sta_connect,
    },
    {
        .cmd = IOCTRL_WM_STA_DISCONNECT,
        .func = _wlan_mgmt_cmd_sta_disconnect,
    },
    {
        .cmd = IOCTRL_WM_STA_IS_CONNECTED,
        .func = _wlan_mgmt_cmd_sta_isconnected,
    },
    {
        .cmd = IOCTRL_WM_STA_GET_MAC,
        .func = _wlan_mgmt_cmd_sta_get_mac,
    },
    {
        .cmd = IOCTRL_WM_STA_SET_MAC,
        .func = _wlan_mgmt_cmd_sta_set_mac,
    },
    {
        .cmd = IOCTRL_WM_STA_GET_AP_INFO,
        .func = _wlan_mgmt_cmd_sta_get_ap_info,
    },
    {
        .cmd = IOCTRL_WM_STA_GET_RSSI,
        .func = _wlan_mgmt_cmd_sta_get_rssi,
    },
    {
        .cmd = IOCTRL_WM_STA_SCAN,
        .func = _wlan_mgmt_cmd_sta_scan,
    },

    // ap
    {
        .cmd = IOCTRL_WM_AP_START,
        .func = _wlan_mgmt_cmd_ap_start,
    },
    {
        .cmd = IOCTRL_WM_AP_STOP,
        .func = _wlan_mgmt_cmd_ap_stop,
    },
    {
        .cmd = IOCTRL_WM_AP_IS_ACTIVE,
        .func = _wlan_mgmt_cmd_ap_isactive,
    },
    {
        .cmd = IOCTRL_WM_AP_GET_INFO,
        .func = _wlan_mgmt_cmd_ap_get_info,
    },
    {
        .cmd = IOCTRL_WM_AP_GET_STA_INFO,
        .func = _wlan_mgmt_cmd_ap_get_stations,
    },
    {
        .cmd = IOCTRL_WM_AP_DEAUTH_STA,
        .func = _wlan_mgmt_cmd_ap_deauth_sta,
    },
    {
        .cmd = IOCTRL_WM_AP_GET_COUNTRY,
        .func = _wlan_mgmt_cmd_ap_get_country,
    },
    {
        .cmd = IOCTRL_WM_AP_SET_COUNTRY,
        .func = _wlan_mgmt_cmd_ap_set_country,
    },
#endif // CONFIG_ENABLE_NETWORK_RT_WLAN

#ifdef CONFIG_ENABLE_NETWORK_RT_LAN
    // lan
    {
        .cmd = IOCTRL_LAN_GET_ISCONNECTED,
        .func = _lan_mgmt_cmd_get_isconnected,
    },
    {
        .cmd = IOCTRL_LAN_GET_ISACTIVE,
        .func = _net_mgmt_cmd_get_isactive,
    },
    {
        .cmd = IOCTRL_LAN_GET_STATUS,
        .func = _lan_mgmt_cmd_get_status,
    },
    {
        .cmd = IOCTRL_LAN_GET_MAC,
        .func = _lan_mgmt_cmd_get_mac,
    },
    {
        .cmd = IOCTRL_LAN_SET_MAC,
        .func = _lan_mgmt_cmd_set_mac,
    },
#endif // CONFIG_ENABLE_NETWORK_RT_LAN

    // network
    {
        .cmd = IOCTRL_NET_IFCONFIG,
        .func = _net_mgmt_dev_cmd_net_ifconfig,
    },
    {
        .cmd = IOCTRL_NET_GETHOSTBYNAME,
        .func = _net_mgmt_dev_cmd_net_gethostbyname,
    },
};

static rt_err_t _net_mgmt_dev_control(rt_device_t dev, int cmd, void *args)
{
    struct rt_net_mgmt_device *mgmt_dev = (struct rt_net_mgmt_device *)dev;
    const size_t handle_count = sizeof(cmd_handles) / sizeof(cmd_handles[0]);

    for(size_t i = 0; i < handle_count; i++) {
        if(cmd == cmd_handles[i].cmd) {
            return cmd_handles[i].func(mgmt_dev, args);
        }
    }

    LOG_E("Unsupport cmd 0x%x\n", cmd);

    return -RT_ERROR;
}

static rt_err_t  _net_mgmt_dev_init(rt_device_t dev)
{
    (void)dev;
#ifdef CONFIG_ENABLE_NETWORK_RT_WLAN
    rt_wlan_config_autoreconnect(RT_TRUE);

    return (rt_err_t)rt_wlan_init();
#else
    return RT_EOK;
#endif
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops net_mgmt_dev_ops =
{
    _net_mgmt_dev_init,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    RT_NULL,
    _net_mgmt_dev_control
};
#endif

static int net_mgmt_dev_init(void)
{
    rt_err_t err = RT_EOK;
    static rt_int8_t _init_flag = 0;

    /* Execute only once */
    if (_init_flag == 0) {
        rt_memset(&net_mgmt_device, 0, sizeof(struct rt_net_mgmt_device));

#ifdef RT_USING_DEVICE_OPS
        net_mgmt_device.device.ops = &net_mgmt_dev_ops;
#else
        net_mgmt_device.device.init       = _wlan_mgmt_init;
        net_mgmt_device.device.open       = RT_NULL;
        net_mgmt_device.device.close      = RT_NULL;
        net_mgmt_device.device.read       = RT_NULL;
        net_mgmt_device.device.write      = RT_NULL;
        net_mgmt_device.device.control    = _rt_wlan_dev_control;
#endif
        net_mgmt_device.device.user_data = RT_NULL;
        net_mgmt_device.device.type = RT_Device_Class_Miscellaneous;

        // rt_mutex_init(&net_mgmt_device.lock, "wlan_mgmt_dev", RT_IPC_FLAG_FIFO);

        if(RT_EOK != (err = rt_device_register(&net_mgmt_device.device, "canmv_net_mgmt", RT_DEVICE_FLAG_RDWR))) {
            LOG_E("net_mgmt_device register failed, %d\n", errno);
        }
    }
    return 0;
}
INIT_APP_EXPORT(net_mgmt_dev_init);
