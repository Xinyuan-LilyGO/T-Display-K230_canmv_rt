#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netdb.h>

#include <unistd.h>    // For close()
#include <sys/ioctl.h>
#include <sys/types.h> // For socket types
#include <sys/socket.h>
#include <poll.h>
#include <netinet/in.h> // For sockaddr_in

#include "network_rt_opt.h"

static int net_mgmt_dev_fd = -1;

int network_rt_init()
{
    int fd = open("/dev/canmv_net_mgmt", O_RDWR);
    if (fd < 0) {
        perror("open /dev/canmv_net_mgmt fail");
        return -1;
    }
    net_mgmt_dev_fd=fd;
    return fd;
}

void network_rt_deinit(void)
{
    close(net_mgmt_dev_fd);
    net_mgmt_dev_fd = -1;
}
int lan_get_mac(uint8_t*mac_buff_addr)
{   int ret_code=-1;
    uint8_t mac_buf[6];
    memset(mac_buf,0,6);
    if(0x00== ioctl(net_mgmt_dev_fd, IOCTRL_LAN_GET_MAC, &mac_buf[0])) { 
    memcpy(mac_buff_addr,mac_buf,6);
    ret_code=0;
    }
    return ret_code;
}

bool lan_is_active()
{
    int isactive = 0;
    if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_LAN_GET_ISACTIVE, &isactive)) {
       if(isactive==1)
        {
        return true;
        }
   }
            
return false;
}

static void format_ipv4_addr_to_str(uint8_t *ip, char*ip_str) {
    char ip_str_arr[16]={0};
    sprintf(ip_str_arr, "%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
    memcpy(ip_str,ip_str_arr,16);
}

static void format_ipv4_str_to_addr(char*ip_str,uint8_t *ip) {

  char *temp = strtok(ip_str,".");
  int i=0;
    while(temp)
    {
    if(i>3)
    {break;}
        ip[3-i]=atoi(temp);
        i++;
        temp = strtok(NULL,".");
    }
 }



int  get_net_ifconfig(int _net_if,char*ip_addr_str,char* netmask_addr_str,char* gw_addr_str,char* dns_addr_str) {
int ret_code=-1;
 struct ifconfig {
        uint16_t net_if;            /* 0: sta, 1: ap, 2:lan, 3:... */
        uint16_t set;               /* 0:get, 1:set */
        ip_addr_t ip;               /* IP address */
        ip_addr_t gw;               /* gateway */
        ip_addr_t netmask;          /* subnet mask */
        ip_addr_t dns;              /* DNS server */
    };
    struct ifconfig ifconfig;
    ifconfig.net_if = _net_if;
        // get ifconfig info
    ifconfig.set = 0;

    if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_NET_IFCONFIG, &ifconfig)) {
    ret_code=0;
    format_ipv4_addr_to_str((uint8_t *)&ifconfig.ip.addr, ip_addr_str);
    format_ipv4_addr_to_str((uint8_t *)&ifconfig.netmask.addr, netmask_addr_str);
    format_ipv4_addr_to_str((uint8_t *)&ifconfig.gw.addr, gw_addr_str);
    format_ipv4_addr_to_str((uint8_t *)&ifconfig.dns.addr, dns_addr_str);
    }            
return ret_code;
}
int  set_net_ifconfig(int _net_if,char*ip_addr_str,char* netmask_addr_str,char* gw_addr_str,char* dns_addr_str) {
int ret_code=-1;
 struct ifconfig {
        uint16_t net_if;            /* 0: sta, 1: ap, 2:lan, 3:... */
        uint16_t set;               /* 0:get, 1:set */
        ip_addr_t ip;               /* IP address */
        ip_addr_t gw;               /* gateway */
        ip_addr_t netmask;          /* subnet mask */
        ip_addr_t dns;              /* DNS server */
    };
    struct ifconfig ifconfig;
    ifconfig.net_if = _net_if;
    
    uint8_t ip_addr[4]={0};
    uint8_t netmask_addr[4]={0};
    uint8_t gw_addr[4]={0};
    uint8_t dns_addr[4]={0};
    format_ipv4_str_to_addr(ip_addr_str,ip_addr);
    format_ipv4_str_to_addr(netmask_addr_str,netmask_addr);
    format_ipv4_str_to_addr(gw_addr_str,gw_addr);
    format_ipv4_str_to_addr(dns_addr_str,dns_addr);
    ifconfig.ip.addr=(uint32_t)((ip_addr[0]<<24)|(ip_addr[1]<<16)|(ip_addr[2]<<8)|(ip_addr[3]<<0));
    ifconfig.netmask.addr=(uint32_t)((netmask_addr[0]<<24)|(netmask_addr[1]<<16)|(netmask_addr[2]<<8)|(netmask_addr[3]<<0));
    ifconfig.gw.addr=(uint32_t)((gw_addr[0]<<24)|(gw_addr[1]<<16)|(gw_addr[2]<<8)|(gw_addr[3]<<0));
    ifconfig.dns.addr=(uint32_t)((dns_addr[0]<<24)|(dns_addr[1]<<16)|(dns_addr[2]<<8)|(dns_addr[3]<<0));
        // set ifconfig info
    ifconfig.set = 1;
    if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_NET_IFCONFIG, &ifconfig)) {
    ret_code=0;
    }            
return ret_code;



}
int  set_net_ifconfig_dhcp(int _net_if) {
int ret_code=-1;
 struct ifconfig {
        uint16_t net_if;            /* 0: sta, 1: ap, 2:lan, 3:... */
        uint16_t set;               /* 0:get, 1:set */
        ip_addr_t ip;               /* IP address */
        ip_addr_t gw;               /* gateway */
        ip_addr_t netmask;          /* subnet mask */
        ip_addr_t dns;              /* DNS server */
    };
    struct ifconfig ifconfig;
    ifconfig.net_if = _net_if;
        // set ifconfig info
    ifconfig.set = 2;
    if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_NET_IFCONFIG, &ifconfig)) {
    ret_code=0;
    }            
return ret_code;
}


















int wlan_get_mac(uint8_t*mac_buff_addr)
{   int ret_code=-1;
    uint8_t mac_buf[6];
    memset(mac_buf,0,6);
    if(0x00== ioctl(net_mgmt_dev_fd, IOCTRL_WM_STA_GET_MAC, &mac_buf[0])) {  
    memcpy(mac_buff_addr,mac_buf,6);
    ret_code=0;
    }
    return ret_code;
}
#if 0
int network_rt_wlan_scan(struct rt_wlan_scan_result *_scan_result) {
    int ret_code=-1;
    struct rt_wlan_scan_result *scan_result = NULL;
    size_t scan_result_info_size = 0;
  
    scan_result_info_size = sizeof(struct rt_wlan_info) * RT_WLAN_STA_SCAN_MAX_AP;
    scan_result = (struct rt_wlan_scan_result *)malloc(sizeof(struct rt_wlan_scan_result) + scan_result_info_size);
    memset(scan_result, 0, sizeof(struct rt_wlan_scan_result) + scan_result_info_size);

    scan_result->num = RT_WLAN_STA_SCAN_MAX_AP;
    scan_result->info = (struct rt_wlan_info *)(((uint8_t *)scan_result) + sizeof(struct rt_wlan_scan_result));
    INVALID_INFO(&scan_result->info[0]);

    if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_WM_STA_SCAN, scan_result)) {
    ret_code=0;
    memcpy(_scan_result,scan_result,sizeof(struct rt_wlan_scan_result) + scan_result_info_size);
      /*printf("scan_result->num:%d\n",scan_result->num);
      char ssid_buf[32]; 
      for(int32_t i = 0; i < scan_result->num; i++) {
        struct rt_wlan_info item_wlan_info=scan_result->info[i];
        memset(ssid_buf,0,32);
        memcpy(ssid_buf,item_wlan_info.ssid.val,item_wlan_info.ssid.len);
          printf("ssid:%s\n",ssid_buf);
    } */
    }
    free(scan_result);
    return ret_code; 
}
#endif
int network_rt_wlan_scan(struct rt_wlan_info* rt_wlan_info_list,int wlan_num, int *wlan_scan_num)
{

 int ret_code=-1;
    struct rt_wlan_scan_result *scan_result = NULL;
    size_t scan_result_info_size = 0;
  
    scan_result_info_size = sizeof(struct rt_wlan_info) * RT_WLAN_STA_SCAN_MAX_AP;
    scan_result = (struct rt_wlan_scan_result *)malloc(sizeof(struct rt_wlan_scan_result) + scan_result_info_size);
    memset(scan_result, 0, sizeof(struct rt_wlan_scan_result) + scan_result_info_size);

    scan_result->num = RT_WLAN_STA_SCAN_MAX_AP;
    scan_result->info = (struct rt_wlan_info *)(((uint8_t *)scan_result) + sizeof(struct rt_wlan_scan_result));
    INVALID_INFO(&scan_result->info[0]);

    if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_WM_STA_SCAN, scan_result)) {
       ret_code=0;
       *wlan_scan_num=scan_result->num;
       if(wlan_num<scan_result->num)
       {
           memcpy(rt_wlan_info_list,scan_result->info,sizeof(struct rt_wlan_info)*wlan_num);
       }
       else
       {
          memcpy(rt_wlan_info_list,scan_result->info,sizeof(struct rt_wlan_info)*scan_result->num);
       }
    }
    free(scan_result);
    return ret_code; 
}









int network_rt_wlan_connect(char* ssid,char*key) {
 int ret_code=-1;
    struct rt_wlan_connect_config {
        int use_info;
        union {
            rt_wlan_ssid_t ssid;
            struct rt_wlan_info info;
        };
        rt_wlan_key_t key;
    };
    struct rt_wlan_connect_config config;
    memset(&config, 0, sizeof(config));

        int ssid_len = strlen(ssid);
        if ((0x00 == ssid_len) || (RT_WLAN_SSID_MAX_LENGTH < ssid_len)) {
            ret_code=-2;
            return ret_code;
        }
        config.use_info = 0;
        config.ssid.len = ssid_len;
        strncpy((char *)&config.ssid.val[0], ssid, RT_WLAN_SSID_MAX_LENGTH);
        config.ssid.val[ssid_len] = '\0';
    int key_len = strlen(key);
     if((0x08 > key_len) || (RT_WLAN_PASSWORD_MAX_LENGTH < key_len)) {
       ret_code=-3;
            return ret_code; 
    }
    config.key.len = key_len;
    strncpy((char *)&config.key.val[0], key, RT_WLAN_PASSWORD_MAX_LENGTH);
    config.key.val[config.key.len] = '\0';

   
   if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_WM_STA_CONNECT, &config)) {
      ret_code=0;       
    }
     
  return ret_code; 
}

int network_rt_wlan_disconnect() {
int ret_code=-1;
   if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_WM_STA_DISCONNECT, NULL)) {
         ret_code=0;
    }
    return ret_code;
}

int network_rt_wlan_isconnected(int *_status) {
    int ret_code=-1;
    int status = -1;
    if(0x00 == ioctl(net_mgmt_dev_fd, IOCTRL_WM_STA_IS_CONNECTED, &status)) {
    *_status=status;
        ret_code=0;
    }

    return ret_code;
}






#if 0
STATIC mp_obj_t network_rt_net_ifconfig(size_t n_args, const mp_obj_t *args) {
    struct ifconfig {
        uint16_t net_if;            /* 0: sta, 1: ap, 2:lan, 3:... */
        uint16_t set;               /* 0:get, 1:set */
        ip_addr_t ip;               /* IP address */
        ip_addr_t gw;               /* gateway */
        ip_addr_t netmask;          /* subnet mask */
        ip_addr_t dns;              /* DNS server */
    };
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    struct ifconfig ifconfig;

    ifconfig.net_if = self->itf;

    if (n_args == 1) {
        // get ifconfig info
        ifconfig.set = 0;

        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_NET_IFCONFIG, &ifconfig)) {
            mp_printf(&mp_plat_print, "run ifconfig failed.\n");
            return mp_const_none;
        }

        mp_obj_t tuple[4] = {
            netutils_format_ipv4_addr((uint8_t *)&ifconfig.ip.addr, NETUTILS_BIG),
            netutils_format_ipv4_addr((uint8_t *)&ifconfig.netmask.addr, NETUTILS_BIG),
            netutils_format_ipv4_addr((uint8_t *)&ifconfig.gw.addr, NETUTILS_BIG),
            netutils_format_ipv4_addr((uint8_t *)&ifconfig.dns.addr, NETUTILS_BIG),
        };

        return mp_obj_new_tuple(4, tuple);
    } else {
        if (mp_obj_is_str(args[1])) {
            switch (mp_obj_str_get_qstr(args[1])) {
                case MP_QSTR_dhcp: {
                    ifconfig.set = 2; // enable dhcp
                } break;
                default: {
                    mp_raise_ValueError(MP_ERROR_TEXT("unknown config param"));
                } break;
            }
        } else {
            ifconfig.set = 1;   // disable dhcp, set static ip

            mp_obj_t *items;
            mp_obj_get_array_fixed_n(args[1], 4, &items);
            netutils_parse_ipv4_addr(items[0], (uint8_t *)&ifconfig.ip.addr, NETUTILS_BIG);
            netutils_parse_ipv4_addr(items[1], (uint8_t *)&ifconfig.netmask.addr, NETUTILS_BIG);
            netutils_parse_ipv4_addr(items[2], (uint8_t *)&ifconfig.gw.addr, NETUTILS_BIG);
            netutils_parse_ipv4_addr(items[3], (uint8_t *)&ifconfig.dns.addr, NETUTILS_BIG);
        }

        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_NET_IFCONFIG, &ifconfig)) {
            mp_printf(&mp_plat_print, "run ifconfig failed.\n");
            return mp_const_false;
        }

        return mp_const_true;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_rt_net_ifconfig_obj, 1, 2, network_rt_net_ifconfig);

/* NIC Protocol **************************************************************/
STATIC void network_rt_wlan_socket_close(struct _mod_network_socket_obj_t *socket);



STATIC int network_rt_wlan_socket_poll(mod_network_socket_obj_t *_socket, uint32_t rwf, int *_errno) {

    return 0;
}

STATIC int network_rt_wlan_socke_setblocking(mod_network_socket_obj_t *_socket, bool blocking, int *_errno) {
    int nonblocking = !blocking;
    // set socket in non-blocking mode
    if (ioctl(_socket->fileno, FIONBIO, &nonblocking) < 0) {
        *_errno = errno;
        network_rt_wlan_socket_close(_socket->fileno);
        return -1;
    }

    return 0;
}

STATIC int network_rt_wlan_socket_listening(mod_network_socket_obj_t *_socket, int *_errno) {
    int optval;
    socklen_t optlen = sizeof(optval);

    if (getsockopt(_socket->fileno, SOL_SOCKET, SO_ACCEPTCONN, &optval, &optlen) < 0) {
        *_errno = errno;
        debug_printf("socket_getsockopt() -> errno %d\n", errno);
        return -1;
    }

    return optval;
}

STATIC mp_uint_t network_rt_wlan_socket_auto_bind(mod_network_socket_obj_t *_socket, int *_errno) {

    return 0;
}

// API for non-socket operations
STATIC int network_rt_wlan_socket_gethostbyname(mp_obj_t nic, const char *name, mp_uint_t len, uint8_t *ip_out)
{
    struct result {
        uint8_t ip[4];
        int name_len; // max 256
        char name[0];
    };

    CHECK_FD_VALID();

    uint64_t buffer[(sizeof(struct result) + 256) / sizeof(uint64_t) + 1];
    struct result *result = (struct result *)buffer;
    int err = 0;

    if(len > 256) {
        mp_printf(&mp_plat_print, "Host name length max 256 bytes.\n");
        return -3;
    }

    result->name_len = len;
    memcpy(result->name, name, len);

    if(0x00 != (err = ioctl(s_net_mgmt_dev_fd, IOCTRL_NET_GETHOSTBYNAME, result))) {
        debug_printf("socket_getsockopt() -> err %d\n", err);
        return err;
    }

    memcpy(ip_out, result->ip, sizeof(result->ip));

    return 0;
}

// API for socket operations; return -1 on error
STATIC int network_rt_wlan_socket_socket(struct _mod_network_socket_obj_t *_socket, int *_errno)
{
    debug_printf("socket_socket(%d %d %d)\n", _socket->domain, _socket->type, _socket->proto);

    int fd, domain, type;

    switch (_socket->type) {
        case MOD_NETWORK_SOCK_STREAM:
            type = SOCK_STREAM;
            break;

        case MOD_NETWORK_SOCK_DGRAM:
            type = SOCK_DGRAM;
            break;

        case MOD_NETWORK_SOCK_RAW:
            type = SOCK_RAW;
            break;

        default:
            *_errno = MP_EINVAL;
            return -1;
    }

    if (MOD_NETWORK_AF_INET == _socket->domain) {
        domain = AF_INET;
    } else if(MOD_NETWORK_AF_INET6 == _socket->domain) {
        domain = AF_INET6;
    } else {
        *_errno = MP_EAFNOSUPPORT;
        return -1;
    }

    fd = socket(domain, type, _socket->proto);
    if (fd < 0) {
        *_errno = errno;
        mp_printf(&mp_plat_print,"socket_socket() -> errno %d\n", errno);
        return -1;
    }

    // set socket state
    _socket->fileno = fd;
    _socket->bound = false;
    _socket->callback = MP_OBJ_NULL;

    // default set recv timeout
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50 * 1000;

    return setsockopt(_socket->fileno, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // return 0; // network_rt_wlan_socke_setblocking(_socket, false, _errno);
}

STATIC void network_rt_wlan_socket_close(struct _mod_network_socket_obj_t *socket)
{
    if (socket->callback != MP_OBJ_NULL) {
        // mp_sched_lock();
        // socket->callback = MP_OBJ_NULL;
        // mp_obj_list_remove(MP_STATE_PORT(mp_wifi_sockpoll_list), socket);
        // mp_sched_unlock();
    }

    if (socket->fileno >= 0) {
        close(socket->fileno);
        socket->fileno = -1; // Mark socket FD as invalid
    }
}

STATIC int network_rt_wlan_socket_bind(struct _mod_network_socket_obj_t *_socket, byte *ip, mp_uint_t port, int *_errno)
{
    debug_printf("socket_bind(%d, %d)\n", _socket->fileno, port);

    struct sockaddr_in addr;
    addr.sin_family = _socket->domain;
    addr.sin_port = htons(port);
    memcpy(&addr.sin_addr, ip, MOD_NETWORK_IPADDR_BUF_SIZE);

    int ret = bind(_socket->fileno, (struct sockaddr*)&addr, sizeof(addr));
    if (ret < 0) {
        *_errno = errno;
        // network_rt_wlan_socket_close(_socket);
        debug_printf("socket_bind(%d, %d) -> errno: %d\n", _socket->fileno, port, *_errno);
        return -1;
    }

    // Mark socket as bound to avoid auto-binding.
    _socket->bound = true;

    return 0;
}

STATIC int network_rt_wlan_socket_listen(struct _mod_network_socket_obj_t *_socket, mp_int_t backlog, int *_errno)
{
    debug_printf("socket_listen(%d, %d)\n", _socket->fileno, backlog);

    int ret = listen(_socket->fileno, backlog);
    if (ret < 0) {
        *_errno = errno;
        // network_rt_wlan_socket_close(_socket);
        debug_printf("socket_listen() -> errno %d\n", *_errno);
        return -1;
    }
    return 0;
}

STATIC int network_rt_wlan_socket_accept(struct _mod_network_socket_obj_t *_socket,
    struct _mod_network_socket_obj_t *socket2, byte *ip, mp_uint_t *port, int *_errno)
{
    debug_printf("socket_accept(%d)\n", _socket->fileno);

    if (network_rt_wlan_socket_poll(_socket, POLLIN, _errno) != 0) {
        return -1;
    }

    struct sockaddr_in addr;
    int addrlen = sizeof(addr);
    addr.sin_family = _socket->domain;

    *port = 0;
    int fd = -1;

    do {
        fd = accept(_socket->fileno, (struct sockaddr*)&addr, (socklen_t*)&addrlen);
        if((0 <= fd) || (EAGAIN != errno)) {
            break;
        }
        mp_hal_delay_ms(100);
    } while(0 > fd);

    if (fd < 0) {
        *_errno = errno;
        // network_rt_wlan_socket_close(_socket);
        debug_printf("socket_accept() -> errno %d\n", *_errno);
        return -1;
    }

    *port = ntohs(addr.sin_port);
    memcpy(ip, &addr.sin_addr.s_addr, sizeof(addr.sin_addr));

    // set socket state
    socket2->fileno = fd;
    socket2->bound = false;
    socket2->timeout = -1;
    socket2->callback = MP_OBJ_NULL;

    // default set recv timeout
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50 * 1000;

    return setsockopt(socket2->fileno, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // return 0; // network_rt_wlan_socke_setblocking(socket2, false, _errno);
}

STATIC int network_rt_wlan_socket_connect(struct _mod_network_socket_obj_t *_socket, byte *ip, mp_uint_t port, int *_errno)
{
    debug_printf("socket_connect(%d)\n", _socket->fileno);

    struct sockaddr_in addr;
    addr.sin_family = _socket->domain;
    addr.sin_port = htons(port);
    memcpy(&addr.sin_addr, ip, MOD_NETWORK_IPADDR_BUF_SIZE);

    int ret = connect(_socket->fileno, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        *_errno = errno;
        debug_printf("socket_connect() -> errno %d\n", *_errno);

        // network_rt_wlan_socket_close(_socket);

        // Poll for write.
        // if (_socket->timeout == 0 ||
        //     network_rt_wlan_socket_poll(_socket, POLLOUT, _errno) != 0) {
        //     return -1;
        // }

        return -1;
    }

    return 0;
}

STATIC mp_uint_t network_rt_wlan_socket_send(struct _mod_network_socket_obj_t *_socket, const byte *buf, mp_uint_t len, int *_errno)
{
    debug_printf("socket_send(%d, %d)\n", _socket->fileno, len);

    if (network_rt_wlan_socket_poll(_socket, POLLOUT, _errno) != 0) {
        return -1;
    }

    int ret = send(_socket->fileno, buf, len, 0);
    if (ret < 0) {
        *_errno = errno;
        // network_rt_wlan_socket_close(_socket);
        debug_printf("socket_send() -> errno %d\n", *_errno);
        return -1;
    }
    return ret;
}

STATIC mp_uint_t network_rt_wlan_socket_recv(struct _mod_network_socket_obj_t *_socket, byte *buf, mp_uint_t len, int *_errno)
{
    debug_printf("socket_recv(%d), len %d\n", _socket->fileno, len);

    // check if socket in listening state.
    if (network_rt_wlan_socket_listening(_socket, _errno) == 1) {
        *_errno = MP_ENOTCONN;
        return -1;
    }

    if (network_rt_wlan_socket_poll(_socket, POLLIN, _errno) != 0) {
        return -1;
    }

#if 0
    int ret = recv(_socket->fileno, buf, len, 0);
    if (ret < 0) {
        *_errno = errno;
        debug_printf("socket_recv() -> errno %d %d\n", *_errno, ret);
        // network_rt_wlan_socket_close(_socket);

        if(EAGAIN == *_errno) {
            return 0;
        }

        return -1;
    }

    return ret;
#else
    int ret;
    mp_uint_t received = 0;
    mp_uint_t curr_tick_ms = mp_hal_ticks_ms();
    mp_uint_t stop_ms = curr_tick_ms + 3000; // timeout 3s

    do {
        ret = recv(_socket->fileno, buf + received, len - received, 0);

        if(0 > ret) {
            *_errno = errno;

            debug_printf("socket_recv() -> errno %d %d\n", *_errno, ret);

            if(EAGAIN == *_errno) {
                curr_tick_ms = mp_hal_ticks_ms();
                if(curr_tick_ms > stop_ms) {
                    return received;
                }
                continue;
            }
            break;
        } else {
            received += ret;
        }
    } while(received < len);

    return received;
#endif
}

STATIC mp_uint_t network_rt_wlan_socket_sendto(struct _mod_network_socket_obj_t *_socket, const byte *buf, mp_uint_t len, byte *ip, mp_uint_t port, int *_errno)
{
    debug_printf("socket_sendto(%d)\n", _socket->fileno);
    // Auto-bind the socket first if the socket is unbound.
    if (network_rt_wlan_socket_auto_bind(_socket, _errno) != 0) {
        return -1;
    }

    if (network_rt_wlan_socket_poll(_socket, POLLOUT, _errno) != 0) {
        return -1;
    }

    struct sockaddr_in addr;
    addr.sin_family = _socket->domain;
    addr.sin_port = htons(port);
    memcpy(&addr.sin_addr, ip, MOD_NETWORK_IPADDR_BUF_SIZE);

    int ret = sendto(_socket->fileno, buf, len, 0, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        *_errno = errno;
        // network_rt_wlan_socket_close(_socket);
        return -1;
    }
    return ret;
}

STATIC mp_uint_t network_rt_wlan_socket_recvfrom(struct _mod_network_socket_obj_t *_socket, byte *buf, mp_uint_t len, byte *ip, mp_uint_t *port, int *_errno)
{
    debug_printf("socket_recvfrom(%d), len %d\n", _socket->fileno, len);
    // Auto-bind the socket first if the socket is unbound.
    if (network_rt_wlan_socket_auto_bind(_socket, _errno) != 0) {
        return -1;
    }

    if (network_rt_wlan_socket_poll(_socket, POLLIN, _errno) != 0) {
        return -1;
    }

    struct sockaddr_in addr;
    socklen_t server_addr_len = sizeof(addr);
    addr.sin_family = _socket->domain;

    *port = 0;

#if 0
    int ret = recvfrom(_socket->fileno, buf, len, 0, (struct sockaddr *)&addr, &server_addr_len);
    if (ret < 0) {
        *_errno = errno;
        debug_printf("socket_recvfrom() -> errno %d\n", *_errno);
        // network_rt_wlan_socket_close(_socket);

        if(EAGAIN == *_errno) {
            return 0;
        }

        return -1;
    }

    *port = ntohs(addr.sin_port);
    memcpy(ip, &addr.sin_addr.s_addr, sizeof(addr.sin_addr));

    return ret;
#else
    int ret = -1;
    mp_uint_t received = 0;
    mp_uint_t curr_tick_ms = mp_hal_ticks_ms();
    mp_uint_t stop_ms = curr_tick_ms + 3000; // timeout 3s

    do {
        ret = recvfrom(_socket->fileno, buf + received, len - received, 0, (struct sockaddr *)&addr, &server_addr_len);

        if(0 > ret) {
            *_errno = errno;
            debug_printf("socket_recvfrom() -> errno %d\n", *_errno);

            if(EAGAIN == *_errno) {
                curr_tick_ms = mp_hal_ticks_ms();
                if(curr_tick_ms > stop_ms) {
                    return received;
                }
                continue;
            }
        } else {
            received += ret;
        }
    } while(received < len);

    return received;
#endif
}

STATIC int network_rt_wlan_socket_setsockopt(struct _mod_network_socket_obj_t *_socket, mp_uint_t level, mp_uint_t opt, const void *optval, mp_uint_t optlen, int *_errno)
{
    debug_printf("socket_setsockopt(%d, %d)\n", _socket->fileno, opt);
    // if (opt == 20) {
    //     mp_sched_lock();
    //     socket->callback = (void *)optval;
    //     if (socket->callback != MP_OBJ_NULL) {
    //         mp_obj_list_append(MP_STATE_PORT(mp_wifi_sockpoll_list), socket);
    //     }
    //     mp_sched_unlock();
    //     return 0;
    // }
    int ret = setsockopt(_socket->fileno, level, opt, optval, optlen);
    if (ret < 0) {
        *_errno = errno;
        // network_rt_wlan_socket_close(_socket);
        debug_printf("socket_setsockopt() -> errno %d\n", *_errno);
        return -1;
    }
    return 0;
}

STATIC int network_rt_wlan_socket_settimeout(struct _mod_network_socket_obj_t *_socket, mp_uint_t timeout_ms, int *_errno)
{
    int ret = 0;
    debug_printf("socket_settimeout(%d, %d)\n", _socket->fileno, timeout_ms);
    if (timeout_ms == 0 || timeout_ms == UINT32_MAX) {
        ret |= network_rt_wlan_socke_setblocking(_socket, false, _errno);
    } else {
        struct timeval timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;
        ret |= setsockopt(_socket->fileno, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        ret |= setsockopt(_socket->fileno, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    }
    if (ret < 0) {
        *_errno = errno;
        debug_printf("socket_settimeout() -> errno %d\n", *_errno);
    }
    _socket->timeout = timeout_ms;
    return 0;
}

STATIC int network_rt_wlan_socket_ioctl(struct _mod_network_socket_obj_t *_socket, mp_uint_t request, mp_uint_t arg, int *_errno)
{
    mp_uint_t ret = 0;
    debug_printf("socket_ioctl(%d, %d)\n", _socket->fileno, request);
    if (request == MP_STREAM_POLL) {
        uint8_t flags = 0;

        struct pollfd fd[1];
        fd[0].fd = _socket->fileno;
        fd[0].events = POLLIN;
        fd[0].revents = 0;

        if (poll(_socket->fileno, 1, 1) < 0) {
            *_errno = errno;
            ret = MP_STREAM_ERROR;
            debug_printf("socket_ioctl() -> errno %d\n", *_errno);
        }

        flags = fd[0].revents;
        if ((arg & MP_STREAM_POLL_RD) && (flags & POLLIN)) {
            ret |= MP_STREAM_POLL_RD;
        }
        if ((arg & MP_STREAM_POLL_WR) && (flags & POLLOUT)) {
            ret |= MP_STREAM_POLL_WR;
        }
    } else {
        // NOTE: FIONREAD and FIONBIO are supported as well.
        *_errno = MP_EINVAL;
        ret = MP_STREAM_ERROR;
    }
    return ret;
}

STATIC const mod_network_nic_protocol_t mod_network_nic_protocol_rtt_posix = {
    // API for non-socket operations
    .gethostbyname = network_rt_wlan_socket_gethostbyname,

    // API for socket operations; return -1 on error
    .socket = network_rt_wlan_socket_socket,
    .close = network_rt_wlan_socket_close,
    .bind = network_rt_wlan_socket_bind,
    .listen = network_rt_wlan_socket_listen,
    .accept = network_rt_wlan_socket_accept,
    .connect = network_rt_wlan_socket_connect,
    .send = network_rt_wlan_socket_send,
    .recv = network_rt_wlan_socket_recv,
    .sendto = network_rt_wlan_socket_sendto,
    .recvfrom = network_rt_wlan_socket_recvfrom,
    .setsockopt = network_rt_wlan_socket_setsockopt,
    .settimeout = network_rt_wlan_socket_settimeout,
    .ioctl = network_rt_wlan_socket_ioctl,
};

#ifdef CONFIG_ENABLE_NETWORK_RT_LAN
/* network_rt_lan ************************************************************/
STATIC mp_obj_t network_rt_lan_isconnected(mp_obj_t self_in) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(self_in);
    (void)self;

    int isconnected = 0;
    if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_LAN_GET_ISCONNECTED, &isconnected)) {
        mp_printf(&mp_plat_print, "run get isconnected failed.\n");
        return mp_const_false;
    }
    return mp_obj_new_bool(0x00 != isconnected);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_rt_lan_isconnected_obj, network_rt_lan_isconnected);


STATIC mp_obj_t network_rt_lan_status(size_t n_args, const mp_obj_t *args) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    (void)self;

    if (n_args == 1) {
        int status = 0;

        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_LAN_GET_STATUS, &status)) {
            status = 0; // failed

            mp_printf(&mp_plat_print, "run get status failed.\n");
        }
        return MP_OBJ_NEW_SMALL_INT(status);
    }
    mp_raise_ValueError(MP_ERROR_TEXT("unknown status param"));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_rt_lan_status_obj, 1, 2, network_rt_lan_status);

STATIC mp_obj_t network_rt_lan_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    (void)self;

    if (kwargs->used == 0) {
        // Get config value
        if (n_args != 2) {
            mp_raise_TypeError(MP_ERROR_TEXT("must query one param"));
        }

        switch (mp_obj_str_get_qstr(args[1])) {
            case MP_QSTR_mac: {
                uint8_t buf[6];

                if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_LAN_GET_MAC, &buf[0])) {
                    memset(&buf[0], 0, sizeof(buf)); // failed

                    mp_printf(&mp_plat_print, "run get mac failed.\n");
                }

                return mp_obj_new_bytes(buf, 6);
            }
            default:
                mp_raise_ValueError(MP_ERROR_TEXT("unknown config param"));
        }
    } else {
        // Set config value(s)
        if (n_args != 1) {
            mp_raise_TypeError(MP_ERROR_TEXT("can't specify pos and kw args"));
        }

        for (size_t i = 0; i < kwargs->alloc; ++i) {
            if (MP_MAP_SLOT_IS_FILLED(kwargs, i)) {
                mp_map_elem_t *e = &kwargs->table[i];
                switch (mp_obj_str_get_qstr(e->key)) {
                    case MP_QSTR_mac: {
                        mp_buffer_info_t buf;
                        mp_get_buffer_raise(e->value, &buf, MP_BUFFER_READ);
                        if (buf.len != 6) {
                            mp_raise_ValueError(NULL);
                        }

                        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_LAN_SET_MAC, buf.buf)) {
                            mp_printf(&mp_plat_print, "run set mac failed.\n");
                        }

                        break;
                    }
                    default:
                        mp_raise_ValueError(MP_ERROR_TEXT("unknown config param"));
                }
            }
        }

        return mp_const_none;
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_rt_lan_config_obj, 1, network_rt_lan_config);

const struct _mp_obj_type_t network_type_eth_lan;

STATIC py_rt_net_obj_t network_rt_eth_lan = {{(mp_obj_type_t *)&network_type_eth_lan}, 2};

STATIC mp_obj_t py_rt_eth_lan_type_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    mp_obj_t rt_net_obj;

    if(0 > s_net_mgmt_dev_fd) {
        if(0 > (s_net_mgmt_dev_fd = open("/dev/canmv_net_mgmt", O_RDWR))) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("could not open /dev/canmv_net_mgmt"));
        }
    }

    rt_net_obj = MP_OBJ_FROM_PTR(&network_rt_eth_lan);

    // Register with network module
    mod_network_register_nic(rt_net_obj);

    return rt_net_obj;
}

STATIC const mp_rom_map_elem_t network_type_lan_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&network_rt_net_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_isconnected), MP_ROM_PTR(&network_rt_lan_isconnected_obj) },
    { MP_ROM_QSTR(MP_QSTR_status), MP_ROM_PTR(&network_rt_lan_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_ifconfig), MP_ROM_PTR(&network_rt_net_ifconfig_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&network_rt_lan_config_obj) },
};
STATIC MP_DEFINE_CONST_DICT(network_type_lan_locals_dict, network_type_lan_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    network_type_eth_lan,
    MP_QSTR_eth_lan,
    MP_TYPE_FLAG_NONE,
    make_new, py_rt_eth_lan_type_make_new,
    locals_dict, &network_type_lan_locals_dict,
    protocol, &mod_network_nic_protocol_rtt_posix
    );

#endif // CONFIG_ENABLE_NETWORK_RT_LAN

#ifdef CONFIG_ENABLE_NETWORK_RT_WLAN
/* rt_wlan_info **************************************************************/
STATIC const mp_obj_type_t py_rt_wlan_info_type;

typedef struct _py_rt_wlan_info_obj_t {
    mp_obj_base_t base;
    struct rt_wlan_info _cobj;
} py_rt_wlan_info_obj_t;

STATIC const char *rt_wlan_security_string(rt_wlan_security_t security)
{
    switch (security) {
        case SECURITY_OPEN: return "SECURITY_OPEN";
        case SECURITY_WEP_PSK: return "SECURITY_WEP_PSK";
        case SECURITY_WEP_SHARED: return "SECURITY_WEP_SHARED";
        case SECURITY_WPA_TKIP_PSK: return "SECURITY_WPA_TKIP_PSK";
        case SECURITY_WPA_TKIP_8021X: return "SECURITY_WPA_TKIP_8021X";
        case SECURITY_WPA_AES_PSK: return "SECURITY_WPA_AES_PSK";
        case SECURITY_WPA_AES_8021X: return "SECURITY_WPA_AES_8021X";
        case SECURITY_WPA2_AES_PSK: return "SECURITY_WPA2_AES_PSK";
        case SECURITY_WPA2_AES_8021X: return "SECURITY_WPA2_AES_8021X";
        case SECURITY_WPA2_TKIP_PSK: return "SECURITY_WPA2_TKIP_PSK";
        case SECURITY_WPA2_TKIP_8021X: return "SECURITY_WPA2_TKIP_8021X";
        case SECURITY_WPA2_MIXED_PSK: return "SECURITY_WPA2_MIXED_PSK";
        case SECURITY_WPA_WPA2_MIXED_PSK: return "SECURITY_WPA_WPA2_MIXED_PSK";
        case SECURITY_WPA_WPA2_MIXED_8021X: return "SECURITY_WPA_WPA2_MIXED_8021X";
        case SECURITY_WPA2_AES_CMAC: return "SECURITY_WPA2_AES_CMAC";
        case SECURITY_WPS_OPEN: return "SECURITY_WPS_OPEN";
        case SECURITY_WPS_SECURE: return "SECURITY_WPS_SECURE";
        case SECURITY_WPA3_AES_PSK: return "SECURITY_WPA3_AES_PSK";
        default: return "UNKNOWN";
    }

    return NULL;
}

STATIC const char *rt_wlan_band_string(rt_802_11_band_t band)
{
    switch(band) {
        case RT_802_11_BAND_5GHZ: return "5G";
        case RT_802_11_BAND_2_4GHZ: return "2.4G";
        default: return "UNKNOWN";
    }

    return NULL;
}

STATIC mp_obj_t py_rt_wlan_info_from_struct(struct rt_wlan_info *info)
{
    py_rt_wlan_info_obj_t *o = m_new_obj_with_finaliser(py_rt_wlan_info_obj_t);
    o->base.type = &py_rt_wlan_info_type;
    o->_cobj = *info;
    return o;
}

STATIC mp_obj_t py_rt_wlan_info_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_ssid, ARG_bssid, ARG_channel, ARG_security, ARG_band, ARG_hidden };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid,         MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_bssid,        MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_channel,      MP_ARG_INT, {.u_int = 1} },
        { MP_QSTR_rssi,         MP_ARG_INT, {.u_int = -99} },
        { MP_QSTR_security,     MP_ARG_INT, {.u_int = SECURITY_OPEN} },
        { MP_QSTR_band,         MP_ARG_INT, {.u_int = RT_802_11_BAND_2_4GHZ} },
        { MP_QSTR_hidden,       MP_ARG_INT, {.u_int = 0} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    struct rt_wlan_info info;
    INVALID_INFO(&info);

    if(mp_const_none != args[ARG_ssid].u_obj) {
        const char *ssid = mp_obj_str_get_str(args[ARG_ssid].u_obj);
        int ssid_len = strlen(ssid);
        if ((0x00 == ssid_len) || (RT_WLAN_SSID_MAX_LENGTH < ssid_len)) {
            mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("SSID can't be empty, and can't longer than %d"), RT_WLAN_SSID_MAX_LENGTH);
        }
        SSID_SET(&info, ssid);
    }

    if(mp_const_none != args[ARG_bssid].u_obj) {
        uint8_t bssid[RT_WLAN_BSSID_MAX_LENGTH];
        mp_obj_t *items;
        mp_obj_get_array_fixed_n(args[ARG_bssid].u_obj, RT_WLAN_BSSID_MAX_LENGTH, &items);
        for(int i = 0; i < RT_WLAN_BSSID_MAX_LENGTH; i++) {
            bssid[i] = (uint8_t)mp_obj_get_int(items[i]);
        }
        memcpy(&info.bssid[0], bssid, RT_WLAN_BSSID_MAX_LENGTH);
    }

    info.channel = args[ARG_channel].u_int;
    info.security = args[ARG_security].u_int;
    info.band = args[ARG_band].u_int;
    info.hidden = args[ARG_hidden].u_int;

    return py_rt_wlan_info_from_struct(&info);
}

STATIC void *py_rt_wlan_info_cobj(mp_obj_t rt_wlan_info) {
    PY_ASSERT_TYPE(rt_wlan_info, &py_rt_wlan_info_type);
    return &((py_rt_wlan_info_obj_t *) rt_wlan_info)->_cobj;
}

STATIC void py_rt_wlan_info_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_rt_wlan_info_obj_t *self = self_in;
    struct rt_wlan_info *info = py_rt_wlan_info_cobj(self);

    mp_printf(print,
              "{\"ssid\":\"%s\", \"bssid\":%02X:%02X:%02X:%02X:%02X:%02X, \"channel\":%d, \"rssi\":%d, \"security\":\"%s\", \"band\":\"%s\", \"hidden\":%d}",
              info->ssid.val, \
              info->bssid[0], info->bssid[1], info->bssid[2], \
              info->bssid[3], info->bssid[4], info->bssid[5], \
              info->channel, info->rssi, rt_wlan_security_string(info->security), rt_wlan_band_string(info->band), info->hidden);
}

STATIC void py_rt_wlan_info_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    py_rt_wlan_info_obj_t *self = self_in;
    struct rt_wlan_info *info = py_rt_wlan_info_cobj(self);

    if(MP_OBJ_NULL == dest[0]) {
        // load attribute
        switch(attr) {
            case MP_QSTR_ssid:
                dest[0] = mp_obj_new_bytes(info->ssid.val, info->ssid.len);
                break;
            case MP_QSTR_bssid:
                dest[0] = mp_obj_new_bytes(info->bssid, RT_WLAN_BSSID_MAX_LENGTH);
                break;
            case MP_QSTR_channel:
                dest[0] = MP_OBJ_NEW_SMALL_INT(info->channel);
                break;
            case MP_QSTR_rssi:
                dest[0] = MP_OBJ_NEW_SMALL_INT(info->rssi);
                break;
            case MP_QSTR_security:
                dest[0] = MP_OBJ_NEW_SMALL_INT(info->security);
                break;
            case MP_QSTR_band:
                dest[0] = MP_OBJ_NEW_SMALL_INT(info->band);
                break;
            case MP_QSTR_hidden:
                dest[0] = MP_OBJ_NEW_SMALL_INT(info->hidden);
                break;
        }
    } else if(MP_OBJ_SENTINEL == dest[0]) {
        // store attribute
        switch(attr) {
            case MP_QSTR_ssid:
                const char *ssid = mp_obj_str_get_str(dest[1]);
                SSID_SET(info, ssid);
                break;
            case MP_QSTR_bssid:
                uint8_t bssid[RT_WLAN_BSSID_MAX_LENGTH];
                mp_obj_t *items;
                mp_obj_get_array_fixed_n(dest[1], RT_WLAN_BSSID_MAX_LENGTH, &items);
                for(int i = 0; i < RT_WLAN_BSSID_MAX_LENGTH; i++) {
                    bssid[i] = (uint8_t)mp_obj_get_int(items[i]);
                }
                memcpy(info->bssid, bssid, RT_WLAN_BSSID_MAX_LENGTH);
                break;
            case MP_QSTR_channel:
                info->channel = MP_OBJ_SMALL_INT_VALUE(dest[1]);
                break;
            case MP_QSTR_rssi:
                info->rssi = MP_OBJ_SMALL_INT_VALUE(dest[1]);
                break;
            case MP_QSTR_security:
                info->security = MP_OBJ_SMALL_INT_VALUE(dest[1]);
                break;
            case MP_QSTR_band:
                info->band = MP_OBJ_SMALL_INT_VALUE(dest[1]);
                break;
            case MP_QSTR_hidden:
                info->hidden = MP_OBJ_SMALL_INT_VALUE(dest[1]);
                break;
            default:
                // not support set, directly return
                return;
        }
        dest[0] = MP_OBJ_NULL;
    }
}

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    py_rt_wlan_info_type,
    MP_QSTR_rt_wlan_info,
    MP_TYPE_FLAG_NONE,
    make_new, py_rt_wlan_info_make_new,
    print, py_rt_wlan_info_print,
    attr, py_rt_wlan_info_attr
);

/* network_rt_wlan ***********************************************************/
STATIC mp_obj_t network_rt_wlan_scan(size_t n_args, const mp_obj_t *args) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_t scan_list;

    struct rt_wlan_scan_result *scan_result = NULL;
    size_t scan_result_info_size = 0;
    const char *ssid = NULL;

    if(MOD_NETWORK_STA_IF != self->itf) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ap mode not support scan"));
    }

    scan_result_info_size = sizeof(struct rt_wlan_info) * RT_WLAN_STA_SCAN_MAX_AP;
    scan_result = (struct rt_wlan_scan_result *)malloc(sizeof(struct rt_wlan_scan_result) + scan_result_info_size);
    if(NULL == scan_result) {
        mp_printf(&mp_plat_print, "no memory for scan result.\n");
        return mp_const_none;
    }
    memset(scan_result, 0, sizeof(struct rt_wlan_scan_result) + scan_result_info_size);

    scan_result->num = RT_WLAN_STA_SCAN_MAX_AP;
    scan_result->info = (struct rt_wlan_info *)(((uint8_t *)scan_result) + sizeof(struct rt_wlan_scan_result));
    INVALID_INFO(&scan_result->info[0]);

    if(0x02 == n_args) {
        ssid = mp_obj_str_get_str(args[1]);

        if(0x00 != strlen(ssid)) {
            SSID_SET(&scan_result->info[0], ssid);
        }
    }

    if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_SCAN, scan_result)) {
        scan_result->num = 0;
        mp_printf(&mp_plat_print, "run scan failed.\n");
    }
    scan_list = mp_obj_new_list(0, NULL);

    for(int32_t i = 0; i < scan_result->num; i++) {
        mp_obj_t info = py_rt_wlan_info_from_struct(&scan_result->info[i]);
        mp_obj_list_append(scan_list, info);
    }

    free(scan_result);

    return scan_list;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_rt_wlan_scan_obj, 1, 2, network_rt_wlan_scan);

STATIC mp_obj_t network_rt_wlan_connect(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    struct rt_wlan_connect_config {
        int use_info;
        union {
            rt_wlan_ssid_t ssid;
            struct rt_wlan_info info;
        };
        rt_wlan_key_t key;
    };

    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    enum { ARG_ssid, ARG_key, ARG_info };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid,     MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_key,      MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_info,     MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    struct rt_wlan_connect_config config;
    memset(&config, 0, sizeof(config));

    if (mp_const_none != args[ARG_ssid].u_obj) {
        const char *ssid = mp_obj_str_get_str(args[ARG_ssid].u_obj);
        int ssid_len = strlen(ssid);
        if ((0x00 == ssid_len) || (RT_WLAN_SSID_MAX_LENGTH < ssid_len)) {
            mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("SSID can't be empty, and can't longer than %d"), RT_WLAN_SSID_MAX_LENGTH);
        }
        config.use_info = 0;

        config.ssid.len = ssid_len;
        strncpy((char *)&config.ssid.val[0], ssid, RT_WLAN_SSID_MAX_LENGTH);
        config.ssid.val[ssid_len] = '\0';
    } else if (mp_const_none != args[ARG_info].u_obj) {
        struct rt_wlan_info *info = py_rt_wlan_info_cobj(args[ARG_info].u_obj);
        config.use_info = 1;
        memcpy(&config.info, info, sizeof(struct rt_wlan_info));
    } else {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("should set ssid or ap"));
    }

    int key_len = 0;
    const char *key = NULL;
    if (mp_const_none != args[ARG_key].u_obj) {
        key = mp_obj_str_get_str(args[ARG_key].u_obj);
        key_len = strlen(key);
    }

    if((0x08 > key_len) || (RT_WLAN_PASSWORD_MAX_LENGTH < key_len)) {
        mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("Key length(%d) should be 8 - %d"), key_len, RT_WLAN_PASSWORD_MAX_LENGTH);
    }

    config.key.len = key_len;
    strncpy((char *)&config.key.val[0], key, RT_WLAN_PASSWORD_MAX_LENGTH);
    config.key.val[config.key.len] = '\0';

    if(MOD_NETWORK_STA_IF == self->itf) {
        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_CONNECT, &config)) {
            mp_printf(&mp_plat_print, "run connect failed.\n");
            return mp_const_false;
        }
    } else {
        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_START, &config)) {
            mp_printf(&mp_plat_print, "start ap failed.\n");
            return mp_const_false;
        }
    }

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_rt_wlan_connect_obj, 1, network_rt_wlan_connect);

STATIC mp_obj_t network_rt_wlan_disconnect(size_t n_args, const mp_obj_t *args) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if(MOD_NETWORK_STA_IF == self->itf) {
        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_DISCONNECT, NULL)) {
            mp_printf(&mp_plat_print, "run disconnect failed.\n");
            return mp_const_false;
        }
    } else {
        uint8_t mac[6];
        mp_obj_t *items;
        mp_obj_get_array_fixed_n(args[1], 6, &items);
        for(int i = 0; i < 6; i++) {
            mac[i] = (uint8_t)mp_obj_get_int(items[i]);
        }
        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_DEAUTH_STA, mac)) {
            mp_printf(&mp_plat_print, "run ap deauth failed.\n");
            return mp_const_false;
        }
    }

    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_rt_wlan_disconnect_obj, 1, 2, network_rt_wlan_disconnect);

STATIC mp_obj_t network_rt_wlan_isconnected(mp_obj_t self_in) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(self_in);
    int status = -1;

    if(MOD_NETWORK_STA_IF != self->itf) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("ap mode not support connect"));
    }

    if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_IS_CONNECTED, &status)) {
        status = false;
        mp_printf(&mp_plat_print, "run isconnected failed.\n");
    }

    return mp_obj_new_bool(status);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_rt_wlan_isconnected_obj, network_rt_wlan_isconnected);

STATIC mp_obj_t _network_rt_wlan_sta_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    CHECK_FD_VALID();

    // py_rt_net_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);

    if (kw_args->used == 0) {
        // Get config value
        if (n_args != 2) {
            mp_raise_TypeError(MP_ERROR_TEXT("must query one param.\n"));
        }
        qstr attr = mp_obj_str_get_qstr(pos_args[1]);

        switch(attr) {
            case MP_QSTR_mac: {
                uint8_t mac[6];

                if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_GET_MAC, mac)) {
                    mp_printf(&mp_plat_print, "run get mac failed.\n");
                    return mp_const_none;
                }

                return mp_obj_new_bytes(mac, sizeof(mac));
            } break;
            case MP_QSTR_auto_reconnect: {
                int auto_reconnect = 0;

                if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_GET_AUTO_RECONNECT, &auto_reconnect)) {
                    auto_reconnect = 0;
                    mp_printf(&mp_plat_print, "run get auto reconnect failed.\n");
                }
                return mp_obj_new_bool(auto_reconnect);
            } break;
        }
    } else {
        enum { ARG_mac, ARG_auto_reconnect };
        static const mp_arg_t allowed_args[] = {
            { MP_QSTR_mac,                  MP_ARG_OBJ, {.u_obj = mp_const_none} },
            { MP_QSTR_auto_reconnect,       MP_ARG_OBJ, {.u_obj = mp_const_none} },
        };
        mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
        mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

        if(mp_const_none != args[ARG_mac].u_obj) {
            uint8_t mac[6];
            mp_obj_t *items;
            mp_obj_get_array_fixed_n(args[ARG_mac].u_obj, 6, &items);
            for(int i = 0; i < 6; i++) {
                mac[i] = (uint8_t)mp_obj_get_int(items[i]);
            }

            if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_SET_MAC, mac)) {
                mp_printf(&mp_plat_print, "run set mac failed.\n");
                return mp_const_false;
            }
        }

        if(mp_const_none != args[ARG_auto_reconnect].u_obj) {
            int auto_reconnect = 0;

            if(mp_obj_is_true(args[ARG_auto_reconnect].u_obj)) {
                auto_reconnect = 1;
            }

            if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_SET_AUTO_RECONNECT, &auto_reconnect)) {
                mp_printf(&mp_plat_print, "run set auto_reconnect failed.\n");
                return mp_const_false;
            }
        }
    }

    return mp_const_true;
}

STATIC mp_obj_t _network_rt_wlan_ap_config(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    CHECK_FD_VALID();

    // py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if (kw_args->used == 0) {
        // Get config value
        if (n_args != 2) {
            mp_raise_TypeError(MP_ERROR_TEXT("must query one param"));
        }
        qstr attr = mp_obj_str_get_qstr(pos_args[1]);

        switch(attr) {
            case MP_QSTR_info: {
                struct rt_wlan_info info;

                if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_GET_INFO, &info)) {
                    mp_printf(&mp_plat_print, "run get ap info failed.\n");
                    return mp_const_none;
                }

                return py_rt_wlan_info_from_struct(&info);
            } break;
            case MP_QSTR_country: {
                int country = RT_COUNTRY_CHINA;
                if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_GET_COUNTRY, &country)) {
                    country = -1;
                    mp_printf(&mp_plat_print, "run get ap country failed.\n");
                }
                return MP_OBJ_NEW_SMALL_INT(country);
            } break;
        }
    } else {
        mp_map_elem_t *ssid = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_ssid), MP_MAP_LOOKUP); // same in network_rt_wlan_connect
        mp_map_elem_t *key = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_key), MP_MAP_LOOKUP);   // same in network_rt_wlan_connect
        mp_map_elem_t *info = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_info), MP_MAP_LOOKUP); // same in network_rt_wlan_connect

        if((NULL != key) && ((NULL != ssid) || (NULL != info))) {
            // Call connect to set WiFi access point.
            return network_rt_wlan_connect(n_args, pos_args, kw_args);
        }

        enum { ARG_country };
        static const mp_arg_t allowed_args[] = {
            { MP_QSTR_country,                  MP_ARG_INT, {.u_int = -1} },
        };
        mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
        mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

        if((-1) != args[ARG_country].u_int) {
            int country = args[ARG_country].u_int;
            if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_SET_COUNTRY, &country)) {
                mp_printf(&mp_plat_print, "run set ap country failed.\n");
                return mp_const_false;
            }
        }
    }

    return mp_const_true;
}

STATIC mp_obj_t network_rt_wlan_config(size_t n_args, const mp_obj_t *args, mp_map_t *kwargs) {
    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if(MOD_NETWORK_STA_IF == self->itf) {
        return _network_rt_wlan_sta_config(n_args, args, kwargs);
    }
    return _network_rt_wlan_ap_config(n_args, args, kwargs);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(network_rt_wlan_config_obj, 1, network_rt_wlan_config);

STATIC mp_obj_t _network_rt_wlan_get_sta_status(size_t n_args, const mp_obj_t *args) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if(0x01 == n_args) {
        return network_rt_wlan_isconnected(MP_OBJ_FROM_PTR(self));
    }

    switch (mp_obj_str_get_qstr(args[1])) {
        case MP_QSTR_rssi: {
            int rssi = -99;

            if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_GET_RSSI, &rssi)) {
                rssi = -99;
                mp_printf(&mp_plat_print, "run sta get rssi failed.\n");
            }
            return MP_OBJ_NEW_SMALL_INT(rssi);
        } break;
        case MP_QSTR_ap: {
            struct rt_wlan_info info;

            if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_STA_GET_AP_INFO, &info)) {
                mp_printf(&mp_plat_print, "run get ap info failed.\n");
                return mp_const_none;
            }

            return py_rt_wlan_info_from_struct(&info);
        } break;
    }

    mp_raise_ValueError(MP_ERROR_TEXT("unknown status param"));
}

STATIC mp_obj_t _network_rt_wlan_get_ap_status(size_t n_args, const mp_obj_t *args) {
    CHECK_FD_VALID();

    if(0x01 == n_args) {
        // check is actived
        int active = 0;

        if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_IS_ACTIVE, &active)) {
            mp_printf(&mp_plat_print, "run ap isactive failed.\n");
            return mp_const_false;
        }
        return mp_obj_new_bool(active);
    }

    switch (mp_obj_str_get_qstr(args[1])) {
        case MP_QSTR_stations: {
            struct rt_wlan_scan_result *station_result = NULL;
            size_t station_result_info_size = 0;

            const int max_ap_num = 10;

            station_result_info_size = sizeof(struct rt_wlan_info) * max_ap_num;
            station_result = (struct rt_wlan_scan_result *)malloc(sizeof(struct rt_wlan_scan_result) + station_result_info_size);
            if(NULL == station_result) {
                mp_printf(&mp_plat_print, "no memory for connected station info.\n");
                return mp_const_none;
            }
            memset(station_result, 0, sizeof(struct rt_wlan_scan_result) + station_result_info_size);

            station_result->num = max_ap_num;
            station_result->info = (struct rt_wlan_info *)(((uint8_t *)station_result) + sizeof(struct rt_wlan_scan_result));

            if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_GET_STA_INFO, station_result)) {
                free(station_result);
                mp_printf(&mp_plat_print, "run ap get connected station info failed.\n");
                return mp_const_none;
            }

            mp_obj_t station_list = mp_obj_new_list(0, NULL);

            for(int32_t i = 0; i < station_result->num; i++) {
                struct rt_wlan_info *info = &station_result->info[i];
                mp_obj_t bssid = mp_obj_new_bytes(info->bssid, RT_WLAN_BSSID_MAX_LENGTH);
                mp_obj_list_append(station_list, bssid);
            }

            free(station_result);

            return station_list;
        } break;
    }

    mp_raise_ValueError(MP_ERROR_TEXT("unknown status param"));
}

STATIC mp_obj_t network_rt_wlan_status(size_t n_args, const mp_obj_t *args) {
    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(args[0]);

    if(MOD_NETWORK_STA_IF == self->itf) {
        return _network_rt_wlan_get_sta_status(n_args, args);
    }
    return _network_rt_wlan_get_ap_status(n_args, args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_rt_wlan_status_obj, 1, 2, network_rt_wlan_status);

STATIC mp_obj_t network_rt_wlan_stop(mp_obj_t self_in) {
    CHECK_FD_VALID();

    py_rt_net_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if(MOD_NETWORK_AP_IF != self->itf) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("sta mode not support stop"));
    }

    if(0x00 != ioctl(s_net_mgmt_dev_fd, IOCTRL_WM_AP_STOP, NULL)) {
        mp_printf(&mp_plat_print, "run stop failed.\n");
        return mp_const_false;
    }

    return mp_const_true;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(network_rt_wlan_stop_obj, network_rt_wlan_stop);
/* Constants *****************************************************************/
STATIC const mp_rom_map_elem_t rt_wlan_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_active),              MP_ROM_PTR(&network_rt_net_active_obj) },
// { MP_ROM_QSTR(MP_QSTR_scan),                    MP_ROM_PTR(&network_rt_wlan_scan_obj) },             // only for sta
// { MP_ROM_QSTR(MP_QSTR_connect),                 MP_ROM_PTR(&network_rt_wlan_connect_obj) },          // only for sta
    { MP_ROM_QSTR(MP_QSTR_disconnect),              MP_ROM_PTR(&network_rt_wlan_disconnect_obj) },
// { MP_ROM_QSTR(MP_QSTR_isconnected),             MP_ROM_PTR(&network_rt_wlan_isconnected_obj) },      // only for sta
    { MP_ROM_QSTR(MP_QSTR_ifconfig),                MP_ROM_PTR(&network_rt_net_ifconfig_obj) },
    { MP_ROM_QSTR(MP_QSTR_config),                  MP_ROM_PTR(&network_rt_wlan_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),                  MP_ROM_PTR(&network_rt_wlan_status_obj) },
// { MP_ROM_QSTR(MP_QSTR_stop),                    MP_ROM_PTR(&network_rt_wlan_stop_obj) },             // only for ap
// { MP_ROM_QSTR(MP_QSTR_info),                    MP_ROM_PTR(&py_rt_wlan_info_type) },                 // only for ap

    // security
    { MP_ROM_QSTR(MP_QSTR_SECURITY_OPEN),           MP_ROM_INT(SECURITY_OPEN) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WEP_PSK),        MP_ROM_INT(SECURITY_WEP_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WEP_SHARED),     MP_ROM_INT(SECURITY_WEP_SHARED) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA_TKIP_PSK),   MP_ROM_INT(SECURITY_WPA_TKIP_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA_TKIP_8021X), MP_ROM_INT(SECURITY_WPA_TKIP_8021X) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA_AES_PSK),    MP_ROM_INT(SECURITY_WPA_AES_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA_AES_8021X),  MP_ROM_INT(SECURITY_WPA_AES_8021X) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA2_AES_PSK),   MP_ROM_INT(SECURITY_WPA2_AES_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA2_AES_8021X), MP_ROM_INT(SECURITY_WPA2_AES_8021X) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA2_TKIP_PSK),  MP_ROM_INT(SECURITY_WPA2_TKIP_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA2_TKIP_8021X), MP_ROM_INT(SECURITY_WPA2_TKIP_8021X) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA2_MIXED_PSK), MP_ROM_INT(SECURITY_WPA2_MIXED_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA_WPA2_MIXED_PSK), MP_ROM_INT(SECURITY_WPA_WPA2_MIXED_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA_WPA2_MIXED_8021X), MP_ROM_INT(SECURITY_WPA_WPA2_MIXED_8021X) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPA2_AES_CMAC),  MP_ROM_INT(SECURITY_WPA2_AES_CMAC) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPS_OPEN),       MP_ROM_INT(SECURITY_WPS_OPEN) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_WPS_SECURE),     MP_ROM_INT(SECURITY_WPS_SECURE) },
	{ MP_ROM_QSTR(MP_QSTR_SECURITY_WPA3_AES_PSK),   MP_ROM_INT(SECURITY_WPA3_AES_PSK) },
    { MP_ROM_QSTR(MP_QSTR_SECURITY_UNKNOWN),        MP_ROM_INT(SECURITY_UNKNOWN) },

    // bandrate
    { MP_ROM_QSTR(MP_QSTR_BAND_5GHZ),               MP_ROM_INT(RT_802_11_BAND_5GHZ) },
    { MP_ROM_QSTR(MP_QSTR_BAND_2_4GHZ),             MP_ROM_INT(RT_802_11_BAND_2_4GHZ) },
};

const mp_obj_type_t network_type_wlan_sta, network_type_wlan_ap;

STATIC py_rt_net_obj_t network_rt_wlan_sta = {{(mp_obj_type_t *)&network_type_wlan_sta}, MOD_NETWORK_STA_IF};
STATIC py_rt_net_obj_t network_rt_wlan_ap = {{(mp_obj_type_t *)&network_type_wlan_ap}, MOD_NETWORK_AP_IF};

STATIC mp_obj_t py_rt_wlan_type_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    mp_obj_t rt_net_obj;

    if(type == &network_type_wlan_sta) {
        rt_net_obj = MP_OBJ_FROM_PTR(&network_rt_wlan_sta);
    } else {
        rt_net_obj = MP_OBJ_FROM_PTR(&network_rt_wlan_ap);
    }

    // Register with network module
    mod_network_register_nic(rt_net_obj);

    return rt_net_obj;
}

STATIC int network_type_sta_init_flag = 0;

STATIC mp_map_elem_t network_type_sta_locals_dict_table[MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 3];
STATIC MP_DEFINE_CONST_DICT_WITH_SIZE(network_type_sta_locals_dict, network_type_sta_locals_dict_table, MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 3);

MP_DEFINE_CONST_OBJ_TYPE(
    network_type_wlan_sta,
    MP_QSTR_rt_wlan_sta,
    MP_TYPE_FLAG_NONE,
    make_new, py_rt_wlan_type_make_new,
    locals_dict, &network_type_sta_locals_dict,
    protocol, &mod_network_nic_protocol_rtt_posix
);

STATIC int network_type_ap_init_flag = 0;

STATIC mp_map_elem_t network_type_ap_locals_dict_table[MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 2];
STATIC MP_DEFINE_CONST_DICT_WITH_SIZE(network_type_ap_locals_dict, network_type_ap_locals_dict_table, MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 2);

MP_DEFINE_CONST_OBJ_TYPE(
    network_type_wlan_ap,
    MP_QSTR_rt_wlan_ap,
    MP_TYPE_FLAG_NONE,
    make_new, py_rt_wlan_type_make_new,
    locals_dict, &network_type_ap_locals_dict,
    protocol, &mod_network_nic_protocol_rtt_posix
);

STATIC mp_obj_t network_wlan_make_new(size_t n_args, const mp_obj_t *args)
{
    int itf = MOD_NETWORK_STA_IF;

    if(0 > s_net_mgmt_dev_fd) {
        if(0 > (s_net_mgmt_dev_fd = open("/dev/canmv_net_mgmt", O_RDWR))) {
            mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("could not open /dev/canmv_net_mgmt"));
        }
    }

    if(0x01 == n_args) {
        itf = mp_obj_get_int(args[0]);
    }

    if(MOD_NETWORK_STA_IF == itf) {
        if(0x00 == network_type_sta_init_flag) {
            network_type_sta_init_flag = 1;

            const mp_map_elem_t *base_dict_map = (const mp_map_elem_t *)&rt_wlan_locals_dict_table;
            mp_map_elem_t *sta_dict_map = (mp_map_elem_t *)&network_type_sta_locals_dict_table;
            memcpy(sta_dict_map, base_dict_map, MP_ARRAY_SIZE(rt_wlan_locals_dict_table) * sizeof(mp_map_elem_t));

            network_type_sta_locals_dict_table[MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 0] = \
                (mp_map_elem_t){ MP_ROM_QSTR(MP_QSTR_scan), MP_OBJ_FROM_PTR(&network_rt_wlan_scan_obj) };
            network_type_sta_locals_dict_table[MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 1] = \
                (mp_map_elem_t){ MP_ROM_QSTR(MP_QSTR_connect), MP_OBJ_FROM_PTR(&network_rt_wlan_connect_obj) };
            network_type_sta_locals_dict_table[MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 2] = \
                (mp_map_elem_t){ MP_ROM_QSTR(MP_QSTR_isconnected), MP_OBJ_FROM_PTR(&network_rt_wlan_isconnected_obj) };
        }

        return MP_OBJ_TYPE_GET_SLOT(&network_type_wlan_sta, make_new)(&network_type_wlan_sta, 0, 0, MP_OBJ_NULL);
    } else {
        if(0x00 == network_type_ap_init_flag) {
            network_type_ap_init_flag = 1;

            const mp_map_elem_t *base_dict_map = (const mp_map_elem_t *)&rt_wlan_locals_dict_table;
            mp_map_elem_t *ap_dict_map = (mp_map_elem_t *)&network_type_ap_locals_dict_table;
            memcpy(ap_dict_map, base_dict_map, MP_ARRAY_SIZE(rt_wlan_locals_dict_table) * sizeof(mp_map_elem_t));

            network_type_ap_locals_dict_table[MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 0] = \
                (mp_map_elem_t){ MP_ROM_QSTR(MP_QSTR_stop), MP_OBJ_FROM_PTR(&network_rt_wlan_stop_obj) };
            network_type_ap_locals_dict_table[MP_ARRAY_SIZE(rt_wlan_locals_dict_table) + 1] = \
                (mp_map_elem_t){ MP_ROM_QSTR(MP_QSTR_info), MP_OBJ_FROM_PTR(&py_rt_wlan_info_type) };
        }

        return MP_OBJ_TYPE_GET_SLOT(&network_type_wlan_ap, make_new)(&network_type_wlan_ap, 0, 0, MP_OBJ_NULL);
    }
}
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(network_wlan_make_new_obj, 0, 1, network_wlan_make_new);

#endif // CONFIG_ENABLE_NETWORK_RT_WLAN

void network_rt_wlan_deinit(void)
{
    close(s_net_mgmt_dev_fd);
    s_net_mgmt_dev_fd = -1;

#ifdef CONFIG_ENABLE_NETWORK_RT_WLAN
    network_type_sta_init_flag = 0;
    network_type_ap_init_flag = 0;
#endif // CONFIG_ENABLE_NETWORK_RT_WLAN
}
#endif
