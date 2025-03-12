#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include "network_rt_opt.h"
#include <stdlib.h>

int main(int argc, char* argv[]) {  

network_init();
/*
uint8_t mac_buf[6]={0};
lan_get_mac(mac_buf);

printf("eth mac:%02X-%02X-%02X-%02X-%02X-%02X\n",mac_buf[0],mac_buf[1],mac_buf[2],mac_buf[3],mac_buf[4],mac_buf[5]);

char ip_addr[16]={0};
char netmask_addr[16]={0};
char gw_addr[16]={0};
char dns_addr[16]={0};
get_net_ifconfig(2,ip_addr,netmask_addr,gw_addr,dns_addr);
printf("before if_config:%s,%s,%s,%s\n",ip_addr,netmask_addr,gw_addr,dns_addr);

set_net_ifconfig(2,"192.168.4.10","255.255.255.0","192.168.4.1","8.8.8.8");


get_net_ifconfig(2,ip_addr,netmask_addr,gw_addr,dns_addr);
printf("after if_config:%s,%s,%s,%s\n",ip_addr,netmask_addr,gw_addr,dns_addr);
*/



uint8_t mac_buf[6]={0};
wlan_get_mac(mac_buf);

printf("wifi mac:%02X-%02X-%02X-%02X-%02X-%02X\n",mac_buf[0],mac_buf[1],mac_buf[2],mac_buf[3],mac_buf[4],mac_buf[5]);
//wlan_scan();
/*
size_t scan_result_info_size = 0;
scan_result_info_size = sizeof(struct rt_wlan_info) * RT_WLAN_STA_SCAN_MAX_AP;
struct rt_wlan_scan_result* scan_result=(struct rt_wlan_scan_result *)malloc(sizeof(struct rt_wlan_scan_result) + scan_result_info_size);
memset(scan_result, 0, sizeof(struct rt_wlan_scan_result) + scan_result_info_size);
scan_result->num = RT_WLAN_STA_SCAN_MAX_AP;
scan_result->info = (struct rt_wlan_info *)(((uint8_t *)scan_result) + sizeof(struct rt_wlan_scan_result));;
network_rt_wlan_scan(scan_result);
printf("scan_result->num:%d\n",scan_result->num);
      char ssid_buf[32]; 
      for(int32_t i = 0; i < scan_result->num; i++) {
        struct rt_wlan_info item_wlan_info=scan_result->info[i];
        memset(ssid_buf,0,32);
        memcpy(ssid_buf,item_wlan_info.ssid.val,item_wlan_info.ssid.len);
          printf("ssid:%s\n",ssid_buf);
    } 
free(scan_result);
*/
int wlan_scan_num;
struct rt_wlan_info rt_wlan_info_list[16];
network_rt_wlan_scan(rt_wlan_info_list,16, &wlan_scan_num);
printf("scan_result->num:%d\n",wlan_scan_num);
char ssid_buf[32]; 
int show_wlan_num=wlan_scan_num>16?16:wlan_scan_num;
for(int32_t i = 0; i < show_wlan_num; i++) {
        struct rt_wlan_info item_wlan_info=rt_wlan_info_list[i];
        memset(ssid_buf,0,32);
        memcpy(ssid_buf,item_wlan_info.ssid.val,item_wlan_info.ssid.len);
          printf("ssid:%s\n",ssid_buf);
    } 










int ret_code=network_rt_wlan_connect("c3","12345678");
if(ret_code==0)
{
printf("network_rt_wlan_connect success\n");
}
else
{
printf("network_rt_wlan_connect fail\n");
}









#if 0
    int net_mgmt_dev_fd = open("/dev/canmv_net_mgmt", O_RDWR);
    if (net_mgmt_dev_fd < 0) {
        perror("open /dev/canmv_net_mgmt");
        return -1;
    }
    uint8_t mac_buf[6];
    memset(mac_buf,0,6);
    if(0x00== ioctl(net_mgmt_dev_fd, IOCTRL_LAN_GET_MAC, &mac_buf[0])) {
    printf("eth mac:%02X-%02X-%02X-%02X-%02X-%02X\n",mac_buf[0],mac_buf[1],mac_buf[2],mac_buf[3],mac_buf[4],mac_buf[5]);
    }
   //uint8_t mac_buf[6];
    memset(mac_buf,0,6);
    if(0x00== ioctl(net_mgmt_dev_fd, IOCTRL_WM_STA_GET_MAC, &mac_buf[0])) {
    printf("wifi mac:%02X-%02X-%02X-%02X-%02X-%02X\n",mac_buf[0],mac_buf[1],mac_buf[2],mac_buf[3],mac_buf[4],mac_buf[5]);
    }
   
   
   int status = 0;
   if(0x00 == ioctl(s_net_mgmt_dev_fd, IOCTRL_LAN_GET_STATUS, &status)) { 
   printf("lan status:%d\n",status);
   }
   
   #endif
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
   
    while(1)
    {}
    //close(net_mgmt_dev_fd);
    return 0;
}
