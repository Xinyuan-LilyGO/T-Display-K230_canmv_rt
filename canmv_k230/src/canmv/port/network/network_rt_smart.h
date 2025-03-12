#ifndef __NETWOTK_RT_WLAN_H__
#define __NETWOTK_RT_WLAN_H__

#include <stdint.h>

typedef enum
{
    RT_WLAN_DEV_EVT_INIT_DONE = 0,
    RT_WLAN_DEV_EVT_CONNECT,
    RT_WLAN_DEV_EVT_CONNECT_FAIL,
    RT_WLAN_DEV_EVT_DISCONNECT,
    RT_WLAN_DEV_EVT_AP_START,
    RT_WLAN_DEV_EVT_AP_STOP,
    RT_WLAN_DEV_EVT_AP_ASSOCIATED,
    RT_WLAN_DEV_EVT_AP_DISASSOCIATED,
    RT_WLAN_DEV_EVT_AP_ASSOCIATE_FAILED,
    RT_WLAN_DEV_EVT_SCAN_REPORT,
    RT_WLAN_DEV_EVT_SCAN_DONE,
    RT_WLAN_DEV_EVT_MAX,
} rt_wlan_dev_event_t;

#define WEP_ENABLED        0x0001
#define TKIP_ENABLED       0x0002
#define AES_ENABLED        0x0004
#define WSEC_SWFLAG        0x0008
#define AES_CMAC_ENABLED    0x0010

#define SHARED_ENABLED  0x00008000
#define WPA_SECURITY    0x00200000
#define WPA2_SECURITY   0x00400000
#define WPA3_SECURITY	0x00800000
#define WPS_ENABLED     0x10000000

#define IEEE_8021X_ENABLED   0x80000000


#define RT_WLAN_FLAG_STA_ONLY    (0x1 << 0)
#define RT_WLAN_FLAG_AP_ONLY     (0x1 << 1)

#define RT_WLAN_STA_SCAN_MAX_AP     (64)

#ifndef RT_WLAN_SSID_MAX_LENGTH
#define RT_WLAN_SSID_MAX_LENGTH  (32)   /* SSID MAX LEN */
#endif

#ifndef RT_WLAN_BSSID_MAX_LENGTH
#define RT_WLAN_BSSID_MAX_LENGTH (6)    /* BSSID MAX LEN (default is 6) */
#endif

#ifndef RT_WLAN_PASSWORD_MAX_LENGTH
#define RT_WLAN_PASSWORD_MAX_LENGTH   (32)   /* PASSWORD MAX LEN*/
#endif

#ifndef RT_WLAN_DEV_EVENT_NUM
#define RT_WLAN_DEV_EVENT_NUM  (2)   /* EVENT GROUP MAX NUM */
#endif

// wlan basic
#define IOCTRL_WM_GET_AUTO_RECONNECT 0x00
#define IOCTRL_WM_SET_AUTO_RECONNECT 0x01

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

// lan
#define IOCTRL_LAN_GET_ISCONNECTED 0x80
#define IOCTRL_LAN_GET_ISACTIVE   0x81
#define IOCTRL_LAN_GET_STATUS   0x82
#define IOCTRL_LAN_GET_MAC   0x83
#define IOCTRL_LAN_SET_MAC   0x84

// network util
#define IOCTRL_NET_IFCONFIG 0x100
#define IOCTRL_NET_GETHOSTBYNAME 0x101


#define INVALID_INFO(_info)                                                    \
  do {                                                                         \
    memset((_info), 0, sizeof(struct rt_wlan_info));                           \
    (_info)->band = RT_802_11_BAND_UNKNOWN;                                    \
    (_info)->security = SECURITY_UNKNOWN;                                      \
    (_info)->channel = -1;                                                     \
  } while (0)

#define SSID_SET(_info, _ssid)                                                 \
  do {                                                                         \
    strncpy((char *)(_info)->ssid.val, (_ssid), RT_WLAN_SSID_MAX_LENGTH);      \
    (_info)->ssid.len = strlen((char *)(_info)->ssid.val);                     \
  } while (0)

/**
 * Enumeration of Wi-Fi security modes
 */
typedef enum
{
    SECURITY_OPEN           = 0,                                                            /**< Open security                                 */
    SECURITY_WEP_PSK        = WEP_ENABLED,                                                  /**< WEP PSK Security with open authentication     */
    SECURITY_WEP_SHARED     = ( WEP_ENABLED | SHARED_ENABLED ),                             /**< WEP PSK Security with shared authentication   */
    SECURITY_WPA_TKIP_PSK   = ( WPA_SECURITY  | TKIP_ENABLED ),                             /**< WPA PSK Security with TKIP                    */
    SECURITY_WPA_TKIP_8021X   = ( IEEE_8021X_ENABLED | WPA_SECURITY  | TKIP_ENABLED ),      /**< WPA 8021X Security with TKIP                  */
    SECURITY_WPA_AES_PSK    = ( WPA_SECURITY  | AES_ENABLED ),                              /**< WPA PSK Security with AES                     */
    SECURITY_WPA_AES_8021X    = ( IEEE_8021X_ENABLED | WPA_SECURITY  | AES_ENABLED ),       /**< WPA 8021X Security with AES                   */
    SECURITY_WPA2_AES_PSK   = ( WPA2_SECURITY | AES_ENABLED ),                              /**< WPA2 PSK Security with AES                    */
    SECURITY_WPA2_AES_8021X   = ( IEEE_8021X_ENABLED | WPA2_SECURITY | WEP_ENABLED ),       /**< WPA2 8021X Security with AES                  */
    SECURITY_WPA2_TKIP_PSK  = ( WPA2_SECURITY | TKIP_ENABLED ),                             /**< WPA2 PSK Security with TKIP                   */
    SECURITY_WPA2_TKIP_8021X  = ( IEEE_8021X_ENABLED | WPA2_SECURITY | TKIP_ENABLED ),      /**< WPA2 8021X Security with TKIP                 */
    SECURITY_WPA2_MIXED_PSK = ( WPA2_SECURITY | AES_ENABLED | TKIP_ENABLED ),               /**< WPA2 PSK Security with AES & TKIP             */
    SECURITY_WPA_WPA2_MIXED_PSK = ( WPA_SECURITY  | WPA2_SECURITY ),                        /**< WPA/WPA2 PSK Security                         */
    SECURITY_WPA_WPA2_MIXED_8021X = ( IEEE_8021X_ENABLED | WPA_SECURITY  | WPA2_SECURITY ), /**< WPA/WPA2 8021X Security                       */
    SECURITY_WPA2_AES_CMAC = ( WPA2_SECURITY | AES_CMAC_ENABLED),                           /**< WPA2 Security with AES and Management Frame Protection                 */

    SECURITY_WPS_OPEN       = WPS_ENABLED,                                                  /**< WPS with open security                  */
    SECURITY_WPS_SECURE     = (WPS_ENABLED | AES_ENABLED),                                  /**< WPS with AES security                   */

	SECURITY_WPA3_AES_PSK 	= (WPA3_SECURITY | AES_ENABLED),						        /**< WPA3-AES with AES security  */

    SECURITY_UNKNOWN        = -1,                                                           /**< May be returned by scan function if security is unknown. Do not pass this to the join function! */

} rt_wlan_security_t;

typedef enum
{
    RT_802_11_BAND_5GHZ  =  0,             /* Denotes 5GHz radio band   */
    RT_802_11_BAND_2_4GHZ =  1,            /* Denotes 2.4GHz radio band */
    RT_802_11_BAND_UNKNOWN = 0x7fffffff,   /* unknown */
} rt_802_11_band_t;

typedef enum
{
    RT_COUNTRY_AFGHANISTAN,
    RT_COUNTRY_ALBANIA,
    RT_COUNTRY_ALGERIA,
    RT_COUNTRY_AMERICAN_SAMOA,
    RT_COUNTRY_ANGOLA,
    RT_COUNTRY_ANGUILLA,
    RT_COUNTRY_ANTIGUA_AND_BARBUDA,
    RT_COUNTRY_ARGENTINA,
    RT_COUNTRY_ARMENIA,
    RT_COUNTRY_ARUBA,
    RT_COUNTRY_AUSTRALIA,
    RT_COUNTRY_AUSTRIA,
    RT_COUNTRY_AZERBAIJAN,
    RT_COUNTRY_BAHAMAS,
    RT_COUNTRY_BAHRAIN,
    RT_COUNTRY_BAKER_ISLAND,
    RT_COUNTRY_BANGLADESH,
    RT_COUNTRY_BARBADOS,
    RT_COUNTRY_BELARUS,
    RT_COUNTRY_BELGIUM,
    RT_COUNTRY_BELIZE,
    RT_COUNTRY_BENIN,
    RT_COUNTRY_BERMUDA,
    RT_COUNTRY_BHUTAN,
    RT_COUNTRY_BOLIVIA,
    RT_COUNTRY_BOSNIA_AND_HERZEGOVINA,
    RT_COUNTRY_BOTSWANA,
    RT_COUNTRY_BRAZIL,
    RT_COUNTRY_BRITISH_INDIAN_OCEAN_TERRITORY,
    RT_COUNTRY_BRUNEI_DARUSSALAM,
    RT_COUNTRY_BULGARIA,
    RT_COUNTRY_BURKINA_FASO,
    RT_COUNTRY_BURUNDI,
    RT_COUNTRY_CAMBODIA,
    RT_COUNTRY_CAMEROON,
    RT_COUNTRY_CANADA,
    RT_COUNTRY_CAPE_VERDE,
    RT_COUNTRY_CAYMAN_ISLANDS,
    RT_COUNTRY_CENTRAL_AFRICAN_REPUBLIC,
    RT_COUNTRY_CHAD,
    RT_COUNTRY_CHILE,
    RT_COUNTRY_CHINA,
    RT_COUNTRY_CHRISTMAS_ISLAND,
    RT_COUNTRY_COLOMBIA,
    RT_COUNTRY_COMOROS,
    RT_COUNTRY_CONGO,
    RT_COUNTRY_CONGO_THE_DEMOCRATIC_REPUBLIC_OF_THE,
    RT_COUNTRY_COSTA_RICA,
    RT_COUNTRY_COTE_DIVOIRE,
    RT_COUNTRY_CROATIA,
    RT_COUNTRY_CUBA,
    RT_COUNTRY_CYPRUS,
    RT_COUNTRY_CZECH_REPUBLIC,
    RT_COUNTRY_DENMARK,
    RT_COUNTRY_DJIBOUTI,
    RT_COUNTRY_DOMINICA,
    RT_COUNTRY_DOMINICAN_REPUBLIC,
    RT_COUNTRY_DOWN_UNDER,
    RT_COUNTRY_ECUADOR,
    RT_COUNTRY_EGYPT,
    RT_COUNTRY_EL_SALVADOR,
    RT_COUNTRY_EQUATORIAL_GUINEA,
    RT_COUNTRY_ERITREA,
    RT_COUNTRY_ESTONIA,
    RT_COUNTRY_ETHIOPIA,
    RT_COUNTRY_FALKLAND_ISLANDS_MALVINAS,
    RT_COUNTRY_FAROE_ISLANDS,
    RT_COUNTRY_FIJI,
    RT_COUNTRY_FINLAND,
    RT_COUNTRY_FRANCE,
    RT_COUNTRY_FRENCH_GUINA,
    RT_COUNTRY_FRENCH_POLYNESIA,
    RT_COUNTRY_FRENCH_SOUTHERN_TERRITORIES,
    RT_COUNTRY_GABON,
    RT_COUNTRY_GAMBIA,
    RT_COUNTRY_GEORGIA,
    RT_COUNTRY_GERMANY,
    RT_COUNTRY_GHANA,
    RT_COUNTRY_GIBRALTAR,
    RT_COUNTRY_GREECE,
    RT_COUNTRY_GRENADA,
    RT_COUNTRY_GUADELOUPE,
    RT_COUNTRY_GUAM,
    RT_COUNTRY_GUATEMALA,
    RT_COUNTRY_GUERNSEY,
    RT_COUNTRY_GUINEA,
    RT_COUNTRY_GUINEA_BISSAU,
    RT_COUNTRY_GUYANA,
    RT_COUNTRY_HAITI,
    RT_COUNTRY_HOLY_SEE_VATICAN_CITY_STATE,
    RT_COUNTRY_HONDURAS,
    RT_COUNTRY_HONG_KONG,
    RT_COUNTRY_HUNGARY,
    RT_COUNTRY_ICELAND,
    RT_COUNTRY_INDIA,
    RT_COUNTRY_INDONESIA,
    RT_COUNTRY_IRAN_ISLAMIC_REPUBLIC_OF,
    RT_COUNTRY_IRAQ,
    RT_COUNTRY_IRELAND,
    RT_COUNTRY_ISRAEL,
    RT_COUNTRY_ITALY,
    RT_COUNTRY_JAMAICA,
    RT_COUNTRY_JAPAN,
    RT_COUNTRY_JERSEY,
    RT_COUNTRY_JORDAN,
    RT_COUNTRY_KAZAKHSTAN,
    RT_COUNTRY_KENYA,
    RT_COUNTRY_KIRIBATI,
    RT_COUNTRY_KOREA_REPUBLIC_OF,
    RT_COUNTRY_KOSOVO,
    RT_COUNTRY_KUWAIT,
    RT_COUNTRY_KYRGYZSTAN,
    RT_COUNTRY_LAO_PEOPLES_DEMOCRATIC_REPUBIC,
    RT_COUNTRY_LATVIA,
    RT_COUNTRY_LEBANON,
    RT_COUNTRY_LESOTHO,
    RT_COUNTRY_LIBERIA,
    RT_COUNTRY_LIBYAN_ARAB_JAMAHIRIYA,
    RT_COUNTRY_LIECHTENSTEIN,
    RT_COUNTRY_LITHUANIA,
    RT_COUNTRY_LUXEMBOURG,
    RT_COUNTRY_MACAO,
    RT_COUNTRY_MACEDONIA_FORMER_YUGOSLAV_REPUBLIC_OF,
    RT_COUNTRY_MADAGASCAR,
    RT_COUNTRY_MALAWI,
    RT_COUNTRY_MALAYSIA,
    RT_COUNTRY_MALDIVES,
    RT_COUNTRY_MALI,
    RT_COUNTRY_MALTA,
    RT_COUNTRY_MAN_ISLE_OF,
    RT_COUNTRY_MARTINIQUE,
    RT_COUNTRY_MAURITANIA,
    RT_COUNTRY_MAURITIUS,
    RT_COUNTRY_MAYOTTE,
    RT_COUNTRY_MEXICO,
    RT_COUNTRY_MICRONESIA_FEDERATED_STATES_OF,
    RT_COUNTRY_MOLDOVA_REPUBLIC_OF,
    RT_COUNTRY_MONACO,
    RT_COUNTRY_MONGOLIA,
    RT_COUNTRY_MONTENEGRO,
    RT_COUNTRY_MONTSERRAT,
    RT_COUNTRY_MOROCCO,
    RT_COUNTRY_MOZAMBIQUE,
    RT_COUNTRY_MYANMAR,
    RT_COUNTRY_NAMIBIA,
    RT_COUNTRY_NAURU,
    RT_COUNTRY_NEPAL,
    RT_COUNTRY_NETHERLANDS,
    RT_COUNTRY_NETHERLANDS_ANTILLES,
    RT_COUNTRY_NEW_CALEDONIA,
    RT_COUNTRY_NEW_ZEALAND,
    RT_COUNTRY_NICARAGUA,
    RT_COUNTRY_NIGER,
    RT_COUNTRY_NIGERIA,
    RT_COUNTRY_NORFOLK_ISLAND,
    RT_COUNTRY_NORTHERN_MARIANA_ISLANDS,
    RT_COUNTRY_NORWAY,
    RT_COUNTRY_OMAN,
    RT_COUNTRY_PAKISTAN,
    RT_COUNTRY_PALAU,
    RT_COUNTRY_PANAMA,
    RT_COUNTRY_PAPUA_NEW_GUINEA,
    RT_COUNTRY_PARAGUAY,
    RT_COUNTRY_PERU,
    RT_COUNTRY_PHILIPPINES,
    RT_COUNTRY_POLAND,
    RT_COUNTRY_PORTUGAL,
    RT_COUNTRY_PUETO_RICO,
    RT_COUNTRY_QATAR,
    RT_COUNTRY_REUNION,
    RT_COUNTRY_ROMANIA,
    RT_COUNTRY_RUSSIAN_FEDERATION,
    RT_COUNTRY_RWANDA,
    RT_COUNTRY_SAINT_KITTS_AND_NEVIS,
    RT_COUNTRY_SAINT_LUCIA,
    RT_COUNTRY_SAINT_PIERRE_AND_MIQUELON,
    RT_COUNTRY_SAINT_VINCENT_AND_THE_GRENADINES,
    RT_COUNTRY_SAMOA,
    RT_COUNTRY_SANIT_MARTIN_SINT_MARTEEN,
    RT_COUNTRY_SAO_TOME_AND_PRINCIPE,
    RT_COUNTRY_SAUDI_ARABIA,
    RT_COUNTRY_SENEGAL,
    RT_COUNTRY_SERBIA,
    RT_COUNTRY_SEYCHELLES,
    RT_COUNTRY_SIERRA_LEONE,
    RT_COUNTRY_SINGAPORE,
    RT_COUNTRY_SLOVAKIA,
    RT_COUNTRY_SLOVENIA,
    RT_COUNTRY_SOLOMON_ISLANDS,
    RT_COUNTRY_SOMALIA,
    RT_COUNTRY_SOUTH_AFRICA,
    RT_COUNTRY_SPAIN,
    RT_COUNTRY_SRI_LANKA,
    RT_COUNTRY_SURINAME,
    RT_COUNTRY_SWAZILAND,
    RT_COUNTRY_SWEDEN,
    RT_COUNTRY_SWITZERLAND,
    RT_COUNTRY_SYRIAN_ARAB_REPUBLIC,
    RT_COUNTRY_TAIWAN_PROVINCE_OF_CHINA,
    RT_COUNTRY_TAJIKISTAN,
    RT_COUNTRY_TANZANIA_UNITED_REPUBLIC_OF,
    RT_COUNTRY_THAILAND,
    RT_COUNTRY_TOGO,
    RT_COUNTRY_TONGA,
    RT_COUNTRY_TRINIDAD_AND_TOBAGO,
    RT_COUNTRY_TUNISIA,
    RT_COUNTRY_TURKEY,
    RT_COUNTRY_TURKMENISTAN,
    RT_COUNTRY_TURKS_AND_CAICOS_ISLANDS,
    RT_COUNTRY_TUVALU,
    RT_COUNTRY_UGANDA,
    RT_COUNTRY_UKRAINE,
    RT_COUNTRY_UNITED_ARAB_EMIRATES,
    RT_COUNTRY_UNITED_KINGDOM,
    RT_COUNTRY_UNITED_STATES,
    RT_COUNTRY_UNITED_STATES_REV4,
    RT_COUNTRY_UNITED_STATES_NO_DFS,
    RT_COUNTRY_UNITED_STATES_MINOR_OUTLYING_ISLANDS,
    RT_COUNTRY_URUGUAY,
    RT_COUNTRY_UZBEKISTAN,
    RT_COUNTRY_VANUATU,
    RT_COUNTRY_VENEZUELA,
    RT_COUNTRY_VIET_NAM,
    RT_COUNTRY_VIRGIN_ISLANDS_BRITISH,
    RT_COUNTRY_VIRGIN_ISLANDS_US,
    RT_COUNTRY_WALLIS_AND_FUTUNA,
    RT_COUNTRY_WEST_BANK,
    RT_COUNTRY_WESTERN_SAHARA,
    RT_COUNTRY_WORLD_WIDE_XX,
    RT_COUNTRY_YEMEN,
    RT_COUNTRY_ZAMBIA,
    RT_COUNTRY_ZIMBABWE,
    RT_COUNTRY_UNKNOWN
} rt_country_code_t;

struct rt_wlan_ssid
{
    uint8_t len;
    uint8_t val[RT_WLAN_SSID_MAX_LENGTH + 1];
};
typedef struct rt_wlan_ssid rt_wlan_ssid_t;

struct rt_wlan_key
{
    uint8_t len;
    uint8_t val[RT_WLAN_PASSWORD_MAX_LENGTH + 1];
};
typedef struct rt_wlan_key rt_wlan_key_t;

struct rt_wlan_info
{
    /* security type */
    rt_wlan_security_t security;
    /* 2.4G/5G */
    rt_802_11_band_t band;
    /* maximal data rate */
    uint32_t datarate;
    /* radio channel */
    int16_t channel;
    /* signal strength */
    int16_t  rssi;
    /* ssid */
    rt_wlan_ssid_t ssid;
    /* hwaddr */
    uint8_t bssid[RT_WLAN_BSSID_MAX_LENGTH];
    uint8_t hidden;
};

struct rt_sta_info
{
    rt_wlan_ssid_t ssid;
    rt_wlan_key_t key;
    uint8_t bssid[6];
    uint16_t channel;
    rt_wlan_security_t security;
};

struct rt_ap_info
{
    rt_wlan_ssid_t ssid;
    rt_wlan_key_t key;
    bool hidden;
    uint16_t channel;
    rt_wlan_security_t security;
};

struct rt_scan_info
{
    rt_wlan_ssid_t ssid;
    uint8_t bssid[6];
    int16_t channel_min;
    int16_t channel_max;
    bool passive;
};

struct rt_wlan_scan_result
{
    int32_t num;
    struct rt_wlan_info *info;
};


typedef struct ip4_addr
{
    uint32_t addr;
} ip4_addr_t;
typedef ip4_addr_t ip_addr_t;

#endif // __NETWOTK_RT_WLAN_H__
