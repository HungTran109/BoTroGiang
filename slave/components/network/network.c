#include "network.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "lwip/netif.h"
// #include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp32/rom/gpio.h"
#include "esp32/rom/gpio.h"
#include "esp_netif.h"
// #include "tcpip_adapter.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_eth.h"
#include <string.h>
#include "esp_smartconfig.h"
#include "app_mdns.h"
#include "esp_modem.h"
#include "ec2x.h"
#include "app_mqtt.h"
#include "main.h"
#include "board_def.h"

#define IDF_VERSION_BELOW_3V3       0
#define CONFIG_ETH_PHY_IP101        1
#if IDF_VERSION_BELOW_3V3

// Các define nằm trong sdkconfig (thay đổi qua menuconfig)
#if CONFIG_PHY_LAN8720
#include "eth_phy/phy_lan8720.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_lan8720_default_ethernet_config
#elif CONFIG_PHY_TLK110
#include "eth_phy/phy_tlk110.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_tlk110_default_ethernet_config
#elif CONFIG_PHY_IP101
#include "eth_phy/phy_ip101.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_ip101_default_ethernet_config
#endif

#else
#include "esp_eth.h"
#endif /* IDF_VERSION_BELOW_3V3 */

// Power pin for Ethernet PHY chip (define if used gpio pin)
#ifndef CONFIG_PHY_USE_POWER_PIN
#define CONFIG_PHY_USE_POWER_PIN
#endif

#if IDF_VERSION_BELOW_3V3
// Các define nằm trong sdkconfig (thay đổi qua menuconfig)
#if CONFIG_PHY_LAN8720
#include "eth_phy/phy_lan8720.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_lan8720_default_ethernet_config
#elif CONFIG_PHY_TLK110
#include "eth_phy/phy_tlk110.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_tlk110_default_ethernet_config
#elif CONFIG_PHY_IP101
#include "eth_phy/phy_ip101.h"
#define DEFAULT_ETHERNET_PHY_CONFIG phy_ip101_default_ethernet_config
#endif
#endif /* IDF_VERSION_BELOW_3V3 */

#define PIN_SMI_MDC CONFIG_PHY_SMI_MDC_PIN
#define PIN_SMI_MDIO CONFIG_PHY_SMI_MDIO_PIN
#define PIN_PHY_POWER -1//CONFIG_PHY_POWER_PIN

#define BIT_WIFI_GOT_IP             BIT0
#define BIT_ETH_GOT_IP              BIT1
#define BIT_PPP_GOT_IP              BIT2
#define BIT_ESPTOUCH_DONE           BIT3
#define BIT_ETH_DRIVER_ERROR        BIT4

#define RECONNECT_WIFI_IN_STREAM    30000
#define RECONNECT_WIFI_IN_IDLE      5000

static const char *TAG = "network";

static network_event_cb_t m_ip_evt_cb;
static EventGroupHandle_t m_event_bit_network;
static int s_retry_num = 0;
static bool m_wifi_connected = false;
static int32_t reconnect_wifi_interval_s = -1;
static bool m_wifi_started = false;
static network_interface_t m_network_interface = NETWORK_INTERFACE_UNKNOWN;
static char * m_wifi_ssid;
static char * m_wifi_pass;
static uint32_t m_last_reconnect_wifi_tick = 0;
static esp_netif_t *m_eth_netif;
static void smartconfig_task(void * parm);
static bool m_enable_start_config = false;
static bool m_smart_config_task_created = false;
uint8_t m_smart_cfg_ssid[33] = { 0 };
uint8_t m_smart_cfg_password[65] = { 0 };
static modem_dte_t *dte = NULL;
static modem_dce_t *dce = NULL;

static esp_netif_t *m_wifi_if;
static esp_netif_dns_info_t m_wifi_main_dns, m_wifi_backup_dns, m_wifi_fallback_dns;

static esp_netif_t *m_eth_netif;
static esp_netif_dns_info_t m_eth_main_dns, m_eth_backup_dns, m_eth_fallback_dns;
// ETH auto reset
static uint32_t m_auto_reset_eth_on_error = 0;
static uint32_t m_auto_reset_eth_interval = 30000;
esp_eth_handle_t m_eth_handle = NULL;
static bool is_eth_err =  false;
static char m_ip[2][24];

static void app_eth_store_dns(void)
{
    ESP_ERROR_CHECK(esp_netif_get_dns_info(m_eth_netif, ESP_NETIF_DNS_MAIN, &m_eth_main_dns));
    ESP_ERROR_CHECK(esp_netif_get_dns_info(m_eth_netif, ESP_NETIF_DNS_BACKUP, &m_eth_backup_dns));
    ESP_ERROR_CHECK(esp_netif_get_dns_info(m_eth_netif, ESP_NETIF_DNS_FALLBACK, &m_eth_fallback_dns));

    ESP_LOGD(TAG, "DNS servers : " IPSTR " , " IPSTR " , " IPSTR,
    IP2STR(&m_eth_main_dns.ip.u_addr.ip4),
    IP2STR(&m_eth_backup_dns.ip.u_addr.ip4),
    IP2STR(&m_eth_fallback_dns.ip.u_addr.ip4));
}

void app_eth_restore_dns(void)
{
    if (m_eth_netif == NULL)
    {
        return;
    }
    esp_netif_set_dns_info(m_eth_netif, ESP_NETIF_DNS_MAIN, &m_eth_main_dns);
    esp_netif_set_dns_info(m_eth_netif, ESP_NETIF_DNS_BACKUP, &m_eth_backup_dns);
    esp_netif_set_dns_info(m_eth_netif, ESP_NETIF_DNS_FALLBACK, &m_eth_fallback_dns);
}

static void app_wifi_store_dns(void)
{
    ESP_ERROR_CHECK(esp_netif_get_dns_info(m_wifi_if, ESP_NETIF_DNS_MAIN, &m_wifi_main_dns));
    ESP_ERROR_CHECK(esp_netif_get_dns_info(m_wifi_if, ESP_NETIF_DNS_BACKUP, &m_wifi_backup_dns));
    ESP_ERROR_CHECK(esp_netif_get_dns_info(m_wifi_if, ESP_NETIF_DNS_FALLBACK, &m_wifi_fallback_dns));

    ESP_LOGD(TAG, "DNS servers : " IPSTR " , " IPSTR " , " IPSTR,
    IP2STR(&m_wifi_main_dns.ip.u_addr.ip4),
    IP2STR(&m_wifi_backup_dns.ip.u_addr.ip4),
    IP2STR(&m_wifi_fallback_dns.ip.u_addr.ip4));
}

void app_wifi_restore_dns(void)
{
    if (m_wifi_if == NULL)
    {
        return;
    }
    
    esp_netif_set_dns_info(m_wifi_if, ESP_NETIF_DNS_MAIN, &m_wifi_main_dns);
    esp_netif_set_dns_info(m_wifi_if, ESP_NETIF_DNS_BACKUP, &m_wifi_backup_dns);
    esp_netif_set_dns_info(m_wifi_if, ESP_NETIF_DNS_FALLBACK, &m_wifi_fallback_dns);
}


static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        if (m_wifi_ssid && m_wifi_pass && m_enable_start_config == false)
        {
            esp_wifi_connect();
            m_wifi_connected = false;
            m_wifi_started = false;
        }
        else if (m_enable_start_config && m_smart_config_task_created == false)
        {
            m_smart_config_task_created = true;
            xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
        }
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START\r\n");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        xEventGroupClearBits(m_event_bit_network, BIT_WIFI_GOT_IP);
        m_wifi_connected = false;
        ESP_LOGI(TAG, "retry to connect to the AP %s", m_wifi_ssid);
        EventBits_t ip_status = xEventGroupGetBits(m_event_bit_network);
        xEventGroupClearBits(m_event_bit_network, BIT_WIFI_GOT_IP);

        if (s_retry_num < 0xFFFF)
        {
            // If device do not have any ip
            if ((ip_status & BIT_ETH_GOT_IP) == 0 && (ip_status & BIT_PPP_GOT_IP) == 0 && m_smart_config_task_created == false)
            {
                m_last_reconnect_wifi_tick = xTaskGetTickCount();
                ESP_LOGD(TAG, "esp_wifi_connect\n");
                esp_wifi_connect();
            }
            else
            {
                reconnect_wifi_interval_s = RECONNECT_WIFI_IN_IDLE;
            }
            s_retry_num++;
        }
        ESP_LOGD(TAG, "connect to the AP fail, reconnect value %d\n", reconnect_wifi_interval_s);
        if (m_ip_evt_cb)
        {
            network_interface_src_t src = NETWORK_INTERFACE_SRC_WIFI;
            m_ip_evt_cb(&src, (void*)SYSTEM_EVENT_STA_DISCONNECTED, &ip_status, sizeof(ip_status));
        }
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        ESP_LOGI(TAG, "Connected");
    }
    else if (event_base == WIFI_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGD(TAG, "Got ip0");
    }
    
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        m_network_interface = NETWORK_INTERFACE_WIFI;
        if (network_is_eth_got_ip())
        {
            network_change_interface(NETWORK_INTERFACE_ETH);
        }
        // esp_netif_ip_info_t *ip_info = &event->ip_info;
        app_wifi_store_dns();
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        sprintf(m_ip[NETWORK_LOCAL_IP_WIFI], IPSTR,  IP2STR(&event->ip_info.ip));

        s_retry_num = 0;
        m_wifi_connected = true;
        xEventGroupSetBits(m_event_bit_network, BIT_WIFI_GOT_IP);
        if (m_ip_evt_cb)
        {
            uint32_t ip = xEventGroupGetBits(m_event_bit_network);
            network_interface_src_t src = NETWORK_INTERFACE_SRC_WIFI;
            m_ip_evt_cb(&src, (void*)SYSTEM_EVENT_STA_GOT_IP, &ip, sizeof(ip));
        }
    }

    if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) 
    {
        ESP_LOGI(TAG, "Scan done");
    } 
    else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) 
    {
        ESP_LOGI(TAG, "Found channel");
    } 
    else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) 
    {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t rvd_data[33] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) 
        {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(m_smart_cfg_ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(m_smart_cfg_password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", m_smart_cfg_ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", m_smart_cfg_password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) 
        {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i=0; i<33; i++) 
            {
                printf("%02x ", rvd_data[i]);
            }
            printf("\r\n");
        }

        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_wifi_connect();
    } 
    else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) 
    {
        ESP_LOGI(TAG, "ACK done");
        xEventGroupSetBits(m_event_bit_network, BIT_ESPTOUCH_DONE);
    }
}

static void smartconfig_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1) 
    {
        uxBits = xEventGroupWaitBits(m_event_bit_network, BIT_WIFI_GOT_IP | BIT_ESPTOUCH_DONE, true, false, portMAX_DELAY);
        if(uxBits & BIT_WIFI_GOT_IP) 
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
            // xEventGroupClearBits(m_event_bit_network, BIT_WIFI_GOT_IP);
        }
        if(uxBits & BIT_ESPTOUCH_DONE) 
        {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            m_smart_config_task_created = false;
            xEventGroupClearBits(m_event_bit_network, BIT_ESPTOUCH_DONE);
            if (strlen((char*)m_smart_cfg_ssid) && strlen((char*)m_smart_cfg_password))
            {
                if (m_ip_evt_cb)
                {
                    network_interface_src_t src = NETWORK_INTERFACE_SRC_SMART_CFG;
                    network_wifi_info_t info;
                    info.ssid = (char*)m_smart_cfg_ssid;
                    info.password = (char*)m_smart_cfg_password;
                    m_ip_evt_cb(&src, (void*)SYSTEM_EVENT_STA_GOT_IP, &info, sizeof(info));
                }
            }
            vTaskDelete(NULL); 
        }
    }
}

static void wifi_got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    // esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

    s_retry_num = 0;
    m_wifi_connected = true;
    xEventGroupSetBits(m_event_bit_network, BIT_WIFI_GOT_IP);
    if (m_ip_evt_cb)
    {
        uint32_t ip = xEventGroupGetBits(m_event_bit_network);
        network_interface_src_t src = NETWORK_INTERFACE_SRC_WIFI;
        m_ip_evt_cb(&src, (void*)SYSTEM_EVENT_STA_GOT_IP, &ip, sizeof(ip));
    }
}

static void wifi_create_default(void)
{
    static bool created = false;
    if (created == false)
    {
        created = true;
        m_wifi_if = esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        app_wifi_store_dns();

        esp_event_handler_instance_t instance_any_id;
        esp_event_handler_instance_t instance_got_ip;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &instance_any_id));
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &instance_got_ip));

        ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_got_ip_event_handler, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    }
}

void wifi_init_sta(char *ssid, char *password, bool stop_wifi_before_connect)
{
    ESP_LOGI(TAG, "Wifi user name %s, pass %s\r\n", ssid, password);

    if (stop_wifi_before_connect)
    {
        esp_wifi_stop();
    }

    wifi_create_default();
    wifi_config_t wifi_config = 
    {
        .sta =
        {
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg =
            {
                .capable = true,
                .required = false
            },
        },
    };


    sprintf((char *)wifi_config.sta.ssid, "%s", ssid);
    sprintf((char *)wifi_config.sta.password, "%s", password);
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wifi started\r\n");
}


static void eth_got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");

    sprintf(m_ip[NETWORK_LOCAL_IP_WIFI], IPSTR, IP2STR(&ip_info->ip));

    m_auto_reset_eth_interval = 30000;
    app_eth_store_dns();
    xEventGroupSetBits(m_event_bit_network, BIT_ETH_GOT_IP);

    EventBits_t network_status = xEventGroupGetBits(m_event_bit_network);
    if (network_status & BIT_PPP_GOT_IP)
    {
            /* Switch to PPP interface */
            /*
                Dirty fix : mot so case ETH van co IP nhung ko co mang
                Ma chua bo tri dc thoi gian toi uu -> chon mac dinh 4G de khac phuc case trne
             */
            network_change_interface(NETWORK_INTERFACE_PPP);
    }
    else
    {
        /* Switch to Ethernet interface */
        network_change_interface(NETWORK_INTERFACE_ETH);
    }
    if (m_ip_evt_cb)
    {
        network_interface_src_t src = NETWORK_INTERFACE_SRC_ETH;
        m_ip_evt_cb(&src, (void*)SYSTEM_EVENT_ETH_GOT_IP, NULL, 0);
    }
}

/******************************************************************************************/
/**
 * @brief 	: Event handler for ethernet
 *
 * @param ctx
 * @param event
 * @return esp_err_t
 */
static void ethernet_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{  
    uint8_t mac_addr[6] = {0};

    switch (event_id)
    {
    /* =================================== CÁC EVENT TỪ ETHERNET ==================================*/
    case ETHERNET_EVENT_CONNECTED:  //SYSTEM_EVENT_ETH_CONNECTED:
        //	        ESP_LOGI(TAG, "Ethernet Link Up - Connected");
        ESP_LOGD(TAG,"[ZIG] ETHERNET: Link Up - Connected\r\n");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        m_auto_reset_eth_interval = 30000;
        break;

    case ETHERNET_EVENT_DISCONNECTED: //SYSTEM_EVENT_ETH_DISCONNECTED:
    {
        //	        ESP_LOGI(TAG, "Ethernet Link Down - Disconnected");
        ESP_LOGD(TAG,"[ZIG] ETHERNET: Link Down - Disconnected\r\n");
        xEventGroupClearBits(m_event_bit_network, BIT_ETH_GOT_IP);
        EventBits_t network_status = xEventGroupGetBits(m_event_bit_network);

        /* Switch to other interface available */
        if (network_status & BIT_WIFI_GOT_IP)
        {
            network_change_interface(NETWORK_INTERFACE_WIFI);
        }
        else if (network_status & BIT_PPP_GOT_IP)
        {
            network_change_interface(NETWORK_INTERFACE_PPP);
        }

        if (m_ip_evt_cb)
        {
            network_interface_src_t src = NETWORK_INTERFACE_SRC_ETH;
            m_ip_evt_cb(&src, (void*)SYSTEM_EVENT_ETH_DISCONNECTED, NULL, 0);
        }
    }
        break;

    case ETHERNET_EVENT_START: //SYSTEM_EVENT_ETH_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;

    case ETHERNET_EVENT_STOP:// SYSTEM_EVENT_ETH_STOP:
        //	        ESP_LOGI(TAG, "Ethernet Stopped");
        ESP_LOGD(TAG,"[ZIG] ETHERNET: Stopped\r\n");
        xEventGroupClearBits(m_event_bit_network, BIT_ETH_GOT_IP);
        break;

    default:
        // // if (m_wifi_enable)
        // {
        //     wifi_event_handle(ctx, event);
        // }
        break;
    }

    return;
}

bool m_debug_ppp_disconnect = false;
bool network_debug_and_clear_ppp_disconnect_status()
{
    bool retval = m_debug_ppp_disconnect;
    m_debug_ppp_disconnect = false;
    return retval;
}

void network_eth_restart(void)
{
    gpio_set_level(PIN_PHY_POWER, 0);
    vTaskDelay(10);
    gpio_set_level(PIN_PHY_POWER, 1);
}

static void modem_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case MODEM_EVENT_PPP_START:
        ESP_LOGI(TAG, "Modem PPP Started");
        break;

    case MODEM_EVENT_PPP_CONNECT:
        /* 
            https://github.com/espressif/esp-idf/issues/8080
            When PPP is connected, esp lwip will clear and change dns of others interface
            =>> WiFi and ETH couldnot resolve hostname anymore
            =>> Solution : Restore dns of wifi and ppp everytimes ppp got ip
        */
        app_wifi_restore_dns();
        app_eth_restore_dns();
        ESP_LOGI(TAG, "Modem Connect to PPP Server");
        ppp_client_ip_info_t *ipinfo = (ppp_client_ip_info_t *)(event_data);
        ESP_LOGD(TAG, "~~~~~~~~~~~~~~");
        ESP_LOGD(TAG, "IP          : " IPSTR, IP2STR(&ipinfo->ip));
        ESP_LOGD(TAG, "Netmask     : " IPSTR, IP2STR(&ipinfo->netmask));
        ESP_LOGD(TAG, "Gateway     : " IPSTR, IP2STR(&ipinfo->gw));
        ESP_LOGD(TAG, "Name Server1: " IPSTR, IP2STR(&ipinfo->ns1));
        ESP_LOGD(TAG, "Name Server2: " IPSTR, IP2STR(&ipinfo->ns2));
        ESP_LOGD(TAG, "~~~~~~~~~~~~~~");
        xEventGroupSetBits(m_event_bit_network, BIT_PPP_GOT_IP);

        /* Khi link up 1 network interface thì netif_default mặc định switch về cái đó
         * Muốn chuyển sang dùng interface khác thì phải changeNetInterface
         */
        m_network_interface = NETWORK_INTERFACE_PPP;

        // /** Nếu đang kết nối Ethernet/WiFi và -> thử chuyển sang dùng Ethernet/WiFi
        //  * (WiFi connected chưa chắc có Internet!)
        //  */
        // if (network_is_eth_got_ip())
        // {
        //     network_change_interface(NETWORK_INTERFACE_ETH);
        // }
        // else if (network_is_wifi_got_ip())
        // {
        //     network_change_interface(NETWORK_INTERFACE_WIFI);
        // }

        if (m_ip_evt_cb)
        {
            network_interface_src_t src = NETWORK_INTERFACE_SRC_PPP;
            m_ip_evt_cb(&src, (void*)MODEM_EVENT_PPP_CONNECT, ipinfo, sizeof(ipinfo));
        }
        break;

    case MODEM_EVENT_PPP_DISCONNECT:
        m_debug_ppp_disconnect = true;
        ESP_LOGI(TAG, "Modem Disconnect from PPP Server");
        xEventGroupClearBits(m_event_bit_network, BIT_PPP_GOT_IP);

        /** 
         * Nếu đang kết nối Ethernet/WiFi và -> thử chuyển sang dùng Ethernet/WiFi
         * (WiFi connected chưa chắc có Internet!)
         */
        if (network_is_eth_got_ip())
        {
            network_change_interface(NETWORK_INTERFACE_ETH);
        }
        else if (network_is_wifi_got_ip())
        {
            network_change_interface(NETWORK_INTERFACE_WIFI);
        }
        if (m_ip_evt_cb)
        {
            uint32_t ip = xEventGroupGetBits(m_event_bit_network);
            network_interface_src_t src = NETWORK_INTERFACE_SRC_PPP;
            m_ip_evt_cb(&src, (void*)MODEM_EVENT_PPP_DISCONNECT, &ip, sizeof(ip));
        }
        break;

    case MODEM_EVENT_PPP_STOP:
        m_debug_ppp_disconnect = true;
        ESP_LOGI(TAG, "Modem PPP Stopped");
        xEventGroupClearBits(m_event_bit_network, BIT_PPP_GOT_IP);
        // Reopen ppp connection
        esp_modem_setup_ppp(dte);
        if (m_ip_evt_cb)
        {
            uint32_t ip = xEventGroupGetBits(m_event_bit_network);
            network_interface_src_t src = NETWORK_INTERFACE_SRC_PPP;
            m_ip_evt_cb(&src, (void*)MODEM_EVENT_PPP_STOP, &ip, sizeof(ip));
        }
        break;

    case MODEM_EVENT_INIT_DONE: /* Khởi tạo xong module 4G */
        ESP_LOGI(TAG, "Modem Init DONE");
        if (m_ip_evt_cb)
        {
            network_interface_src_t src = NETWORK_INTERFACE_SRC_PPP;
            m_ip_evt_cb(&src, (void*)MODEM_EVENT_INIT_DONE, dce, sizeof(dce));
        }
        break;

    case MODEM_EVENT_UNKNOWN:
        ESP_LOGW(TAG, "Unknown line received: %s", (char *)event_data);
        break;

    default:
        break;
    }
}

static bool m_wifi_enable = false;
static bool do_reconnect_wifi = false;
void network_enable_wifi(bool enable)
{
    m_wifi_enable = enable;
}

static void wifi_process(void)
{
    if (m_wifi_enable == false)
    {
        return;
    }
    if (do_reconnect_wifi && m_smart_config_task_created == false)
    {
        do_reconnect_wifi = false;
        wifi_init_sta(m_wifi_ssid, m_wifi_pass, m_wifi_started);
    }
    else if (!m_wifi_connected && (reconnect_wifi_interval_s > -1) && m_smart_config_task_created == false)
    {
        uint32_t now = xTaskGetTickCount();
        if (now - m_last_reconnect_wifi_tick > (int32_t)reconnect_wifi_interval_s)
        {
            m_last_reconnect_wifi_tick = now;
            reconnect_wifi_interval_s = -1;
            esp_wifi_connect();
            // ESP_LOGW(TAG, "Reconnect wifi");
        }
    }

    static bool is_wifi_got_ip = false;
    if (m_wifi_connected)
    {
        if (!is_wifi_got_ip)
        {
            ESP_LOGI(TAG, "\t\t--- WiFi is Connected ---");
            // app_mdns_init();
            is_wifi_got_ip = true;

            /** Nếu đang không chạy bằng Ethernet, va ko stream  thì mới chuyển sang WiFi */
            if (m_network_interface != NETWORK_INTERFACE_ETH && !slave_at_least_one_master_is_streaming())
            {
                network_change_interface(NETWORK_INTERFACE_WIFI);
            }
        }
    }
    else
    {
        if (is_wifi_got_ip)
        {
            ESP_LOGW(TAG, "\t\t--- WiFi is Disconnected ---");
            is_wifi_got_ip = false;

            /* If ETH or PPP got IP -> Switch to ETH/PPP */
            EventBits_t network = xEventGroupGetBits(m_event_bit_network);
            if (network & BIT_ETH_GOT_IP)
            {
                if (m_network_interface != NETWORK_INTERFACE_ETH)
                {
                    network_change_interface(NETWORK_INTERFACE_ETH);
                }
            }
            else if (network & BIT_PPP_GOT_IP)
            {
                if (m_network_interface != NETWORK_INTERFACE_PPP)
                {
                    network_change_interface(NETWORK_INTERFACE_PPP);
                }
            }
        }
    }
}


bool network_is_connected(void)
{
    EventBits_t network = xEventGroupGetBits(m_event_bit_network);
    if (network & BIT_ETH_GOT_IP
        || network & BIT_WIFI_GOT_IP
        || network & BIT_PPP_GOT_IP)
    {
        return true;
    }
    return false;
}


/******************************************************************************************/
/**
 * @brief   : network_change_interface. Có thể giới hạn nếu đang stream thì ko chuyển mạng!!!
 * @param   :
 * @retval  : ESP_OK or ESP_FAIL
 * @author  :
 * @created :
 */
void network_change_interface(network_interface_t interface)
{
    ESP_LOGD(TAG, "network_change_interface...");
    // if (interface == m_network_interface)
    // {
    //     ESP_LOGI(TAG, "Already using interface %d\r\n", m_network_interface);
    //     return;
    // }

    if (interface == NETWORK_INTERFACE_WIFI)
    {
        ESP_LOGI(TAG, "Change network interface to WIFI");

        for (struct netif *pri = netif_list; pri != NULL; pri = pri->next)
        {
            if ((pri->name[0] == 's') && (pri->name[1] == 't'))
            {
                /* Network mask matches? */
                if (netif_is_up(pri))
                {
                    ESP_LOGI(TAG, "netif_is_up: true");

                    ESP_LOGI(TAG, "netif_set_default to wifi");
                    netif_set_default(pri);

                    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
                    ESP_LOGI(TAG, "Interface priority has been set to %.2s%d", pri->name, pri->num);

                    m_network_interface = NETWORK_INTERFACE_WIFI;
                }
                else
                {
                    ESP_LOGI(TAG, "netif_is_up: false");
                }
            }
        }
    }
    else if (interface == NETWORK_INTERFACE_ETH)
    {
        ESP_LOGI(TAG, "Change network interface to Ethernet");

        for (struct netif *pri = netif_list; pri != NULL; pri = pri->next)
        {
            if ((pri->name[0] == 'e') && (pri->name[1] == 'n'))
            {
                /* Network mask matches? */
                if (netif_is_up(pri))
                {
                    ESP_LOGI(TAG, "netif_is_up: true");
                    ESP_LOGI(TAG, "netif_set_default to ethernet");
                    netif_set_default(pri);

                    ESP_LOGI(TAG, "~~~~~~~~~~~~~~");
                    ESP_LOGI(TAG, "Interface priority has been set to %.2s%d", pri->name, pri->num);

                    m_network_interface = NETWORK_INTERFACE_ETH;
                }
                else
                {
                    ESP_LOGI(TAG, "netif_is_up: false");
                }
            }
        }
    }
    else if (interface == NETWORK_INTERFACE_PPP)
    {
        ESP_LOGI(TAG, "Change network interface to PPP");
        for (struct netif *pri = netif_list; pri != NULL; pri = pri->next)
        {
            if ((pri->name[0] == 'p') && (pri->name[1] == 'p'))
            {
                /* Network mask matches? */
                if (netif_is_up(pri))
                {
                    ESP_LOGW(TAG, "netif_is_up: true");
                    ESP_LOGW(TAG, "netif_set_default to ppp");
                    netif_set_default(pri);

                    ESP_LOGW(TAG, "~~~~~~~~~~~~~~");
                    ESP_LOGW(TAG, "Interface priority has been set to %.2s%d", pri->name, pri->num);

                    m_network_interface = NETWORK_INTERFACE_PPP;
                }
                else
                {
                    ESP_LOGW(TAG, "netif_is_up: false");
                }
            }
        }
    }
}

void network_smart_cfg_start(void)
{
    m_enable_start_config = true;
    if (m_smart_config_task_created == false)
    {
        ESP_LOGI(TAG, "Start smart config\r\n");
        esp_wifi_stop();
        wifi_create_default();
        ESP_ERROR_CHECK(esp_wifi_start());
    }
}

static void ec2x_start_task(void* arg)
{
    esp_modem_dte_config_t modem_config = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte = esp_modem_dte_init(&modem_config);

    /* Register event handler */
    ESP_ERROR_CHECK(esp_modem_add_event_handler(dte, modem_event_handler, NULL));

    /* create dce object */
    dce = ec2x_init(dte);
    vTaskDelete(NULL);
}

eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
esp_eth_mac_t *mac;
esp_eth_phy_t *phy;

static void start_eth_task(void *param)
{
    /* attach Ethernet driver to TCP/IP stack */
    ESP_ERROR_CHECK(esp_netif_attach(m_eth_netif, esp_eth_new_netif_glue(m_eth_handle)));
    /* start Ethernet driver state machine */
    ESP_ERROR_CHECK(esp_eth_start(m_eth_handle));
    vTaskDelete(NULL);
}

void network_initialize(network_wifi_info_t *info, network_event_cb_t on_ip_callback)
{
    m_ip_evt_cb = on_ip_callback;
    m_wifi_ssid = info->ssid;
    m_wifi_pass = info->password;
    m_event_bit_network = xEventGroupCreate();
    xEventGroupClearBits(m_event_bit_network, BIT_WIFI_GOT_IP | BIT_WIFI_GOT_IP | BIT_ETH_GOT_IP | BIT_ESPTOUCH_DONE);

#if 1
    // Initialize TCP/IP network interface (should be called only once in application)
    ESP_ERROR_CHECK(esp_netif_init());
    // Create default event loop that running in background
    esp_event_loop_create_default();
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    m_eth_netif = esp_netif_new(&cfg);
    app_eth_store_dns();
#ifndef BOARD_HW_DO_NOT_USE_GSM
    xTaskCreate(ec2x_start_task, "ec2x_task", 4096, NULL, 5, NULL);
#endif
// //    char *static_ip = net_info_load_static_ethernet_ip();
//     char *static_ip = NULL;
//     if (static_ip != NULL)
//     {
// 		// esp_netif_dhcpc_stop(m_eth_netif);
// 		// ESP_LOGI(TAG, "ETH: static IP %s", static_ip);
// 		// esp_netif_ip_info_t ip_info;
// 		// inet_aton(static_ip, &ip_info.ip);
// 		// IP4_ADDR(&ip_info.gw, 192, 168, 0, 1);
// 		// IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
// 		// esp_netif_set_ip_info(m_eth_netif, &ip_info);
//     }
    // Set default handlers to process TCP/IP stuffs
    ESP_ERROR_CHECK(esp_eth_set_default_handlers(m_eth_netif));
    // Register user defined event handers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &ethernet_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &eth_got_ip_event_handler, NULL));

    phy_config.reset_gpio_num = PIN_PHY_POWER;

    mac_config.smi_mdc_gpio_num = PIN_SMI_MDC;
    mac_config.smi_mdio_gpio_num = PIN_SMI_MDIO;
    mac = esp_eth_mac_new_esp32(&mac_config);
    phy = esp_eth_phy_new_ip101(&phy_config);


    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    if (ESP_OK != esp_eth_driver_install(&config, &m_eth_handle))
    {
        ESP_LOGI(TAG, "Install eth driver error");
        is_eth_err = true;
        xEventGroupSetBits(m_event_bit_network, BIT_ETH_DRIVER_ERROR);
    }
    else
    {
// ETH start is slow =>> Create task to optimize startup time
#if 0
        /* attach Ethernet driver to TCP/IP stack */
        ESP_ERROR_CHECK(esp_netif_attach(m_eth_netif, esp_eth_new_netif_glue(m_eth_handle)));
        /* start Ethernet driver state machine */
        ESP_ERROR_CHECK(esp_eth_start(m_eth_handle));
#else
        is_eth_err = false;
        xTaskCreate(start_eth_task, "eth_task", 4096, NULL, 3, NULL);
#endif
    }
#endif

    //	/* =================== Init WiFi ====================================== */
    if (strlen(m_wifi_ssid) > 0 && strlen(m_wifi_pass) > 0 && m_wifi_enable)
    {
        wifi_init_sta(m_wifi_ssid, m_wifi_pass, false);
    }
    else if (m_wifi_enable)
    {
        network_smart_cfg_start();
    }
}


void network_manager_poll(void)
{
    wifi_process();
    uint32_t now = xTaskGetTickCount();
    if (now - m_auto_reset_eth_on_error >= (uint32_t)30000)
    {
        m_auto_reset_eth_on_error = now;
        m_auto_reset_eth_interval += 30000;
        if (m_auto_reset_eth_interval >= 600000)
        {
            m_auto_reset_eth_interval = 600000;
        }
        EventBits_t network = xEventGroupGetBits(m_event_bit_network);
        if (network & BIT_ETH_DRIVER_ERROR)
        {
            // gpio_set_level(CONFIG_PHY_POWER_PIN, 0);
            // vTaskDelay(100/portTICK_RATE_MS);
            // gpio_set_level(CONFIG_PHY_POWER_PIN, 1);
            esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
            if (ESP_OK != esp_eth_driver_install(&config, &m_eth_handle))
            {
                ESP_LOGI(TAG, "Install eth driver error");
                xEventGroupSetBits(m_event_bit_network, BIT_ETH_DRIVER_ERROR);
            }
            else
            {   
                xEventGroupClearBits(m_event_bit_network, BIT_ETH_DRIVER_ERROR);
            }
        }
    }
}

bool network_is_wifi_got_ip(void)
{
    EventBits_t network = xEventGroupGetBits(m_event_bit_network);
    return (network & BIT_WIFI_GOT_IP);
}

bool network_is_ppp_got_ip(void)
{
    EventBits_t network = xEventGroupGetBits(m_event_bit_network);
    return (network & BIT_PPP_GOT_IP);
}

bool network_is_eth_got_ip(void)
{
    EventBits_t network = xEventGroupGetBits(m_event_bit_network);
    return (network & BIT_ETH_GOT_IP);
}

network_interface_t network_get_current_interface(void)
{
    return m_network_interface;
}

bool network_is_eth_phy_err(void)
{
    return is_eth_err;
}

char *network_get_local_ip(network_local_ip_t type)
{
    if (type == NETWORK_LOCAL_IP_WIFI)
    {
        return m_ip[NETWORK_LOCAL_IP_WIFI];
    }
    else
    {
        return m_ip[NETWORK_LOCAL_IP_ETH];
    }
}
