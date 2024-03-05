#ifndef APP_WIFI_ETH_H
#define APP_WIFI_ETH_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    char *ssid;
    char *password;
} network_wifi_info_t;
typedef enum
{
    NETWORK_INTERFACE_UNKNOWN,
    NETWORK_INTERFACE_WIFI,
    NETWORK_INTERFACE_ETH,
    NETWORK_INTERFACE_PPP,
} network_interface_t;

typedef enum
{
    NETWORK_INTERFACE_SRC_WIFI,
    NETWORK_INTERFACE_SRC_PPP,
    NETWORK_INTERFACE_SRC_ETH,
    NETWORK_INTERFACE_SRC_SMART_CFG
} network_interface_src_t;

typedef enum
{
    NETWORK_LOCAL_IP_WIFI,
    NETWORK_LOCAL_IP_ETH
} network_local_ip_t;

typedef void (*network_event_cb_t)(void *src, void *event, void *data, uint32_t size);


void network_initialize(network_wifi_info_t *info, network_event_cb_t on_ip_callback);

/**
 * @brief       Get network connection status
 * @retval      TRUE At least 1 network interface is connected
 *              FALSe No network interface is connected
 */
bool network_is_connected(void);

/**
 * @brief       Enable/disable WiFi
 * @param[in]   TRUE Enable wifi
 *              FALSE Disable wifi
 */
void network_enable_wifi(bool enable);

/**
 * @brief       Network manager process
 */
void network_manager_poll(void);


/**
 * @brief       Get current network interface
 * @retval      Current interface
 */
network_interface_t network_get_current_interface(void);


/**
 * @brief       Check if wifi is connected to AP and got IP
 * @retval      TRUE WiFi got ip
 *              FALSE WiFi doesnot got ip
 */
bool network_is_wifi_got_ip(void);

/**
 * @brief       Check if eth is connected to AP and got IP
 * @retval      TRUE ETH got ip
 *              FALSE ETH doesnot got ip
 */
bool network_is_eth_got_ip(void);

/**
 * @brief       Check PPP interface got IP
 * @retval      TRUE PPP got ip
 *              FALSE PPP doesnot got ip
 */
bool network_is_ppp_got_ip(void);

/**
 * @brief       Change network interface
 * @param[in]   interface New interface to change
 */
void network_change_interface(network_interface_t interface);

/**
 * @brief       Start smart config task
 */
void network_smart_cfg_start(void);

/**
 * @brief       Debug PPP only
 */
bool network_debug_and_clear_ppp_disconnect_status(void);

/**
 * @brief       Restart ETH driver
 */
void network_eth_restart(void);

/**
 * @brief       Check if ETH driver is error
 */
bool network_is_eth_phy_err(void);

/**
 * @brief       Get Local IP address
 * @retval      IP address, null on error
 */
char *network_get_local_ip(network_local_ip_t type);

#endif /* APP_WIFI_ETH_H */
