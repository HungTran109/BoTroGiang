/*
 * app_mdns.c
 *
 *  Created on: Mar 26, 2019
 *      Author: HuyTV
 */

#include <string.h>
#include "mdns.h"
#include <sys/socket.h>
#include <netdb.h>
#include "esp_log.h"
#include "sdkconfig.h"

// #define TAG "app_mdns"

static bool m_mdsn_init = false;
bool app_mdns_init(void)
{
    if (m_mdsn_init)
    {
        return true;
    }
    esp_err_t err;

    err = mdns_init();

    if (err != ESP_OK)
    {
        return false;
    }
    m_mdsn_init = true;

    return true;
}

bool app_mdns_discovery_host(char *host_name, char *return_ip, uint32_t timeout_ms)
{
    ip4_addr_t addr;

    addr.addr = 0;

    esp_err_t err = mdns_query_a(host_name, timeout_ms, &addr);

    if (err)
        return false;

    sprintf(return_ip, IPSTR, IP2STR(&addr));

    return true;
}
