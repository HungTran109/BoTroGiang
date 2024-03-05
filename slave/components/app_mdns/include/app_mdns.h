/*
 * app_mdns.h
 *
 *  Created on: Mar 26, 2019
 *      Author: HuyTV
 */

#ifndef APP_MDNS_APP_MDNS_H_
#define APP_MDNS_APP_MDNS_H_

#include <stdbool.h>
#include <stdint.h>

extern char app_mdns_local_host[];

/**
 * @brief       Initialize mdns service 
 * @retval      TRUE Initialize success
 *              FALSE Initialize failure
 */
bool app_mdns_init(void);

/**
 * @brief       Start mdns to discovery host
 * @param[in]   host_name     Host name
 * @param[out]  return_ip    Ip of host
 * @param[in]   timeout_ms    Max timeout in millisecond to find host
 * @retval      TRUE DNS host found
 *              FALSE Device didnot found host
 * 
 */
bool app_mdns_discovery_host(char *host_name, char* return_ip, uint32_t timeout_ms);

#endif /* APP_MDNS_APP_MDNS_H_ */
