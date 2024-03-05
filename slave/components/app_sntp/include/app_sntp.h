/*
 * app_sntp.h
 *
 *  Created on: Apr 27, 2021
 *      Author: huybk
 */

#ifndef COMPONENTS_APP_SNTP_INCLUDE_APP_SNTP_H_
#define COMPONENTS_APP_SNTP_INCLUDE_APP_SNTP_H_

#include <stdint.h>

/**
 * @brief		Start SNTP
 */
void app_sntp_start(void);

/**
 * @brief		Poll sntp every 1 second
 */
void app_sntp_poll();

/**
 * @brief		Debug current time
 */
void app_sntp_debug_timenow(void);

/**
 * @brief		Get current system timestamp
 * @retval		Counter
 */
uint32_t app_sntp_timestamp();

#endif /* COMPONENTS_APP_SNTP_INCLUDE_APP_SNTP_H_ */
