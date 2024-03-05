#ifndef APP_OTA_H
#define APP_OTA_H

#include <stdint.h>
#include <stdbool.h>
/**
 * @brief           Start OTA task
 * @param[in]       url OTA url
 */
void app_ota_start(void *url);

/**
 * @brief           Check if ota is running
 * @retval          TRUE OTA is running
 *                  FALSE OTA is not running
 */
bool app_ota_is_running(void);

#endif /* APP_OTA_H */
