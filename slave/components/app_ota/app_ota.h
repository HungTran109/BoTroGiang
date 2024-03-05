

#ifndef APP_OTA_H
#define APP_OTA_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    APP_OTA_DEVICE_ESP32,
    APP_OTA_DEVICE_GD32,
    APP_OTA_DEVICE_FM,
    APP_OTA_DEVICE_INVALID
} app_ota_device_t;


typedef struct
{
    const char *url;
    app_ota_device_t type;
} app_ota_info_t;

typedef enum
{
    APP_OTA_DOWNLOAD_STATE_INIT,
    APP_OTA_DOWNLOAD_STATE_PROCESSING,
    APP_OTA_DOWNLOAD_STATE_CONNECTED,
    APP_OTA_DOWNLOAD_STATE_DOWNLOADING,
    APP_OTA_DOWNLOAD_STATE_SUCCESS,
    APP_OTA_DOWNLOAD_STATE_FAILED
} app_ota_download_state_t;

/**
 * @brief       GD32 or FM transfer ack callback
 */
void app_ota_on_slave_frame_callback(void *msg);

/**
 * @brief       Task ota
 */
void app_ota_download_task(void *pvParameter);

/**
 * @brief       TGet OTA task status
 * @retval      TRUE OTA task is running
 *              FALSE OTA task is stopped
 */
bool app_ota_is_running(void);

#endif /* APP_OTA_H */