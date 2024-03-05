#ifndef MIN_ID_H
#define MIN_ID_H

#include <stdint.h>

typedef struct
{
    uint8_t connection_state;
    uint8_t interface;
} __attribute__((packed)) min_connection_state_t;

typedef struct
{
    struct 
    {
        uint32_t server : 1;
        uint32_t eth : 1;
        uint32_t codec : 1;
        uint32_t gsm : 1;
        uint32_t NA : 28;
    } name;
    uint32_t value;
} __attribute__((packed)) min_jig_status_t;

typedef union
{
    min_jig_status_t jig_status;
    uint8_t gsm_imei[26];
    uint8_t sim_imei[26];
} __attribute__((packed)) min_jig_data_t;

#define MIN_ID_ESP_32_INFO                      0x01
/*OTA update state*/
#define MIN_ID_OTA_UPDATE_START                 0x02
#define MIN_ID_OTA_UPDATE_TRANSFER              0x03
#define MIN_ID_OTA_UPDATE_END                   0x04
#define MIN_ID_OTA_ACK                          0x05
#define MIN_ID_OTA_FAILED                       0x06
#define MIN_ID_OTA_REQUEST_MORE                 0x07
/*LCD converstation*/
#define MIN_ID_LCD_FM_DATA                      0x0B
#define MIN_ID_STREAM_STATE                     0x08
#define MIN_ID_CONECTTION_STATE                 0x09
#define MIN_ID_VOLUME                           0x0A
#define MIN_ID_RESET                            0x0C
#define MIN_ID_TIME                             0x0D
#define MIN_ID_AUDIO_DETECT_STATUS              0x1A
/**/
#define MIN_ID_PING                             0x0E
#define MIN_ID_FORWARD                          0x0F
#define MIN_ID_NEXTION_LCD                      0x10
#define MIN_ID_RESET_GD32_MAIN                  0x11
#define MIN_ID_RESET_FM                         0x11
#define MIN_ID_CONTROL_GPIO                     0x12
#define MIN_ID_JIG_DATA                         0x13
#define MIN_ID_MCU_EXPANDER_REPORT_CHIP         0x14

/* Trang thai hoat dong cua esp32 */
#define ESP32_MODE_IDLE 0
#define ESP32_MODE_FM 1
#define ESP32_MODE_MIC 2
#define ESP32_MODE_INTERNET 3
#define ESP32_MODE_NONE 4
#define ESP32_MODE_4G   5

/* Trang thai ket noi 4G */
#define ESP32_INTERNET_DISCONNECTED 0
#define ESP32_INTERNET_CONNECTED 1

/* Giao thuc ket noi */
#define ESP32_INTERNET_INTERFACE_UNKNOWN 0
#define ESP32_INTERNET_INTERFACE_WIFI 1
#define ESP32_INTERNET_INTERFACE_ETH 2
#define ESP32_INTERNET_INTERFACE_4G 3

#define MIN_ID_CTRL_EXT_GPIO 59

#define MIN_ID_FM_SHOW_IMEI 0x0D

#endif /* MIN_ID_H */
