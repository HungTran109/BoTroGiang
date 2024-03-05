// For OTA
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "app_ota.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/task.h"
#include "version_ctrl.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
// #include "app_mqtt.h"
// #include "app_io.h"
#include "utilities.h"
#include "app_ota.h"
#include "min.h"
#include "min_id.h"
#include "app_io.h"
// #include "sys_ctx.h"
#include "app_mqtt.h"
#include "main.h"
#include "app_flash.h"
#include "app_debug.h"

#define OTA_TIMEOUT_MASTER              (200000)
#define OTA_TIMEOUT_SLAVE               (400000)
#define EXPANDER_HTTP_RECV_BUFFER       (128)       // IO expander rx http_rx_buffer is small, so please carefully when increase http rx http_rx_buffer
#define ESP32_HTTP_RECV_BUFFER          (1024)
#define OTA_SLAVE_START_TIMEOUT_MS      (5000)
#define OTA_SLAVE_TRANSFER_TIMEOUT_MS   (500)

#define BIT_RECEIVED_FRAME_ACK          (BIT0)
#define BIT_RECEIVED_FRAME_FAILED       (BIT1)
#define BIT_ALL                         (BIT0 | BIT1)

static TimerHandle_t m_timer_ota = NULL;
static EventGroupHandle_t m_io_ota_event_group = NULL;
static uint32_t m_downloaded_size;
static uint32_t m_binary_size = 0;
static uint32_t m_downloaded_300kb = 0;
static app_ota_download_state_t m_ota_download_state = APP_OTA_DOWNLOAD_STATE_INIT;
static uint32_t m_size = 0;
static esp_err_t http_ota_event_handler(esp_http_client_event_t *evt);
static bool m_ota_is_running = false;

static void on_ota_timeout(void *timer)
{
    ets_printf("OTA timeout\r\n");
    system_software_reset(SW_RESET_REASON_OTA_FINISH);
    esp_restart();
}

static void ota_report_download(uint32_t downloaded)
{
    char *ota_report = malloc(64);
    if (!ota_report)
    {
        DEBUG_ERROR("OTA mem allocate error!\r\n");
        return;
    }
    sprintf(ota_report, "OTA: %u/%u", downloaded, m_binary_size);
    mqtt_publish_message("DBG", ota_report);
    free(ota_report);
}

static esp_err_t http_ota_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        DEBUG_ERROR("HTTP_EVENT_ERROR\r\n");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        DEBUG_INFO("HTTP_EVENT_ON_CONNECTED\r\n");
        m_downloaded_size = 0;
        m_downloaded_300kb = 0;
        m_binary_size = 0;
        mqtt_publish_message("DBG", "OTA: connected");
        break;

    case HTTP_EVENT_HEADER_SENT:
        DEBUG_VERBOSE("HTTP_EVENT_HEADER_SENT\r\n");
        break;
    case HTTP_EVENT_ON_HEADER:
        // HTTP_EVENT_ON_HEADER, key=Content-Length, value=1224336
        DEBUG_VERBOSE("HTTP_EVENT_ON_HEADER, key=%s, value=%s\r\n", evt->header_key, evt->header_value);
        if (strstr(evt->header_key, "Content-Length"))
        {
            m_binary_size = GetNumberFromString(0, evt->header_value);
            DEBUG_INFO("OTA Binary size: %u\r\n", m_binary_size);
            ota_report_download(0);
        }
        break;
    case HTTP_EVENT_ON_DATA:
        //		DEBUG_INFO("HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        m_downloaded_300kb += evt->data_len;
        m_downloaded_size += evt->data_len;
        if (m_downloaded_300kb >= 300000)
        {
            m_downloaded_300kb = 0;
            DEBUG_INFO("HTTP_EVENT_ON_DATA, downloaded=%d\r\n", m_downloaded_size);
            ota_report_download(m_downloaded_size);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        DEBUG_VERBOSE("HTTP_EVENT_ON_FINISH. Total: %u bytes\r\n", m_downloaded_size);
        ota_report_download(m_downloaded_size);
        mqtt_publish_message("DBG", "OTA Finish");
        break;
    case HTTP_EVENT_DISCONNECTED:
        DEBUG_VERBOSE("HTTP_EVENT_DISCONNECTED\r\n");
        break;
    default:
        break;
    }
    return ESP_OK;
}

app_ota_download_state_t app_ota_get_state(void)
{
    return m_ota_download_state;
}

bool app_ota_is_running(void)
{
    return m_ota_is_running;
}

void app_ota_on_slave_frame_callback(void *msg)
{
    switch (((min_msg_t*)msg)->id)
    {
        case MIN_ID_OTA_ACK:
        case MIN_ID_OTA_REQUEST_MORE:
            xEventGroupSetBits(m_io_ota_event_group, BIT_RECEIVED_FRAME_ACK);
            break;

        case MIN_ID_OTA_FAILED:
            xEventGroupSetBits(m_io_ota_event_group, BIT_RECEIVED_FRAME_FAILED);
            break;

        case MIN_ID_OTA_UPDATE_START:
            break;
        case MIN_ID_OTA_UPDATE_TRANSFER:
            break;
        case MIN_ID_OTA_UPDATE_END:
        {
            uint8_t result = *((uint8_t*)(((min_msg_t*)msg)->payload));
            if (result)
            {
                mqtt_publish_message("DBG", "OTA slave success");
                xEventGroupSetBits(m_io_ota_event_group, BIT_RECEIVED_FRAME_ACK);
            }
            else
            {
                mqtt_publish_message("DBG", "OTA slave failed");
            }
        }
            break;
        default:
            break;
    }
}

void app_ota_download_task(void *pvParameter)
{
    m_ota_is_running = true;
    bool is_ota_download_failed = false;
    uint8_t download_timeout = 0;
    uint8_t retries = 3;
    esp_err_t ret;
    char *http_rx_buffer;

    DEBUG_INFO("\t\t--- ota_download_task is started ---\r\n");

    app_ota_info_t *ota_info = (app_ota_info_t*)pvParameter;
    
        // Create timer for ota timeout
    if (!m_timer_ota)
    {
        m_timer_ota = xTimerCreate("ota_timeout",
                                    OTA_TIMEOUT_SLAVE,
                                    pdFALSE,
                                    NULL,
                                    on_ota_timeout);
        assert(m_timer_ota);
        assert(xTimerStart(m_timer_ota, 0));
    }

    esp_http_client_config_t config = 
    {
        .url = ota_info->url,
        //        .cert_pem = (char *)server_cert_pem_start,	//Không sử dụng SSL -> Cấu hình: CONFIG_OTA_ALLOW_HTTP=y trong menu 'ESP HTTPS OTA'
        .auth_type = HTTP_AUTH_TYPE_NONE, // Không sử dụng xác thực
        .event_handler = http_ota_event_handler,
        .skip_cert_common_name_check = true,
        .timeout_ms = 5000,
        .buffer_size = 1024,
    };

    if (ota_info->type == APP_OTA_DEVICE_ESP32)
    {
        //#ifdef CONFIG_EXAMPLE_SKIP_COMMON_NAME_CHECK
        //    config.skip_cert_common_name_check = true;
        //#endif

    OTA_DOWNLOAD_BIN:
        ret = esp_https_ota(&config);
        if (ret == ESP_OK)
        {
            DEBUG_INFO("'esp_https_ota' successfully. OTA size: %u\r\n", m_downloaded_size);
            ota_report_download(m_downloaded_size);

            download_timeout = 5;
        }
        else
        {
            DEBUG_ERROR("Firmware upgrade failed\r\n");
            if (retries > 0)
            {
                retries--;
                DEBUG_INFO("Retry download %d\r\n", retries);

                mqtt_publish_message("DBG", "OTA Failed!");

                vTaskDelay(5000 / portTICK_PERIOD_MS);
                goto OTA_DOWNLOAD_BIN;
            }
            else
            {
                is_ota_download_failed = true;
                esp_restart();
            }
        }

        while (1)
        {
            if (download_timeout > 0)
            {
                DEBUG_INFO("System will restart to upgrade in %d seconds\r\n", download_timeout);
                download_timeout--;
                if (download_timeout == 0)
                {
                    DEBUG_INFO("Restart system to upgrade...\r\n");
                    vTaskDelay(1000);
                    // SoftResetSystem(SW_RESET_REASON_OTA_FINISH);
                    esp_restart();
                }
            }

            if (is_ota_download_failed)
                break;

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    else
    {
        // Create event group for ota
        if (!m_io_ota_event_group)
        {
            m_io_ota_event_group = xEventGroupCreate();
            DEBUG_INFO("Create event group\r\n");
        }

        xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);
        http_rx_buffer = malloc(EXPANDER_HTTP_RECV_BUFFER);
        if (http_rx_buffer == NULL) 
        {
            DEBUG_ERROR("Cannot malloc http receive http_rx_buffer\r\n");
            vTaskDelay(1000);
            esp_restart();
            return;
        }

        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_err_t err;
        if ((err = esp_http_client_open(client, 0)) != ESP_OK) 
        {
            DEBUG_ERROR("Failed to open HTTP connection: %s\r\n", esp_err_to_name(err));
            free(http_rx_buffer);
            esp_restart();
            return;
        }

        // Send data to slave
        m_ota_download_state = APP_OTA_DOWNLOAD_STATE_CONNECTED;
        int content_length = esp_http_client_fetch_headers(client);
        int total_read_len = 0, read_len;
        if (content_length >= 0 && total_read_len < content_length) 
        {
            m_ota_download_state = APP_OTA_DOWNLOAD_STATE_DOWNLOADING;
            xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);
            DEBUG_INFO("Send ota begin frame, size %u\r\n", content_length);
            // Send id to begin ota update
            if (ota_info->type == APP_OTA_DEVICE_GD32)
            {
                app_io_send_custom_frame_to_gd32(MIN_ID_OTA_UPDATE_START, (uint8_t*)&content_length, 4);
            }
            else
            {
                app_io_send_custom_frame_to_fm(MIN_ID_OTA_UPDATE_START, (uint8_t*)&content_length, 4);
            }
            DEBUG_INFO("Wait for slave ack\r\n");
            // Wait for ack
            xEventGroupWaitBits(m_io_ota_event_group,  
                                BIT_RECEIVED_FRAME_ACK | BIT_RECEIVED_FRAME_FAILED,
                                pdFALSE,
                                pdFALSE,
                                OTA_SLAVE_START_TIMEOUT_MS);
            
            EventBits_t ack_status_result = xEventGroupGetBits(m_io_ota_event_group);
            if (ack_status_result & BIT_RECEIVED_FRAME_FAILED)
            {
                DEBUG_ERROR("OTA start failed\r\n");
                m_ota_download_state = APP_OTA_DOWNLOAD_STATE_FAILED;
                vTaskDelay(5000/portTICK_PERIOD_MS);
            }
            else
            {
                m_size = 0;
                while (m_size < content_length)
                {
                    vTaskDelay(5/portTICK_PERIOD_MS);
                    read_len = esp_http_client_read(client, http_rx_buffer, EXPANDER_HTTP_RECV_BUFFER);
                    if (read_len <= 0) 
                    {
                        DEBUG_ERROR("Error read data from http\r\n");
                        mqtt_publish_message("DBG", "HTTP OTA for slave error");
                        break;
                    }
                    else
                    {
                        m_size += read_len;
                        DEBUG_INFO("read_len = %d/%d\r\n", m_size, content_length);

                        // Send data to slave   
                        xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);
                        if (ota_info->type == APP_OTA_DEVICE_GD32)
                        {
                            app_io_send_custom_frame_to_gd32(MIN_ID_OTA_UPDATE_TRANSFER, (uint8_t*)http_rx_buffer, read_len);
                        }
                        else
                        {
                            app_io_send_custom_frame_to_fm(MIN_ID_OTA_UPDATE_TRANSFER, (uint8_t*)http_rx_buffer, read_len);
                        }

                        // Wait for ack
                        xEventGroupWaitBits(m_io_ota_event_group,  BIT_RECEIVED_FRAME_ACK | BIT_RECEIVED_FRAME_FAILED,
                                        pdFALSE,
                                        pdFALSE,
                                        OTA_SLAVE_TRANSFER_TIMEOUT_MS);
                        ack_status_result = xEventGroupGetBits(m_io_ota_event_group);
                        if (!(ack_status_result & BIT_RECEIVED_FRAME_ACK))
                        {
                            DEBUG_WARN("OTA ack failed\r\n");
                            mqtt_publish_message("DBG", "OTA ack failed");
                            xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);
                            m_ota_download_state = APP_OTA_DOWNLOAD_STATE_FAILED;
                            vTaskDelay(5000/portTICK_PERIOD_MS);
                            break;
                        }
                    }
                }

                if (m_size && m_size == content_length)
                {
                    DEBUG_INFO("All data received\r\n");

                    xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);

                    // Send final frame
                    if (ota_info->type == APP_OTA_DEVICE_GD32)
                    {
                        app_io_send_custom_frame_to_gd32(MIN_ID_OTA_UPDATE_END, NULL, 0);
                    }
                    else
                    {
                        app_io_send_custom_frame_to_fm(MIN_ID_OTA_UPDATE_END, NULL, 0);
                    }

                    // Wait for ack
                    xEventGroupWaitBits(m_io_ota_event_group,  BIT_RECEIVED_FRAME_ACK | BIT_RECEIVED_FRAME_FAILED,
                                        pdFALSE,
                                        pdFALSE,
                                        OTA_SLAVE_TRANSFER_TIMEOUT_MS);
                                        ack_status_result = xEventGroupGetBits(m_io_ota_event_group);
                    // If we didnot received an ack =>> clear all bit
                    if (!(ack_status_result & BIT_RECEIVED_FRAME_ACK))
                    {
                        DEBUG_WARN("Final frame : OTA ack failed\r\n");
                        m_ota_download_state = APP_OTA_DOWNLOAD_STATE_FAILED;
                        mqtt_publish_message("DBG", "OTA ack failed");
                        xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);
                    }
                    else
                    {
                        m_ota_download_state = APP_OTA_DOWNLOAD_STATE_SUCCESS;
                        mqtt_publish_message("DBG", "Slave all data downloaded");
                    }
                    vTaskDelay(5000/portTICK_PERIOD_MS);
                }
            }

            if (m_io_ota_event_group)
            {
                xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);
            }

            DEBUG_WARN("HTTP Stream reader Status = %d, content_length = %d\r\n",
                            esp_http_client_get_status_code(client),
                            esp_http_client_get_content_length(client));

            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            free(http_rx_buffer);
            
            xTimerStop(m_timer_ota, 0);
            xTimerDelete(m_timer_ota, 0);
            vTaskDelay(1000);
            esp_restart();
        }
        else
        {
            DEBUG_WARN("Invalid content_length\r\n");
            if (m_io_ota_event_group)
            {
                xEventGroupClearBits(m_io_ota_event_group, BIT_ALL);
            }
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            m_ota_download_state = APP_OTA_DOWNLOAD_STATE_FAILED;
            xTimerStop(m_timer_ota, 0);
            xTimerDelete(m_timer_ota, 0);
            free(http_rx_buffer);
            vTaskDelay(5000/portTICK_PERIOD_MS);
        }

    }

    DEBUG_INFO("\t\t--- ota_download_task is exit ---\r\n");
    m_ota_is_running = false;
    vTaskDelete(NULL);
}
