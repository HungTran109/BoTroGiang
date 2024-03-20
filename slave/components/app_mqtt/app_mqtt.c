#include "app_mqtt.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "network.h"
#include "app_flash.h"
#include "app_ota.h"
#include "app_io.h"
#include "version_ctrl.h"
#include "lwip/init.h"
#include "esp_modem.h"
#include "esp_modem_dte.h"
#include "ec2x.h"
#include "utilities.h"
#include "app_audio.h"
#include "main.h"
#include "esp32/rom/rtc.h"
#include "tcp_console.h"
#include "app_debug.h"
#include "esp_http_client.h"
#include "httn.h"

static const char *TAG = "app_mqtt";

static uint8_t m_mqtt_sub_req_tick = 0;
// MQTT
static esp_mqtt_client_handle_t m_mqtt_client;
static esp_mqtt_client_config_t m_mqtt_config;
static char m_slave_sub_topic[96] = {0};
static int m_mqtt_sub_cfg_msg_id = 0;
static char m_ota_http_url[256];
bool m_is_mqtt_subscribed = false;
static int m_mqtt_sub_master_msg_id[APP_FLASH_MASTER_TOTAL] = {0};
static bool m_mqtt_is_master_subscribed[APP_FLASH_MASTER_TOTAL] = {false};
static uint8_t m_mqtt_send_resp_config_url_num = 0;

static app_mqtt_state_t m_mqtt_fsm = APP_MQTT_DISCONNECTED;
static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);
static bool do_reinit_mqtt = false;
static const char *m_audio_class_des[] = {"CLASS NONE", "CLASS AB", "CLASS D"};
static ping_message_t m_ping_msg;
static QueueHandle_t m_mqtt_queue = NULL;
static int m_last_http_code = 0;
static int v1_post_to_server(char *ping);

void mqtt_queue_streaming_init(void)
{
    if (m_mqtt_queue == NULL)
    {
        m_mqtt_queue = xQueueCreate(10, sizeof(mqtt_queue_stream_event_t));
    }
}
void mqtt_queue_streaming_put(mqtt_queue_stream_event_t *event)
{
    if (!xQueueSend(m_mqtt_queue, event, 0))
    {
        if (event->need_free)
        {
            free(event->payload);
            free(event->topic);
        }
    }
}

mqtt_queue_stream_event_t *mqtt_queue_streaming_get(void)
{
    static mqtt_queue_stream_event_t rx_queue;
    if (m_mqtt_queue == NULL)
    {
        return NULL;
    }
    if (xQueueReceive(m_mqtt_queue, &rx_queue, 0))
    {
        return &rx_queue;
    }
    return NULL;
}



bool app_mqtt_is_master_subscribed(uint8_t master_index)
{
    return m_mqtt_is_master_subscribed[master_index];
}

app_mqtt_state_t app_mqtt_get_state(void)
{
    return m_mqtt_fsm;
}

bool app_mqtt_is_connected_to_server(void)
{
    return (m_mqtt_fsm == APP_MQTT_CONNECTED) ? true : false;
}

void app_mqtt_set_state(app_mqtt_state_t state)
{
    m_mqtt_fsm = state;
}


/**
 * Gửi bản tin sau reset, check từng giây
 * Chỉ gửi khi đã lấy được thông tin IMEI của module GSM hoặc SIM (trường hợp online bằng Ethernet)
 */
void app_mqtt_send_reset_message(void)
{
    static bool is_sent = false;
    static uint8_t timeout_get_sim_imei = 35;

    modem_dce_t *dce = slave_get_modem_dce();

    if (is_sent)
        return;
    if (!dce)
        return;

    // Nếu chưa lấy được SIM IMEI thì chờ thêm...
    if (!IsDigitString(dce->sim_imei))
    {
        if (timeout_get_sim_imei > 0)
        {
            timeout_get_sim_imei--;
            return;
        }
    }

    char *reset_info = malloc(512+256);
    if (!reset_info)
    {
        ESP_LOGE(TAG, "Can't allocate mem!");
        esp_restart();
        return;
    }

    uint8_t uid[6];
    esp_efuse_mac_get_default(uid);
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t gsm_err = 0;
    app_flash_node_nvs_read_u32(APP_FLASH_KEY_GSM_ERR, &gsm_err);

    app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, 0);

    sprintf(reset_info, "RESET|MAC:%02X%02X%02X%02X%02X%02X,"
            "GSM:%s-%s,SIM:%s-%s,FW:%s-%s,HW:%d,Reason:%u-%u-%u,Count:%u,IO:%02X,VOL:%d,SpkDetect:(%s),SIM_ERR:%u,"
            "EXP_FW:%u,EXP_HW:%u,FM_FW:%u,FM_HW:%u,BUILD:%s-%s",
            uid[0], uid[1], uid[2],
            uid[3], uid[4], uid[5],
            app_flash_get_imei(), dce->name, dce->imsi, dce->sim_imei,
            __FIRMWARE_VERSION__, app_flash_get_protocol_priority_des(), app_io_get_hardware_version(),
            system_get_software_reset_reason(), rtc_get_reset_reason(0), rtc_get_reset_reason(1),
            app_flash_get_total_reset_time(), app_io_get_io_value()->Value,
            app_audio_get_current_output_vol(),
            m_audio_class_des[app_flash_speaker_audio_class_get()],
            gsm_err,
            app_io_get_last_gd32_msg()->fw_version,
            app_io_get_last_gd32_msg()->hw_version,
            app_io_get_last_fm_msg()->fw_version,
            app_io_get_last_fm_msg()->hw_version,
            __DATE__, __TIME__);

    int res = mqtt_publish_message("DBG", reset_info);
    free(reset_info);

    if (res >= 0)
    {
        DEBUG_INFO("mqtt send 'reset_msg' OK\r\n");
        is_sent = true;

        // Clear nguyên nhân reset trong NVS
        system_clear_software_reset_reason();
        app_flash_write_u8(APP_FLASH_SELF_RESET_KEY, 0);
        // DEBUG_INFO("Clear nguyen nhan reset %s", err == ESP_OK ? "OK" : "ERR");
    }
}

void app_mqtt_initialize(void)
{
    /* =================== End of Init Peripheral ========================== */

    /* =================== Config MQTT ===================================== */
    static char client_id[32] = {0};
    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(client_id, "VSSL_%s", app_flash_get_imei());
    }
    else
    {
        uint8_t uid[6];
        esp_efuse_mac_get_default(uid);

        sprintf(client_id, "VSSL_%02X%02X%02X%02X%02X%02X",
                uid[0], uid[1], uid[2], 
                uid[3], uid[4], uid[5]);
    }
    m_mqtt_config.client_id = client_id;
    m_mqtt_config.uri = app_flash_get_mqtt_server_url();
    m_mqtt_config.username = app_flash_get_mqtt_server_username();
    m_mqtt_config.password = app_flash_get_mqtt_server_password();
    m_mqtt_config.keepalive = APP_MQTT_KEEP_ALIVE_INTERVAL; /* seconds */
    m_mqtt_config.event_handle = mqtt_event_handler;
    DEBUG_INFO("Mqtt broker %s\r\n", m_mqtt_config.uri);
    DEBUG_INFO("Mqtt username %s\r\n", m_mqtt_config.username);
    DEBUG_INFO("Mqtt password %s\r\n", m_mqtt_config.password);
    DEBUG_INFO("Mqtt client_id %s\r\n", m_mqtt_config.client_id);
    if (strstr(m_mqtt_config.uri, "mqtts://"))
    {
        // m_mqtt_config.cert_pem = root_ca;
    }
    m_mqtt_client = esp_mqtt_client_init(&m_mqtt_config);
    if (!m_mqtt_client)
    {
        ESP_LOGE(TAG, "esp_mqtt_client_init: ERR!");
    }
}

int mqtt_publish_message(char *header, const char *format, ...)
{
    int len = 0;
    char *body = malloc(512);
    char topic[32];
    if (!body)
    {
        DEBUG_WARN("Can't allocate mem!\r\n");
        return -1;
    }
    int index = 0;
    int msg_id = 0;

    if (app_flash_get_protocol_priority() != APP_FLASH_PROTOCOL_V2_JSON)
    {
        index = sprintf(body, "%s,", header);
        va_list arg_ptr;

        if (strlen(app_flash_get_imei()) >= 15)
        {
            sprintf(topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, app_flash_get_imei());
        }
        else
        {
            sprintf(topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, "NA");
        }

        va_start(arg_ptr, format);
        len = vsnprintf(body+index, 512-index, format, arg_ptr);
        va_end(arg_ptr);

        
        msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index+len, 1, 0);
        DEBUG_VERBOSE("slave publish dbg OK, msg_id=%d\r\n", msg_id);
        free(body);
    }
    else
    {
        char time_debug[32];
        time_t t = time(NULL);
        struct tm tm = *localtime(&t);
        sprintf(time_debug, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

        index += sprintf(body+index, "{\"T\":\"%s\",\"C\":\"", time_debug);
        va_list arg_ptr;

        if (strlen(app_flash_get_imei()) >= 15)
        {
            sprintf(topic, "tx/%s/info", app_flash_get_imei());
        }
        else
        {
            sprintf(topic, "tx/%s/info", "NA");
        }

        va_start(arg_ptr, format);
        len = vsnprintf(body+index, 1024-index, format, arg_ptr);
        va_end(arg_ptr);
        
        index += len;
        index += sprintf(body+index, "%s", "\"}");

        msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index, 1, 0);
        DEBUG_VERBOSE("slave publish dbg OK, msg_id=%d\r\n", msg_id);
        free(body);
    }
    
    return msg_id;
}

int app_mqtt_publish_ttn_debug_msg(const char *format, ...)
{
    int len = 0;
    char *body = malloc(1024);
    char topic[32];
    char time_debug[32];
    if (!body)
    {
        DEBUG_WARN("Can't allocate mem!\r\n");
        return -1;
    }
    int index = 0;

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    sprintf(time_debug, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

    index += sprintf(body+index, "{\"T\":\"%s\",\"C\":\"", time_debug);
    va_list arg_ptr;

    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(topic, "tx/%s/info", app_flash_get_imei());
    }
    else
    {
        sprintf(topic, "tx/%s/info", "NA");
    }

    va_start(arg_ptr, format);
    len = vsnprintf(body+index, 1024-index, format, arg_ptr);
    va_end(arg_ptr);
    
    index += len;
    index += sprintf(body+index, "%s", "\"}");

    int msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index, 1, 0);
    DEBUG_VERBOSE("slave publish dbg OK, msg_id=%d\r\n", msg_id);
    free(body);
    
    return msg_id;
}

int mqtt_publish_event_msg(int evt_code, char *evt_name, const char *format, ...)
{
    int len = 0;
    char *body = malloc(1024);
    char topic[32];
    char time_debug[32];
    if (!body)
    {
        DEBUG_WARN("Can't allocate mem!\r\n");
        return -1;
    }
    int index = 0;

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    sprintf(time_debug, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

    index += sprintf(body+index, "{\"Id\":%d,\"Time\":\"%s\",\"Sender\":\"%s\",\"Message\":", 
                                    MSG_ID_EVENT, 
                                    time_debug,
                                    app_flash_get_imei());
    index += sprintf(body+index, "{\"Time\":\"%s\",\"EvCode\":%d,\"EvName\":\"%s\",\"Msg\":\"", 
                                    time_debug,
                                    evt_code,
                                    evt_name);

    va_list arg_ptr;

    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(topic, "tx/%s/info", app_flash_get_imei());
    }
    else
    {
        sprintf(topic, "tx/%s/info", "NA");
    }

    va_start(arg_ptr, format);
    len = vsnprintf(body+index, 1024-index, format, arg_ptr);
    va_end(arg_ptr);
    
    index += len;
    index += sprintf(body+index, "%s", "\"}}");

    int msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index, 1, 0);
    DEBUG_VERBOSE("slave publish event msg OK, msg_id=%d\r\n", msg_id);
    free(body);
    
    return msg_id;
}

int app_mqtt_send_pwr_lost(uint32_t vin_mv)
{
    uint8_t priority = app_flash_get_protocol_priority();
    if (priority != APP_FLASH_PROTOCOL_V2_JSON)
        mqtt_publish_message("DBG", "Power lost, Vin %umv", vin_mv);

    m_ping_msg.sys_err.name.pwr_lost = 1;
    return mqtt_publish_event_msg(EVT_CODE_PWR_LOST, "Power", "Power lost, vin %u", vin_mv);
}

bool app_mqtt_is_subscribed(void)
{
    return m_is_mqtt_subscribed;
}

esp_err_t app_mqtt_start(void)
{
    return esp_mqtt_client_start(m_mqtt_client);
}

void app_mqtt_close(void)
{
    esp_mqtt_client_stop(m_mqtt_client);
}

/******************************************************************************************/
/**
 * @brief   : Lưu cấu hình WiFi từ lệnh MQTT
 * @param   : data: WIFINAME(name),PASS(pass),ENABLE:1
 * @retval  :
 * @author  :
 * @created :
 */
bool do_reconnect_wifi = false;
void app_mqtt_write_wifi_config(char *data)
{
    // WIFINAME(name),PASS(pass),ENABLE:1

    char wifi_name[28] = {0};
    char wifi_pass[28] = {0};
    uint8_t wifi_en = 0;
    char *name = strstr(data, "WIFINAME(");
    uint32_t same_wifi = 0;
    if (name != NULL)
    {
        if (CopyParameter(name, wifi_name, '(', ')'))
        {
            if (strcmp(app_flash_get_wifi_name(), wifi_name))
            {
                DEBUG_INFO("New wiFi name: %s", wifi_name);
                sprintf(app_flash_get_wifi_name(), "%s", wifi_name);
                do_reconnect_wifi = true;
            }
            else
            {
                same_wifi++;
            }
        }
    }
    char *pass = strstr(data, "PASS(");
    if (pass != NULL)
    {
        if (CopyParameter(pass, wifi_pass, '(', ')'))
        {
            if (strcmp(app_flash_get_wifi_pass(), wifi_pass))
            {
                DEBUG_INFO("New wiFi pass: %s", wifi_pass);
                sprintf(app_flash_get_wifi_pass(), "%s", wifi_pass);
                do_reconnect_wifi = true;
            }
            else
            {
                same_wifi++;
            }
        }
    }
    char *enable = strstr(data, "ENABLE:");
    if (enable != NULL)
    {
        wifi_en = GetNumberFromString(7, enable);
        DEBUG_INFO("WiFi enable: %d", wifi_en);
        if (app_flash_is_wifi_enable() != wifi_en)
        {
            same_wifi = 0;
        }

        app_flash_wifi_enable(wifi_en);
        if (!app_flash_is_wifi_enable())
        {
            do_reconnect_wifi = false;
        }
    }
    esp_err_t err = ESP_OK;
    // if (!same_wifi)
    {
        app_flash_write_wifi_info(app_flash_get_wifi_name(),
                                app_flash_get_wifi_pass(), wifi_en);
    }
    DEBUG_INFO("Write wifi config to NVS: %s", err == ESP_OK ? "OK" : "ERR");

    if (err == ESP_OK && same_wifi != 2)
    {
        DEBUG_INFO("Reset system...");
        app_audio_change_codec_vol(0);
        app_audio_simple_terminate_pipeline();
        vTaskDelay(1000 / portTICK_RATE_MS);
        system_software_reset(SW_RESET_REASON_CONFIG_WIFI);
    }
    else if (err == ESP_OK && same_wifi == 2)
    {
        DEBUG_WARN("Save WiFi");
        mqtt_publish_message("DBG", "Same wifi username and password, no need to change");
    }
    else
    {
        ESP_LOGE(TAG, "Save WiFi config ERROR!");
        mqtt_publish_message("DBG", "Config WiFi ERR");
    }
}

/******************************************************************************************/
/**
 * @brief   : Xử lý lệnh SET cấu hình
 * @param   : data: SET,Lệnh 1,Lệnh 2,....,Lệnh N
 * @retval  :
 * @author  :
 * @created :
 */
static void mqtt_process_set_config_msg(char *configuration_data, char *topic)
{
    uint32_t update_slave_info = 0;
    char tmp_str[80] = {0};
    char *p_res = malloc(1024);
    uint16_t index = 0;
    uint8_t has_new_url = 0;
    bool report_invalid_msg = false;
    DEBUG_INFO("Config: %s\r\n", configuration_data);

    if (!p_res)
    {
        DEBUG_WARN("Can't alocate mem p_res!\r\n");
        return;
    }

#if 1
    // Lệnh cấu hình http stream url
    char *stream_url = strstr(configuration_data, "STREAM_URL(");
    if (stream_url != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));
        if (CopyParameter(stream_url, tmp_str, '(', ')'))
        {
            DEBUG_INFO("Set URL header: %s\r\n", tmp_str);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_http_stream_header(), tmp_str) != 0)
            {
                update_slave_info++;
                sprintf(app_flash_get_http_stream_header(), "%s", tmp_str);

                DEBUG_INFO("New URL header: %s\r\n", app_flash_get_http_stream_header());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_HTTP_STREAM_HEADER_KEY, app_flash_get_http_stream_header());
                index += sprintf(&p_res[index], "STREAM_URL=%s,", app_flash_get_http_stream_header());

                has_new_url = 1;
            }
            else
            {
                DEBUG_INFO("Same URL header: %s\r\n", app_flash_get_http_stream_header());
                index += sprintf(&p_res[index], "STREAM_URL=%s,", app_flash_get_http_stream_header());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }

    stream_url = strstr(configuration_data, "HTTP_POST(");
    if (stream_url != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));
        if (CopyParameter(stream_url, tmp_str, '(', ')'))
        {
            DEBUG_INFO("Set Post header: %s\r\n", tmp_str);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_http_post_url(), tmp_str) != 0)
            {
                update_slave_info++;
                sprintf(app_flash_get_http_post_url(), "%s", tmp_str);

                DEBUG_INFO("New POST header: %s\r\n", app_flash_get_http_post_url());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_HTTP_POST_HEADER_KEY, app_flash_get_http_post_url());
                index += sprintf(&p_res[index], "POST_URL(%s),", app_flash_get_http_post_url());

                // has_new_url = 1;
            }
            else
            {
                DEBUG_INFO("Same POST header: %s\r\n", app_flash_get_http_post_url());
                index += sprintf(&p_res[index], "POST_URL(%s),", app_flash_get_http_post_url());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }
#endif

    stream_url = strstr(configuration_data, "API_KEY(");
    if (stream_url != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));
        if (CopyParameter(stream_url, tmp_str, '(', ')'))
        {
            DEBUG_INFO("Set api key: %s\r\n", tmp_str);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_api_key(), tmp_str) != 0)
            {
                update_slave_info++;
                sprintf(app_flash_get_api_key(), "%s", tmp_str);

                DEBUG_INFO("New URL header: %s\r\n", app_flash_get_api_key());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_API, app_flash_get_api_key());
                index += sprintf(&p_res[index], "API_KEY=%s,", app_flash_get_api_key());

                // has_new_url = 1;
            }
            else
            {
                DEBUG_INFO("Same api key: %s\r\n", app_flash_get_api_key());
                index += sprintf(&p_res[index], "API_KEY=%s,", app_flash_get_api_key());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }

    // SET,MQTT_URL(mqtt://smart.radiotech.vn:1883),MQTT_USER(village-speaker),MQTT_PASS(vs.bytech@2019)
    char *mqtt_url = strstr(configuration_data, "MQTT_URL(");
    if (mqtt_url != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));
        if (CopyParameter(mqtt_url, tmp_str, '(', ')'))
        {
            update_slave_info++;
            DEBUG_INFO("Set URL header: %s\r\n", tmp_str);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_mqtt_server_url(), tmp_str) != 0)
            {
                memset(app_flash_get_mqtt_server_url(), 0, APP_FLASH_MQTT_URL_SIZE);
                snprintf(app_flash_get_mqtt_server_url(), APP_FLASH_MQTT_URL_SIZE, "%s", tmp_str);

                DEBUG_INFO("New mqtt url: %s\r\n", app_flash_get_mqtt_server_url());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_URL, app_flash_get_mqtt_server_url());
                index += sprintf(&p_res[index], "MQTT_URL=%s,", app_flash_get_mqtt_server_url());
            }
            else
            {
                DEBUG_INFO("Same URL header: %s\r\n", app_flash_get_mqtt_server_url());
                index += sprintf(&p_res[index], "MQTT_URL=%s,", app_flash_get_mqtt_server_url());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }


    char *mqtt_username = strstr(configuration_data, "MQTT_USER(");
    if (mqtt_username != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));
        if (CopyParameter(mqtt_username, tmp_str, '(', ')'))
        {
            DEBUG_INFO("Set URL header: %s\r\n", tmp_str);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_mqtt_server_username(), tmp_str) != 0)
            {
                update_slave_info++;
                memset(app_flash_get_mqtt_server_username(), 0, APP_FLASH_MQTT_USERNAME_SIZE);
                snprintf(app_flash_get_mqtt_server_username(), APP_FLASH_MQTT_USERNAME_SIZE, "%s", tmp_str);

                DEBUG_INFO("New mqtt username: %s\r\n", app_flash_get_mqtt_server_username());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_USERNAME, app_flash_get_mqtt_server_username());
                index += sprintf(&p_res[index], "MQTT_USER=%s,", app_flash_get_mqtt_server_username());
            }
            else
            {
                DEBUG_INFO("Same URL username: %s\r\n", app_flash_get_mqtt_server_username());
                index += sprintf(&p_res[index], "MQTT_USER=%s,", app_flash_get_mqtt_server_username());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }

    char *mqtt_pass = strstr(configuration_data, "MQTT_PASS(");
    if (mqtt_pass != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));
        if (CopyParameter(mqtt_pass, tmp_str, '(', ')'))
        {
            DEBUG_INFO("Set URL header: %s\r\n", tmp_str);

            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_mqtt_server_password(), tmp_str) != 0)
            {
                update_slave_info++;
                memset(app_flash_get_mqtt_server_password(), 0, APP_FLASH_MQTT_PASSWORD_SIZE);
                snprintf(app_flash_get_mqtt_server_password(), APP_FLASH_MQTT_PASSWORD_SIZE, "%s", tmp_str);

                DEBUG_INFO("New mqtt pass: %s\r\n", app_flash_get_mqtt_server_password());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_PASSWORD, app_flash_get_mqtt_server_password());
                index += sprintf(&p_res[index], "MQTT_PASS=%s,", app_flash_get_mqtt_server_password());
            }
            else
            {
                DEBUG_INFO("Same mqtt pass: %s\r\n", app_flash_get_mqtt_server_password());
                index += sprintf(&p_res[index], "MQTT_PASS=%s,", app_flash_get_mqtt_server_password());
            }

            // Giới hạn số lần gửi phản hồi config vì bản tin retain nhận được mỗi khi subscribe
            if (m_mqtt_send_resp_config_url_num <= 3)
                m_mqtt_send_resp_config_url_num++;
        }
    }

    // Mod: Lệnh cấu hình Master sẽ gửi nhóm IMEI: MASTER1(imei1|imei2|imei3)
    char *master1 = strstr(configuration_data, "MASTER1(");
    if (master1 != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));

        if (CopyParameter(master1, tmp_str, '(', ')'))
        {
            update_slave_info++;
            DEBUG_INFO("Set MASTER1: %s\r\n", tmp_str);
            index += sprintf(&p_res[index], "SVR_M1=%s,", tmp_str);

            char list_imei_T[4][25];
            uint8_t list_imei_index = 0;
            char *m_token = strtok(tmp_str, "|");
            while (m_token != NULL)
            {
                snprintf(list_imei_T[list_imei_index++], 25, "%s", m_token);
                m_token = strtok(NULL, "|");

                if (list_imei_index >= 3)
                    break;
            }

            for (uint8_t i = 0; i < 3; i++)
            {
                if (IsDigitString(list_imei_T[i]) && strlen(list_imei_T[i]) >= 3)
                { /* Cho phép clear imei master = "000" */
                    memset(app_flash_get_master(APP_FLASH_MASTER_TINH1 + i), 0, APP_FLASH_MASTER_IMEI_SIZE);
                    memcpy(app_flash_get_master(APP_FLASH_MASTER_TINH1 + i), list_imei_T[i], strlen(list_imei_T[i]));

                    // Write to NVS
                    if (i == 0)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_T1_KEY, app_flash_get_master(APP_FLASH_MASTER_TINH1));
                        index += sprintf(&p_res[index], "MASTER1=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH1));
                    }
                    else if (i == 1)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_T2_KEY, app_flash_get_master(APP_FLASH_MASTER_TINH2));
                        index += sprintf(&p_res[index], "MASTER1_2=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH2));
                    }
                    else if (i == 2)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_T3_KEY, app_flash_get_master(APP_FLASH_MASTER_TINH3));
                        index += sprintf(&p_res[index], "MASTER1_3=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH3));
                    }

                    // Send subscribe master1
                    app_mqtt_send_subscribe_command_topic(APP_FLASH_MASTER_TINH1 + i);
                }
            }
        }
    }
    char *master2 = strstr(configuration_data, "MASTER2(");
    if (master2 != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));

        if (CopyParameter(master2, tmp_str, '(', ')'))
        {
            DEBUG_INFO("Set MASTER2: %s\r\n", tmp_str);
            index += sprintf(&p_res[index], "SVR_M2=%s,", tmp_str);
            update_slave_info++;
            char list_imei_H[4][25];
            uint8_t list_imei_index = 0;
            char *m_token = strtok(tmp_str, "|");
            while (m_token != NULL)
            {
                snprintf(list_imei_H[list_imei_index++], 25, "%s", m_token);
                m_token = strtok(NULL, "|");

                if (list_imei_index >= 3)
                    break;
            }

            for (uint8_t i = 0; i < 3; i++)
            {
                if (IsDigitString(list_imei_H[i]) && strlen(list_imei_H[i]) >= 3)
                { /* Cho phép clear imei master = "000" */
                    memset(app_flash_get_master(APP_FLASH_MASTER_HUYEN1 + i), 0, APP_FLASH_MASTER_IMEI_SIZE);
                    memcpy(app_flash_get_master(APP_FLASH_MASTER_HUYEN1 + i), list_imei_H[i], strlen(list_imei_H[i]));

                    // Write to NVS
                    if (i == 0)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_H1_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
                        index += sprintf(&p_res[index], "MASTER2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
                    }
                    else if (i == 1)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_H2_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN2));
                        index += sprintf(&p_res[index], "MASTER2_2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN2));
                    }
                    else if (i == 2)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_H3_KEY, app_flash_get_master(APP_FLASH_MASTER_HUYEN3));
                        index += sprintf(&p_res[index], "MASTER2_3=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN3));
                    }

                    // Send subscribe master1
                    app_mqtt_send_subscribe_command_topic(APP_FLASH_MASTER_HUYEN1 + i);
                }
            }
        }
    }
    char *master3 = strstr(configuration_data, "MASTER3(");
    if (master3 != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));

        if (CopyParameter(master3, tmp_str, '(', ')'))
        {
            index += sprintf(&p_res[index], "SVR_M3=%s,", tmp_str);
            update_slave_info++;
            DEBUG_INFO("Set MASTER3: %s\r\n", tmp_str);

            char list_imei_X[4][25];
            uint8_t list_imei_index = 0;
            char *m_token = strtok(tmp_str, "|");
            while (m_token != NULL)
            {
                snprintf(list_imei_X[list_imei_index++], 25, "%s", m_token);
                m_token = strtok(NULL, "|");

                if (list_imei_index >= 3)
                    break;
            }

            for (uint8_t i = 0; i < 3; i++)
            {
                if (IsDigitString(list_imei_X[i]) && strlen(list_imei_X[i]) >= 3)
                { /* Cho phép clear imei master = "000" */
                    memset(app_flash_get_master(APP_FLASH_MASTER_XA1 + i), 0, APP_FLASH_MASTER_IMEI_SIZE);
                    memcpy(app_flash_get_master(APP_FLASH_MASTER_XA1 + i), list_imei_X[i], strlen(list_imei_X[i]));

                    // Write to NVS
                    if (i == 0)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_X1_KEY, app_flash_get_master(APP_FLASH_MASTER_XA1));
                        index += sprintf(&p_res[index], "MASTER3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA1));
                    }
                    else if (i == 1)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_X2_KEY, app_flash_get_master(APP_FLASH_MASTER_XA2));
                        index += sprintf(&p_res[index], "MASTER3_2=%s,", app_flash_get_master(APP_FLASH_MASTER_XA2));
                    }
                    else if (i == 2)
                    {
                        app_flash_node_nvs_write_string(APP_FLASH_MASTER_X3_KEY, app_flash_get_master(APP_FLASH_MASTER_XA3));
                        index += sprintf(&p_res[index], "MASTER3_3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA3));
                    }

                    // Send subscribe master1
                    app_mqtt_send_subscribe_command_topic(APP_FLASH_MASTER_XA1 + i);
                }
            }
        }
    }

    char *fmFreq1 = strstr(configuration_data, "FM_FREQ1("); /** Tần số đài Tỉnh/Trung ương */
    if (fmFreq1 != NULL)
    {
        uint32_t freq = GetNumberFromString(9, fmFreq1);
        if (freq > 0)
        {
            update_slave_info++;
            DEBUG_INFO("Set FM_FREQ1: %u\r\n", freq);
            app_flash_set_fm_freq1(freq); /* KHz */

            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ1_KEY, freq);
        }
    }
    char *fmFreq2 = strstr(configuration_data, "FM_FREQ2("); /** Tần số đài huyện */
    if (fmFreq2 != NULL)
    {
        uint32_t freq = GetNumberFromString(9, fmFreq2);
        if (freq > 0)
        {
            update_slave_info++;
            DEBUG_INFO("Set FM_FREQ2: %u\r\n", freq);
            app_flash_set_fm_freq2(freq); /* KHz */

            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ2_KEY, freq);
        }
    }
    char *fmFreq3 = strstr(configuration_data, "FM_FREQ3("); /** Tần số đài xã */
    if (fmFreq3 != NULL)
    {
        uint32_t freq = GetNumberFromString(9, fmFreq3);
        if (freq > 0)
        {
            update_slave_info++;
            DEBUG_INFO("Set FM_FREQ3: %u\r\n", freq);
            app_flash_set_fm_freq3(freq); /* KHz */

            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ3_KEY, freq);
        }
    }

    char *volume = strstr(configuration_data, "VOLUME(");
    if (volume != NULL)
    {
        uint8_t vol = GetNumberFromString(7, volume);
        if (vol > 100)
            vol = 100;
        update_slave_info++;
        DEBUG_INFO("Set VOLUME: %u\r\n", vol);
        app_flash_set_volume(vol); /* 0 - 100 */

        // Write to NVS
        app_flash_write_u8(APP_FLASH_VOLUME_KEY, vol);

        // Set volume cho audio codec
        uint8_t readback_vol = app_audio_change_codec_vol(app_flash_get_volume());

        // Add to response
        index += sprintf(&p_res[index], "VOLUME=%u,VOLUME_CODEC=%u,", app_flash_get_volume(), readback_vol);
        if (app_flash_get_volume() == 0)
        {
            app_io_control_pa(APP_IO_PA_OFF);
        }
    }

    /* Relay delay time */
    char *relay_delay_on = strstr(configuration_data, "DELAY_ON(");
    if (relay_delay_on != NULL)
    {
        update_slave_info++;
        uint8_t delay_on_sec = GetNumberFromString(9, relay_delay_on);

        DEBUG_INFO("Set Relay delay on: %u\r\n", delay_on_sec);

        if (delay_on_sec > 0 && delay_on_sec <= APP_FLASH_RELAY_DELAY_MAX_TIME)
        { /* 1 - 250 */
            app_flash_set_delay_turn_on_relay(delay_on_sec);

            // Write to NVS
            app_flash_write_u8(APP_FLASH_RELAY_DELAY_ON_KEY, delay_on_sec);

            // Add to response
            if (p_res)
            {
                index += sprintf(&p_res[index], "DELAY_ON=%u,", app_flash_get_delay_turn_on_relay());
            }
        }
    }

    char *relay1_delay_off = strstr(configuration_data, "DELAY_OFF1(");
    if (relay1_delay_off != NULL)
    {
        update_slave_info++;
        uint8_t delay_off_1 = GetNumberFromString(11, relay1_delay_off);

        DEBUG_INFO("Set Relay1 delay off: %u\r\n", delay_off_1);

        if (delay_off_1 > 0 && delay_off_1 <= APP_FLASH_RELAY_DELAY_MAX_TIME)
        { /* 1 - 250 */
            app_flash_set_relay1_turn_off_delay(delay_off_1);

            // Write to NVS
            app_flash_write_u8(APP_FLASH_RELAY1_DELAY_OFF_KEY, delay_off_1);

            // Add to response
            if (p_res)
            {
                index += sprintf(&p_res[index], "DELAY_OFF1=%u,", app_flash_get_relay1_turn_off_delay());
            }
        }
    }

    char *relay2_delay_off = strstr(configuration_data, "DELAY_OFF2(");
    if (relay2_delay_off != NULL)
    {
        update_slave_info++;
        uint16_t delay_off_2 = GetNumberFromString(11, relay2_delay_off);

        DEBUG_INFO("Set Relay2 delay off: %u\r\n", delay_off_2);

        if (delay_off_2 > 0 && delay_off_2 <= APP_FLASH_RELAY2_DELAY_MAX_TIME)
        { /* 1 - 250 */
            app_flash_set_relay2_turn_off_delay(delay_off_2);

            // Write to NVS
            app_flash_write_u16(APP_FLASH_RELAY2_DELAY_OFF_KEY, delay_off_2);

            // Add to response
            if (p_res)
            {
                index += sprintf(&p_res[index], "DELAY_OFF2=%u,", app_flash_get_relay2_turn_off_delay());
            }
        }
    }

    char *op_mode = strstr(configuration_data, "MODE(");
    if (op_mode != NULL)
    {
        update_slave_info++;
        change_mode_cmd(op_mode+5);
    }

    bool is_gpio_cfg = false;
    if (strstr(configuration_data, "IO1(ON)"))
    {
        update_slave_info++;
        is_gpio_cfg = true;
        app_io_get_io_value()->Name.IO1 = IO_ON;

        // Add to response
        index += sprintf(&p_res[index], "%s", "IO1=ON,");

        // Control relay
        app_io_control_opto_output1(APP_IO_OPTO_ON);
    }

    if (strstr(configuration_data, "IO1(OFF)") && app_io_get_io_value()->Name.IO1 == IO_ON)
    {
        if (slave_get_delay_turn_off_relay1_remain_time())
        {
            report_invalid_msg = true;
        }
        // else
        {
            app_io_get_io_value()->Name.IO1 = IO_OFF;
            // Add to response
            index += sprintf(&p_res[index], "%s", "IO1=OFF,");
            // Control relay
            app_io_control_opto_output1(APP_IO_OPTO_OFF);
            is_gpio_cfg = true;
        }
        update_slave_info++;
    }
    
    // Set speaker
    if (strstr(configuration_data, "SPK_DETECT(OFF)"))
    {
        update_slave_info++;
        DEBUG_INFO("Speaker off\r\n");
        if (app_flash_speaker_audio_class_get())
        {
            app_flash_write_u8(APP_FLASH_SPK_CLASS, APP_FLASH_AUDIO_CLASS_NONE);
            app_flash_speaker_audio_class_set(APP_FLASH_AUDIO_CLASS_NONE);
        }
    }
    else if (strstr(configuration_data, "SPK_DETECT(AB)"))
    {
        update_slave_info++;
        if (!app_flash_speaker_audio_class_get())
        {
            app_flash_write_u8(APP_FLASH_SPK_CLASS, APP_FLASH_AUDIO_CLASS_AB);
            app_flash_speaker_audio_class_set(APP_FLASH_AUDIO_CLASS_AB);
        }
    }
    else if (strstr(configuration_data, "SPK_DETECT(D)"))
    {
        update_slave_info++;
        if (!app_flash_speaker_audio_class_get())
        {
            app_flash_write_u8(APP_FLASH_SPK_CLASS, APP_FLASH_AUDIO_CLASS_D);
            app_flash_speaker_audio_class_set(APP_FLASH_AUDIO_CLASS_D);
        }
    }

    if (strstr(configuration_data, "IO2(ON)"))
    {
        update_slave_info++;
        is_gpio_cfg = true;
        app_io_get_io_value()->Name.IO2 = IO_ON;

        // Add to response
        index += sprintf(&p_res[index], "%s", "IO2=ON,");

        // Control relay
        app_io_control_opto_output2(APP_IO_OPTO_ON);
    }

    if (strstr(configuration_data, "IO2(OFF)"))
    {
        if (slave_get_delay_turn_off_relay2_remain_time() && app_io_get_io_value()->Name.IO2 == IO_ON)
        {
            report_invalid_msg = true;
        }
        // else
        {
            app_io_get_io_value()->Name.IO2 = IO_OFF;
            // Add to response
            index += sprintf(&p_res[index], "%s", "IO2=OFF,");
            // Control relay
            app_io_control_opto_output2(APP_IO_OPTO_OFF);
            is_gpio_cfg = true;
        }

        update_slave_info++;
    }

    if (is_gpio_cfg)
    {
        // Write to NVS
        app_flash_write_u8(APP_FLASH_IO_STATE_KEY, app_io_get_io_value()->Value);
    }

    // TCP console
    if (strstr(configuration_data, "CONSOLE(ON)"))
    {
        update_slave_info++;
        app_flash_tcp_console_enable();
        index += sprintf(&p_res[index], "%s", "TCP(1),");
        DEBUG_INFO("TCP console enable");
    }
    else if (strstr(configuration_data, "CONSOLE(OFF)"))
    {
        update_slave_info++;
        DEBUG_INFO("TCP console disable\r\n");
        index += sprintf(&p_res[index], "%s", "TCP(0),");
        app_flash_tcp_console_disable();
    }

    if (strstr(configuration_data, "FACTORY_RESET"))
    {
        DEBUG_INFO("TCP console disable\r\n");
    }

    if (strstr(configuration_data, "RELAY_FM(1)"))
    {
        app_flash_set_turn_on_relay_in_fm(true);
        index += sprintf(&p_res[index], "%s", "RL_FM(1),");
    }
    else if (strstr(configuration_data, "RELAY_FM(0)"))
    {
        app_flash_set_turn_on_relay_in_fm(false);
        index += sprintf(&p_res[index], "%s", "RL_FM(0),");
    }

    if (strstr(configuration_data, "TEST_AB(1)"))
    {
        DEBUG_INFO("Test class AB=1\r\n");
        index += sprintf(&p_res[index], "%s", "AB_TEST=1,");
        app_flash_set_test_ab(1);
    }
    else if (strstr(configuration_data, "TEST_AB(0)"))
    {
        index += sprintf(&p_res[index], "%s", "AB_TEST=0,");
        DEBUG_INFO("Test class AB=0\r\n");
        app_flash_set_test_ab(0);
    }

    char dtmf_str[10] = {0};
    char *p_dtmf = strstr(configuration_data, "DTMF1(");
    if (p_dtmf)
    {
        if (CopyParameter(p_dtmf, dtmf_str, '(', ')'))
        {
            uint8_t dtmf_delay = (uint8_t)GetNumberFromString(0, dtmf_str);
            DEBUG_INFO("DTMF_DELAY1=%d", dtmf_delay);
            app_flash_set_dtmf_t1(dtmf_delay);
            index += sprintf(&p_res[index], "DTMF1=%u,", dtmf_delay);
        }
    }

    
    p_dtmf = strstr(configuration_data, "DTMF2(");
    if (p_dtmf)
    {
        if (CopyParameter(p_dtmf, dtmf_str, '(', ')'))
        {
            uint8_t dtmf_delay = (uint8_t)GetNumberFromString(0, dtmf_str);
            DEBUG_INFO("DTMF_DELAY2=%d", dtmf_delay);
            app_flash_set_dtmf_t2(dtmf_delay);
            index += sprintf(&p_res[index], "DTMF2=%u,", dtmf_delay);
        }
    }

    p_dtmf = strstr(configuration_data, "DTMF_REPEAT(");
    if (p_dtmf)
    {
        if (CopyParameter(p_dtmf, dtmf_str, '(', ')'))
        {
            uint8_t dtmf_repeat = (uint8_t)GetNumberFromString(0, dtmf_str);
            DEBUG_INFO("DTMF_REPEAT=%d", dtmf_repeat);
            app_flash_set_dtmf_retires(dtmf_repeat);
            index += sprintf(&p_res[index], "DTMF_REPEAT=%u,", dtmf_repeat);
        }
    }

    p_dtmf = strstr(configuration_data, "DTMF_STARTUP(");
    if (p_dtmf)
    {
        if (CopyParameter(p_dtmf, dtmf_str, '(', ')'))
        {
            uint8_t dtmf_startup = (uint8_t)GetNumberFromString(0, dtmf_str);
            DEBUG_INFO("DTMF_STARTUP=%d", dtmf_startup);
            app_flash_set_dtmf_startup(dtmf_startup);
            index += sprintf(&p_res[index], "DTMF_STARTUP=%u,", dtmf_startup);
        }
    }

    char *protocol_priority = strstr(configuration_data, "PROTOCOL_PRIORITY(");
    if (protocol_priority != NULL)
    {
        update_slave_info++;
        uint8_t priority = GetNumberFromString(18, protocol_priority);
        app_flash_set_protocol_priority(priority);
        index += sprintf(&p_res[index], "PR_PRIORITY=%u,", priority);
        DEBUG_INFO("Set protocol priority: %u\r\n", priority);
    }

    /** Mặc định phản hồi các tần số cấu hình để trên web có danh sách tần số */
    index += sprintf(&p_res[index], "FM_FREQ1=%u,FM_FREQ2=%u,FM_FREQ3=%u,CLASS:(%s),",
                    app_flash_get_fm_freq1(), 
                    app_flash_get_fm_freq2(), 
                    app_flash_get_fm_freq3(), 
                    m_audio_class_des[app_flash_speaker_audio_class_get()]);

    /* Gửi phản hồi kết quả cấu hình */
    if (index > 0)
    {
        // Nếu là bản tin config url -> chỉ gửi vài lần mỗi khi kết nối
        if (stream_url == NULL || (stream_url != NULL && m_mqtt_send_resp_config_url_num <= 3))
        {
            DEBUG_INFO("SET reply: %s\r\n", p_res);
            mqtt_publish_message("CFG", p_res);
        }
    }

    /* Nếu có cấu hình url mới */
    if (has_new_url > 0)
    {
        // Reset cho mat
        esp_restart();
    }

    char *alternative_imei = strstr(configuration_data, "ALT_IMEI(");
    if (alternative_imei != NULL)
    {
        memset(tmp_str, 0, sizeof(tmp_str));
        if (CopyParameter(alternative_imei, tmp_str, '(', ')'))
        {
            // So sánh với giá hiện hiện tại, nếu khác thì apply
            if (strcmp(app_flash_get_alt_imei(), tmp_str) != 0)
            {
                ESP_LOGI(TAG, "Set ALT IMEI : %s", tmp_str);
                strcpy(app_flash_get_alt_imei(), tmp_str);

                ESP_LOGI(TAG, "New alternative imei: %s", app_flash_get_alt_imei());

                // Lưu bộ nhớ
                app_flash_node_nvs_write_string(APP_FLASH_KEY_ALTERNATIVE_IMEI, tmp_str);
                index += sprintf(&p_res[index], "ALT_IMEI=%s,", tmp_str);
            }
            else
            {
                ESP_LOGI(TAG, "Same alternative imei: %s", app_flash_get_alt_imei());
                index += sprintf(&p_res[index], "ALT_IMEI=%s,", app_flash_get_alt_imei());
            }
        }
    }
    
    /* Free mem */
    free(p_res);

    if (update_slave_info)
    {
        slave_report_info_now();
    }

    if (report_invalid_msg)
    {
        // char *reply = malloc(256);
        // if (reply)
        // {
        //     snprintf(reply, 256, "Invalid message: %s", configuration_data);
        //     mqtt_publish_message("DGB", reply);
        //     free(reply);
        // }
    }
}




/******************************************************************************************/
/**
 * @brief   : Xử lý lệnh GET cấu hình
 * @param   : data: GET,Lệnh 1,Lệnh 2,....,Lệnh N
 * @retval  :
 * @author  :
 * @created :
 */
static void process_get_config(char *configuration_data)
{
    char *p_res = malloc(512+256);
    uint16_t index = 0;

    if (!p_res)
    {
        DEBUG_WARN("Can't alocate mem p_res!\r\n");
        return;
    }

    bool is_get_all = false;
    if (strstr(configuration_data, "ALL"))
    {
        is_get_all = true;
    }

    if (strstr(configuration_data, "MASTER") || is_get_all)
    {
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_TINH1)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER1=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_TINH2)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER1_2=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_TINH3)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER1_3=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH3));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN1)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN2)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER2_2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN3)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER2_3=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN3));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA1)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA2)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER3_2=%s,", app_flash_get_master(APP_FLASH_MASTER_XA2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA3)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER3_3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA3));
        }
    }
    if (strstr(configuration_data, "FM_FREQ") || is_get_all)
    {
        index += sprintf(&p_res[index], "FM_FREQ=%u,", app_io_get_current_fm_freq());
    }
    if (strstr(configuration_data, "FM_FREQ1") || is_get_all)
    {
        index += sprintf(&p_res[index], "FM_FREQ1=%u,", app_flash_get_fm_freq1());
    }
    if (strstr(configuration_data, "FM_FREQ2") || is_get_all)
    {
        index += sprintf(&p_res[index], "FM_FREQ2=%u,", app_flash_get_fm_freq2());
    }
    if (strstr(configuration_data, "FM_FREQ3") || is_get_all)
    {
        index += sprintf(&p_res[index], "FM_FREQ3=%u,", app_flash_get_fm_freq3());
    }

    if (strstr(configuration_data, "VOLUME") || is_get_all)
    {
        index += sprintf(&p_res[index], "VOLUME=%u,VOLUME_CODEC=%u,", 
                        app_flash_get_volume(), 
                        app_audio_get_current_output_vol());
    }
    if (strstr(configuration_data, "MODE") || is_get_all)
    {
        index += sprintf(&p_res[index], "MODE=%u,", app_flash_get_operate_mode());
    }
    if (strstr(configuration_data, "IO1") || is_get_all)
    {
        index += sprintf(&p_res[index], "IO1=%s,", app_io_get_io_value()->Name.IO1 ? "ON" : "OFF");
    }
    if (strstr(configuration_data, "IO2") || is_get_all)
    {
        index += sprintf(&p_res[index], "IO2=%s,", app_io_get_io_value()->Name.IO2 ? "ON" : "OFF");
    }
    if (strstr(configuration_data, "DELAY_ON") || is_get_all)
    {
        index += sprintf(&p_res[index], "DELAY_ON=%u,", app_flash_get_delay_turn_on_relay());
    }
    if (strstr(configuration_data, "DELAY_OFF") || is_get_all)
    {
        index += sprintf(&p_res[index], "DELAY_OFF1=%u,DELAY_OFF2=%u,", 
                        app_flash_get_relay1_turn_off_delay(), 
                        app_flash_get_relay2_turn_off_delay());
    }
    if (strstr(configuration_data, "WIFI") || is_get_all)
    {
        index += sprintf(&p_res[index], "WIFI=%s:%s,", app_flash_get_wifi_name(), app_flash_get_wifi_pass());
    }

    // last config
    if (strstr(configuration_data, "STREAM") || is_get_all)
    {
        index += sprintf(&p_res[index], "STREAM_URL=%s,", app_flash_get_http_stream_header());
    }

    if (strstr(configuration_data, "mqttInfo") || is_get_all)
    {
        index += sprintf(&p_res[index], "MQTT_SERVER=%s,", app_flash_get_mqtt_server_url());
        index += sprintf(&p_res[index], "MQTT_USERNAME=%s,", app_flash_get_mqtt_server_username());
        index += sprintf(&p_res[index], "MQTT_PASSWORD=%s,", app_flash_get_mqtt_server_password());
    }

    if (strstr(configuration_data, "httpPost") || is_get_all)
    {
        index += sprintf(&p_res[index], "POST_URL(%s),", app_flash_get_http_post_url());
    }

    if (strstr(configuration_data, "protocolPriority") || is_get_all)
    {
        index += sprintf(&p_res[index], "PRO_PRI(%d),", app_flash_get_protocol_priority());
    }

    /* Gửi phản hồi kết quả lấy cấu hình */
    if (index > 0)
    {
        DEBUG_INFO("GET reply: %s\r\n", p_res);
        mqtt_publish_message("CFG", p_res);
    }

    /* Free mem */
    free(p_res);
}

void app_mqtt_send_all_config_after_reset(void)
{
    if (app_flash_get_protocol_priority() != APP_FLASH_PROTOCOL_V2_JSON)
    {
        char *p_res = malloc(512+256);
        uint16_t index = 0;

        if (!p_res)
        {
            DEBUG_ERROR("Can't alocate mem p_res!\r\n");
            system_software_reset(SW_RESET_REASON_MEM);
            esp_restart();
            return;
        }

        if (strlen(app_flash_get_master(APP_FLASH_MASTER_TINH1)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER1=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_TINH2)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER1_2=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_TINH3)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER1_3=%s,", app_flash_get_master(APP_FLASH_MASTER_TINH3));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN1)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN2)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER2_2=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_HUYEN3)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER2_3=%s,", app_flash_get_master(APP_FLASH_MASTER_HUYEN3));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA1)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA1));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA2)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER3_2=%s,", app_flash_get_master(APP_FLASH_MASTER_XA2));
        }
        if (strlen(app_flash_get_master(APP_FLASH_MASTER_XA3)) >= 3)
        {
            index += sprintf(&p_res[index], "MASTER3_3=%s,", app_flash_get_master(APP_FLASH_MASTER_XA3));
        }

        index += sprintf(&p_res[index], "FM_FREQ=%u,", app_io_get_current_fm_freq());
        index += sprintf(&p_res[index], "FM_FREQ1=%u,", app_flash_get_fm_freq1());
        index += sprintf(&p_res[index], "FM_FREQ2=%u,", app_flash_get_fm_freq2());
        index += sprintf(&p_res[index], "FM_FREQ3=%u,", app_flash_get_fm_freq3());
        index += sprintf(&p_res[index], "VOLUME=%u,VOLUME_CODEC=%u,", 
                                                        app_flash_get_volume(), 
                                                        app_audio_get_current_output_vol());
        index += sprintf(&p_res[index], "MODE=%u,", app_flash_get_operate_mode());
        index += sprintf(&p_res[index], "IO1=%s,", app_io_get_io_value()->Name.IO1 ? "ON" : "OFF");
        index += sprintf(&p_res[index], "IO2=%s,", app_io_get_io_value()->Name.IO2 ? "ON" : "OFF");
        index += sprintf(&p_res[index], "DL_ON=%u,", app_flash_get_delay_turn_on_relay());
        index += sprintf(&p_res[index], "DL_OFF1=%u,DL_OFF2=%u,", 
                        app_flash_get_relay1_turn_off_delay(), 
                        app_flash_get_relay2_turn_off_delay());
        // index += sprintf(&p_res[index], "WIFI=%s:%s,", app_flash_get_wifi_name(), app_flash_get_wifi_pass());
        // index += sprintf(&p_res[index], "STREAM_URL=%s,", app_flash_get_http_stream_header());
        // index += sprintf(&p_res[index], "MQTT_SERVER=%s,", app_flash_get_mqtt_server_url());
        // index += sprintf(&p_res[index], "MQTT_USERNAME=%s,", app_flash_get_mqtt_server_username());
        // index += sprintf(&p_res[index], "MQTT_PASSWORD=%s,", app_flash_get_mqtt_server_password());
        index += sprintf(&p_res[index], "DTMF_R=%d,", app_flash_get_dtmf_retires());
        index += sprintf(&p_res[index], "DTMF_T1=%d,", app_flash_get_dtmf_t1());
        index += sprintf(&p_res[index], "DTMF_T2=%d,", app_flash_get_dtmf_t2());
        index += sprintf(&p_res[index], "DTMF_STARTUP=%d,", app_flash_get_dtmf_startup());
        index += sprintf(&p_res[index], "PRO_PRI=%d", app_flash_get_protocol_priority());
        // char *username;
        // char *password;
        // app_flash_get_console_info(&username, &password);
        // index += sprintf(&p_res[index], "CONSOLE_USER=%s,", username);
        // index += sprintf(&p_res[index], "CONSOLE_PASS=%s", password);
        // if (app_flash_get_protocol_priority() == 0)     // mqtt
        mqtt_publish_message("CFG", p_res);
        // else 
        //     v1_post_to_server(p_res);
        
        /* Free mem */
        free(p_res);
        return;
    }

    
    char *body = malloc(1024+512);
    char *group = malloc(1024);
    uint16_t index = 0;

    if (!body || !group)
    {
        DEBUG_ERROR("Can't alocate mem\r\n");
        system_software_reset(SW_RESET_REASON_MEM);
        esp_restart();
        return;
    }

    char topic[32];
    char time_debug[32];

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    sprintf(time_debug, "%02d:%02d:%02d", tm.tm_hour, tm.tm_min, tm.tm_sec);

    index += sprintf(body+index, "{\"Id\":%d,\"Time\":\"%s\",\"Sender\":\"%s\",\"Message\":", 
                                    MSG_ID_EVENT, 
                                    time_debug,
                                    app_flash_get_imei());
    index += sprintf(body+index, "{\"Time\":\"%s\",\"C\":[", 
                                    time_debug);
    
    // Mode
    index += sprintf(body+index, "{\"C\":101,\"V\":\"%d\"},", 
                                    app_flash_get_operate_mode());
    // Group info
    int tmp = 0;
    int found_master = 0;

    if (group)
    {
        tmp += sprintf(group+tmp, "{\\\"Group\\\":%s", "[");
        // tmp += sprintf(group+tmp, "%s", "}");

        for (int i = 0; i < APP_FLASH_MAX_GROUP_SUPPORT; i++)
        {
            app_flash_group_info_t *gr = app_flash_get_group_info(i);
            if (gr && strlen(gr->group_id))
            {   
                found_master++;
                tmp += sprintf(group+tmp, "{\\\"ListDevices\\\":[],\\\"Id\\\":\\\"%s\\\",\\\"Priority\\\":%d},", 
                                        gr->group_id,
                                        gr->priority);
            }
        }
        
        if (found_master)
        {
            tmp--;
            group[tmp] = 0;
        }
        tmp += sprintf(group+tmp, "%s", "]");
    }
    else
    {
        group = "NA";
    }
    
    index += sprintf(body+index, "{\"C\":95,\"V\":\"%s\"}", group);
    index += sprintf(body+index, "%s", 
                                    "]}}");
    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(topic, "tx/%s/config", app_flash_get_imei());
    }
    else
    {
        sprintf(topic, "tx/%s/config", "NA");
    }

    int msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index, 0, 0);
    DEBUG_VERBOSE("slave publish config msg OK, msg_id=%d\r\n", msg_id);
}

bool app_mqtt_need_restart_mqtt(void)
{
    return do_reinit_mqtt;
}

void app_mqtt_need_clear_restart_mqtt_flag()
{
    do_reinit_mqtt = false;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    // esp_mqtt_client_handle_t client = event->client;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        app_mqtt_set_state(APP_MQTT_CONNECTED);
        ESP_LOGD(TAG, "MQTT_EVENT_CONNECTED");
        ESP_LOGD(TAG, "[ZIG] SERVER: CONNECTED\r\n");

        // Send subscribe 'config_topic'
        app_mqtt_send_subscribe_config_topic(app_flash_get_imei());
        slave_set_mqtt_state_timeout(56); /* Publish info sau khi connected 5s */
        app_mqtt_send_all_config_after_reset();
        do_reinit_mqtt = false;
        break;

    case MQTT_EVENT_DISCONNECTED:
        app_mqtt_set_state(APP_MQTT_DISCONNECTED);
        DEBUG_INFO("MQTT_EVENT_DISCONNECTED\r\n");
        ESP_LOGD(TAG, "[ZIG] SERVER: DISCONNECTED\r\n");
        m_mqtt_sub_req_tick = 0;
        m_is_mqtt_subscribed = false;
        m_mqtt_send_resp_config_url_num = 0;
        do_reinit_mqtt = true;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        // DEBUG_INFO("MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        if (event->msg_id == m_mqtt_sub_cfg_msg_id)
        {
            // DEBUG_INFO("MQTT subscribe CONFIG topic: OK");
            m_is_mqtt_subscribed = true;

            // Send subscribe 'command_topic'
            app_mqtt_send_subscribe_command_topic(APP_FLASH_MASTER_TINH1);
        }
        else
        {
            uint8_t pro = app_flash_get_protocol_priority();
            if (pro != APP_FLASH_PROTOCOL_V2_JSON)
            {
                for (uint8_t i = APP_FLASH_MASTER_TINH1; i < APP_FLASH_MASTER_TOTAL; i++)
                {
                    if (event->msg_id == m_mqtt_sub_master_msg_id[i])
                    {
                        // DEBUG_INFO("MQTT subscribe MASTER%d: OK", i);
                        m_mqtt_is_master_subscribed[i] = true;

                        // Send sub master tiếp theo
                        uint8_t next_master = i + 1;
                        if (next_master < APP_FLASH_MASTER_TOTAL && !m_mqtt_is_master_subscribed[next_master])
                        {
                            app_mqtt_send_subscribe_command_topic(next_master);
                        }
                        break;
                    }
                }
            }
            else
            {
                for (uint8_t i = 0; i < APP_FLASH_MAX_GROUP_SUPPORT; i++)
                {
                    if (event->msg_id == m_mqtt_sub_master_msg_id[i])
                    {
                        DEBUG_INFO("MQTT subscribe MASTER%d: OK", i);
                        m_mqtt_is_master_subscribed[i] = true;

                        // Send sub master tiếp theo
                        uint8_t next_master = i + 1;
                        if (next_master >= APP_FLASH_MAX_GROUP_SUPPORT)
                        {
                            break;
                        }
                        if (next_master < APP_FLASH_MAX_GROUP_SUPPORT 
                            && !m_mqtt_is_master_subscribed[next_master])
                        {
                            app_mqtt_send_subscribe_command_topic(next_master);
                        }
                        break;
                    }
                }
            }
        }
        m_mqtt_sub_req_tick = 0;
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        DEBUG_INFO("UNSUBSCRIBED, msg_id=%d\r\n", event->msg_id);
        m_is_mqtt_subscribed = false;
        m_mqtt_send_resp_config_url_num = 0;
        for (uint8_t i = 0; i < APP_FLASH_MASTER_TOTAL; i++)
        {
            m_mqtt_is_master_subscribed[i] = false;
        }
        m_mqtt_sub_req_tick++;
        if (m_mqtt_sub_req_tick > 10)
        {
            m_mqtt_sub_req_tick = 0;
            esp_mqtt_client_stop(m_mqtt_client);
        }
        else
        {
            // Send subscribe 'config_topic'
            app_mqtt_send_subscribe_config_topic(app_flash_get_imei());
        }
        break;

    case MQTT_EVENT_PUBLISHED:
        // DEBUG_INFO("MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGD(TAG, "MQTT_EVENT_DATA");
        // get topic name
        char topic_name[100] = {0};
        uint8_t topic_len = event->topic_len;
        if (topic_len > 99)
            topic_len = 99;
        memcpy(topic_name, event->topic, topic_len);

        // clear last byte of received data
        event->data[event->data_len] = 0;

        DEBUG_VERBOSE("TOPIC=%s, leng=%u\r\n", /*event->topic*/ topic_name, event->topic_len);
        DEBUG_VERBOSE("DATA=%s, leng=%u-%u\r\n", event->data, event->data_len, strlen(event->data));

        /* ==================================== From slave 'config_topic' ======================================= */
        if (strlen(app_flash_get_imei()) >= 15 && strstr(topic_name, app_flash_get_imei()))
        {
            ESP_LOGD(TAG, "From 'config_topic'\r\n");
            // event->data[event->data_len] = 0;

            /* ================ Lệnh RESET thiết bị ===================*/
            if (strstr(event->data, "REBOOT#"))
            {
                DEBUG_INFO("\t--- Reset System ---\r\n");
                mqtt_publish_message("DBG", "Reset by reboot cmd");
                app_audio_change_codec_vol(0);
                vTaskDelay(1000 / portTICK_RATE_MS);
                app_audio_simple_terminate_pipeline();
                vTaskDelay(1000 / portTICK_RATE_MS);

                system_software_reset(SW_RESET_REASON_REBOOT_CMD);
            }

            // Ignore server msg
            if (strstr(event->data, "STREAM_RUNNING_SCHEDULE") || strstr(event->data, ",RUNNING,SRV,") )
            {
                return ESP_OK;
            }

            /* ================ Lệnh Cấu hình WiFi ===================*/
            if (strstr(event->data, "WIFINAME("))
            {
                DEBUG_INFO("\t--- Set WiFi info ---");

                // Write config to NVS
                app_mqtt_write_wifi_config(event->data);
            }

            /* ================ Lệnh OTA Firmware ===================*/
            char *ota_for_esp32 = strstr(event->data, "UDFW(");
            char *ota_for_gd32 = strstr(event->data, "UDFW_GD32(");
            char *ota_for_fm = strstr(event->data, "UDFW_FM(");
            char *force_ota = strstr(event->data, "FORCE_UDFW(");
            if (ota_for_esp32 || ota_for_gd32 || ota_for_fm)
            { /* UDFW(http link) */

                DEBUG_INFO("\t--- Update firmware ---\r\n");

                /* Không nhận lệnh liên tiếp, hoặc khi đang chạy task OTA rồi */
                if (app_ota_is_running())
                {
                    DEBUG_INFO("WAR: OTA is running...\r\n");
                    return ESP_OK;
                }

                /* Không OTA khi đang streaming */
                if (app_audio_is_http_audio_stream_running() && !force_ota)
                {
                    // mqtt_publish_message("DBG", "It's STREAMING. Don't OTA! Try later\r\n");
                    return ESP_OK;
                }

                if (CopyParameter(event->data, m_ota_http_url, '(', ')'))
                {
                    DEBUG_INFO("OTA link: %s", m_ota_http_url);
                    static app_ota_info_t ota_url;
                    ota_url.url = m_ota_http_url;
                    ota_url.type = APP_OTA_DEVICE_INVALID;

                    if (!ota_for_esp32)
                    {
                        DEBUG_WARN("OTA for GD32\r\n");
                        if (ota_for_gd32 && 
                            app_io_get_gd32_protocol_method() == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
                        {
                            ota_url.type = APP_OTA_DEVICE_GD32;
                        }

                        if (ota_for_fm 
                            && app_io_get_gd32_protocol_method() == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL
                            && app_io_get_fm_protocol_method() == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
                        {
                            ota_url.type = APP_OTA_DEVICE_FM;
                        }
                    }
                    else
                    {
                        DEBUG_WARN("OTA for esp32\r\n");
                        ota_url.type = APP_OTA_DEVICE_ESP32;
                    }

                    if (ota_url.type != APP_OTA_DEVICE_INVALID)
                    {
                        // Create ota task
                        xTaskCreate(app_ota_download_task, "ota_tsk", 8192, (void *)&ota_url, 5, NULL);
                    }
                    else
                    {
                        mqtt_publish_message("DBG", "Invalid OTA device type\r\n");
                    }
                }
                break;
            }

            /* ================ Lệnh SET cấu hình ===================*/
            if (strstr(event->data, "SET,") && event->retain == false)
            {
                DEBUG_INFO("\t--- Set Parameter ---\r\n");
                mqtt_process_set_config_msg(event->data, topic_name);
                break;
            }

            if (strstr(event->data, "FACTORY,") && event->retain == false)
            {
                DEBUG_INFO("\t--- Factory reset ---\r\n");
                mqtt_publish_message("DBG", "Factory reset");
                app_flash_do_factory_reset();
                vTaskDelay(1000);
                esp_restart();
                break;
            }


            /* ================ Lệnh GET cấu hình ===================*/
            if (strstr(event->data, "GET,"))
            {
                DEBUG_INFO("\t--- Get Parameter ---\r\n");
                process_get_config(event->data);
                break;
            }

            if (strstr(event->data, "DBG,"))
            {
                break;
            }
        }

        /* Nếu đang OTA không nhận lệnh stream */
        if (app_ota_is_running())
            return ESP_OK;

        char *p_run = strstr(event->data, "STREAM_");
        if (p_run || (strstr(event->data, "{") && (strstr(event->topic, "rx/") || strstr(event->topic, "tx/"))))
        {
            mqtt_queue_stream_event_t tx_event;
            tx_event.need_free = true;
            tx_event.payload = calloc(event->data_len+1, 1);
            tx_event.topic = calloc(strlen(topic_name)+1, 1);
            tx_event.need_free = true;
            if (tx_event.payload == NULL || tx_event.topic == NULL)
            {
                esp_restart();
            }
            sprintf(tx_event.topic, "%s", topic_name);
            sprintf(tx_event.payload, "%s", event->data);
            mqtt_queue_streaming_put(&tx_event);
        }
        break;

    case MQTT_EVENT_ERROR:
        DEBUG_WARN("MQTT_EVENT_ERROR\r\n");
        break;

    default:
        DEBUG_VERBOSE("MQTT other event id: %d\r\n", event->event_id);
        break;
    }
    return ESP_OK;
}

void app_mqtt_send_subscribe_config_topic(char *imei)
{
    /* Nếu đã có GSM_IMEI -> Sub luôn topic Slave config */
    if (strlen(imei) >= 15)
    {
        memset(m_slave_sub_topic, 0, sizeof(m_slave_sub_topic));
        if (app_flash_get_protocol_priority() != APP_FLASH_PROTOCOL_V2_JSON)
        {
            sprintf(m_slave_sub_topic, "%s%s", SLAVE_SUB_TOPIC_CONF_HEADER, imei);
        }
        else
        {
            sprintf(m_slave_sub_topic, "rx/%s", imei);
        }
        m_mqtt_sub_cfg_msg_id = esp_mqtt_client_subscribe(m_mqtt_client, m_slave_sub_topic, 1);

        DEBUG_VERBOSE("sent subscribe %s OK, msg_id=%d\r\n", m_slave_sub_topic, m_mqtt_sub_cfg_msg_id);
        m_mqtt_sub_req_tick = 1;
    }
    else
    {
        m_mqtt_sub_cfg_msg_id = 0;
        DEBUG_WARN("GSM_IMEI is empty!\r\n");
    }
}

uint8_t app_mqtt_send_subscribe_command_topic(uint8_t master_id)
{
    if (app_flash_get_protocol_priority() != APP_FLASH_PROTOCOL_V2_JSON)
    {
        if (master_id < APP_FLASH_MASTER_TINH1 || master_id > APP_FLASH_MASTER_XA3)
            return 0;

        /* Đã có cấu hình MASTER -> Sub topic 'master_command' */
        if (strlen(app_flash_get_master(master_id)) >= 15)
        {
            memset(m_slave_sub_topic, 0, sizeof(m_slave_sub_topic));
            sprintf(m_slave_sub_topic, "%s%s", SLAVE_SUB_TOPIC_CMD_HEADER, app_flash_get_master(master_id));
            m_mqtt_sub_master_msg_id[master_id] = esp_mqtt_client_subscribe(m_mqtt_client, m_slave_sub_topic, 1);

            DEBUG_VERBOSE("sent subscribe master%d - %s' OK, msg_id=%d\r\n", master_id, m_slave_sub_topic, m_mqtt_sub_master_msg_id[master_id]);
            m_mqtt_sub_req_tick = 1;
            return 1;
        }
    }
    else
    {
        app_flash_group_info_t *info = app_flash_get_group_info(master_id);
        /* Đã có cấu hình MASTER -> Sub topic 'master_command' */
        if (info && strlen(info->group_id) >= 3)
        {
            memset(m_slave_sub_topic, 0, sizeof(m_slave_sub_topic));
            sprintf(m_slave_sub_topic, "tx/%s", info->group_id);
            m_mqtt_sub_master_msg_id[master_id] = esp_mqtt_client_subscribe(m_mqtt_client, m_slave_sub_topic, 1);
            m_mqtt_sub_req_tick = 1;
            DEBUG_INFO("Subscribe [%d] : %s\r\n", master_id, m_slave_sub_topic);
            return 1;
        }
    }
    return 0;
}

esp_err_t http_post_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id) 
    {
        case HTTP_EVENT_ERROR:
            DEBUG_WARN("HTTP_EVENT_ERROR");
            break;

        case HTTP_EVENT_ON_CONNECTED:
            DEBUG_VERBOSE("HTTP_EVENT_ON_CONNECTED");
            break;

        case HTTP_EVENT_HEADER_SENT:
            DEBUG_VERBOSE("HTTP_EVENT_HEADER_SENT");
            break;

        case HTTP_EVENT_ON_HEADER:
            DEBUG_VERBOSE("HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;

        case HTTP_EVENT_ON_DATA:
            DEBUG_INFO("HTTP_EVENT_ON_DATA, len=%d, data %.*s", evt->data_len,  evt->data_len, (char*)evt->data);
            break;

        case HTTP_EVENT_ON_FINISH:
            DEBUG_INFO("HTTP_EVENT_ON_FINISH\r\n");
            break;

        case HTTP_EVENT_DISCONNECTED:
        {
            DEBUG_VERBOSE("HTTP_EVENT_DISCONNECTED\r\n");
        }
            break;
        default:
            break;
    }
    return ESP_OK;
}

uint32_t app_sntp_get_timestamp()
{
    return (uint32_t)(time(NULL)) + 25200;     // GMT+7
}

/*
{"Message":{"Ver":"1.0.5_lp","C":10,"E":4539,"F":{"D":0,"F":0,"R":0,"S":0},
            "G":0,"Hw":10,"Ip":"rm[10.150.18.75]","Cam":false,"Join":false,"Mic":false,"Spk":true,
            "L":{"V":0.0,"K":0.0},"M":{"F":1,"H":3,"I":0,"O":0,"2":4904,"3":3781,"1":23761},
            "Mp":0,"N":"Viettel,2G,B1900,18",
            "Id":"863462063248569","Name":"Alice","O":1,"P":0,"R":210,"Type":0,
            "Room":"","Scid":"","Scst":0,"Scf":"","Sn":"863462063248569",
            "S":{"D":0,"E":0,"V":0,"C":1},"Stst":0,"Stmt":"","A":39,"T":45,"U":0,"Vl1":60,"Vl2":60},
            "Id":10,"Time":0,"Sender":"863462063248569"}
*/
static int v1_post_to_server(char *ping)
{
    int status_code = -1;
    char *body = malloc(512+256);
    int index = 0;
    assert(body);
    index += sprintf(body+index, "{\"imei\":%s,\"payload\":\"%s\"}", app_flash_get_imei(), ping);
    // //                 "\"Stlk\":\"%s\",\"Stst\":%d,\"Mic\":%d,\"Spk\":%d,\"Vl2\":%d,"
    // //                 "\"A\":%d,\"N\":\"%s\",\"P\":%u,\"O\":%d,\"L\":\"%s,%s\",",
    // //                 ping->serial, ping->role, ping->ip, ping->streaming_master, 
    // //                 ping->streaming_link, ping->stream_state, ping->mic_on, ping->spk_on, ping->spk_on, ping->vol_music,
    // //                 ping->temp_air, ping->network, ping->play_time_of_day, ping->operating_mode, ping->gps_lat, ping->gps_long);
    // index += sprintf(body+index, "{\"Id\":10,\"Time\":%u,\"Sender\":\"%s\",\"Message\":",
    //                 app_sntp_get_timestamp(),
    //                 ping->serial);

    // index += sprintf(body+index, "{\"Id\":\"%s\",\"Sn\":\"%s\",\"Type\":%d,\"Ip\":\"%s\",\"Stmt\":\"%s\","
    //                 "\"Stlk\":\"%s\",\"Stst\":%d,\"Mic\":%d,\"Spk\":%d,\"Vl1\":%d,\"Vl2\":%d,"
    //                  "\"A\":%d,\"N\":\"%s\",\"P\":%u,\"O\":%d,\"L\":{\"V\":%s,\"K\":%s},\"Ver\":\"%s\",",
    //                 ping->serial, ping->serial, ping->role, ping->ip, ping->streaming_master, 
    //                 ping->streaming_link, ping->stream_state, ping->mic_on, ping->spk_on, ping->vol_music,  ping->vol_music,
    //                 ping->temp_air, ping->network, ping->play_time_of_day, ping->operating_mode, ping->gps_lat, ping->gps_long,
    //                 ping->app_version);

    // index += sprintf(body+index, "\"F\":{\"F\":%u,\"S\":%d,\"R\":%d,\"D\":%d},",
    //                                 ping->fm_info.freq, ping->fm_info.snr, 
    //                                 ping->fm_info.rssi, ping->fm_info.dBm);

    // index += sprintf(body+index, "\"M\":{\"F\":%d,\"H\":%d,\"1\":%u,\"2\":%u,\"3\":%u,\"O\":%u,\"I\":%u},",
    //                 ping->mcu_info.firmware_version, ping->mcu_info.hardware_version, 
    //                 ping->mcu_info.vin, ping->mcu_info.v5v, ping->mcu_info.vgsm, 
    //                 ping->mcu_info.output_control, ping->mcu_info.input_state);

    // index += sprintf(body+index, "\"S\":{\"C\":%d,\"D\":%d,\"E\":%u},\"Name\":\"%s\",",
    //         ping->speaker.audio_class, ping->speaker.detect_value, 
    //         ping->speaker.err_state,
    //         ping->device_name);

    // index += sprintf(body+index, "\"Mp\":%u}}", ping->mic_plug_state);


     esp_http_client_config_t config = {
        .host = app_flash_get_http_post_url(),
        .event_handler = http_post_event_handler,
        .path = "/POST",
        .query = "slave",
        .user_data = NULL,        // Pass address of local buffer to get response
        .disable_auto_redirect = false,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_url(client, app_flash_get_http_post_url());
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "API-KEY", app_flash_get_api_key());
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t err = esp_http_client_perform(client);
    status_code = esp_http_client_get_status_code(client);
    if (err == ESP_OK && status_code == 200) 
    {
        m_last_http_code = status_code;
        DEBUG_INFO("HTTP status code = %d\r\n", status_code);
    } 
    else 
    {
        if (m_last_http_code != status_code)
        {
            m_last_http_code = status_code;
            mqtt_publish_message("DBG", "HTTP POST request failed: %s, code %d", 
                                        esp_err_to_name(err), status_code);
        }
    }

    esp_http_client_cleanup(client);


    free(body);
    return status_code;
}

uint8_t translate_spk_err_code(uint8_t value)
{
    // Convert lai gia tri day len server de server ko can sua code trang thai loa
    if (value == 8)    // short
    {
        value = 13;
    }
    else if (value == 0)    // normal
    {
        value = 12;
    }
    else if (value == 6 || value == 15)    // open
    {
        value = 14;
    }

    return value;
}

char *build_mqtt_body(void)
{
    char *body = calloc(512, 1);
    assert(body);
    modem_dce_t *dce = slave_get_modem_dce();
    /* Nội dung info: INF,<IMEI>,<FW version>,<Stream State>,<Network Interface>,<Stream time in second>,<Stream data usage in byte>,
     *		<Running Mode>,<FM Freq>,<Volume>,<MIC state>,<IO1 state>,<IO2 state>,<FM SNR>,<RSSI>,<RSSI in dBm>,<GPS lat>,<GPS lng>,
     *		<GSM CSQ>,<GSM network name>,<Tech>,<Band>,<Speaker state>,<MCU FwVersion>,PA level
     */
    const char *state_str = app_audio_get_audio_state_description();
    if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
    {
        state_str = "STOP";
    }

    int bodylen = sprintf(body, "INF,%s,%s-%s,%s,%s,%u,%u,", (strlen(app_flash_get_imei()) >= 15) ? app_flash_get_imei() : "NA",
                          __FIRMWARE_VERSION__, app_flash_get_protocol_priority_des(), 
                          state_str,
                          NET_IF_TAB[network_get_current_interface()], 
                          slave_get_stream_time(), 
                          slave_get_total_streaming_received());

    // Chế độ đang chạy
    if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET)
    {
        char *master_str = "000";
        // uint8_t isFoundCurMaster = 0;
        for (uint8_t i = APP_FLASH_MASTER_TINH1; i < APP_FLASH_MASTER_TOTAL; i++)
        {
            if (strlen(master_str) < 5 && strlen(app_flash_get_master(i)) > 5)
            {
                master_str = app_flash_get_master(i);
            }
            if (app_flash_get_current_streaming_master_index() == i 
                && strlen(app_flash_get_master(i)) > 5)
            {
                // bodylen += sprintf(&body[bodylen], "%s,", app_flash_get_master(i));
                // isFoundCurMaster = 1;
                master_str = app_flash_get_master(i);
                break;
            }
        }
        // // Nếu chưa có curMaster -> hiển thị "unknow"
        // if (!isFoundCurMaster)
        // {
        //     bodylen += sprintf(&body[bodylen], "%s,", "000");
        // }
        bodylen += sprintf(&body[bodylen], "%s,", master_str);
    }
    else if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_FM)
    {
        bodylen += sprintf(&body[bodylen], "%s,", "FM");
    }
    else if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_MIC)
    {
        bodylen += sprintf(&body[bodylen], "%s,", "MIC");
    }
    else if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
    {
        bodylen += sprintf(&body[bodylen], "%s,", "NONE");
    }

    // FM Freq - Tần số FM mà module FM đang thu
    if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_FM 
        || app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_FM)
    {
        bodylen += sprintf(&body[bodylen], "%.1f,", (float)app_io_get_freq_at_slave() / 1000);
    }
    else
    {
        bodylen += sprintf(&body[bodylen], "%u,", 0);
    }

    if (app_io_get_hardware_version() == 2)
    {
        // Volume hiện tại, MIC state, IO1, IO2
        bodylen += sprintf(&body[bodylen], "%u,%s,%s,%s,",
                           app_audio_get_current_output_vol(),
                           app_io_is_mic_plugged() ? "MIC" : "NA",
                           (app_io_get_i2c_exp_value()->BitName.ISO_OUT1 == APP_IO_OPTO_ON) ? "ON" : "OFF",
                           (app_io_get_i2c_exp_value()->BitName.ISO_OUT2 == APP_IO_OPTO_ON) ? "ON" : "OFF");
    }
    else if (app_io_get_hardware_version() == 3)
    {
        // Volume hiện tại, MIC state, IO1, IO2
        bodylen += sprintf(&body[bodylen], "%u,%s,%s,%s,",
                           app_audio_get_current_output_vol(),
                           app_io_is_mic_plugged() ? "MIC" : "NA",
                           (app_io_get_mcu_exp_value()->BitName.ISO_OUT1 == APP_IO_STM_OPTO_ON) ? "ON" : "OFF",
                           (app_io_get_mcu_exp_value()->BitName.ISO_OUT2 == APP_IO_STM_OPTO_ON) ? "ON" : "OFF");
    }

    char *gps_info = app_io_get_last_gps_info();
    if (strlen(gps_info) < 3)
    {
        gps_info = "0.000,0.000";
    }  
    uint32_t snr, rssi, dbm;
    app_io_get_fm_snr_info(&snr, &rssi, &dbm);
    // FM SNR, RSSI, RSSI in dBm, GPS lat, GPS lng
    bodylen += sprintf(&body[bodylen], "%d,%d,%d,%s,",
                       snr, rssi, dbm, gps_info);

    // GSM CSQ
    bodylen += sprintf(&body[bodylen], "CSQ:%d,", slave_get_gsm_csq());

    // OperatorNetwork, Technology access, band, channel
    if (dce)
    {
        bodylen += sprintf(&body[bodylen], "%s,%s,%s,%s,",
                           dce->oper, dce->access_tech, dce->band, dce->channel);
    }
    else
    {
        bodylen += sprintf(&body[bodylen], "%s,%s,%s,%s,", "NA", "NA", "NA", "NA");
    }
#if 1
    // Speaker state
    uint8_t class = app_flash_speaker_audio_class_get();
    if (class != APP_FLASH_AUDIO_CLASS_NONE)
    {
        uint8_t value = app_io_get_speaker_detect_value().value;
        if (class == APP_FLASH_AUDIO_CLASS_AB)
        {
            bodylen += sprintf(&body[bodylen], "%d,", value);
        }
        else
        {
            // Convert lai gia tri day len server de server ko can sua code trang thai loa
            value = translate_spk_err_code(value);

            bodylen += sprintf(&body[bodylen], "%d,", value);
        }
    }
    else
    {
        bodylen += sprintf(&body[bodylen], "%d,", 7);       // 7 hoac 15 the hien trang thai khong xac dinh
    }
#else
    bodylen += sprintf(&body[bodylen], "%d,", 7);       // 7 hoac 15 the hien trang thai khong xac dinh
#endif
    // MCU firmware version
    bodylen += sprintf(&body[bodylen], "%d,", app_io_get_worker_version());
    bodylen += sprintf(&body[bodylen], "%d,", app_io_get_pa_state());

    // Vin and vgsm
    bodylen += sprintf(&body[bodylen], "%d,", app_io_get_current_vin());
    if (dce)
    {
        bodylen += sprintf(&body[bodylen], "%d,", dce->gsm_vbat);
    }
    else
    {
        bodylen += sprintf(&body[bodylen], "%d,", 0);
    }
    // Temp
    bodylen += sprintf(&body[bodylen], "%d,", app_io_get_temperature());
    bodylen += sprintf(&body[bodylen], "%d", app_io_is_power_lost());
    return body;
}

int pub_mcu_info(char *body)
{
    if (m_mqtt_client == NULL || m_mqtt_fsm != APP_MQTT_CONNECTED || body == NULL)
        return -1;

    modem_dce_t *dce = slave_get_modem_dce();
    char *pub_topic = malloc(64);
   
    if (!pub_topic)
    {
        if (pub_topic)
            free(pub_topic);
        DEBUG_ERROR("Can't allocate mem!\r\n");
        return -1;
    }
    slave_set_mqtt_state_timeout(0);

    if (strlen(app_flash_get_imei()) >= 15)
    {
        sprintf(pub_topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, app_flash_get_imei());
    }
    else
    {
        sprintf(pub_topic, "%s%s", SLAVE_PUB_TOPIC_HEADER, "NA");
    }

    
    // // Publish mqtt
    // int msg_id = esp_mqtt_client_publish(m_mqtt_client, pub_topic, body, bodylen, 1, 0);
    // DEBUG_VERBOSE("slave publish 'info' OK, msg_id=%d, topic %s\r\n", msg_id, pub_topic);
    esp_mqtt_client_publish(m_mqtt_client, pub_topic, body, strlen(body), 1, 0);
    free(pub_topic);
    return 0;
}

int mqtt_publish_ttn_info(ping_message_t *ping)
{
    int len = 0;
    int msg_id = -1;
    char *body = malloc(512+256);
    int index = 0;
    assert(body);
    char topic[128];
    // index += sprintf(body+index, "{\"Sn\":%s,\"Type\":%d,\"Ip\":\"%s\",\"Stmt\":\"%s\","
    //                 "\"Stlk\":\"%s\",\"Stst\":%d,\"Mic\":%d,\"Spk\":%d,\"Vl2\":%d,"
    //                 "\"A\":%d,\"N\":\"%s\",\"P\":%u,\"O\":%d,\"L\":\"%s,%s\",",
    //                 ping->serial, ping->role, ping->ip, ping->streaming_master, 
    //                 ping->streaming_link, ping->stream_state, ping->mic_on, ping->spk_on, ping->spk_on, ping->vol_music,
    //                 ping->temp_air, ping->network, ping->play_time_of_day, ping->operating_mode, ping->gps_lat, ping->gps_long);

    index += sprintf(body+index, "{\"Id\":10,\"Time\":%u,\"Sender\":\"%s\",\"Message\":",
                    app_sntp_get_timestamp(),
                    ping->serial);

    index += sprintf(body+index, "{\"Type\":%d,\"Ip\":\"%s\",\"Stmt\":\"%s\","
                    "\"Stlk\":\"%s\",\"Stst\":%d,\"Mic\":%d,\"Spk\":%d,\"Vl1\":%d,\"Vl2\":%d,"
                     "\"A\":%d,\"T\":%d,\"N\":\"%s\",\"P\":%u,\"O\":%d,\"L\":{\"V\":%s,\"K\":%s},\"Ver\":\"%s\",",
                    ping->role, ping->ip, ping->streaming_master, 
                    ping->streaming_link, ping->stream_state, ping->mic_on, ping->spk_on, ping->vol_music,  ping->vol_music,
                    ping->temp_air, ping->temp_air, ping->network, ping->play_time_of_day, ping->operating_mode, ping->gps_lat, ping->gps_long,
                    ping->app_version);

    uint8_t spk_value = app_io_get_speaker_detect_value().value;
    if (app_flash_speaker_audio_class_get() == APP_FLASH_AUDIO_CLASS_AB)
    {
        // Convert lai gia tri day len server de server ko can sua code trang thai loa
        if (spk_value == 13)    // short
        {
            ping->sys_err.name.spk_open = 0;
            ping->sys_err.name.spk_short = 1;
        }
        else if (spk_value == 12)    // normal
        {
            ping->sys_err.name.spk_open = 0;
            ping->sys_err.name.spk_short = 0;
        }
        else if (spk_value == 14)    // open
        {
            ping->sys_err.name.spk_open = 1;
            ping->sys_err.name.spk_short = 0;
        }
        else
        {
            ping->sys_err.name.spk_open = 0;
            ping->sys_err.name.spk_short = 0;
        }
    }
    else
    {
        ping->sys_err.name.spk_open = 0;
        ping->sys_err.name.spk_short = 0;
    }
    index += sprintf(body+index, "\"F\":{\"E\":%u,\"F\":%u,\"S\":%d,\"R\":%d,\"D\":%d},",
                                    ping->sys_err.err,
                                    ping->fm_info.freq, ping->fm_info.snr, 
                                    ping->fm_info.rssi, ping->fm_info.dBm);

    index += sprintf(body+index, "\"M\":{\"F\":%d,\"H\":%d,\"1\":%u,\"2\":%u,\"3\":%u,\"O\":%u,\"I\":%u},",
                    ping->mcu_info.firmware_version, ping->mcu_info.hardware_version, 
                    ping->mcu_info.vin, ping->mcu_info.v5v, ping->mcu_info.vgsm, 
                    ping->mcu_info.output_control, ping->mcu_info.input_state);


    index += sprintf(body+index, "\"S\":{\"C\":%d,\"D\":%d,\"E\":%u,\"PA\":%d},\"Name\":\"%s\",",
            ping->speaker.audio_class, ping->speaker.detect_value, 
            ping->speaker.err_state,
            app_io_is_pa_off() ? 0 : 1,
            ping->device_name);

    index += sprintf(body+index, "\"Mp\":%u}}", ping->mic_plug_state);

    int found_gr = 0;
    for (int i = 0; i < APP_FLASH_MAX_GROUP_SUPPORT; i++)
    {
        app_flash_group_info_t *gr = app_flash_get_group_info(i);
        if (gr && strlen(gr->group_id))
        {   
            found_gr++;
            sprintf(topic, "tx/%s/%s", gr->group_id, app_flash_get_imei());
            msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index+len, 1, 0);
        }
    }
    if (found_gr == 0)
    {
        sprintf(topic, "tx/gid/%s", app_flash_get_imei());
        msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index+len, 1, 0);
    }
    
    DEBUG_VERBOSE("slave publish dbg OK, msg_id=%d\r\n", msg_id);
    free(body);
    return msg_id;
}

int mqtt_publish_ttn_debug(const char *format, ...)
{
    // int len = 0;
    // char *body = malloc(512);
    // char topic[32];
    // if (!body)
    // {
    //     DEBUG_WARN("Can't allocate mem!\r\n");
    //     return -1;
    // }
    // int index = 0;
    // index = sprintf(body, "{\"time\"", header);
    // va_list arg_ptr;

    // char *imei = app_flash_get_imei();

    // if (strlen(imei) < 15)
    // {
    //     imei = "NA";
    // }

    // sprintf(topic, "tx/%s/info", imei);

    // va_start(arg_ptr, format);
    // len = vsnprintf(body+index, 512-index, format, arg_ptr);
    // va_end(arg_ptr);
    // int msg_id = esp_mqtt_client_publish(m_mqtt_client, topic, body, index+len, 1, 0);
    // DEBUG_VERBOSE("slave publish dbg OK, msg_id=%d\r\n", msg_id);
    // free(body);
    // return msg_id;
    return 0;
}

// static bool is_higher_priority_master(char *new_master, char *current_master)
// {
//     if (app_flash_find_group_priority(new_master) > app_flash_find_group_priority(current_master))
//         return true;
//     return false;
// }

void app_mqtt_publish_slave_info(void)
{
    modem_dce_t *dce = slave_get_modem_dce();
    strcpy(m_ping_msg.device_name, app_flash_get_device_name());
    m_ping_msg.mic_on = app_io_is_mic_plugged() ? 1 : 0;
    m_ping_msg.mic_plug_state = m_ping_msg.mic_on;
    m_ping_msg.speaker.audio_class = app_flash_speaker_audio_class_get();
    m_ping_msg.speaker.detect_value = app_io_get_speaker_detect_value().value;
    m_ping_msg.speaker.err_state = app_io_is_speaker_error();
    sprintf(m_ping_msg.gps_lat, "%.6f", app_io_get_last_fm_msg()->gps_lat);
    sprintf(m_ping_msg.gps_long, "%.6f", app_io_get_last_fm_msg()->gps_long);
    m_ping_msg.operating_mode = app_flash_get_operate_mode();
    m_ping_msg.play_time_of_day = slave_get_stream_time_in_day();
    m_ping_msg.fm_info.dBm = app_io_get_last_fm_msg()->dbm;
    m_ping_msg.fm_info.snr = app_io_get_last_fm_msg()->rssi;
    m_ping_msg.fm_info.snr = app_io_get_last_fm_msg()->snr;
    m_ping_msg.fm_info.freq = app_io_get_current_fm_freq();

    if (dce)
    {
        sprintf(m_ping_msg.network, "%s,%s,%s,%s,%s,%d",
                NET_IF_TAB[network_get_current_interface()],
                dce->oper, dce->access_tech, dce->band, dce->channel, dce->rssi);
    }
    else
    {
        sprintf(m_ping_msg.network, "%s", NET_IF_TAB[network_get_current_interface()]);
    }
    
    m_ping_msg.temp_air = app_io_get_temperature();
    sprintf(m_ping_msg.app_version, "%s-%s", __FIRMWARE_VERSION__, app_flash_get_protocol_priority_des());
    m_ping_msg.vol_music = app_flash_get_volume();
    m_ping_msg.spk_on = app_io_is_pa_off() ? 0 : 1;
    m_ping_msg.stream_state = app_audio_get_http_state() == AEL_STATE_RUNNING ? AEL_STATE_RUNNING : 0;

    if (strstr(app_audio_get_stream_url(), APP_FLASH_INVALID_LINK))
    {
        sprintf(m_ping_msg.streaming_link, "%s", "");
    }
    else
    {
        sprintf(m_ping_msg.streaming_link, "%s", app_audio_get_stream_url());
    }
    
    int tmp = 0;
    m_ping_msg.ip[0] = 0;
    if (strlen(network_get_local_ip(NETWORK_LOCAL_IP_WIFI)))
    {
        tmp += sprintf(&m_ping_msg.ip[tmp], "w[%s]", network_get_local_ip(NETWORK_LOCAL_IP_WIFI));
    }
    if (strlen(network_get_local_ip(NETWORK_LOCAL_IP_ETH)))
    {
        tmp += sprintf(&m_ping_msg.ip[tmp], "e[%s]", network_get_local_ip(NETWORK_LOCAL_IP_ETH));
    }

    m_ping_msg.role = 0;        // 0 = slave, 1 = master
    m_ping_msg.serial = app_flash_get_imei();

    m_ping_msg.mcu_info.firmware_version = app_io_get_last_gd32_msg()->fw_version;
    m_ping_msg.mcu_info.hardware_version = app_io_get_last_gd32_msg()->hw_version;
    m_ping_msg.mcu_info.vin = app_io_get_last_gd32_msg()->vin;

    if (dce)
    {
        m_ping_msg.mcu_info.vgsm = dce->gsm_vbat;
    }
    else
    {
        m_ping_msg.mcu_info.vgsm = 0;
    }
    m_ping_msg.mcu_info.v5v = 5000;
    m_ping_msg.mcu_info.input_state = 0;
    uint8_t output = 0;
    output |= app_io_get_mcu_exp_value()->BitName.ISO_OUT1;
    output |= app_io_get_mcu_exp_value()->BitName.ISO_OUT2 << 1;
    output |= app_io_get_mcu_exp_value()->BitName.en_pa << 2;
    m_ping_msg.mcu_info.output_control = output;
    char *master = app_flash_get_master(app_flash_get_current_streaming_master_index());
    sprintf(m_ping_msg.streaming_master, "%s", master); 

    uint8_t priority = app_flash_get_protocol_priority();
    if (priority != APP_FLASH_PROTOCOL_V2_JSON)
    {
        char *body = build_mqtt_body();
        assert(body);
        if (priority == APP_FLASH_PROTOCOL_V1_MQTT)  // mqtt
        {
            if (pub_mcu_info(body) == -1)
            {
                v1_post_to_server(body);
            }
        }
        else if (priority == APP_FLASH_PROTOCOL_V1_HTTP)       // http
        {
            if (v1_post_to_server(body) != 200)
            {
                pub_mcu_info(body);
            }
        }
        free(body);
    }
    else
    {
        mqtt_publish_ttn_info(&m_ping_msg);
    }
}


void app_mqtt_set_streaming_master(char *master)
{
    if (master)
        sprintf(m_ping_msg.streaming_master, "%s", master); 
}
