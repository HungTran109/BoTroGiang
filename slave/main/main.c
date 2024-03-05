/* Play M3U HTTP Living stream

   ============
   Sửa menuconfig
   ESP32 Specific:
       - Tăng các Main task stack size 3584 lên 100KB (102400)

//	   - Nếu chạy song song cả WiFi/PPP -> PPP bị lỗi: assertion "conn->write_offset < conn->current_msg->msg.w.len" failed, api_msg.c", line 1674, function: lwip_netconn_do_writemore
//	   - Thử tăng Main task stack size lên 112KB để chạy song song PPPoS vs Wifi -> vẫn lỗi -> tăng lên 116KB -> Khởi động bị reset liên tục!
//	   - Thử giảm Main task stack size còn 65536 -> chạy được song song, nhưng lỗi ngẫu nhiên -> thử tăng TCP send buffer size?
//   LwIP : Tăng TCP/IP stack size 4096 -> 8192 -> chạy riêng PPP ổn!
//   LwIP -> TCP : tăng send buffer size 5744 -> tăng lên gấp đôi 11488 chạy vài giờ k thấy bị!
//   Lý do là nhiều khi các gói tin gửi đi liên tiếp, kích thước lớn -> over size of buffer
//   Ví dụ:
//   pppos_netif_output[1]: proto=0x21, len = 1064
//   pppos_netif_output[1]: proto=0x21, len = 1440
//   pppos_netif_output[1]: proto=0x21, len = 1440
//   or
//   pppos_netif_output[1]: proto=0x21, len = 1064
//   pppos_netif_output[1]: proto=0x21, len = 1390
//   pppos_netif_output[1]: proto=0x21, len = 1390
//   pppos_netif_output[1]: proto=0x21, len = 1390
//   assertion "last_unsent->oversize_left >= oversize_used" failed: file "../esp/esp-adf/esp-idf/components/lwip/lwip/src/core/tcp_out.c", line 686, function: tcp_write
//   abort() was called at PC 0x400da247 on core 0

- Bật/tắt debug PPP: Component config -> LWIP -> Enable PPP support -> Enable PPP debug log output
*/

/******************************************************************************
                                   INCLUDES
******************************************************************************/
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "http_stream.h"
#include "i2s_stream.h"
#include "aac_decoder.h"
// #include "esp32/rom/rtc.h"
#include "rom/rtc.h"
#include "wav_decoder.h"
#include "mp3_decoder.h"
#include "opus_decoder.h"

#include "esp_peripherals.h"
#include "periph_wifi.h"
#include "board.h"

#include "mqtt_client.h"
#include "esp_modem.h"
#include "ec2x.h"

// For Ethernet //
#include "lwip/netif.h"
// #include "esp_event_loop.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "rom/gpio.h"
// #include "tcpip_adapter.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "mdns.h"
#include "esp_ping.h"
#include "ping/ping.h"
#include "netdb.h"

// SNTP
#include "esp_sntp.h"

// For OTA
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"

// User define
#include "pcf8575.h"
#include "version_ctrl.h"
#include "utilities.h"
#include "audio_hal.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "http_stream.h"
#include "network.h"
#include "app_ota.h"
#include "app_io.h"
#include "app_flash.h"
#include "app_mqtt.h"
#include "app_sntp.h"
#include "esp32/rom/rtc.h"
#include "tcp_console.h"
#include "app_debug.h"
#include "cJSON.h"
#include "httn.h"
#include "main.h"

#define SPEAKER_DETECT_ENABLE   1

#ifndef CONFIG_ADF_LOG_LEVEL
#define CONFIG_ADF_LOG_LEVEL ESP_LOG_INFO
#endif /**< CONFIG_ADF_LOG_LEVEL */

#define LIVE_MASTER 1
#define NO_STREAM 0
/**
 * LED chỉ chị loa phường:
- LED1: nháy xanh: chưa online; sáng xanh: online SIM
- LED2: nháy đỏ: chưa online; sáng đỏ: online ETH; sáng xanh: đang phát hoặc thu
* Version history
https://bytechvn-my.sharepoint.com/:w:/g/personal/biennb_bytech_vn/EdhCqa98dRxBoXqDesn9TLoBmjnM4wAN6pg9UFn-WXRI9A?e=m4J0X6
*/

// Power pin for Ethernet PHY chip (define if used gpio pin)
#ifndef CONFIG_PHY_USE_POWER_PIN
#define CONFIG_PHY_USE_POWER_PIN
#endif

#define TEST_VBAT_GSM 0
#define AAC_STREAM 0
#define WAV_STREAM 0
#define MP3_STREAM 0
#define OPUS_STREAM 1

#define RECONNECT_WIFI_IN_STREAM 30000
#define RECONNECT_WIFI_IN_IDLE 3000

#define SUBSCRIBE_INTERVAL 61
#define MULTI_ELEMENT   0

// Network error timeout
#define RTC_MEMORY_VALID_INFO_KEY		0x55AABB
#define HEARTBEAT_INTERVAL 59

#define DEFAULT_TIMEOUT_AUTO_TERMINATE_STREAM 90

typedef union 
{
	struct Stream_State_t 
    {
		uint16_t MasterTINH1 : 1;
		uint16_t MasterTINH2 : 1;
		uint16_t MasterTINH3 : 1;
		uint16_t MasterHUYEN1 : 1;
		uint16_t MasterHUYEN2 : 1;
		uint16_t MasterHUYEN3 : 1;
		uint16_t MasterXA1 : 1;
		uint16_t MasterXA2 : 1;
		uint16_t MasterXA3 : 1;
		uint16_t NA : 7;
	} __attribute__((packed)) name;
	uint16_t value;
} __attribute__((packed)) app_mqtt_master_streaming_state_t;

typedef struct
{
    uint32_t flag0;
    uint8_t reason;
    uint32_t flag1;
} software_reset_reason_on_rtc_t;

// url stream header
static uint8_t m_timeout_http_stream_check = 0;
// System_t xSystem;
static uint32_t sys_tick_counter = 0;
uint32_t last_sys_tick_counter = 0;
static uint8_t m_main_task_hangout = 0;
// uint8_t HW_VERSION = 3;
static uint32_t m_auto_terminate_pipeline = 0;
static uint32_t m_total_streaming_received = 0;
static uint32_t m_total_stream_time_in_session = 0;
static uint32_t m_total_stream_time_in_one_day = 0;
static uint8_t m_auto_restart_stream_timeout = 0;
static uint8_t m_timeout_turn_off_opto_output = 0;
static uint8_t m_start_stream_cmd_timeout = 0;

static bool m_allow_monitor_stream_downloaded = false;

static uint8_t m_waiting_http_element_stopped_timeout = 0;
static uint8_t m_timeout_waiting_next_master_streaming = 0;
static bool m_stream_emergency = false;
// Case: Bên phát bị rớt mạng -> mất data -> FINISH_TRACK event
//-> http_element chưa kịp nhận FINISHED_STATUS thì bên phát lại lệnh APP_AUDIO_STREAM_START lại
//-> trạng thái http_element vẫn đang RUNNING -> terminate_pipeline và chờ http_stopped
//-> tuy nhiên khi đã xảy ra FINISH_TRACK thì terminate_pipeline sẽ không làm cho http_element
//  về stopped -> chờ mãi cũng không stopped để restart stream được -> Không thu được gì!
//=> Khi nhận được FINISH_TRACK thì không nhận APP_AUDIO_STREAM_START hay STREAM_STOP nữa, chờ xảy ra
//  FINISHED_STATUS và restart lại stream
static uint8_t m_http_get_finish_track_event_timeout = 0;

// Kiểm soát trạng thái streaming, nếu giữa 2 bản tin APP_AUDIO_STREAM_RUNNING của master mà
// dung lượng stream bên thu không thay đổi -> không stream được -> reset system luôn
static uint32_t m_last_m_total_streaming_received = 0;
static uint8_t m_receive_stream_running_state_timeout = 0;
static int8_t m_mqtt_send_heartbeat_counter = 0;
static uint8_t m_mqtt_fast_subscribe_timeout = 0;
static uint16_t m_mqtt_send_slow_subscribe_timeout = (SUBSCRIBE_INTERVAL-20);
static volatile uint32_t m_mqtt_disconnected_timeout = 0;
static uint8_t m_delay_turn_on_relay_prepair_stream = 0; // On relay1 -> delay -> on relay2
static uint16_t m_max_delay_time_wait_for_stream_from_prepare = 0;
// delay2 -> off relay2 -> delay1 -> off relay1
static uint8_t m_delay_turn_off_relay1_stop_stream = 0;
static uint16_t m_delay_turn_off_relay2_stop_stream = 0;
static uint8_t m_http_received_data_timeout;

// Chọn về Internet từ web thì chế độ Local về Idle
static uint32_t m_auto_switch_codec_mode_in_test_mode = 0;
static SemaphoreHandle_t m_debug_lock;

static modem_dce_t *m_dce = NULL;
RTC_NOINIT_ATTR software_reset_reason_on_rtc_t m_reset_reason;
// static bool m_terminate_stream_async = false;
// static bool m_restart_stream_async = false;
static uint32_t m_delay_for_http_server_have_enough_buffer = 1;
static bool m_is_waiting_http_element_stopped = false;
void ttn_on_device_mode_change(char *value);

static app_mqtt_master_streaming_state_t m_master_streaming_info = 
{
    .value = 0
};


void system_software_reset(uint8_t reason)
{
    m_reset_reason.flag0 = RTC_MEMORY_VALID_INFO_KEY;
    m_reset_reason.reason = reason;
    m_reset_reason.flag1 = RTC_MEMORY_VALID_INFO_KEY;
    esp_restart();
}

uint8_t system_get_software_reset_reason(void)
{
    if (m_reset_reason.flag0 == RTC_MEMORY_VALID_INFO_KEY
        && m_reset_reason.flag1 == RTC_MEMORY_VALID_INFO_KEY)
    {
        return m_reset_reason.reason;
    }
    return SW_RESET_REASON_UNKNOWN;
}

void system_clear_software_reset_reason(void)
{
    m_reset_reason.flag0 = RTC_MEMORY_VALID_INFO_KEY;
    m_reset_reason.flag1 = RTC_MEMORY_VALID_INFO_KEY;
    m_reset_reason.reason = SW_RESET_REASON_UNKNOWN;
}

void slave_on_hls_finish_track_cb()
{
    m_http_get_finish_track_event_timeout = 13;  // 15
    // 05/07/20: xóa trạng thái đang có master streaming -> FM hiển thị IDLE
    // m_master_streaming_info.value = 0;

    // 01/02/2021: Nếu chạy nhiều master -> xét trạng thái streaming của từng master
    switch (app_flash_get_current_streaming_master_index())
    {
    case APP_FLASH_MASTER_TINH1:
        m_master_streaming_info.name.MasterTINH1 = 0;
        break;
    case APP_FLASH_MASTER_TINH2:
        m_master_streaming_info.name.MasterTINH2 = 0;
        break;
    case APP_FLASH_MASTER_TINH3:
        m_master_streaming_info.name.MasterTINH3 = 0;
        break;
    case APP_FLASH_MASTER_HUYEN1:
        m_master_streaming_info.name.MasterHUYEN1 = 0;
        break;
    case APP_FLASH_MASTER_HUYEN2:
        m_master_streaming_info.name.MasterHUYEN2 = 0;
        break;
    case APP_FLASH_MASTER_HUYEN3:
        m_master_streaming_info.name.MasterHUYEN3 = 0;
        break;
    case APP_FLASH_MASTER_XA1:
        m_master_streaming_info.name.MasterXA1 = 0;
        break;
    case APP_FLASH_MASTER_XA2:
        m_master_streaming_info.name.MasterXA2 = 0;
        break;
    case APP_FLASH_MASTER_XA3:
        m_master_streaming_info.name.MasterXA3 = 0;
        break;
    default:
        m_master_streaming_info.value = 0;
        break;
    }
}

uint8_t stream_ele_info(void)
{
    uint8_t info = 0;
    info = ((app_audio_is_i2s_running() << 0)
            | (app_audio_is_opus_running() << 1)
            | (app_audio_is_http_audio_stream_running() << 2));
    return info;
}

void slave_reset_delay_turn_off_relay1_when_stop_stream(void)
{
    m_delay_turn_off_relay1_stop_stream = 0;
}

uint32_t slave_get_delay_turn_off_relay1_remain_time(void)
{
    return m_delay_turn_off_relay1_stop_stream;
}

void slave_reset_delay_turn_off_relay2_when_stop_stream(void)
{
    m_delay_turn_off_relay2_stop_stream = 0;
}

uint32_t slave_get_delay_turn_off_relay2_remain_time(void)
{
    return m_delay_turn_off_relay2_stop_stream;
}

void slave_allow_http_element_monitor_downloaded_data_in_streaming_state(bool state)
{
    m_allow_monitor_stream_downloaded = state;
}

void slave_reset_counter_wait_for_http_element_stop(void)
{
    m_waiting_http_element_stopped_timeout = 0;
}

void slave_set_mqtt_state_timeout(uint8_t timeout)
{
    m_mqtt_send_heartbeat_counter = timeout;
}

void slave_set_auto_restart_stream_timeout(uint8_t timeout)
{
    m_auto_restart_stream_timeout = timeout;
}

void slave_set_audio_switch_audio_in_test_mode(void)
{
    m_auto_switch_codec_mode_in_test_mode = 15;
}

uint8_t slave_get_http_finish_track_timeout(void)
{
    return m_http_get_finish_track_event_timeout;
}

void slave_set_http_finish_track_timeout(uint8_t timeout)
{
    m_http_get_finish_track_event_timeout = timeout;
}

void slave_set_received_running_state_timeout(uint32_t timeout)
{
    m_receive_stream_running_state_timeout = timeout;
}

uint8_t slave_get_received_running_state_timeout(void)
{
    return m_receive_stream_running_state_timeout;
}

uint32_t slave_get_stream_time(void)
{
    return m_total_stream_time_in_session;
}

uint32_t slave_get_stream_time_in_day(void)
{
    return m_total_stream_time_in_one_day;
}


void slave_increase_stream_data_downloaded(uint32_t number_of_bytes)
{
    if (number_of_bytes > 0)
    {
        m_timeout_http_stream_check = 0;
        m_total_streaming_received += number_of_bytes;
        m_http_received_data_timeout = 60;    
    }
}

uint8_t slave_get_http_received_data_timeout(void)
{
    return m_http_received_data_timeout;
}

void slave_reset_stream_monitor_data(void)
{
    m_last_m_total_streaming_received = 0;
    m_total_streaming_received = 0;
}

uint32_t slave_get_total_streaming_received(void)
{
    return m_total_streaming_received;
}

void slave_reset_total_streaming_received(void)
{
    m_total_streaming_received = 0;
}

void slave_update_last_stream_data(void)
{
    m_last_m_total_streaming_received = m_total_streaming_received;
}

void slave_set_timeout_number_of_waiting_master_streaming(uint8_t counter)
{
    m_timeout_waiting_next_master_streaming = counter;
}

void slave_set_timeout_when_received_stream_running_command(uint8_t delay_s)
{
    m_auto_terminate_pipeline = delay_s;
}


uint8_t slave_get_number_of_second_timeout_waiting_master_streaming(void)
{
    return m_timeout_waiting_next_master_streaming;
}

void slave_process_stop_onair(void)
{
    // Thêm điều khiển OFF relay theo thứ tự delay2 -> off relay2 -> delay1 -> off relay1
    m_delay_turn_off_relay2_stop_stream = app_flash_get_relay2_turn_off_delay();
    m_stream_emergency = false;
}

void slave_set_start_stream_command_timeout(uint8_t timeout)
{
    m_start_stream_cmd_timeout = timeout;
}

uint8_t slave_get_start_stream_command_timeout(void)
{
    return m_start_stream_cmd_timeout;
}

uint8_t slave_get_timeout_turn_off_opto_output(void)
{
    return m_timeout_turn_off_opto_output;
}

void slave_set_timeout_turn_off_opto_output(uint8_t timeout)
{
    m_timeout_turn_off_opto_output = timeout;   
}

bool slave_at_least_one_master_is_streaming(void)
{
    return m_master_streaming_info.value ? true : false;
}

static uint8_t m_csq = 99;
uint8_t slave_get_gsm_csq(void)
{
    return m_csq;
}

static uint8_t m_band = 0;
uint8_t slave_get_gsm_band()
{
    return m_band;
}

void *slave_get_modem_dce(void)
{
    return m_dce;
}

static char m_ppp_ip_str[24] = {"0.0.0.0"};

char *slave_get_ppp_ip(void)
{
    return m_ppp_ip_str;
}

void network_event_cb(void *src, void *evt, void *data, uint32_t size)
{
    network_interface_src_t *if_src = (network_interface_src_t*)src;
    if (*if_src == NETWORK_INTERFACE_SRC_ETH)
    {
        uint32_t eth_evt = (uint32_t)evt;
        if (eth_evt == SYSTEM_EVENT_ETH_DISCONNECTED)
        {
            /* Test LED Ethernet ON */
            app_io_control_led_eth_state(0);
        }
        else if (eth_evt == SYSTEM_EVENT_ETH_GOT_IP)
        {
            DEBUG_VERBOSE("[ZIG] ETHERNET: Link Up - Got IP\r\n");
            app_io_control_led_eth_state(1);
        }
    }
    else if (*if_src == NETWORK_INTERFACE_SRC_WIFI)
    {
        uint32_t wifi_evt = (uint32_t)evt;
        if (wifi_evt == SYSTEM_EVENT_STA_GOT_IP)
        {
            app_io_control_led_wifi_state(1);
        }
        else if (wifi_evt == SYSTEM_EVENT_STA_DISCONNECTED)
        {
            app_io_control_led_wifi_state(0);
        }
    }
    else if (*if_src == NETWORK_INTERFACE_SRC_SMART_CFG)
    {
        app_flash_write_wifi_info(((network_wifi_info_t*)data)->ssid, ((network_wifi_info_t*)data)->password, 1);
    }
    else if (*if_src == NETWORK_INTERFACE_SRC_PPP)
    {
        esp_modem_event_t modem_event = (esp_modem_event_t)evt;
        switch (modem_event)
        {
            case MODEM_EVENT_PPP_START:
            break;

        case MODEM_EVENT_PPP_CONNECT:
        {
            DEBUG_INFO("Modem connected to PPP Server\r\n");
            ppp_client_ip_info_t *ipinfo = (ppp_client_ip_info_t *)(data);
            sprintf(m_ppp_ip_str, IPSTR, IP2STR(&ipinfo->ip));
            app_io_control_led_4g_state(1);
        }
            break;

        case MODEM_EVENT_PPP_DISCONNECT:
            DEBUG_INFO("Modem Disconnect from PPP Server\r\n");
            app_io_control_led_4g_state(0);
            break;

        case MODEM_EVENT_PPP_STOP:
            DEBUG_WARN("Modem PPP Stoppe\r\n");
            app_io_control_led_4g_state(0);
            // m_csq = 99;
            break;

        case MODEM_EVENT_INIT_DONE: /* Khởi tạo xong module 4G */
        {
            modem_dce_t *dce = (modem_dce_t*)data;
            m_dce = dce;
            /* Print Module ID, Operator, IMEI, IMSI */
            DEBUG_INFO("Module version: %s, IMEI: %s\r\n", dce->name, dce->imei);
            DEBUG_INFO("SIM IMEI: %s, IMSI: %s\r\n", dce->sim_imei, dce->imsi);
            DEBUG_INFO("Operator: %s\r\n", dce->oper);
            DEBUG_INFO( "RSSI: %d\r\n", dce->rssi);
            DEBUG_INFO( "Tech: %s, band: %s\n", dce->access_tech, dce->band);
            m_csq = dce->rssi;
            char *p_band = strstr(dce->band, "BAND ");
            if (p_band)
            {
                m_band = GetNumberFromString(5, p_band);
            }
            else
            {
                p_band = 0;
            }
            DEBUG_VERBOSE("[ZIG] IMEI: GSM: %s, SIM: %s\r\n", dce->imei, dce->imsi);

            // Lưu thông tin IMEI module vào bộ nhớ, dùng cho lần sau
            if (IsDigitString(dce->imei))
            {
                if (strlen(app_flash_get_imei()) < 15 || (strcmp(dce->imei, app_flash_get_imei()) != 0))
                {
                    sprintf(app_flash_get_imei(), "%s", dce->imei);
                    app_flash_node_nvs_write_string(APP_FLASH_GSM_IMEI_KEY, app_flash_get_imei());
                    DEBUG_INFO("Coppied GSM_IMEI: %s\r\n", app_flash_get_imei());
                }
                else
                {
                    DEBUG_INFO( "GSM_IMEI is existed and matched!\r\n");
                }
            }
        }
            break;

        case MODEM_EVENT_UNKNOWN:
            break;
        default:
            break;
        }
    }
}

static void xSystem_timercb(void *timer)
{
    static uint8_t timeout_100ms = 0;

    timeout_100ms++;
    if (timeout_100ms >= 100)
    {
        timeout_100ms = 0;

        /* Timeout mất kết nối MQTT server -> mạng có vấn đề */
        m_mqtt_disconnected_timeout++;
        // DEBUG_INFO( "mqtt disconnect timeout: %d", m_mqtt_disconnected_timeout);
        // Chuyển mạng qua lại mà vẫn không kết nối được -> Reset cho nhanh!
        if (m_mqtt_disconnected_timeout >= NETWORK_ERROR_TIMEOUT_SEC)
        {
            ets_printf("m_mqtt_disconnected_timeout");
            if (m_mqtt_disconnected_timeout > NETWORK_ERROR_TIMEOUT_SEC+3)      // 5s to save critical data to flash
            {
                system_software_reset(SW_RESET_REASON_SRV_TIMEOUT);
            }
        }

        /* Check system tick count */
        if (sys_tick_counter != last_sys_tick_counter)
        {
            last_sys_tick_counter = sys_tick_counter;
            m_main_task_hangout = 0;
        }
        else
        {
            if (m_main_task_hangout++ >= 33)
            {
                ets_printf("!!! OOOOPS... MAIN TASK HANGUP. REBOOT !!!");
                system_software_reset(SW_RESET_REASON_MAIN_TASK_HANG);
            }
        }
    }
}


void slave_set_delay_turn_on_relay_prepare_stream(uint8_t value)
{
    m_max_delay_time_wait_for_stream_from_prepare = value +  120;
    m_delay_turn_on_relay_prepair_stream = value;
}

uint8_t slave_get_delay_turn_on_relay_prepare_stream()
{
    return m_delay_turn_on_relay_prepair_stream;
}

void slave_reset_delay_turn_on_relay_process_on_air(void)
{
    m_delay_turn_on_relay_prepair_stream = 0;
}

void slave_report_info_now(void)
{
    m_mqtt_send_heartbeat_counter = HEARTBEAT_INTERVAL;
}

/******************************************************************************************/
/**
 * @brief   : task quản lý hoạt động của module gsm
 * @param   :
 * @retval  :
 * @author  :
 * @created :
 */
static void main_manager_task(void *arg)
{
    uint8_t mqtt_tick_counter = 0;
    uint8_t task_timeout10s = 0;
    //	uint8_t task_tick3s = 0;

    uint8_t last_stream_state = 0;
    // uint8_t lastMICDetectState = 1;
    uint8_t led_stream_state = 0;
    //	uint8_t testOnOffGSMVbat = 0;
    uint8_t master_sub_index = APP_FLASH_MASTER_TINH1;
    esp_err_t err;

    DEBUG_INFO("\t\t--- main_manager_task is running ---\r\n");

    if (app_flash_speaker_audio_class_get() == APP_FLASH_AUDIO_CLASS_D)
    {
        app_io_control_opto_output1(APP_IO_OPTO_ON);        // mute class D
    }
    bool debug_wifi = true;
    int continues_reset_eth = 0;
    while (1)
    {
        sys_tick_counter++;
        if (continues_reset_eth)
        {
            continues_reset_eth--;
        }

        network_manager_poll();

        /* ==================================== Quản lý MQTT connection ==========================================*/
        if (network_is_connected())
        {
            switch (app_mqtt_get_state())
            {
            case APP_MQTT_DISCONNECTED:
                {
                    if (app_mqtt_need_restart_mqtt())
                    {
                        app_mqtt_close();
                        if (continues_reset_eth == 0)
                        {
                            if (network_is_eth_got_ip())
                            {
                                continues_reset_eth = 60;
                                network_eth_restart();
                            }
                        }
                        else
                        {
                            continues_reset_eth--;
                        }
                        app_mqtt_need_clear_restart_mqtt_flag();
                    }
                    err = app_mqtt_start();
                    if (err == ESP_OK)
                    {
                        mqtt_tick_counter = 0;
                    }
                    else
                    {
                        mqtt_tick_counter = 8;
                    }
                    app_mqtt_set_state(APP_MQTT_CONNECTING);
                    m_mqtt_fast_subscribe_timeout = 0;
                    debug_wifi = true;
                }
                break;
            case APP_MQTT_CONNECTING:
                if (mqtt_tick_counter++ >= 10)
                {
                    err = app_mqtt_start();
                    if (err == ESP_OK)
                    {
                        mqtt_tick_counter = 0;
                    }
                    else
                    {
                        mqtt_tick_counter = 8;
                    }
                }
                break;
            case APP_MQTT_CONNECTED:
                m_mqtt_disconnected_timeout = 0;

                /* Gửi bản tin infor định kỳ mỗi 60s */
                if (++m_mqtt_send_heartbeat_counter >= HEARTBEAT_INTERVAL)
                {
                    m_mqtt_send_heartbeat_counter = 0;
                    app_mqtt_publish_slave_info();
                }

                if (m_mqtt_send_heartbeat_counter > 5 && debug_wifi)
                {
                    if (network_is_wifi_got_ip())
                    {
                        debug_wifi = false;
                        // mqtt_publish_message("DBG", "WIFI connected");
                    }
                }

                uint8_t pro = app_flash_get_protocol_priority();
                /* Gửi yêu cầu subscribe nhanh nếu chưa được sub */
                if (++m_mqtt_fast_subscribe_timeout >= 10)
                { // 10
                    m_mqtt_fast_subscribe_timeout = 0;

                    if (!app_mqtt_is_subscribed())
                    {
                        app_mqtt_send_subscribe_config_topic(app_flash_get_imei());
                    }

                    if (pro != APP_FLASH_PROTOCOL_V2_JSON)
                    {
                        /* subscribe master topic nào đã được cấu hình */
                        for (uint8_t master_index = APP_FLASH_MASTER_TINH1; master_index < APP_FLASH_MASTER_TOTAL; master_index++)
                        {
                            if (!app_mqtt_is_master_subscribed(master_index) 
                                && strlen(app_flash_get_master(master_index)) >= 15)
                            {
                                app_mqtt_send_subscribe_command_topic(master_index);
                                m_mqtt_fast_subscribe_timeout = 9;
                                break;
                            }
                        }
                    }
                    else
                    {
                        /* subscribe master topic nào đã được cấu hình */
                        for (uint8_t master_index = 0; master_index < APP_FLASH_MAX_GROUP_SUPPORT; master_index++)
                        {
                            app_flash_group_info_t *info = app_flash_get_group_info(master_index);
                            if (!app_mqtt_is_master_subscribed(master_index) 
                                && info
                                && (strlen(info->group_id) >= 3))
                            {
                                app_mqtt_send_subscribe_command_topic(master_index);
                                m_mqtt_fast_subscribe_timeout = 9;
                                break;
                            }
                        }
                    }
                }

                /* Gửi yêu cầu subscribe định kỳ nếu đã được sub -> tránh trường hợp mất subcribe không nhận được lệnh */
                m_mqtt_send_slow_subscribe_timeout++;
                if (m_mqtt_send_slow_subscribe_timeout == SUBSCRIBE_INTERVAL)
                {
                    app_mqtt_send_subscribe_config_topic(app_flash_get_imei());
                }
                if (m_mqtt_send_slow_subscribe_timeout > SUBSCRIBE_INTERVAL)
                {
                    if (pro != APP_FLASH_PROTOCOL_V2_JSON)
                    {
                        for (uint8_t i = master_sub_index; i < APP_FLASH_MASTER_TOTAL; i++)
                        {
                            if (strlen(app_flash_get_master(i)) >= 15)
                            {
                                app_mqtt_send_subscribe_command_topic(i);
                                master_sub_index = i;
                                break;
                            }
                        }
                        master_sub_index++;
                        if (master_sub_index >= APP_FLASH_MASTER_TOTAL)
                        {
                            m_mqtt_send_slow_subscribe_timeout = 1;
                            master_sub_index = APP_FLASH_MASTER_TINH1;
                        }
                    }
                    else
                    {
                        for (uint8_t i = master_sub_index; i < APP_FLASH_MAX_GROUP_SUPPORT; i++)
                        {
                            app_flash_group_info_t *info = app_flash_get_group_info(i);
                            if (info)
                            {
                                app_mqtt_send_subscribe_command_topic(i);
                                master_sub_index = i;
                                break;
                            }
                        }
                        master_sub_index++;
                        if (master_sub_index >= APP_FLASH_MAX_GROUP_SUPPORT)
                        {
                            m_mqtt_send_slow_subscribe_timeout = 1;
                            master_sub_index = 0;
                        }
                    }
                }

                static uint32_t send_proto_des = 0;
                send_proto_des++;
                if (send_proto_des == 15)
                {
                    static bool send = false;
                    if (send == false)
                    {
                        static char *protocol_des[] = {"I2C", "string", "min"};
                        send = true;
                        app_io_protocol_type_t fm = app_io_get_fm_protocol_method();
                        char *fm_pro = protocol_des[app_io_get_fm_protocol_method()];
                        if (fm == APP_IO_MCU_PROTOCOL_STATE_UNKNOWN)
                        {
                            fm_pro = "NA";
                        }
                        mqtt_publish_message("DBG", "EXP:%s,FM:%s", 
                                            protocol_des[app_io_get_gd32_protocol_method()], 
                                            fm_pro);
                    }
                }

                if (send_proto_des == 90)
                {
                    bool err = network_is_eth_phy_err();
                    if (err)
                    {
                        mqtt_publish_message("DBG", "ETH:ERR");
                    }
                }

                /* Send reset message */
                app_mqtt_send_reset_message();

                /* Start local TCP debug */
                if (app_flash_is_tcp_console_enable() 
                    && !tcp_console_started()
                    && (network_is_wifi_got_ip() || network_is_eth_got_ip()))
                {
                    mqtt_publish_message("DBG", "WIFI IP = %s, ETH IP = %s", 
                                            network_get_local_ip(NETWORK_LOCAL_IP_WIFI), 
                                            network_get_local_ip(NETWORK_LOCAL_IP_ETH));
                    app_debug_register_callback_print(tcp_send_buffer);
                    tcp_console_start();
                }
                break;

            default:
                break;
            }
        }

        if (m_mqtt_disconnected_timeout > NETWORK_ERROR_TIMEOUT_SEC)
        {
            modem_dce_t *dce = slave_get_modem_dce();

            // NO SIM error
            uint32_t gsm_err = 0;
            if (!(dce && strlen(dce->sim_imei) > 10))
            {
                gsm_err |= 1 << 0;
            }
            if (!(dce && strlen(dce->imei) > 10))
            {
                gsm_err |= 1 << 1;
            }

            if (gsm_err)
            {
                // app_flash_node_nvs_write_u32(APP_FLASH_KEY_GSM_ERR, gsm_err);
            }
            system_software_reset(SW_RESET_REASON_SRV_TIMEOUT);
        }

        /* ============================ Quản lý ngoại vi ================================= */
        if (m_timeout_turn_off_opto_output)
        {
            m_timeout_turn_off_opto_output--;
            if (m_timeout_turn_off_opto_output == 0)
            {
                /* Nếu đang không chạy các mode module Codec MIC/FM thì mới tắt PA */
                if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
                    && app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_IDLE)
                {
                    app_io_control_pa(APP_IO_PA_OFF);
                }

                /* Nếu đang không chạy mode MIC/FM thì tắt luôn relay */
                if (app_flash_get_operate_mode() != APP_AUDIO_OPERATION_MODE_MIC 
                    && app_flash_get_operate_mode() != APP_AUDIO_OPERATION_MODE_LINE 
                    && app_io_get_current_fm_module() != APP_AUDIO_OPERATION_MODE_MIC 
                    && app_io_get_current_fm_module() != APP_AUDIO_OPERATION_MODE_LINE)
                {
                    // Turn off luôn Relays
                    // app_io_opto_control_all(APP_IO_OPTO_OFF);

                    // 19/12/20: Tắt relay theo thứ tự relay2 -> delay 5s -> relay1
                    DEBUG_VERBOSE("Call slave_process_stop_onair\r\n");
                    slave_process_stop_onair();
                }
                m_http_received_data_timeout = 0;
            }
        }

        if (++task_timeout10s >= 200)
        {
            task_timeout10s = 190;
            DEBUG_VERBOSE("Free heap: %u/%u, streaming: %u, netif: %s\r\n", 
                    esp_get_free_heap_size(), esp_get_free_internal_heap_size(),
                    m_total_streaming_received, (netif_default != NULL) ? netif_default->name : "NULL");
            
        }
        if (app_io_is_in_test_mode() && m_auto_switch_codec_mode_in_test_mode)
        {
            m_auto_switch_codec_mode_in_test_mode--;
            DEBUG_INFO("Auto switch mode %d\r\n", m_auto_switch_codec_mode_in_test_mode);
            if (m_auto_switch_codec_mode_in_test_mode == 0)
            {
                m_auto_switch_codec_mode_in_test_mode = 10;
                if (app_audio_get_codec_mode() != APP_AUDIO_CODEC_MODE_LINE_IN)
                {
                    DEBUG_INFO("Change to linein\r\n");
                    app_audio_change_to_local_line_in_mode();
                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);
                }
                else
                {
                    DEBUG_INFO("Change to FM\r\n");
                    app_flash_set_current_fm_freq(app_flash_get_fm_freq3());
                    /* Nếu đang không stream internet thì mới cho chạy FM */
                    if (!stream_ele_info())
                    {
                        DEBUG_INFO("Switch to FM mode, freq: %u\r\n", app_flash_get_current_fm_freq());

                        /** Change to FM mode, PA ON */
                        app_audio_change_to_local_fm_mode();

                        // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                        DEBUG_INFO("app_io_opto_control_all OFF...\r\n");
                        // app_io_opto_control_all(APP_IO_OPTO_OFF);
                        if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                        {
                            app_io_opto_control_all(APP_IO_OPTO_OFF);
                        }
                        else
                        {
                            app_io_opto_control_all(APP_IO_OPTO_ON);
                        }


                        app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_FM); /* 1 - 5 */
                        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);
                    }
                    else
                    {
                        DEBUG_INFO("HTTP is running, can't run FM mode!\r\n");
                    }
                }
            }
        }

        /* Giám sát trạng thái Streaming của http element */
        if (m_allow_monitor_stream_downloaded)
        {
            if (m_timeout_http_stream_check++ >= 10)
            {
                m_timeout_http_stream_check = 0;

                if (m_total_streaming_received >= m_last_m_total_streaming_received)
                {
                    DEBUG_INFO("HTTP's streaming GOOD!\r\n");
                    m_last_m_total_streaming_received = m_total_streaming_received;
                }
                else
                {
                    DEBUG_INFO("ERR: HTTP's streaming BAD! Reboot...\r\n");
                    mqtt_publish_message("DBG", "Long time no receive http data");
                    if (!app_ota_is_running())
                    {
                        app_audio_change_codec_vol(0);
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        system_software_reset(SW_RESET_REASON_STREAM_START_TIMEOUT);
                    }
                }
            }
        }
        else
        {
            m_timeout_http_stream_check = 1;
            m_total_streaming_received = m_last_m_total_streaming_received = 0;
        }

        // Timeout lệnh APP_AUDIO_STREAM_START qua MQTT
        if (m_start_stream_cmd_timeout)
        {
            m_start_stream_cmd_timeout--;
            DEBUG_VERBOSE("STREAM_START cmd timeout: %d\r\n", 
                    m_start_stream_cmd_timeout);
        }

        // Timeout từ lúc nhận được FINISH_TRACK
        if (m_http_get_finish_track_event_timeout)
        {
            m_http_get_finish_track_event_timeout--;
            DEBUG_INFO("HTTP_FINISH_TRACK timeout: %u\r\n", 
                        m_http_get_finish_track_event_timeout);
        }

        if (m_auto_terminate_pipeline)
        {
            if (m_auto_terminate_pipeline < 12
                && m_auto_terminate_pipeline % 3 == 0)
            {
                DEBUG_WARN("Auto stop stream in %us\r\n", 
                            m_auto_terminate_pipeline);
            }
            
            m_auto_terminate_pipeline--;
            if (m_auto_terminate_pipeline == 0 
                && app_audio_get_http_state() == AEL_STATE_RUNNING)
            {
                /* 1.3. Turn off PA after 30s */
                m_timeout_turn_off_opto_output = 15;
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_IDLE);
                m_allow_monitor_stream_downloaded = 0;
                m_timeout_http_stream_check = 0;

                /** 2. Terminate pipeline */
                DEBUG_WARN("Stop stream when no received stream_running\r\n");
                app_audio_pause();
                app_audio_complete_terminate();

                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_STOPPED);
                m_allow_monitor_stream_downloaded = false; /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                m_allow_monitor_stream_downloaded = false;
                // Set PA off
                app_io_control_pa(APP_IO_PA_OFF);
                app_audio_set_invalid_link();
                app_flash_set_last_streaming_url(APP_FLASH_INVALID_LINK);
                mqtt_publish_message("DBG", "Auto stop streaming");
            }
            if (m_auto_terminate_pipeline == 0)
            {
                app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, 0);
                app_audio_set_invalid_link();
                app_flash_set_last_streaming_url(APP_FLASH_INVALID_LINK);
            }
        }
        // Timeout từ thời điểm nhận lệnh APP_AUDIO_STREAM_START/APP_AUDIO_STREAM_RUNNING
        if (m_receive_stream_running_state_timeout)
        {
            if (m_receive_stream_running_state_timeout < 10)
            {
                DEBUG_INFO("Timeout remain = %ds\r\n", 
                            m_receive_stream_running_state_timeout);
            }
            if (app_audio_is_http_audio_stream_running() 
                && app_audio_is_opus_running()
                && (app_audio_is_i2s_running())
                && (m_total_streaming_received > m_last_m_total_streaming_received))
            {
                app_audio_reset_force_turn_off_pa_when_link_404_not_found();
                app_io_control_pa(APP_IO_PA_ON);
                /* Sau 2s mà trạng thái vẫn RUNNING và dung lượng stream tăng thì OK */
                if (m_receive_stream_running_state_timeout < 34)
                {
                    DEBUG_INFO("Streaming's OK\r\n");
                    m_receive_stream_running_state_timeout = 0;
                }
                else
                {
                    m_receive_stream_running_state_timeout--;
                }
            }
            else
            {
                m_receive_stream_running_state_timeout--;
                if (m_receive_stream_running_state_timeout == 0)
                {
                    DEBUG_WARN("ERR! Stream timeout, reset...\r\n");
                    mqtt_publish_message("DBG", "Stream timeout, reset");
                    if (!app_ota_is_running())
                    {
                        app_audio_change_codec_vol(0);
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        system_software_reset(SW_RESET_REASON_STREAM_START_TIMEOUT);
                    }
                }
            }
        }

        /* Đếm thời gian stream ở trạng thái RUNNING */
        if (app_audio_is_http_audio_stream_running())
        {
            m_total_stream_time_in_one_day++;
            m_total_stream_time_in_session++;
            if (last_stream_state == AEL_STATE_STOPPED)
                m_mqtt_send_heartbeat_counter = HEARTBEAT_INTERVAL - 5; /* Chuyển trạng thái sang RUNNING 5s + random thì gửi info */
            last_stream_state = AEL_STATE_RUNNING;

            // Nếu LED Stream không sáng -> ON
            if (!app_io_is_led_streaming_on())
            {
                app_io_control_led_stream(APP_AUDIO_STREAM_RUNNING);
                m_allow_monitor_stream_downloaded = true;
            }
        }
        else
        {
            if (app_io_is_led_streaming_on())
            {
                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                m_allow_monitor_stream_downloaded = false;
            }
            m_total_stream_time_in_session = 0;
            m_total_streaming_received = 0;
            last_stream_state = AEL_STATE_STOPPED;
        }

        if (m_timeout_waiting_next_master_streaming)
            m_timeout_waiting_next_master_streaming--;

        /* Kiểm soát chế độ của codec khi đang streaming internet, nếu đang có 1 master streaming -> slave luôn ở mode DECODE */
        if (m_master_streaming_info.value
            && app_audio_get_codec_mode() != APP_AUDIO_CODEC_MODE_DECODE)
        {
            DEBUG_INFO("CODEC: Force return to INTERNET mode\r\n");
            app_audio_change_codec_to_internet_mode();
            break;
        }

        // /** Trạng thái cắm MIC: HW v1.0.2 chỉ detect được khi Relay input đóng vào đường MIC!
        //  * Không detect được đường LINE IN
        //  */
        // uint8_t micState = gpio_get_level(APP_IO_MIC_DETECT_NUM);
        // if (micState != lastMICDetectState)
        // {
        //     DEBUG_INFO("MIC state: %s", micState == 0 ? "PLUGGED" : "REMOVED");
        // }
        // xSystem.Status.isMICPlugged = !micState;
        // lastMICDetectState = micState;

        // Kiểm tra trạng thái điều khiển PA, ISORelay khi đang chạy các chế độ phát thanh
        if (m_total_stream_time_in_session > 1 
            || app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_MIC 
            || app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_LINE 
            || app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_MIC 
            || app_io_get_current_fm_module() == APP_AUDIO_OPERATION_MODE_LINE)
        {
            // Nếu PA đang OFF -> turn ON
            if (app_io_is_pa_off() && app_audio_get_force_turn_off_pa_remain_time() == 0)
            {
                app_io_control_pa(APP_IO_PA_ON);
            }

            if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
            {
                // Nếu ISO relays đang OFF -> turn ON
                if (app_io_is_iso_relays_off())
                {
                    if (m_delay_turn_on_relay_prepair_stream == 0)
                    {
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                        // mqtt_publish_message("DBG", "Turn on opto");
                    }
                    else
                    {
                        app_io_control_opto_output1(APP_IO_OPTO_ON);
                    }
                }
            }
            else // Class D muc dk logic lai nguoc lai
            {
                // Nếu ISO relays đang OFF -> turn ON
                if (!app_io_is_iso_relays_off())
                {
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                    app_io_control_opto_output1(APP_IO_OPTO_OFF);
                    // mqtt_publish_message("DBG", "Class D - all opto off");
                }
            }
        }

        /* Timeout mất kết nối MQTT server -> mạng có vấn đề */
        // m_mqtt_disconnected_timeout++;

        // // DEBUG_INFO("mqtt disconnect timeout: %d", m_mqtt_disconnected_timeout);
        // // Chuyển mạng qua lại mà vẫn không kết nối được -> Reset cho nhanh!
        // if (m_mqtt_disconnected_timeout >= NETWORK_ERROR_TIMEOUT_SEC)
        // {
        //     DEBUG_WARN("Network is error, reset system...");
        //     app_audio_change_codec_vol(0);
        //     vTaskDelay(1000 / portTICK_RATE_MS);
        //     system_software_reset(SW_RESET_REASON_SRV_TIMEOUT);
        // }

        network_interface_t interface = network_get_current_interface();
        if (m_mqtt_disconnected_timeout % 55 == 3)
        {
            if (interface == NETWORK_INTERFACE_PPP)
            {
                // Switch to other interface
                if (network_is_eth_got_ip())
                {
                    DEBUG_INFO("Broker's disconnected, change to ETH\r\n");
                    network_change_interface(NETWORK_INTERFACE_ETH);
                }
                else if (network_is_wifi_got_ip())
                {
                    DEBUG_INFO("Broker's disconnected, change to WIFI\r\n");
                    network_change_interface(NETWORK_INTERFACE_WIFI);
                }
            }
            else if (interface == NETWORK_INTERFACE_ETH)
            {
                if (network_is_ppp_got_ip())
                {
                    DEBUG_INFO("Broker's disconnected, change to PPP\r\n");
                    network_change_interface(NETWORK_INTERFACE_PPP);
                } 
                else if (network_is_wifi_got_ip())
                {
                    DEBUG_INFO("Broker's disconnected, change to WIFI\r\n");
                    network_change_interface(NETWORK_INTERFACE_WIFI);
                }
            }
            else if (interface == NETWORK_INTERFACE_WIFI)
            {
                if (network_is_ppp_got_ip())
                {
                    DEBUG_INFO("Broker's disconnected, change to PPP\r\n");
                    network_change_interface(NETWORK_INTERFACE_PPP);
                }
                else if (network_is_eth_got_ip())
                {
                    DEBUG_INFO("Broker's disconnected, change to ETH\r\n");
                    network_change_interface(NETWORK_INTERFACE_ETH);
                }
            }
        }

        /** Trạng thái các LED tương ứng từng mạng: Chưa connect server: Nháy 1s, connect: sáng đứng
            - LED1 BLUE: PPP
            - LED1 RED: WIFI
            - LED2 RED: ETH
        */
        switch (interface)
        {
        case NETWORK_INTERFACE_UNKNOWN:
        case NETWORK_INTERFACE_PPP:
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                if (!app_io_is_led_4g_on())
                    app_io_control_led_4g_state(1);
            }
            else
            {
                led_stream_state ^= 1;
                app_io_control_led_4g_state(led_stream_state);
            }
            break;

        case NETWORK_INTERFACE_ETH:
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                if (!app_io_is_led_eth_on())
                    app_io_control_led_eth_state(1);
            }
            else
            {
                led_stream_state ^= 1;
                app_io_control_led_eth_state(led_stream_state);
            }
            break;

        case NETWORK_INTERFACE_WIFI:
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                if (!app_io_is_led_wifi_on())
                    app_io_control_led_wifi_state(1);
            }
            else
            {
                led_stream_state ^= 1;
                app_io_control_led_wifi_state(led_stream_state);
            }
            break;
        }

        if (m_max_delay_time_wait_for_stream_from_prepare)
        {
            m_max_delay_time_wait_for_stream_from_prepare--;
            if (slave_get_total_streaming_received())
            {
                m_max_delay_time_wait_for_stream_from_prepare = 0;
            } 
            else if (m_max_delay_time_wait_for_stream_from_prepare == 0 
                    && ((app_io_get_i2c_exp_value()->BitName.ISO_OUT1 == APP_IO_OPTO_ON)
                        || (app_io_get_i2c_exp_value()->BitName.ISO_OUT2 == APP_IO_OPTO_ON)))
            {
                slave_process_stop_onair();
            }
        }

        // 19/12/20: Delay turn on relay when start stream
        if (m_delay_turn_on_relay_prepair_stream > 0)
        {
            m_delay_turn_on_relay_prepair_stream--;
            if (m_delay_turn_on_relay_prepair_stream % 5 == 0)
            {
                DEBUG_INFO("Delay turn on Relay2: %d\r\n", m_delay_turn_on_relay_prepair_stream);
            }

            if (m_delay_turn_on_relay_prepair_stream == 0)
            {
                // mqtt_publish_message("DBG", "opto_output2(): ON");
                m_mqtt_send_heartbeat_counter = HEARTBEAT_INTERVAL;
                
                if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                {
                    app_io_control_opto_output2(APP_IO_OPTO_ON);
                }
            }
            // O mach class D thi chan opto dung de mute
            // ISO_OUT1 = 1 =>> Enable speaker detect for class D
            // ISO_OUT2 = 1 -> Mute class D
            if (app_flash_speaker_audio_class_get() == APP_FLASH_AUDIO_CLASS_D)
            {
                app_io_control_opto_output2(APP_IO_OPTO_OFF);
            }

        }
        // Quy trinh turn off relay: Delay2 -> turn off relay2 -> delay1 -> turn off relay1
        // Delay turn off relay1
        if (m_delay_turn_off_relay1_stop_stream > 0)
        {
            m_delay_turn_off_relay1_stop_stream--;
            if (m_delay_turn_off_relay1_stop_stream % 10 == 0)
            {
                DEBUG_VERBOSE("Delay turn off Relay1: %d\r\n", m_delay_turn_off_relay1_stop_stream);
            }

            if (m_delay_turn_off_relay1_stop_stream == 0)
            {
                if (app_io_get_i2c_exp_value()->BitName.ISO_OUT1 == APP_IO_OPTO_OFF)
                {
                    DEBUG_VERBOSE("Opto 1 already off\r\n");
                }
                else
                {
                    DEBUG_VERBOSE("Turn off opto1\r\n");
                }
                app_io_control_opto_output1(APP_IO_OPTO_OFF);
                m_mqtt_send_heartbeat_counter = HEARTBEAT_INTERVAL;        // Send info luon
            }
        }
        // Delay turn off relay2
        if (m_delay_turn_off_relay2_stop_stream > 0)
        {
            m_delay_turn_off_relay2_stop_stream--;
            if (m_delay_turn_off_relay2_stop_stream % 5 == 0)
            {
                DEBUG_VERBOSE("Delay turn off Relay2: %d\r\n", m_delay_turn_off_relay2_stop_stream);
            }

            if (m_delay_turn_off_relay2_stop_stream == 0)
            {
                if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                {
                    app_io_control_opto_output2(APP_IO_OPTO_OFF);
                }
                m_delay_turn_off_relay1_stop_stream = app_flash_get_relay1_turn_off_delay();
                m_mqtt_send_heartbeat_counter = HEARTBEAT_INTERVAL;        // Send info luon
            }
            // O mach class D thi chan opto dung de mute
            // ISO_OUT1 = 1 =>> Enable speaker detect for class D
            // ISO_OUT2 = 1 -> Mute class D
            if (app_flash_speaker_audio_class_get() == APP_FLASH_AUDIO_CLASS_D)
            {
                app_io_control_opto_output1(APP_IO_OPTO_ON);
                app_io_control_opto_output2(APP_IO_OPTO_ON);
            }
        }

#if 0
		/* Set lại volume nếu không đúng so với mức cấu hình, chỉ áp dụng khi codec ở chế độ DECODE, 
		* k xét chế độ LINE/MIC vì chế độ này âm lượng thay đổi theo volume ngoài
		*/
		if(++task_tick3s >= 3) {
			task_tick3s = 0;
			if (app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_DECODE) {
				if(abs(app_flash_get_volume() - app_audio_get_current_output_vol()) > 3) {
					app_audio_change_codec_vol(app_flash_get_volume());
				}
			}
		}
#endif

        /* Timeout nhận dữ liệu http khi stream -> chữa bệnh hiển thị "INET" khi master đã stop stream */
        if (m_http_received_data_timeout > 0)
        {
            m_http_received_data_timeout--;
            if (m_http_received_data_timeout == 0)
            {
                m_timeout_turn_off_opto_output = 15;
                // mqtt_publish_message("DBG", "http windows RX = 0");
                DEBUG_INFO("--> windows = 0, auto stop opto in = 15s\r\n");
            }
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    DEBUG_INFO("\t\t--- main_manager_task is exit ---");
    vTaskDelete(NULL);
}

void slave_terminate_stream(void)
{
    app_audio_pause();
    app_audio_complete_terminate();
}

void slave_restart_stream(char *url)
{
    app_audio_restart_pipeline(app_audio_get_stream_url());
    app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RUNNING);
}

static void build_http_stream_url(uint8_t master_index)
{
    if (master_index >= APP_FLASH_MASTER_TINH1 && master_index <= APP_FLASH_MASTER_XA3)
    {
        if (strlen(app_flash_get_master(master_index)) >= 15)
        {
            sprintf(app_audio_get_stream_url(), 
                    "%s%s", 
                    app_flash_get_http_stream_header(), 
                    app_flash_get_master(master_index));
        }
    }
}

static bool v1_is_higher_priority_master(uint8_t new_master, uint8_t current_master)
{
    if (new_master < current_master)
        return true;
    return false;
}



bool stream_if_higher_level(int command_master_level, bool force_new_stream)
{
    /* 1. ================= Nếu đang Streaming ================= //
    * => Xét mức ưu tiên của master, nếu master cao hơn hiện tại -> chuyển luồng stream sang higher master
    */

    if (command_master_level != -1)
    {
        if (stream_ele_info())
        {
            // Nhận lệnh từ master cao hơn -> Stop stream master hiện tại, chuyển link sang master cao hơn
            if (force_new_stream || v1_is_higher_priority_master(command_master_level, 
                                                                app_flash_get_current_streaming_master_index()))
            {
                DEBUG_INFO("Start streaming by higher level master: %u\r\n", command_master_level);
                m_delay_for_http_server_have_enough_buffer = 1;

                if (slave_get_received_running_state_timeout() == 0)
                {
                    DEBUG_INFO("Start counting running state timeout...\r\n");
                    /* Sau 45s mà không start stream được thì reset cho nhanh */
                    slave_set_received_running_state_timeout(45);
                }
                slave_reset_stream_monitor_data();

                /* Dừng tất cả các chế độ khác -> chuyển sang chế độ codec decoder */
                app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                // Switch Relay to Codec output
                app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);


                if (app_io_is_pa_off())
                {
                    app_io_control_pa(APP_IO_PA_ON);
                }

                if (force_new_stream == false)
                {
                    mqtt_publish_message("DBG", "Start streaming by higher master %d : %s", 
                                            command_master_level, app_audio_get_alt_stream_url(command_master_level));
                    /* 2. Save the current Master */
                    app_flash_set_current_streaming_master(command_master_level);
                    app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, command_master_level);
                    build_http_stream_url(command_master_level);
                    app_flash_set_last_streaming_url(app_audio_get_stream_url());
                }
                else
                {
                    // mqtt_publish_message("DBG", "Start streaming directly by URL");
                }
                
                app_io_opto_control_all(APP_IO_OPTO_ON);
                app_audio_simple_terminate_pipeline();
                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_STOPPED);

                // Check if HttpElement is stopped -> try to restart new stream...
                m_is_waiting_http_element_stopped = true;
                slave_reset_counter_wait_for_http_element_stop();

                /* Thời gian chờ chuyển sang higher master streaming, bao gồm cả 5s delay sau khi stop_http_element
                    * Trong thời gian nếu nhận được lệnh Stream của master khác thì không thực thi!
                    */
                slave_set_timeout_number_of_waiting_master_streaming(30);
                /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false); 
                slave_set_timeout_when_received_stream_running_command(80);
            }
            else
            {
                if (command_master_level != app_flash_get_current_streaming_master_index())
                {
                    DEBUG_WARN("Lower master: %u <= %u, %s. Ignore\r\n", 
                                command_master_level, 
                                app_flash_get_current_streaming_master_index(),
                                app_audio_get_alt_stream_url(command_master_level));
                }
                else
                {
                    slave_set_timeout_when_received_stream_running_command(80);
                }
            }
            return true;
        }
        else
        {
            DEBUG_INFO("Already streaming %d\r\n", stream_ele_info());
            slave_set_timeout_when_received_stream_running_command(80);
        }
    }
    else        // stream by url
    {
        if (slave_get_received_running_state_timeout() == 0)
        {
            DEBUG_INFO("Start counting running state timeout...\r\n");
            /* Sau 45s mà không start stream được thì reset cho nhanh */
            slave_set_received_running_state_timeout(45);
        }
        slave_reset_stream_monitor_data();

        /* Dừng tất cả các chế độ khác -> chuyển sang chế độ codec decoder */
        app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

        // Switch Relay to Codec output
        app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

        // Nếu PA đang OFF thì ON
        if (app_io_is_pa_off())
        {
            app_io_control_pa(APP_IO_PA_ON);
        }

        // mqtt_publish_message("DBG", "app_io_opto_control_all(): Start streaming directly by URL");
        slave_set_timeout_when_received_stream_running_command(80);
        app_io_opto_control_all(APP_IO_OPTO_ON);

        app_audio_simple_terminate_pipeline();
        app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_STOPPED);

        // Check if HttpElement is stopped -> try to restart new stream...
        m_is_waiting_http_element_stopped = true;
        slave_reset_counter_wait_for_http_element_stop();

        /* Thời gian chờ chuyển sang higher master streaming, bao gồm cả 5s delay sau khi stop_http_element
            * Trong thời gian nếu nhận được lệnh Stream của master khác thì không thực thi!
            */
        slave_set_timeout_number_of_waiting_master_streaming(30);
        slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false); /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
    }
    return false;
}


void ttn_on_device_mode_change(char *value)
{
    if (!value)
    {
        return;
    }

    DEBUG_INFO("Set MODE: %s\r\n", value);
    char mode[32];
    sprintf(mode, "MODE(%s)", value);
    change_mode_cmd(mode);
    // mqtt_queue_stream_event_t tx_event;
    // tx_event.need_free = true;
    // tx_event.payload = calloc(strlen(configuration_data)+1, 1);
    // tx_event.topic = calloc(strlen(topic)+1, 1);
    // tx_event.need_free = true;
    // if (tx_event.payload == NULL || tx_event.topic == NULL)
    // {
    //     esp_restart();
    // }
    // sprintf(tx_event.topic, "%s", topic);
    // sprintf(tx_event.payload, "%s", configuration_data);
    // mqtt_queue_streaming_put(&tx_event);
}


static void process_stream_with_high_priority(char *stream_url)
{
    char *p = strtok(stream_url, ")");
    if (strcmp(stream_url, app_audio_get_stream_url()) == 0 && stream_ele_info())
    {
        DEBUG_WARN("Same URL\r\n");
        slave_set_timeout_when_received_stream_running_command(80);
        // mqtt_publish_message("DBG", "Same url %s", stream_url);
    }
    else
    {
        mqtt_publish_message("DBG", "Stream directly by url %s - %s", stream_url, app_audio_get_stream_url());
        strcpy(app_audio_get_stream_url(), p);
        app_flash_set_last_streaming_url(p);
        stream_if_higher_level(-1, true);
        slave_set_http_finish_track_timeout(10);
        m_stream_emergency = true;
    }
}

void on_cfg_fm_freq(int index, uint32_t frequency)
{
    switch (index)
    {
        case 0:
        {
            DEBUG_INFO("Set FM_FREQ1: %u\r\n", frequency);
            app_flash_set_fm_freq1(frequency); /* KHz */
            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ1_KEY, frequency);
        }
            break;

        case 1:
        {
            DEBUG_INFO("Set FM_FREQ2: %u\r\n", frequency);
            app_flash_set_fm_freq2(frequency); /* KHz */
            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ2_KEY, frequency);
        }
            break;

        case 2:
        {
            DEBUG_INFO("Set FM_FREQ3: %u\r\n", frequency);
            app_flash_set_fm_freq3(frequency); /* KHz */
            // Write to NVS
            app_flash_node_nvs_write_u32(APP_FLASH_FM_FREQ3_KEY, frequency);  
            
        }
            break;
        default:
            break;
    }
}


void change_mode_cmd(char *op_mode)
{
    uint8_t mode = GetNumberFromString(5, op_mode);
    if (mode >= APP_AUDIO_OPERATION_MODE_INTERNET && mode <= APP_AUDIO_OPERATION_MODE_NO_OPERATION)
    {
        /* Xử lý chuyển audio mode */
        if (mode == APP_AUDIO_OPERATION_MODE_INTERNET)
        {
            app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_IDLE);
            app_flash_set_operate_mode(mode); /* 1 - 4 */
            // Write to NVS
            app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);

            // Change audio codec mode
            app_audio_change_codec_to_internet_mode();

            // Tạm thời tắt PA luôn, nếu sau có lệnh stream thì tự bật
            app_io_control_pa(APP_IO_PA_OFF);

            // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
            DEBUG_INFO("app_io_opto_control_all OFF...\r\n");
            app_io_opto_control_all(APP_IO_OPTO_OFF);

            // Khi chuyển sang mode INTERNET thì FM_BUSY
            app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_IDLE);

            // Chuyển sang Internet -> Gửi phản hồi mạch FM vài lần để về Idle
            app_io_set_number_of_retries_counter(3);

            if (app_audio_get_streaming_logic_step() == APP_AUDIO_STREAM_STOPPED)
            {
                app_audio_set_stream_retry(1);
                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RESTART);
                build_http_stream_url(app_flash_get_current_streaming_master_index());
                // mqtt_publish_message("DBG", "Mode change, start stream");
                slave_set_timeout_when_received_stream_running_command(80);
            }
        }
        else if (mode == APP_AUDIO_OPERATION_MODE_MIC || mode == APP_AUDIO_OPERATION_MODE_LINE)
        {
            /* Nếu đang không stream internet hoặc phát FM thì mới chuyển sang MIC/LINE IN */
            if (!stream_ele_info())
            {
                DEBUG_INFO("Switch to MIC/LINE mode\r\n");

                app_flash_set_operate_mode(mode);   /* 1 - 5 */
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY); // Không cho thay đổi Volume từ bên module FM

                // Write to NVS
                app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);

                /** Change codec mode and input/output relay */
                if (mode == APP_AUDIO_OPERATION_MODE_MIC)
                {
                    /* Change mode to MIC, PA ON */
                    app_audio_change_to_local_mic_mode();
                }
                else if (mode == APP_AUDIO_OPERATION_MODE_LINE)
                {
                    /* Change mode to LINE IN */
                    app_audio_change_to_local_line_in_mode();
                }

                // Bật 2 Relay ngoài
                mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to MIC/LINE mode");
                if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                {
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
                else
                {
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }
            }
            else
            {
                DEBUG_INFO("HTTP is running, can't run MIC/LINE mode!\r\n");
            }
        }
        else if (mode == APP_AUDIO_OPERATION_MODE_FM)
        {
            // Lấy tần số kênh FM được chọn
            char *fm_channel_selected = strstr(op_mode, "FM_FREQ(");
            uint32_t cur_fm_channel = 3;        // default value

            if (fm_channel_selected != NULL)
            {
                cur_fm_channel = GetNumberFromString(8, fm_channel_selected);
            }

            /* Nếu đang không stream internet thì mới cho chạy FM */
            if (!stream_ele_info())
            {

                if (cur_fm_channel >= 1 && cur_fm_channel <= 3)
                {
                    if (cur_fm_channel == 1)
                        app_io_set_current_fm_freq(app_flash_get_fm_freq1());
                    if (cur_fm_channel == 2)
                        app_io_set_current_fm_freq(app_flash_get_fm_freq2());
                    if (cur_fm_channel == 3)
                        app_io_set_current_fm_freq(app_flash_get_fm_freq3());
                }
            
                /** Change to FM mode, PA ON */
                app_audio_change_to_local_fm_mode();

                if (app_flash_need_turn_on_relay_in_fm_mode() == false)
                {
                    // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                    DEBUG_INFO("app_io_opto_control_all OFF...\r\n");
                    mqtt_publish_message("DBG", "Turn off all opto in FM mode");
                    // app_io_opto_control_all(APP_IO_OPTO_OFF);
                    if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                    {
                        app_io_opto_control_all(APP_IO_OPTO_OFF);
                    }
                    else
                    {
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                    }
                }
                else
                {
                    mqtt_publish_message("DBG", "Turn on all opto in FM mode");
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }


                app_flash_set_operate_mode(mode); /* 1 - 5 */
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                // Write mode to NVS
                app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);

                // Write current freq to NVS
                app_flash_node_nvs_write_u32(APP_FLASH_CURRENT_FREQ_KEY, app_io_get_current_fm_freq());
            }
        }
        else if (mode == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
        {
            DEBUG_INFO("Stop audio by set 'NO_OPERATION'\r\n");
            app_flash_set_operate_mode(mode); /* 1 - 4 */
            // Write to NVS
            app_flash_write_u8(APP_FLASH_OPERATE_MODE_KEY, mode);
            slave_terminate_stream();
            // app_audio_pause();
            // app_audio_complete_terminate();

            app_audio_set_http_stream_stopped_flag();

            // Change codec to INTERNET mode
            app_audio_change_codec_to_internet_mode();

            // Turn off PA
            DEBUG_INFO("app_io_opto_control_all OFF...\r\n");
            app_io_control_pa(APP_IO_PA_OFF);

            // Tắt Relay (nếu được bật ở chế độ LINE IN)
            if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
            {
                app_io_opto_control_all(APP_IO_OPTO_OFF);
            }
            else
            {
                app_io_opto_control_all(APP_IO_OPTO_ON);
            }
        }
    }
}

/******************************************************************************************/
/**
 * @brief   : Thực hiện đóng relay theo thứ tự, thời gian delay đồng bộ theo delay của master
 * @param   :   STREAM_PREPAIR(delayTime)
 * @retval  : None
 * @author  :
 * @created : 19/12/20
 */
void slave_process_prepair_onair(char *prepaire_msg)
{
    // // Lấy thời gian Prepair delay từ master (nếu có)
    // char *prepaire_msg = strstr(msg, "STREAM_PREPAIR");
    uint32_t delay = app_flash_get_delay_turn_on_relay();
    if (prepaire_msg != NULL)
    {
        char delay_str[10] = {0};
        if (strstr(prepaire_msg, "("))
        {
            if (CopyParameter(prepaire_msg, delay_str, '(', ')'))
            {
                delay = (uint8_t)GetNumberFromString(0, delay_str);
                slave_set_delay_turn_on_relay_prepare_stream(delay);
                DEBUG_INFO("Prepair delay from master = %d\r\n", slave_get_delay_turn_on_relay_prepare_stream());
            }
            else
            {
                slave_set_delay_turn_on_relay_prepare_stream(delay);
                DEBUG_INFO("Not found prepair delay fom master, use default = %d\r\n", 
                            slave_get_delay_turn_on_relay_prepare_stream());
            }
        }
        else
        {
            slave_set_delay_turn_on_relay_prepare_stream(delay);
        }

    }

    // Thêm điều khiển ON relay theo thứ tự relay1 -> delay -> relay2
    if (!app_audio_is_http_audio_stream_running())
    {
        // mqtt_publish_message("DBG", "Stream prepare IO, delay on %us", delay);
    }

    // neu o mach Class D thi ko dieu khien opto 1 va 2
    if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
    {
        app_io_control_opto_output1(APP_IO_OPTO_ON);
    }
    else
    {
        app_io_control_opto_output1(APP_IO_OPTO_OFF);
    }
}

void slave_resub_all_master()
{
    m_mqtt_fast_subscribe_timeout = 10;
}

static bool ttn_is_higher_priority_master(char *new_master, char *current_master)
{
    if (app_flash_find_group_priority(new_master) > app_flash_find_group_priority(current_master))
        return true;
    return false;
}


void atth_mqtt_process_msq_id(cJSON* root, cJSON* message, uint32_t id)
{
    bool send_cfg = false;
    switch (id)
    {
        case MSG_ID_DEVICE_CONFIG:
        {
            // {
            //     "Id" : 100,
            //     "Time" : 1683535716731,
            //     "Sender" : "Server",
            //     "Message" : {
            //         "Id" : "1683535716731",
            //         "Name" : "HuyTV test thiết bị phần cứng V1",
            //         "DeviceType" : 0,
            //         "Groups" : [ {
            //         "Id" : "255ff21f59da4168809fe0b69ba76f0f",
            //         "MasterType" : 0,
            //         "Priority" : 1,
            //         "Name" : "Nhóm test HuyTV- Cấp Phường"
            //         }, {
            //         "Id" : "409f772c98734fd08a8500ecb89a31b4",
            //         "MasterType" : 0,
            //         "Priority" : 2,
            //         "Name" : "Nhóm RTV2 Cấp Tỉnh - TP"
            //         }, {
            //         "Id" : "638164149986247400",
            //         "MasterType" : 0,
            //         "Priority" : 3,
            //         "Name" : "Nhóm demo TTN Cấp Tỉnh"
            //         } ]
            //     }
            // }
            DEBUG_INFO("Device config msg\r\n");
            cJSON *device_name, *groups, *group_conf;

            device_name = cJSON_GetObjectItem(message, "Name");
            if (device_name)
            {
                DEBUG_INFO("Device name %s\r\n", device_name->valuestring);
                app_flash_set_device_name(device_name->valuestring);
            }

        
            groups = cJSON_GetObjectItem(message, "Groups");
            if (groups && cJSON_IsArray(groups))
            {
                DEBUG_INFO("Group config size %d\r\n", cJSON_GetArraySize(groups));
                app_flash_remove_all_group();
                cJSON_ArrayForEach(group_conf, groups)
                {
                    cJSON *id, *master_type, *priority;
                    id = cJSON_GetObjectItem(group_conf, "Id");
                    master_type = cJSON_GetObjectItem(group_conf, "MasterType");
                    priority = cJSON_GetObjectItem(group_conf, "Priority");
                    if (id && master_type && priority)
                    {
                        DEBUG_INFO("Master %s, type %d, priority %d\r\n", 
                                    id->valuestring, 
                                    master_type->valueint, 
                                    priority->valueint);
                        app_flash_set_group_name(id->valuestring, priority->valueint);
                    }
                }
            }
            send_cfg = true;
            slave_resub_all_master();
        }
            break;

        case MSG_ID_OTA_APP:
        {
            cJSON *download_url;
            download_url = cJSON_GetObjectItem(message, "url");
            if (!app_ota_is_running() && download_url)
            {
                DEBUG_WARN("Update firmware on %s\r\n", download_url->valuestring);
                static app_ota_info_t ota_url;
                static char url[128];
                sprintf(url, "%s", download_url->valuestring);
                ota_url.url = url;
                ota_url.type = APP_OTA_DEVICE_ESP32;
                xTaskCreate(app_ota_download_task, "ota_tsk", 8192, (void *)&ota_url, 5, NULL);
                vTaskDelay(100);    // Delay for OTA task start
            }
        }
            break;

        case MSG_ID_GET_CFG:
            app_mqtt_send_all_config_after_reset();
            break;
            
        case MSG_ID_SET_CFG:
        {
            // {
            // "Id" : 1,
            // "Time" : 1685958423328,
            // "Sender" : "Server",
            // "Message" : {
            //     "Cmd" : null,
            //     "Obj" : [ {
            //     "C" : 0,
            //     "V" : "SET,VOL"
            //     } ]
            // }
            // }
            DEBUG_INFO("Set config msg\r\n");
            cJSON *c_cmd, *c_obj, *c_value;
            char *set_msg = NULL;
            c_value = cJSON_GetObjectItem(message, "V");
            c_obj = cJSON_GetObjectItem(message, "Obj");
            int cfg_count = 0;
            
            if (c_value && cJSON_IsString(c_value))
            {
                set_msg = strstr(c_value->valuestring, "SET,");
            }
            else if (c_obj && cJSON_IsArray(c_obj))       // check object
            {
                cJSON *ele;
                cJSON_ArrayForEach(ele, c_obj)
                {
                    int new_count = 0;
                    c_cmd = cJSON_GetObjectItem(ele, "C");
                    c_value = cJSON_GetObjectItem(ele, "V");
                    if (c_value && cJSON_IsString(c_value))
                    {
                        set_msg = strstr(c_value->valuestring, "SET,");
                    }
                    if (!set_msg && c_cmd && cJSON_IsNumber(c_cmd) && c_value)
                    {
                        switch (c_cmd->valueint)
                        {
                            case CONFIG_MEET_VOLUME_PERCENT:
                            {
                                // Set vol
                                uint8_t vol = GetNumberFromString(0, c_value->valuestring);
                                if (vol > 100)
                                {
                                    vol = 100;
                                }
                                DEBUG_INFO("Set VOLUME: %u\r\n", vol);
                                app_flash_set_volume(vol); /* 0 - 100 */

                                // Write to NVS
                                app_flash_write_u8(APP_FLASH_VOLUME_KEY, vol);
                                app_audio_change_codec_vol(app_flash_get_volume());
                                send_cfg = true;
                            }
                                break;
                            case CONFIG_VS_SPEAKER_ADB_TCPIP:
                            {
                                if (c_value && cJSON_IsTrue(c_value))
                                {
                                    app_flash_tcp_console_enable();
                                }
                                else if (c_value && cJSON_IsFalse(c_value))
                                {
                                    app_flash_tcp_console_disable();
                                }
                                send_cfg = true;
                            }
                                break;

                            case CONFIG_VS_SPEAKER_SPK_DETECT:
                            {
                                DEBUG_INFO("Set spk class: %u\r\n", c_value->valueint);
                                if (c_value && cJSON_IsNumber(c_value))
                                {
                                    app_flash_write_u8(APP_FLASH_SPK_CLASS, c_value->valueint);
                                    app_flash_speaker_audio_class_set(c_value->valueint);
                                }
                                send_cfg = true;
                            }
                                break;

                            case CONFIG_VS_SPEAKER_WORKING_MODE:
                            {         
                                if (c_value && cJSON_IsString(c_value))
                                {
                                    ttn_on_device_mode_change(c_value->valuestring);
                                    slave_set_mqtt_state_timeout(58);
                                }
                                else
                                {
                                    new_count = 0;
                                }
                            }
                                break;
                            case CONFIG_CONNECT_NEW_WIFI:
                            case CONFIG_CONNECT_WIFI:
                            {
                                if (c_value && cJSON_IsString(c_value))
                                {
                                    // WIFINAME,PASSWORD
                                    char *delim = strstr(c_value->valuestring, ",");
                                    char wifiname[64], wifipass[64];
                                    memset(wifiname, 0, sizeof(wifiname));
                                    memset(wifipass, 0, sizeof(wifipass));
                                    if (delim)
                                    {
                                        delim++;
                                        if (delim)
                                        {
                                            memcpy(wifiname, c_value->valuestring, delim-c_value->valuestring-1);
                                            strcpy(wifipass, delim);
                                            DEBUG_INFO("Wifi name %s, password %s\r\n", wifiname, wifipass);
                                            if (strlen(wifiname) && strlen(wifipass))
                                            {
                                                app_flash_write_wifi_info(wifiname, wifipass, 1);
                                            }
                                            else
                                            {
                                                app_flash_write_wifi_info(wifiname, wifipass, 0);
                                            }
                                            vTaskDelay(500);
                                            esp_restart();
                                        }
                                    }
                                }
                                send_cfg = true;
                            }
                                break;
                            case CONFIG_DELETE_WIFI_NAME:
                                app_flash_write_wifi_info("", "", 0);
                                break;
                            case CONFIG_RESTART_APP:
                            case CONFIG_SAFE_REBOOT:
                            case CONFIG_SAFE_SHUTDOWN: 
                            case CONFIG_EMERGENCY_REBOOT: 
                            case CONFIG_EMERGENCY_SHUTDOWN:
                                DEBUG_WARN("--- Reset System ---\r\n");
                                mqtt_publish_message("DBG", "Reset by reboot cmd");
                                vTaskDelay(1000);
                                system_software_reset(SW_RESET_REASON_REBOOT_CMD);
                                break; 

                            case CONFIG_VS_SPEAKER_FM_FREQ1:
                                on_cfg_fm_freq(0, GetNumberFromString(0, c_value->valuestring));
                                send_cfg = true;
                                break;
                            case CONFIG_VS_SPEAKER_FM_FREQ2:
                                on_cfg_fm_freq(1, GetNumberFromString(0, c_value->valuestring));
                                send_cfg = true;
                                break;
                            case CONFIG_VS_SPEAKER_FM_FREQ3:
                                on_cfg_fm_freq(2, GetNumberFromString(0, c_value->valuestring));
                                send_cfg = true;
                                break;
                            case CONFIG_MQTT_SRV:
                            {
                                DEBUG_INFO("Config mqtt broker\r\n");
                                cJSON *c_mqtt_server = cJSON_GetObjectItem(c_value, "url");
                                cJSON *c_mqtt_user = cJSON_GetObjectItem(c_value, "usr");
                                cJSON *c_mqtt_pass = cJSON_GetObjectItem(c_value, "pw");
                                cJSON *c_mqtt_port = cJSON_GetObjectItem(c_value, "port");
                                if (c_mqtt_server && c_mqtt_user && c_mqtt_pass && c_mqtt_port
                                    && cJSON_IsString(c_mqtt_server)
                                    && cJSON_IsString(c_mqtt_user)
                                    && cJSON_IsString(c_mqtt_pass)
                                    && cJSON_IsNumber(c_mqtt_port))
                                {
                                    // (mqtt://smart.radiotech.vn:1883),MQTT_USERNAME(village-speaker),MQTT_PASSWORD(vs.bytech@2019)
                                    char url[APP_FLASH_MQTT_URL_SIZE];
                                    if (strstr(c_mqtt_server->valuestring, "mqtt://")
                                        || strstr(c_mqtt_server->valuestring, "mqtts://"))
                                    {
                                        sprintf(url, "%s:%u", c_mqtt_server->valuestring, c_mqtt_port->valueint);
                                    }
                                    else if (strstr(c_mqtt_server->valuestring, "tcp://"))
                                    {
                                        sprintf(url, "mqtt://%s:%u", c_mqtt_server->valuestring+6, c_mqtt_port->valueint);
                                    }
                                    else
                                    {
                                        sprintf(url, "%s:%u", c_mqtt_server->valuestring, c_mqtt_port->valueint);
                                    }


                                    if (strcmp(app_flash_get_mqtt_server_url(), url) != 0)
                                    {
                                        snprintf(app_flash_get_mqtt_server_url(), APP_FLASH_MQTT_URL_SIZE, "%s", url);

                                        DEBUG_INFO("New mqtt url: %s", app_flash_get_mqtt_server_url());
                                        // Save config
                                        app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_URL, app_flash_get_mqtt_server_url());
                                    }
                                    else
                                    {
                                        DEBUG_INFO("Same broker\r\n");
                                    }

                                    if (strcmp(app_flash_get_mqtt_server_username(), c_mqtt_user->valuestring) != 0)
                                    {
                                        sprintf(app_flash_get_mqtt_server_username(), "%s", c_mqtt_user->valuestring);

                                        DEBUG_INFO("New mqtt username: %s", app_flash_get_mqtt_server_username());

                                        // Save config
                                        app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_USERNAME, app_flash_get_mqtt_server_username());
                                    }
                                    else
                                    {
                                        DEBUG_INFO("Same username\r\n");
                                    }

                                    if (strcmp(app_flash_get_mqtt_server_password(), c_mqtt_pass->valuestring) != 0)
                                    {
                                        sprintf(app_flash_get_mqtt_server_password(), "%s", c_mqtt_pass->valuestring);

                                        DEBUG_INFO("New mqtt password: %s", app_flash_get_mqtt_server_password());

                                        // Save config
                                        app_flash_node_nvs_write_string(APP_FLASH_KEY_MQTT_SERVER_PASSWORD, app_flash_get_mqtt_server_password());
                                    }
                                    else
                                    {
                                        DEBUG_INFO("Same password\r\n");
                                    }
                                    send_cfg = true;
                                }
                            }
                                break;
                            default:
                                new_count = 0;
                                break;
                        }
                        cfg_count += new_count;
                    }
                }
            }

            if (set_msg)
            {
                DEBUG_INFO("Process set message %s\r\n", set_msg);
                while (set_msg)
                {
                    uint32_t set_id = GetNumberFromString(4, set_msg);
                    char tmp_buf[APP_FLASH_MAX_GROUP_NAME_LEN];
                    memset(tmp_buf, 0, APP_FLASH_MAX_GROUP_NAME_LEN);
                    CopyParameter(set_msg, tmp_buf, '(', ')');
                    cfg_count++;

                    switch (set_id)
                    {
                        case CONFIG_REMOVE_MEET_GROUP_ID:
                        {
                            app_flash_remove_group(tmp_buf);
                            send_cfg = true;
                        }
                            break;
                        case CONFIG_VS_SPEAKER_WORKING_MODE:
                        {
                            ttn_on_device_mode_change(tmp_buf);
                        }
                            break;
                        
                        case CONFIG_VS_SPEAKER_FM_FREQ1:
                        {
                            uint32_t freq = GetNumberFromString(0, tmp_buf);
                            if (freq > 0)
                            {
                                on_cfg_fm_freq(0, freq);
                            }
                        }
                            break;

                        case CONFIG_VS_SPEAKER_FM_FREQ2:
                        {
                            uint32_t freq = GetNumberFromString(0, tmp_buf);
                            if (freq > 0)
                            {
                                on_cfg_fm_freq(1, freq);
                            }
                        }
                            break;

                        case CONFIG_VS_SPEAKER_FM_FREQ3:
                        {
                            uint32_t freq = GetNumberFromString(0, tmp_buf);
                            if (freq > 0)
                            {
                                on_cfg_fm_freq(2, freq);
                            }
                        }
                            break;

                        case CONFIG_VS_SPEAKER_SPK_DETECT:
                        {
                            DEBUG_INFO("Set spk detect value %s\r\n", tmp_buf);
                            if (tmp_buf[0] == '0')
                            {
                                app_flash_speaker_audio_class_set(APP_FLASH_AUDIO_CLASS_NONE);
                            }
                            else if (tmp_buf[0] == '1')
                            {
                                app_flash_speaker_audio_class_set(APP_FLASH_AUDIO_CLASS_AB);
                            }
                            else if (tmp_buf[0] == '2')
                            {
                                app_flash_speaker_audio_class_set(APP_FLASH_AUDIO_CLASS_D);
                            }
                        }
                            break;
                        case CONFIG_VS_SPEAKER_ADB_TCPIP:
                            if (tmp_buf[0] == '0')
                            {
                                app_flash_tcp_console_disable();
                            }
                            else if (tmp_buf[0] == '1')
                            {
                                app_flash_tcp_console_enable();
                            }
                            break;
                        case CONFIG_VS_SPEAKER_RELAY_DELAY_ON:
                        {
                            uint32_t delay_time = GetNumberFromString(0, tmp_buf);
                            if (delay_time > 0 && delay_time <= APP_FLASH_RELAY_DELAY_MAX_TIME)
                            { /* 1 - 250 */
                                app_flash_set_delay_turn_on_relay(delay_time);
                                // Write to NVS
                                app_flash_write_u8(APP_FLASH_RELAY_DELAY_ON_KEY, delay_time);
                                DEBUG_INFO("Delay on %us\r\n", delay_time);
                            }
                        }
                            break;
                        case CONFIG_VS_SPEAKER_RELAY_DELAY_OFF1:
                        {
                            uint32_t delay_time = GetNumberFromString(0, tmp_buf);
                            if (delay_time > 0 && delay_time <= APP_FLASH_RELAY_DELAY_MAX_TIME)
                            { /* 1 - 250 */
                                app_flash_set_relay1_turn_off_delay(delay_time);
                                // Write to NVS
                                app_flash_write_u8(APP_FLASH_RELAY1_DELAY_OFF_KEY, delay_time);
                                DEBUG_INFO("Delay off1 %us\r\n", delay_time);
                                send_cfg = true;
                            }
                        }
                            break;
                        case CONFIG_VS_SPEAKER_RELAY_DELAY_OFF2:
                        {
                            uint32_t delay_time = GetNumberFromString(0, tmp_buf);
                            if (delay_time > 0 && delay_time <= APP_FLASH_RELAY_DELAY_MAX_TIME)
                            { /* 1 - 250 */
                                app_flash_set_relay2_turn_off_delay(delay_time);
                                // Write to NVS
                                app_flash_write_u8(APP_FLASH_RELAY2_DELAY_OFF_KEY, delay_time);
                                DEBUG_INFO("Delay off2 %us\r\n", delay_time);
                                send_cfg = true;
                            }
                        }
                            break;
                        case CONFIG_VS_SPEAKER_FM_WORKING_FREQ:
                        {
                            uint32_t channel = GetNumberFromString(0, tmp_buf);
                            if (channel >= 1 && channel <= 3)
                            {
                                if (channel == 1)
                                    app_io_set_current_fm_freq(app_flash_get_fm_freq1());
                                if (channel == 2)
                                    app_io_set_current_fm_freq(app_flash_get_fm_freq2());
                                if (channel == 3)
                                    app_io_set_current_fm_freq(app_flash_get_fm_freq3());
                                send_cfg = true;
                            }
                        }
                            break;
                        case CONFIG_VS_SPEAKER_WIFINAME:
                        case CONFIG_VS_SPEAKER_WIFIPASS:
                        case CONFIG_VS_SPEAKER_WIFI_ENABLE:
                            app_mqtt_write_wifi_config(tmp_buf);
                            send_cfg = true;
                            break;
                        case CONFIG_MQTT_SRV:
                        {

                        }
                            break;
                        default:
                            break;
                    }
                    set_msg += 4;
                    set_msg = strstr(set_msg, "SET,");
                }
            }
            if (cfg_count)
                app_mqtt_send_all_config_after_reset();
        }
            break;

        case MSG_ID_STREAM_COMMAND:
        {
            DEBUG_INFO("Stream cmd\r\n");
            static uint8_t notify_device_state = 0;
            /* Nếu đang OTA không nhận lệnh stream */
            if (app_ota_is_running())
                return;

            /* Nếu chế độ hoạt động không cho phép -> return */
            if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
            {
                DEBUG_INFO("Mode no operation\r\n");
                // Clear các trạng thái của pipeline để khi cho phép chế độ INTERNET trở lại thì STREAM được
                slave_set_http_finish_track_timeout(0);
                m_is_waiting_http_element_stopped = false;
                app_audio_set_stream_retry(0);
                if (notify_device_state % 10 == 0)
                {
                    app_mqtt_publish_ttn_debug_msg("Ignore stream, device in no operation mode\r\n");
                }
                notify_device_state++;
                return;
            }
            else
            {
                notify_device_state = 0;
            }

            cJSON *c_master, *c_cmd, *c_serial, *c_is_all, *c_info, *c_vol, *c_priority, *c_emergency, *c_type, *c_delay_turn_on_relay, *c_uri;
            c_master = cJSON_GetObjectItem(message, "Master");
            c_is_all = cJSON_GetObjectItem(message, "IsAll");
            c_cmd = cJSON_GetObjectItem(message, "CmdCode");
            c_info = cJSON_GetObjectItem(message, "Info");
            c_priority = cJSON_GetObjectItem(message, "P");
            c_emergency = cJSON_GetObjectItem(message, "E");
            c_type = cJSON_GetObjectItem(message, "T");
            c_delay_turn_on_relay = cJSON_GetObjectItem(message, "DelayTurnOnRelay");
            c_vol = cJSON_GetObjectItem(message, "Vol");
            c_serial = cJSON_GetObjectItem(message, "Serials");
            if (c_info && cJSON_IsObject(c_info))
            {
                c_uri = cJSON_GetObjectItem(c_info, "Uri");
            }
            else
            {
                c_uri = NULL;
            }
            
            if (c_master && c_is_all && c_info && c_priority && c_emergency 
                && c_type && c_vol && c_uri && c_cmd)
            {
                bool do_stream = false;
                DEBUG_INFO("Master [%d], serial %s, url %s\r\n", 
                            c_priority->valueint, c_master->valuestring, c_uri->valuestring);
                
                int master_priority = app_flash_find_group_priority(c_master->valuestring);
                if (master_priority == -1)
                {
                    DEBUG_WARN("Invalid master priority\r\n");
                }

                if (c_is_all && cJSON_IsTrue(c_is_all))
                {
                    do_stream = true;
                }
                else if (c_serial)
                {
                    cJSON *ele;
                    cJSON_ArrayForEach(ele, c_serial)
                    {
                        if (strstr(ele->valuestring, app_flash_get_imei()))
                        {
                            do_stream = true;
                            break;
                        }
                    }
                }

                if (do_stream && master_priority != -1)
                {
                    uint32_t delay_on_relay = app_flash_get_delay_turn_on_relay();
                    char delay_str[32]; // backward compatible
                    sprintf(delay_str, "STREAM_PREPAIR(%u)", delay_on_relay);
                    if (c_cmd->valueint == CMD_STREAM_PREPAIR)
                    {
                        if (c_delay_turn_on_relay)
                        {
                            delay_on_relay = c_delay_turn_on_relay->valueint;
                        }
                        sprintf(delay_str, "STREAM_PREPAIR(%u)", delay_on_relay);
                        slave_process_prepair_onair(delay_str);
                    }
                    else if (c_cmd->valueint == CMD_STREAM_START || c_cmd->valueint == CMD_STREAM_RUNNING)
                    {
                        // TODO check stream buffer
                        // if (c_cmd->valueint == CMD_STREAM_START)
                        // {
                        //     m_delay_for_icecast_have_enough_buffer = 5000;
                        // }
                        // else
                        // {
                        //     m_delay_for_icecast_have_enough_buffer = 3000;
                        // }

                        if (c_delay_turn_on_relay)
                        {
                            delay_on_relay = c_delay_turn_on_relay->valueint;
                            sprintf(delay_str, "STREAM_PREPAIR(%u)", delay_on_relay);
                        }
                        slave_process_prepair_onair(delay_str);

                        char *current_group_id = app_flash_get_current_streaming_group_id();
                        bool is_higher = ttn_is_higher_priority_master(c_master->valuestring, 
                                                    current_group_id);

                        // Todo cache master URL
                        // app_audio_set_alt_stream_url(c_master->valuestring, c_uri->valuestring);
                        if (stream_if_higher_level(is_higher, false))
                        {
                            if (c_vol->valueint > 0)
                                app_audio_change_codec_vol(c_vol->valueint);
                            return;
                        }
                        else
                        {
                            DEBUG_WARN("Lower stream %s\r\n", c_uri->valuestring);
                            //             /* 2. ================== Nếu đang Không Streaming -> Nhận lệnh từ tất cả các master ============== */
                            /* Nếu đang trong quá trình chuyển sang stream higher master -> không nhận lệnh nữa */
                            if (slave_get_number_of_second_timeout_waiting_master_streaming())
                            {
                                DEBUG_WARN("Starting from higher master stream, ignore cmd in: %us\r\n", 
                                            slave_get_number_of_second_timeout_waiting_master_streaming());
                                return;
                            }

                            if (slave_get_received_running_state_timeout() == 0)
                            {
                                DEBUG_WARN("Start counting running state timeout...\r\n");
                                slave_set_received_running_state_timeout(40); /* Sau 35s mà không start stream được thì reset cho nhanh */
                            }

                            app_audio_change_codec_vol(c_vol->valueint);
                            /* Đánh dấu dung lượng stream khi nhận lệnh -> so sánh sau 35 giây sau */
                            if (c_cmd->valueint == CMD_STREAM_START)
                            {
                                slave_reset_stream_monitor_data();
                            }
                            slave_update_last_stream_data();

                            // DEBUG
                            DEBUG_INFO("HTTP finish track timeout: %u: %u, Stream retries number: %u\r\n",
                                    slave_get_http_finish_track_timeout(), app_audio_get_reset_stream_retry_number());

                            /** Hiện tượng: Đang khởi động stream với Higher Master, http_element vừa closed, đang delay 5s chuẩn bị START_STREAM
                            * -> Các element vẫn chưa ở STATE_RUNNING -> nhận được lệnh APP_AUDIO_STREAM_RUNNING khác -> khởi tạo 1 STREAM mới -> xung đột
                            * gây treo task -> watchdog reset
                            * ==> Giải pháp: Nếu đang chờ chạy 1 luồng stream thì không khởi tạo luồng khác!
                            */
                            if (slave_get_http_finish_track_timeout() == 0 && /* Nếu nhận được sự kiện HTTP_FINISH_TRACK -> Đang chờ http_finished thì không START STREAM! */
                                !m_is_waiting_http_element_stopped &&        /* Nếu đang chờ http_element stopped sau khi lệnh terminate -> Không start STREAM */
                                app_audio_get_reset_stream_retry_number() == 0)        /* Đang retry start STREAM -> tạm thời chưa nhận lệnh STREAM */
                            {
                                if (!app_audio_is_i2s_running()
                                    || !app_audio_is_opus_running()
                                    || !app_audio_is_http_audio_stream_running())
                                {
                                    app_flash_set_current_streaming_group_id(c_master->valuestring);
                                    /* Dừng tất cả các chế độ khác -> chuyển sang chế độ codec decoder */
                                    app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
                                    app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                                    // Change codec to DECODE mode
                                    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                                    // Set volume to volume setup
                                    app_audio_change_codec_vol(app_flash_get_volume());

                                    // Switch Relay to Codec output
                                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                                    // Nếu PA đang OFF thì ON
                                    if (app_io_is_pa_off() && app_audio_get_force_turn_off_pa_remain_time() == 0)
                                    {
                                        app_io_control_pa(APP_IO_PA_ON);
                                    }

                                    /* 2. Init new app_audio_get_stream_url() */
                                    char *tmp_url = app_audio_get_stream_url();
                                    app_flash_set_last_streaming_url(c_uri->valuestring);
                                    sprintf(tmp_url, "%s",  c_uri->valuestring);
                                    DEBUG_WARN("HTTP stream URL %s\r\n", app_audio_get_stream_url());
                                    app_mqtt_set_streaming_master(c_master->valuestring); 

                                    // Nếu Relays đang OFF thì ON
                                    // if(app_io_is_iso_relays_off()) {
                                    if (slave_get_delay_turn_on_relay_prepare_stream() == 0)
                                    {
                                        // sprintf(msg+msg_size, ", Turn on all opto");
                                        // O mach classD thi ISO out la mute va disable PA
                                        if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                                        {
                                            app_io_opto_control_all(APP_IO_OPTO_ON);
                                        }
                                        else
                                        {
                                            // Enable PA for class D
                                            app_io_opto_control_all(APP_IO_OPTO_OFF);
                                        }
                                    }
                                    else
                                    {
                                        if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                                        {
                                            app_io_control_opto_output1(APP_IO_OPTO_ON);
                                        }
                                        else
                                        {
                                            // Enable PA for class D
                                            app_io_opto_control_all(APP_IO_OPTO_OFF);
                                        }
                                    }
                                    app_mqtt_publish_ttn_debug_msg("Start streaming by 'STREAM START/STREAM RUNNING' on master %s", 
                                                            c_master->valuestring);
                                    slave_set_timeout_when_received_stream_running_command(DEFAULT_TIMEOUT_AUTO_TERMINATE_STREAM);
                                    //}

                                    /** Nếu http_stream_reader đang STOPPED hoặc INIT(sau khi khởi động request mà k có link) thì Start stream được luôn
                                    * Trường hợp: Sau lệnh 'STREAM_STOP' -> terminate -> nhận lệnh 'STREAM_START' nhưng http_stream_reader
                                    * vẫn chưa stopped xong (vẫn đang state RUNNING) -> terminate cho phát nữa và bật cờ 'm_is_waiting_http_element_stopped'
                                    */
                                    audio_element_state_t http_state = app_audio_get_http_state();
                                    DEBUG_VERBOSE("Http state %d\r\n", http_state);
                                    if (http_state == AEL_STATE_STOPPED 
                                        || http_state == AEL_STATE_INIT)
                                    {
                                        if (!slave_get_start_stream_command_timeout())
                                        { /* Case: nhận được nhiều lệnh đến gần nhau -> tránh restart stream liên tục! */
                                            slave_reset_total_streaming_received();
                                            slave_set_auto_restart_stream_timeout(0);
                                            slave_set_start_stream_command_timeout(15);
                                            slave_set_timeout_turn_off_opto_output(0);

                                            /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */
                                            slave_set_timeout_number_of_waiting_master_streaming(30); 

                                            /**01/05/20: delay before start streaming, nếu start stream luôn thì bị hiện tượng HTTP báo
                                            * HTTP_STREAM_FINISH_TRACK -> Có thể do server đặt Latency Buffer lớn -> Stream ngay thì chưa có
                                            * nội dung -> báo FINISH_TRACK. Để delay 5s thì không thấy bị
                                            */
                                            slave_restart_stream(app_audio_get_stream_url());
                                            app_mqtt_publish_ttn_debug_msg("STREAM START %s", app_audio_get_stream_url());
                                            slave_set_timeout_when_received_stream_running_command(DEFAULT_TIMEOUT_AUTO_TERMINATE_STREAM);
                                        }
                                    }
                                    else
                                    {
                                        DEBUG_WARN("'http_element' is not stopped, terminate_pipeline first and waiting for stoped...\r\n");
                                        app_audio_simple_terminate_pipeline();
                                        m_is_waiting_http_element_stopped = true;
                                        slave_reset_counter_wait_for_http_element_stop();

                                        /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */
                                        slave_set_timeout_number_of_waiting_master_streaming(30); 
                                        /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                                        slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false);  
                                    }
                                }
                            }
                            else
                            {
                                DEBUG_WARN("Another stream is opening\r\n");
                            }
                        }
                    }
                    else if (c_cmd->valueint == CMD_STREAM_STOP && (slave_get_http_finish_track_timeout() == 0)) /** Đang không chờ timeout sau sự kiện HTTP_FINISH_TRACK */
                    {
                        /* ====================== Chỉ nhận lệnh STOP từ đúng master đang chạy ======================== */
                        if (strcmp(app_flash_get_current_streaming_group_id(), c_master->valuestring) == 0)
                        {
                            slave_set_received_running_state_timeout(0);

                            /* Nếu có element nào đang running -> terminate pipeline */
                            if (app_audio_is_i2s_running()
                                || app_audio_is_opus_running()
                                || app_audio_is_http_audio_stream_running())
                            {
                                DEBUG_INFO("Stop by owner master: %s\r\n", app_flash_get_current_streaming_group_id());
                                DEBUG_INFO("Have NO any master are streaming, let's relaxing!\r\n");
                                /* 1.3. Turn off PA after 30s */
                                slave_set_timeout_turn_off_opto_output(15);
                                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_IDLE);
                                slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false);
                                m_timeout_http_stream_check = 0;
                                app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, 0);

                                /** 2. Terminate pipeline */
                                DEBUG_WARN("Stop stream from master cmd -> terminate pipeline\r\n");
#if 1
                                /* Khi stream van con 1 it data -> server da goi ham stream stop
                                    -> Delay them 1 chut de phat not am thanh
                                */
                                vTaskDelay(3000);
#endif      
                                app_audio_pause();
                                app_audio_complete_terminate();
                                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_STOPPED);
                                /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                                slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false); 
                                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                                slave_set_mqtt_state_timeout(58); /* Publish info sau khi connected 5s */
                            }
                            else
                            {
                                DEBUG_VERBOSE("There isn't any audio_element's running. Ignore terminate!\r\n");
                            }
                        }
                        else
                        {
                            DEBUG_WARN("[OH NO] %s not my master, DON'T STOP!!!\r\n", c_master->valuestring);
                        }
                    }
                }
            }
        }
            break;
    default:
        break;
    }

    if (send_cfg)
    {
        app_mqtt_send_all_config_after_reset();
    }
}


bool atth_mqtt_parse(const char* msg)
{
    cJSON* root = cJSON_Parse(msg);

    if (root == NULL)
    {
        DEBUG_ERROR("Invalid json format\r\n");
        return false;
    }
    cJSON *id, *sender, *time, *message;

    id = cJSON_GetObjectItem(root, "Id");
    time = cJSON_GetObjectItem(root, "Time");
    sender = cJSON_GetObjectItem(root, "Sender");
    message = cJSON_GetObjectItem(root, "Message");
    if (!id || !time || !sender || !message)
    {
        DEBUG_WARN("Invalid Id/Time/Sender/Message json key\r\n");
        goto end;
    }
    
    if (!(cJSON_IsNumber(id)
        && cJSON_IsNumber(time)
        && cJSON_IsString(sender)
        && cJSON_IsObject(message)))
    {
        DEBUG_WARN("Invalid Id/Time/Sender/Message json value\r\n");
        goto end;
    }

    DEBUG_WARN("Rx ID -> '%d' from '%s'\r\n", id->valueint, sender->valuestring);
    atth_mqtt_process_msq_id(root, message, id->valueint);
end:
    cJSON_Delete(root);
    return true;
}

void poll_mqtt_stream_msq()
{
    mqtt_queue_stream_event_t *event = mqtt_queue_streaming_get();
    if (event)
    {
        char *topic_name = event->topic;
        char *payload = (char*)event->payload;
        uint8_t priority = app_flash_get_protocol_priority();
        if (priority == APP_FLASH_PROTOCOL_V2_JSON)
        {
            atth_mqtt_parse(payload);
            goto free_queue;
        }
        // Mode cmd
        char *op_mode = strstr(payload, "MODE(");
        if (op_mode)
        {
            change_mode_cmd(op_mode);
            app_mqtt_send_all_config_after_reset();
            goto free_queue;
        }


        /* Nếu chế độ hoạt động không cho phép -> return */
        if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_NO_OPERATION)
        {
            DEBUG_INFO("Mode no operation\r\n");
            // Clear các trạng thái của pipeline để khi cho phép chế độ INTERNET trở lại thì STREAM được
            slave_set_http_finish_track_timeout(0);
            m_is_waiting_http_element_stopped = false;
            app_audio_set_stream_retry(0);
            goto free_queue;
        }

        char *p = strstr(payload, "STREAM_HIGH_PRIORITY(");
        if (p)
        {
            p += strlen("STREAM_HIGH_PRIORITY(");
            DEBUG_INFO("Stream by URL\r\n");
            if (memcmp(p, "http://", 7) == 0 && strstr(p, ")"))
            {
                process_stream_with_high_priority(p);
            }
            else
            {
                DEBUG_INFO("\tInvalid stream");
            }
            goto free_queue;
        }   

        char *prepaire_msg = NULL;
        char *stream_relay = strstr(payload, "STREAM_RELAY");
        char fake_prepare[32];
        if (stream_relay)
        {
            int relay = GetNumberFromString(0, stream_relay);
            if (relay == 0)
            {
                app_io_get_io_value()->Name.IO1 = IO_ON;
                sprintf(fake_prepare, "STREAM_PREPAIR(%d)", 30 + app_flash_get_delay_turn_on_relay());
                prepaire_msg = fake_prepare;
                mqtt_publish_message("DBG", "Control relay1 before stream");
            } 
            else if (relay == 1)
            {
                app_io_get_io_value()->Name.IO2 = IO_ON;
                goto free_queue;
            }
        }

        // 20/12/20: Thêm lệnh "STREAM_PREPAIR" để điều khiển bật Relay theo thứ tự: relay1 -> delay xx giây -> relay2
        if (prepaire_msg == NULL) prepaire_msg = strstr(payload, "STREAM_PREPAIR");
        if (prepaire_msg)
        {
            slave_process_prepair_onair(prepaire_msg);
            goto free_queue;
        }

        /* ====================================== Nếu là message từ topic 'master command' ===================================//
         * Ưu tiên xử lý theo thứ tự từng group: (T1 -> T2 -> T3) => (H1 -> H2 -> H3) => (X1 -> X2 -> X3)
         * 07/02/21: WARNING: chỗ này xác định command_master_level bị sai!!!
         * ====================================================================================================================//
         */
        uint8_t command_master_level = 0;
        char *streaming_master = NULL;
        for (uint8_t master_index = APP_FLASH_MASTER_TINH1; master_index < APP_FLASH_MASTER_TOTAL; master_index++)
        {
            DEBUG_VERBOSE("TOPIC=%s,MASTER%d=%s", topic_name, master_index, app_flash_get_master(master_index));
            if ((strlen(app_flash_get_master(master_index)) >= 15) && strstr(topic_name, app_flash_get_master(master_index)))
            {
                command_master_level = master_index;
                streaming_master = app_flash_get_master(master_index);
                break;
            }
        }

        DEBUG_VERBOSE("CMD master level %u\r\n", command_master_level);
        // Message đến từ topic lạ (giả sử)!
        if (command_master_level == 0)
        {
            DEBUG_WARN("Topic name is not allow: %s\r\n", topic_name);
            // if (strstr(topic_name, app_flash_get_imei()) == NULL)
            //     esp_mqtt_client_unsubscribe(m_mqtt_client, topic_name);
            goto free_queue;
        }

        /**
         * ==================== Lệnh STREAM_STOP ===========================//
         * Ghi nhớ trạng thái STOP streaming của 3 đài.
         * Mục đích: Khi đài cấp cao hơn STOP stream, kiểm tra xem các đài cấp thấp hơn có đang phát không,
         * nếu đang phát thì chuyển luồng stream sang luôn
         * Chỉ STOP khi đang stream từ chính các Master Device! (Bỏ Schedule stream!)
         */
        if (strstr(payload, "STREAM_STOP"))
        {
            switch (command_master_level)
            {
            case APP_FLASH_MASTER_TINH1:
                m_master_streaming_info.name.MasterTINH1 = NO_STREAM;
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_TINH1, "");
                break;
            case APP_FLASH_MASTER_TINH2:
                m_master_streaming_info.name.MasterTINH2 = NO_STREAM;
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_TINH2, "");
                break;
            case APP_FLASH_MASTER_TINH3:
                m_master_streaming_info.name.MasterTINH3 = NO_STREAM;
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_TINH3, "");
                break;
            case APP_FLASH_MASTER_HUYEN1:
                m_master_streaming_info.name.MasterHUYEN1 = NO_STREAM;
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_HUYEN1, "");
                break;
            case APP_FLASH_MASTER_HUYEN2:
                m_master_streaming_info.name.MasterHUYEN2 = NO_STREAM;
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_HUYEN2, "");
                break;
            case APP_FLASH_MASTER_HUYEN3:
                m_master_streaming_info.name.MasterHUYEN3 = NO_STREAM;
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_HUYEN3, "");
                break;
            case APP_FLASH_MASTER_XA1:
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_XA1, "");
                m_master_streaming_info.name.MasterXA1 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_XA2:
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_XA2, "");
                m_master_streaming_info.name.MasterXA2 = NO_STREAM;
                break;
            case APP_FLASH_MASTER_XA3:
                app_audio_set_alt_stream_url(APP_FLASH_MASTER_XA3, "");
                m_master_streaming_info.name.MasterXA3 = NO_STREAM;
                break;
            default:
                break;
            }
        }

        /** ==================== Lệnh STREAM_START or STREAM_RUNNING =========================== */
        uint8_t stream_run_val = 0;
        if (strstr(payload, "STREAM_START"))
        {
            stream_run_val = 1 << 0;
        } 
        if (strstr(payload, "STREAM_RUNNING"))
        {
            stream_run_val = 1 << 1;
        }

        if (stream_run_val == 1)
        {
            m_delay_for_http_server_have_enough_buffer = 5000;
        }
        else
        {
            m_delay_for_http_server_have_enough_buffer = 3000;
        } 


        if (stream_run_val)
        {
            DEBUG_INFO("[%s-%d] STREAM_RUNNING\r\n", streaming_master, command_master_level);
            /* Nếu đang PHÁT -> Chỉ xử lý lệnh từ Master cấp bằng hoặc cao hơn
             * Ví dụ: Đang phát ở cấp xã nhận được lệnh cấp Huyện/Tỉnh -> Stop xã chuyển sang Huyện/Tỉnh
             * Nếu đang phát cấp tỉnh mà nhận được START cấp xã/huyện thì bỏ qua
             * Nếu đang DỪNG -> nhận lệnh từ cả 3 cấp!
             */

            /** Ghi nhớ trạng thái streaming của 3 đài.
             * Mục đích: Khi đài cấp cao hơn STOP stream, kiểm tra xem các đài cấp thấp hơn có đang phát không,
             * nếu đang phát thì chuyển luồng stream sang luôn
             */
            switch (command_master_level)
            {
            case APP_FLASH_MASTER_TINH1:
                m_master_streaming_info.name.MasterTINH1 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_TINH2:
                m_master_streaming_info.name.MasterTINH2 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_TINH3:
                m_master_streaming_info.name.MasterTINH3 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_HUYEN1:
                m_master_streaming_info.name.MasterHUYEN1 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_HUYEN2:
                m_master_streaming_info.name.MasterHUYEN2 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_HUYEN3:
                m_master_streaming_info.name.MasterHUYEN3 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_XA1:
                m_master_streaming_info.name.MasterXA1 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_XA2:
                m_master_streaming_info.name.MasterXA2 = LIVE_MASTER;
                break;
            case APP_FLASH_MASTER_XA3:
                m_master_streaming_info.name.MasterXA3 = LIVE_MASTER;
                break;
            default:
                DEBUG_WARN("Unknown master %u\r\n", command_master_level);
                break;
            }
            char *tmp_url = strstr(payload, "STREAM_RUNNING(");
            if (tmp_url)
            {
                // Copy stream url to ALT list
                CopyParameter(tmp_url, app_audio_get_alt_stream_url(command_master_level), '(', ')');
            }
            else
            {
                char old_url_style[APP_AUDIO_HTTP_URL_SIZE];
                sprintf(old_url_style, 
                    "%s%s", 
                    app_flash_get_http_stream_header(), 
                    app_flash_get_master(command_master_level));

                app_audio_set_alt_stream_url(command_master_level, old_url_style);
            }
            if (stream_if_higher_level(command_master_level, false))
            {
                goto free_queue;
            }
            /* 2. ================== Nếu đang Không Streaming -> Nhận lệnh từ tất cả các master ============== */
            /* Nếu đang trong quá trình chuyển sang stream higher master -> không nhận lệnh nữa */
            if (slave_get_number_of_second_timeout_waiting_master_streaming())
            {
                DEBUG_INFO("Starting from higher master stream, ignore cmd in: %us\r\n", 
                            slave_get_number_of_second_timeout_waiting_master_streaming());
                goto free_queue;
            }

            if (slave_get_received_running_state_timeout() == 0)
            {
                DEBUG_INFO("Start counting running state timeout...\r\n");
                slave_set_received_running_state_timeout(40); /* Sau 35s mà không start stream được thì reset cho nhanh */
            }

            /* Đánh dấu dung lượng stream khi nhận lệnh -> so sánh sau 35 giây sau */
            if (strstr(payload, "STREAM_START"))
            {
                slave_reset_stream_monitor_data();
            }
            slave_update_last_stream_data();

            // DEBUG
            DEBUG_INFO("HTTP finish track timeout: %u, Stream retries number: %u\r\n",
                     slave_get_http_finish_track_timeout(), app_audio_get_reset_stream_retry_number());

#if 0
			/** TEST: Nếu nhiều lệnh START/RUNNING đến liền nhau -> thay đổi url theo mức ưu tiên cao hơn */
			if(command_master_level <= app_flash_get_current_streaming_master_index()) {
				DEBUG_INFO("Chuyen url sang master uu tien cao hon: %u -> %u", app_flash_get_current_streaming_master_index(), command_master_level);
				
				/* 1. Save the current Master */
				app_flash_set_current_streaming_master(command_master_level);
				app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, command_master_level);

				/* 2. Init new app_audio_get_stream_url() */
				build_http_stream_url(command_master_level);
			}
			/* ============================ END TEST ==============================*/
#endif

            /** Hiện tượng: Đang khởi động stream với Higher Master, http_element vừa closed, đang delay 5s chuẩn bị START_STREAM
             * -> Các element vẫn chưa ở STATE_RUNNING -> nhận được lệnh APP_AUDIO_STREAM_RUNNING khác -> khởi tạo 1 STREAM mới -> xung đột
             * gây treo task -> watchdog reset
             * ==> Giải pháp: Nếu đang chờ chạy 1 luồng stream thì không khởi tạo luồng khác!
             */
            if (slave_get_http_finish_track_timeout() == 0 && /* Nếu nhận được sự kiện HTTP_FINISH_TRACK -> Đang chờ http_finished thì không START STREAM! */
                !m_is_waiting_http_element_stopped &&        /* Nếu đang chờ http_element stopped sau khi lệnh terminate -> Không start STREAM */
                app_audio_get_reset_stream_retry_number() == 0)        /* Đang retry start STREAM -> tạm thời chưa nhận lệnh STREAM */
            {
                if (!app_audio_is_i2s_running()
                    || !app_audio_is_opus_running()
                    || !app_audio_is_http_audio_stream_running())
                {
                    /* Dừng tất cả các chế độ khác -> chuyển sang chế độ codec decoder */
                    app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
                    app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                    // Change codec to DECODE mode
                    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                    // Set volume to volume setup
                    app_audio_change_codec_vol(app_flash_get_volume());

                    // Switch Relay to Codec output
                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                    // Nếu PA đang OFF thì ON
                    if (app_io_is_pa_off() && app_audio_get_force_turn_off_pa_remain_time() == 0)
                    {
                        app_io_control_pa(APP_IO_PA_ON);
                    }

                    /* 2. Init new app_audio_get_stream_url() */
                    if (tmp_url && strstr(tmp_url, ")"))
                    {
                        memset(app_audio_get_stream_url(), 0, APP_AUDIO_HTTP_URL_SIZE);
                        CopyParameter(tmp_url, app_audio_get_stream_url(), '(', ')');
                        CopyParameter(tmp_url, app_audio_get_alt_stream_url(command_master_level), '(', ')');
                    }
                    else
                    {
                        build_http_stream_url(command_master_level);
                        app_audio_set_alt_stream_url(command_master_level, app_audio_get_stream_url());
                    }
                    DEBUG_VERBOSE("HTTP stream URL %s\r\n", app_audio_get_stream_url());
                    DEBUG_VERBOSE("ALT stream URL %s\r\n", app_audio_get_alt_stream_url(command_master_level));

                    // Nếu Relays đang OFF thì ON
                    // if(app_io_is_iso_relays_off()) {
                    if (slave_get_delay_turn_on_relay_prepare_stream() == 0)
                    {
                        // sprintf(msg+msg_size, ", Turn on all opto");
                        // O mach classD thi ISO out la mute va disable PA
                        if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                        {
                            app_io_opto_control_all(APP_IO_OPTO_ON);
                        }
                        else
                        {
                            // Enable PA for class D
                            app_io_opto_control_all(APP_IO_OPTO_OFF);
                        }
                    }
                    else
                    {
                        if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                        {
                            app_io_control_opto_output1(APP_IO_OPTO_ON);
                        }
                        else
                        {
                            // Enable PA for class D
                            app_io_opto_control_all(APP_IO_OPTO_OFF);
                        }
                    }
                    mqtt_publish_message("DBG", 
                                        "Start streaming on master %s, link %s", 
                                        app_flash_get_master(command_master_level),
                                        app_audio_get_stream_url());
                    slave_set_timeout_when_received_stream_running_command(80);
                    //}

#if 1 // TEST: Chuyển khởi tạo url ra ngoài if
                    /** Lưu thông tin của Master ra lệnh START và khởi tạo URL tương ứng -> có thể đưa ra ngoài điều kiện check if
                     * Mục đích: Khi chưa khởi tạo stream xong mà có lệnh từ master cao hơn -> thay đổi url luôn, đến lúc stream sẽ lấy
                     * theo url cao nhất luôn -> phải check điều kiện isMasterHigherLevel
                     */
                    /* 1. Save the current Master */
                    app_flash_set_current_streaming_master(command_master_level);
                    app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, command_master_level);
                    app_flash_set_last_streaming_url(app_audio_get_stream_url());

#endif

                    /** Nếu http_stream_reader đang STOPPED hoặc INIT(sau khi khởi động request mà k có link) thì Start stream được luôn
                     * Trường hợp: Sau lệnh 'STREAM_STOP' -> terminate -> nhận lệnh 'STREAM_START' nhưng http_stream_reader
                     * vẫn chưa stopped xong (vẫn đang state RUNNING) -> terminate cho phát nữa và bật cờ 'm_is_waiting_http_element_stopped'
                     */
                    audio_element_state_t http_state = app_audio_get_http_state();
                    DEBUG_INFO("Http state %d\r\n", http_state);
                    if (http_state == AEL_STATE_STOPPED 
                        || http_state == AEL_STATE_INIT)
                    {
                        if (!slave_get_start_stream_command_timeout())
                        { /* Case: nhận được nhiều lệnh đến gần nhau -> tránh restart stream liên tục! */
                            slave_reset_total_streaming_received();
                            slave_set_auto_restart_stream_timeout(0);
                            slave_set_start_stream_command_timeout(15);
                            slave_set_timeout_turn_off_opto_output(0);

                            /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */
                            slave_set_timeout_number_of_waiting_master_streaming(30); 

                            /**01/05/20: delay before start streaming, nếu start stream luôn thì bị hiện tượng HTTP báo
                             * HTTP_STREAM_FINISH_TRACK -> Có thể do server đặt Latency Buffer lớn -> Stream ngay thì chưa có
                             * nội dung -> báo FINISH_TRACK. Để delay 5s thì không thấy bị
                             */
                            slave_restart_stream(app_audio_get_stream_url());
                            mqtt_publish_message("DBG", "STREAM START %s", app_audio_get_stream_url());
                            slave_set_timeout_when_received_stream_running_command(80);
                        }
                    }
                    else
                    {
                        DEBUG_INFO("'http_element' is not stopped, terminate_pipeline first and waiting for stoped...\r\n");
                        app_audio_simple_terminate_pipeline();
                        m_is_waiting_http_element_stopped = true;
                        slave_reset_counter_wait_for_http_element_stop();

                        /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */
                        slave_set_timeout_number_of_waiting_master_streaming(30); 
                        /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                        slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false);  
                    }
                }
            }
            else
            {
                DEBUG_WARN("Another stream is opening\r\n");
                // mqtt_publish_message("DBG", "Another stream is opening");
            }
        }
        /** ==================== Lệnh STREAM_STOP =========================== */
        else if (strstr(payload, "STREAM_STOP") 
                && (slave_get_http_finish_track_timeout() == 0)) /** Đang không chờ timeout sau sự kiện HTTP_FINISH_TRACK */
        {
            DEBUG_INFO("MQTT: Stop streaming by 'STREAM_STOP' command...\r\n");

            /* ====================== Chỉ nhận lệnh STOP từ đúng master đang chạy ======================== */
            if (app_flash_get_current_streaming_master_index() == command_master_level)
            {
                slave_set_received_running_state_timeout(0);

                /* Nếu có element nào đang running -> terminate pipeline */
                if (stream_ele_info())
                {
                    DEBUG_INFO("Stop by owner master: %u\r\n", command_master_level);
                    if (m_stream_emergency)
                    {
                        mqtt_publish_message("DBG", "Stop by owner master: %u, but in emergency call", command_master_level);
                        goto free_queue;
                    }
                    
                    mqtt_publish_message("DBG", "Stop by owner master: %u", command_master_level, app_flash_get_master(command_master_level));
                    app_audio_set_alt_stream_url(command_master_level, "");

                    /** 1. --------- Xét chuyển luồng stream sang đài cấp thấp hơn nếu đang streaming ---------------
                     * Nếu đài TỈNH dừng phát -> xét đài HUYỆN, XÃ...
                     */
                    uint8_t next_master_level = 0;
                    switch (command_master_level)
                    {
                    case APP_FLASH_MASTER_TINH1:
                        if (m_master_streaming_info.name.MasterTINH2)
                            next_master_level = APP_FLASH_MASTER_TINH2;
                        else if (m_master_streaming_info.name.MasterTINH3)
                            next_master_level = APP_FLASH_MASTER_TINH3;
                        else if (m_master_streaming_info.name.MasterHUYEN1)
                            next_master_level = APP_FLASH_MASTER_HUYEN1;
                        else if (m_master_streaming_info.name.MasterHUYEN2)
                            next_master_level = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streaming_info.name.MasterHUYEN3)
                            next_master_level = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streaming_info.name.MasterXA1)
                            next_master_level = APP_FLASH_MASTER_XA1;
                        else if (m_master_streaming_info.name.MasterXA2)
                            next_master_level = APP_FLASH_MASTER_XA2;
                        else if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_TINH2:
                        if (m_master_streaming_info.name.MasterTINH3)
                            next_master_level = APP_FLASH_MASTER_TINH3;
                        else if (m_master_streaming_info.name.MasterHUYEN1)
                            next_master_level = APP_FLASH_MASTER_HUYEN1;
                        else if (m_master_streaming_info.name.MasterHUYEN2)
                            next_master_level = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streaming_info.name.MasterHUYEN3)
                            next_master_level = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streaming_info.name.MasterXA1)
                            next_master_level = APP_FLASH_MASTER_XA1;
                        else if (m_master_streaming_info.name.MasterXA2)
                            next_master_level = APP_FLASH_MASTER_XA2;
                        else if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_TINH3:
                        if (m_master_streaming_info.name.MasterHUYEN1)
                            next_master_level = APP_FLASH_MASTER_HUYEN1;
                        else if (m_master_streaming_info.name.MasterHUYEN2)
                            next_master_level = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streaming_info.name.MasterHUYEN3)
                            next_master_level = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streaming_info.name.MasterXA1)
                            next_master_level = APP_FLASH_MASTER_XA1;
                        else if (m_master_streaming_info.name.MasterXA2)
                            next_master_level = APP_FLASH_MASTER_XA2;
                        else if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_HUYEN1:
                        if (m_master_streaming_info.name.MasterHUYEN2)
                            next_master_level = APP_FLASH_MASTER_HUYEN2;
                        else if (m_master_streaming_info.name.MasterHUYEN3)
                            next_master_level = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streaming_info.name.MasterXA1)
                            next_master_level = APP_FLASH_MASTER_XA1;
                        else if (m_master_streaming_info.name.MasterXA2)
                            next_master_level = APP_FLASH_MASTER_XA2;
                        else if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_HUYEN2:
                        if (m_master_streaming_info.name.MasterHUYEN3)
                            next_master_level = APP_FLASH_MASTER_HUYEN3;
                        else if (m_master_streaming_info.name.MasterXA1)
                            next_master_level = APP_FLASH_MASTER_XA1;
                        else if (m_master_streaming_info.name.MasterXA2)
                            next_master_level = APP_FLASH_MASTER_XA2;
                        else if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_HUYEN3:
                        if (m_master_streaming_info.name.MasterXA1)
                            next_master_level = APP_FLASH_MASTER_XA1;
                        else if (m_master_streaming_info.name.MasterXA2)
                            next_master_level = APP_FLASH_MASTER_XA2;
                        else if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_XA1:
                        if (m_master_streaming_info.name.MasterXA2)
                            next_master_level = APP_FLASH_MASTER_XA2;
                        else if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_XA2:
                        if (m_master_streaming_info.name.MasterXA3)
                            next_master_level = APP_FLASH_MASTER_XA3;
                        break;
                    case APP_FLASH_MASTER_XA3:
                        break;
                    default:
                        break;
                    }

                    /** Nếu tìm thấy master cấp thấp hơn đang stream -> chuyển về listen master đó */
                    if (next_master_level)
                    {
                        char *remmeber_url = app_audio_get_alt_stream_url(next_master_level);
                        DEBUG_INFO("MASTER %u is streaming, listen to him on %s\r\n", 
                                    next_master_level,
                                    remmeber_url);

                        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                        // Set flag HttpElement, check when it's stopped later -> try to restart new stream...
                        m_is_waiting_http_element_stopped = true;
                        slave_reset_counter_wait_for_http_element_stop();
                        m_timeout_http_stream_check = 0;
                        /* Don't turn off PA */
                        slave_set_timeout_turn_off_opto_output(0);

                        /* 1.1. Save the current Master */
                        app_flash_set_current_streaming_master(next_master_level);
                        app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, next_master_level);

                        /* 1.2. Init new app_audio_get_stream_url() */
                        if (strlen(remmeber_url) > 10)
                        {
                            sprintf(app_audio_get_stream_url(), 
                                    "%s", 
                                    remmeber_url);
                        }
                        else
                        {
                            build_http_stream_url(next_master_level);
                        }
                        mqtt_publish_message("DBG", "Found next master %d, listen to %s", 
                                                next_master_level, app_audio_get_stream_url());
                        app_flash_set_last_streaming_url(app_audio_get_stream_url());

                        /* Trong thời gian chờ Start Streaming -> không thực thi lệnh Start khác */
                        slave_set_timeout_number_of_waiting_master_streaming(30); 
                        slave_set_timeout_when_received_stream_running_command(80);
                    }
                    else
                    {
                        DEBUG_INFO("Have NO any master are streaming, let's relaxing!\r\n");
                        /* 1.3. Turn off PA after 30s */
                        slave_set_timeout_turn_off_opto_output(30);
                        app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_IDLE);
                        slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false);
                        m_timeout_http_stream_check = 0;
                        app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, 0);
                        mqtt_publish_message("DBG", "No more master streaming");
                        // Test: chuyển currentMaster về master nhỏ nhất
                        // app_flash_set_current_streaming_master(APP_FLASH_MASTER_XA3);
                    }
                    /** --------- END of chuyển luồng stream sang đài cấp thấp hơn nếu đang streaming ---------------*/

                    /** 2. Terminate pipeline */
                    DEBUG_WARN("Stop stream from master cmd -> terminate pipeline\r\n");
#if 1
                    /* Khi stream van con 1 it data -> server da goi ham stream stop
                        -> Delay them 1 chut de phat not am thanh
                    */
                    vTaskDelay(2000);
#endif      
                    app_audio_pause();
                    app_audio_complete_terminate();
                    app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_STOPPED);
                    /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                    slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false); 
                    app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                    slave_set_mqtt_state_timeout(58); /* Publish info sau 2s */
                    app_audio_set_stream_retry(0);
                    if (m_auto_terminate_pipeline > 1)
                    {
                        m_auto_terminate_pipeline = 1;
                    }
                    DEBUG_VERBOSE("Monitor stream =>> disable\r\n");
                }
                else
                {
                    DEBUG_VERBOSE("There isn't any audio_element's running. Ignore terminate!\r\n");
                }
            }
            else
            {
                DEBUG_WARN("[OH NO] %s not my master, DON'T STOP!!!\r\n", topic_name);
            }
        }
free_queue:
        if (event->need_free)
        {
            free(event->payload);
            free(event->topic);
        }
    }
}

static void live_streaming_down_task(void *arg)
{
    // esp_err_t err;
    DEBUG_VERBOSE("\t\t--- live_streaming_down_task is running --\r\n");
    m_http_received_data_timeout = 0;
    app_audio_start();
    bool restart_stream_when_network_disconnect = false;
    uint8_t delay_stream_after_network_connected = 3;
    bool is_network_connected = false;
    mqtt_queue_streaming_init();

    while (1)
    {
        if (!network_is_connected())
        {
            DEBUG_INFO("WiFi/Eth/PPP is not connected\r\n");
            vTaskDelay(1000 / portTICK_RATE_MS);

            /* Nếu đang có mạng -> mất mạng => terminate pipeline luôn */
            if (is_network_connected)
            {
                DEBUG_WARN("Network is not connected. Terminate pipeline...\r\n");
                restart_stream_when_network_disconnect = true;
            }
            is_network_connected = false;
            delay_stream_after_network_connected = 3;
            continue;
        }

        /* Sau khi có mạng chờ vài giây cho ổn định rồi mới connect */
        if (delay_stream_after_network_connected)
        {
            DEBUG_VERBOSE(TAG,  "Delay after network connected: %us\r\n", 
                        delay_stream_after_network_connected);
            delay_stream_after_network_connected--;
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }
        
        if (restart_stream_when_network_disconnect)
        {
            restart_stream_when_network_disconnect = false;
            if (network_debug_and_clear_ppp_disconnect_status())
            {
                mqtt_publish_message("DBG", "Network PPP disconnect, reconnect again");
            }
            else
            {
                mqtt_publish_message("DBG", "Network PPP unknown error");
            }
        }
        is_network_connected = true;

        // if (m_terminate_stream_async)
        // {
        //     m_terminate_stream_async = false;
        //     app_audio_pause();
        //     app_audio_complete_terminate();
        // }

        // if (m_restart_stream_async)
        // {
        //     m_restart_stream_async = false;

        //     vTaskDelay(1000 / portTICK_RATE_MS);
        //     app_audio_restart_pipeline(app_audio_get_stream_url());
        //     app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RUNNING);
        // }

        /** Chờ HttpElement stopped sau lệnh terminate_pipeline khi nhận lệnh 'STREAM_START' hoặc 'STREAM_RUNNING'
         * Khi stopped thì delay vài giây rồi mới restart STREAM
         */
        if (m_is_waiting_http_element_stopped)
        {
            audio_element_state_t el_http_state = app_audio_get_http_state();
            if (el_http_state == AEL_STATE_STOPPED || el_http_state == AEL_STATE_INIT)
            {
                DEBUG_WARN("HTTP is already stopped, can restart stream now...\r\n");

                m_is_waiting_http_element_stopped = false;
                m_waiting_http_element_stopped_timeout = 0;

                /**01/05/20: test delay before start streaming, nếu start stream luôn thì bị hiện tượng HTTP báo
                 * HTTP_STREAM_FINISH_TRACK -> Có thể do server đặt Latency Buffer lớn -> Stream ngay thì chưa có
                 * nội dung -> báo FINISH_TRACK. Để delay 5s thì không thấy bị
                 */
                // DEBUG_WARN("STREAM: Delay 0s before start streaming!\r\n");
                // mqtt_publish_message("DBG", "HTTP is already stopped, restart stream now");
                // vTaskDelay(5000 / portTICK_RATE_MS);
                m_main_task_hangout = 0;
                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RESTART);
            }
            else
            {
                vTaskDelay(100 / portTICK_RATE_MS);

                if (m_waiting_http_element_stopped_timeout++ >= 150)
                { /* 15s */
                    // Chờ lâu mà httpElement không stopped được -> reset cho nhanh!
                    DEBUG_WARN("m_waiting_http_element_stopped_timeout is too long! Reset system...\r\n");
                    if (!app_ota_is_running())
                    {
                        app_audio_change_codec_vol(0);
                        vTaskDelay(1000 / portTICK_RATE_MS);
                        system_software_reset(SW_RESET_REASON_HTTP_EL_STOP_TIMEOUT);
                    }
                }
                else
                {
                    continue;
                }
            }
        }

        if (m_auto_restart_stream_timeout)
        {
            // vTaskDelay(1000 / portTICK_PERIOD_MS);

            m_auto_restart_stream_timeout--;
            // if (m_auto_restart_stream_timeout == 0)
            // {
            //     DEBUG_INFO("Auto restart pipeline -> %s\r\n", app_audio_get_stream_url());
            //     app_audio_run_pipeline();
            //     app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RUNNING);
            // }
        }

        /* ============================ Chỉ chạy stream internet khi chế độ hoạt động cho phép ============================== */
        if (app_flash_get_operate_mode() != APP_AUDIO_OPERATION_MODE_NO_OPERATION)
        {
            /* Sau khi khởi động thử stream luôn, nếu không có link -> http_err_open */
            if (app_audio_get_streaming_logic_step() == APP_AUDIO_STREAM_NOT_INIT)
            {
                char *url = app_audio_get_stream_url();
                bool invalid_url = strstr(url, APP_FLASH_INVALID_LINK) ? true : false;

                // Kiem tra them xem url co chua master imei ko
                if (!invalid_url)
                {
                    bool allow_stream = false;
                    for (int i = 0; i < APP_FLASH_MAX_SUPPORT_MASTER; i++)
                    {
                        char *tmp_master = app_flash_get_master(i);
                        if (strlen(tmp_master) > 10 && strstr(tmp_master, url))
                        {
                            allow_stream = true;
                            break;
                        }
                    }
                    if (!allow_stream)  // neu ko cho phep stream -> set invalid linl
                    {
                        app_audio_set_invalid_link();
                        app_flash_set_last_streaming_url(APP_FLASH_INVALID_LINK);
                        invalid_url = true;
                    }
                }

                // if (!invalid_url)
                // {
                //     DEBUG_INFO("[ * ] Start stream the 1st time -> %s", url);
                // }

                // audio_element_set_uri(m_http_stream_reader, app_audio_get_stream_url());
                // audio_pipeline_run(pipeline);
                app_audio_run_new_url(url);

                if (m_auto_terminate_pipeline == 0)
                {
                    slave_set_timeout_when_received_stream_running_command(DEFAULT_TIMEOUT_AUTO_TERMINATE_STREAM);
                } 

                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RUNNING);
                if (!invalid_url)
                {
                    mqtt_publish_message("DBG", "STREAM_ONREBOOT %s", url);
                }
            }

            if (app_audio_get_streaming_logic_step() == APP_AUDIO_STREAM_START)
            {
                DEBUG_INFO("Start pipeline -> %s\r\n", app_audio_get_stream_url());

                // audio_element_set_uri(m_http_stream_reader, app_audio_get_stream_url());
                // audio_pipeline_run(pipeline);
                app_audio_run_new_url(app_audio_get_stream_url());

                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RUNNING);
                // mqtt_publish_message("DBG", "STREAM START");
            }

            if (app_audio_get_streaming_logic_step() == APP_AUDIO_STREAM_RESTART)
            {
                DEBUG_INFO("Restart pipeline -> %s\r\n", app_flash_get_last_streaming_url());

                app_audio_pause();
                app_audio_complete_terminate();
                // Neu terminate luon thi bi bug stack no chua terminate xong ma da cau hinh
                // TODO
                vTaskDelay(3000 / portTICK_RATE_MS);

                DEBUG_INFO("Stream url %s\r\n", app_flash_get_last_streaming_url());
                app_audio_run_new_url((char*)app_flash_get_last_streaming_url());

                app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_RUNNING);
                mqtt_publish_message("DBG", "STREAM RESTART %s", app_flash_get_last_streaming_url());
            }
        }

        poll_mqtt_stream_msq();

        app_audio_wait_for_event(1000 / portTICK_RATE_MS);
    }
    DEBUG_INFO("\t\t--- app_streaming_task is exit ---\r\n");
    system_software_reset(SW_RESET_AUDIO);
    esp_restart();
}

/*
 * This function is called by task_wdt_isr function (ISR for when TWDT times out).
 * It can be redefined in user code to handle twdt events.
 * Note: It has the same limitations as the interrupt function.
 *       Do not use ESP_LOGI functions inside.
 */
void esp_task_wdt_isr_user_handler(void)
{
    ets_printf("\n\n\n!!! System is hangup. Restart now...!!!\n\n\n");

    /* reset system */
    system_software_reset(SW_RESET_REASON_TWDT_TIMEOUT);
}

uint32_t debug_get_ms()
{
    return xTaskGetTickCount();
}


bool debug_get_lock(bool lock, uint32_t timeout_ms)
{
    if (lock)
    {
        return xSemaphoreTake(m_debug_lock, timeout_ms);
    }
    xSemaphoreGive(m_debug_lock);
    return true;
}

uint32_t debug_serial_print(const void *buffer, uint32_t len)
{
    char *ptr = (char*)buffer;
    for (int i = 0; i < len; i++)
    {
        putchar(ptr[i]);
    }
    // fflush(stdout);
    return len;
}

void do_factory_reset(void)
{
    
}

void app_main(void)
{
    esp_err_t err;

    m_debug_lock = xSemaphoreCreateMutex();
    xSemaphoreGive(m_debug_lock);

    // esp_log_level_set(TAG, ESP_LOG_DEBUG | ESP_LOG_INFO | ESP_LOG_ERROR | ESP_LOG_WARN);
    // esp_log_level_set(TAG, ESP_LOG_NONE);

    /* Reconfig Debug UART0 */
    gpio_set_direction(GPIO_NUM_1, GPIO_MODE_DEF_OUTPUT);
    uart_set_pin(UART_NUM_0, 1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //	ESP_LOGI*TAG,  "\t\t============ Firmware Version ", __FIRMWARE_VERSION__);
    DEBUG_INFO("FW_VERSION: %s\r\n", __FIRMWARE_VERSION__);
    app_debug_init(debug_get_ms, debug_get_lock);
    app_debug_register_callback_print(debug_serial_print);

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    DEBUG_INFO("ESP32 chip with %d CPU cores, WiFi%s%s, silicon revision %d, %dMB %s flash\r\n",
                chip_info.cores,
                (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
                (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
                chip_info.revision, spi_flash_get_chip_size() / (1024 * 1024),
                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // /* Get Unique ID */
    // uint8_t uid[6];
    // err = esp_efuse_mac_get_default(uid);
    // if (err == ESP_OK)
    // {
    //     //		ESP_LOGI*TAG,  "ESP unique MAC: %02X:%02X:%02X:%02X:%02X:%02X", uid[0],
    //     //			uid[1], uid[2], uid[3],
    //     //			uid[4], uid[5]);

    //     DEBUG_INFO("[ZIG] MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
    //             uid[0], uid[1], uid[2], 
    //             uid[3], uid[4], uid[5]);
    // }
    // else
    // {
    //     //		DEBUG_WARN("ESP unique MAC error!");
    //     DEBUG_WARN("[ZIG] MAC: ERROR\r\n");
    // }

    /* =========================== Init GPIO ============================== */
    // Khởi tạo GPIO & Timer luôn để reset watchdog, nếu k reset kịp sẽ bị wdg reset!
    app_io_initialize();

    /* Timer xử lý các tác vụ phụ */
    TimerHandle_t timer = xTimerCreate("system_timer", 10 / portTICK_RATE_MS, true, NULL, xSystem_timercb);
    xTimerStart(timer, 0);

    app_flash_node_nvs_initialize();

    app_flash_slave_nvs_read_params("ALL");

    /* Get reset reason
     * Tắt nguồn bật lại: 9 - 14 hoặc 16 - 14 hoặc 1 - 14
     * Nhấn nút reset mạch nạp: 1 - 14
     * Reset mềm: 12 - 12 (SW_CPU_RESET)
     */
    uint8_t cpu0_reset_reason = rtc_get_reset_reason(0);
    uint8_t cpu1_reset_reason = rtc_get_reset_reason(1);
    DEBUG_WARN("CPU reset reason: %d - %d\r\n", cpu0_reset_reason, cpu1_reset_reason);

    /* Init m_http_stream_url */
    if (app_flash_get_current_streaming_master_index() >= APP_FLASH_MASTER_TINH1 
        && app_flash_get_current_streaming_master_index() <= APP_FLASH_MASTER_XA3 
        && strlen(app_flash_get_master(app_flash_get_current_streaming_master_index())) >= 15)
    {
        char *m_last_url = (char*)app_flash_get_last_streaming_url();
        if (strlen(m_last_url) > 10)
        {
            sprintf(app_audio_get_stream_url(), "%s", m_last_url);
        }
        else
        {
        sprintf(app_audio_get_stream_url(),
                "%s%s", 
                app_flash_get_http_stream_header(), 
                app_flash_get_master(app_flash_get_current_streaming_master_index()));
        }
    }
    else
    {
        app_audio_set_invalid_link();
    }

    network_enable_wifi(app_flash_is_wifi_enable());
    network_wifi_info_t info;
    info.ssid = app_flash_get_wifi_name();
    info.password = app_flash_get_wifi_pass();

    /* Khởi tạo audio codec để khởi tạo I2C điều khiển PCF */
    bool retval = app_audio_board_init();
    /*Known issued:
    (327) psram: This chip is ESP32-D0WD
    (328) spiram: SPI RAM enabled but initialization failed. Bailing out.
    (328) cpu_start: Failed to init external RAM; continuing without it.
    //Case khởi tạo thành công PSRAM:
    (299) spiram: Found 64MBit SPI RAM device
    (299) spiram: SPI RAM mode: flash 80m sram 80m
    (301) spiram: PSRAM initialized, cache is in low/high (2-core) mode.
    ....
    (444) AUDIO_BOARD_BYT_V102: audio_board_init...
    (454) AUDIO_BOARD_BYT_V102: ../esp/esp-adf-v2.0-beta2/components/audio_board/lyrat_v4_3_byt_v102/board.c:47 (audio_board_init): Memory exhausted[1B][0m
    (464) i2c: ../esp/esp-adf-v2.0-beta2/esp-idf/components/driver/i2c.c:1267 (i2c_master_cmd_begin):i2c driver not installed[1B][0m
    (484) [LIVEDOWN, 5857]: PCF8575 init: ERR[1B][0m
    (*) REASON: Do lỗi khởi tạo PSRAM dẫn đến khi khởi tạo audio_board_init() không đủ bộ nhớ để calloc -> board_handle null
        => không chạy được hàm audio_board_codec_init() -> không khởi tạo I2C và audio codec được -> init PCF8575 cũng failed luôn!
    (*) TODO: check board_handle sau khi call audio_board_init(), nếu = NULL là FAILED, lưu lại nguyên nhân và reset!
    */
    if (!retval)
    {
        DEBUG_WARN("---------> [ERROR] Cannot init audio_board_init() because PSRAM init failed. Reboot...\r\n");
        DEBUG_VERBOSE("[ZIG] PSRAM: Error\r\n");
        system_software_reset(SW_RESET_REASON_PSRAM_FAIL);
    }
    else
    {
        // DEBUG_INFO("[ 2 ] Init board_handle: OK!");
        // DEBUG_VERBOSE("[ZIG] PSRAM: OK\r\n");
    }

    app_io_set_hardware_version(3);

    /* ========================= Khởi tạo GPIO mở rộng kèm detect version PCF ====================== */
    /**
     * Nếu khởi động do tắt nguồn và bật lại -> mặc định OFF các relay
     * Sau khi update firmware -> mặc định OFF các relay
     */
    bool save_opto_state = false;
    if (cpu1_reset_reason == 14)
    {
        if (cpu0_reset_reason == 1 || cpu0_reset_reason == 16)
        {
            DEBUG_INFO("CPU reset by power on, turn off all relay...\r\n");
            app_io_get_io_value()->Name.IO1 = APP_IO_OPTO_OFF;
            app_io_get_io_value()->Name.IO2 = APP_IO_OPTO_OFF;
            save_opto_state = true;
        }
    }
    if (system_get_software_reset_reason() == SW_RESET_REASON_OTA_FINISH)
    {
        DEBUG_INFO("CPU reset by OTA, turn off all relay...\r\n");
        app_io_get_io_value()->Name.IO1 = APP_IO_OPTO_OFF;
        app_io_get_io_value()->Name.IO2 = APP_IO_OPTO_OFF;
        save_opto_state = true;
    }
    if (save_opto_state)
    {
        app_flash_write_u8(APP_FLASH_IO_STATE_KEY, app_io_get_io_value()->Value);
    }

    app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_OFF;
    app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
    app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;
    app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;

    /* Các chân ISO Ouput khởi tạo theo cấu hình cài đặt */
    /* Mặc định khởi tạo OFF, khi nào chạy stream hoặc MIC/LINE thì bật */
    // app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_OPTO_OFF;
    // app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_OPTO_OFF;
    //-> Điều khiển relay về trạng thái ON/OFF trước đó
    app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = app_io_get_io_value()->Name.IO1;
    app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = app_io_get_io_value()->Name.IO2;

    // GSM OFF
    app_io_get_i2c_exp_value()->BitName.gsm_pw_en = 0;
    app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 0;

    // Swith Audio input relay
    app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 0; // Default to AUX input

    // Switch Ouput Codec - FM
    app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 0; // Relay to Codec ouput

    // PA enable
    app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;

    // Input
    app_io_get_i2c_exp_value()->BitName.ISO_IN1 = 1;
    app_io_get_i2c_exp_value()->BitName.ISO_IN2 = 1;
    app_io_get_i2c_exp_value()->BitName.BTN_RESET = 1;
    app_io_get_i2c_exp_value()->BitName.BTN_SET = 1;

    app_io_button_on_air_adc_start();

    Int_t ioEx;
    ioEx.value = app_io_get_i2c_exp_value()->Value;
    uint8_t retry_detect_num = 10;
    while (retry_detect_num > 0)
    {
        retry_detect_num--;
        err = pcf_i2c_write(I2C_NUM_0, PCF8575_I2C_ADDR, ioEx.bytes, 2);
        // DEBUG_INFO("[ 3 ] Init PCF8575: %s", err == ESP_OK ? "OK" : "ERR");

        if (err != ESP_OK)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        else
        {
            app_io_set_hardware_version(2);
            break;
        }
    }
    DEBUG_INFO("[ 3 ] Detect hardware version = %d", app_io_get_hardware_version());
    DEBUG_VERBOSE("[ZIG] IOEXT_VERSION: %s\r\n", app_io_get_hardware_version() == 2 ? "PCF85xx" : "MCU");

    for (int32_t i = 10; i > 0; i--)
    {
        app_audio_change_codec_vol(app_flash_get_volume()/i);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    app_audio_change_codec_vol(app_flash_get_volume());

    app_io_adc_poll();

    // if (HW_VERSION == 2) {
    //	//========================== Khởi tạo PCF8575 ===========================//
    //	app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_OFF;
    //	app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
    //	app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;
    //	app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
    //
    //	/* Các chân ISO Ouput khởi tạo theo cấu hình cài đặt */
    //	/* Mặc định khởi tạo OFF, khi nào chạy stream hoặc MIC/LINE thì bật */
    //	// app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_OPTO_OFF;
    //	// app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_OPTO_OFF;
    //	//-> Điều khiển relay về trạng thái ON/OFF trước đó
    //	app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = app_io_get_io_value()->Name.IO1;
    //	app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = app_io_get_io_value()->Name.IO2;
    //
    //	//GSM OFF
    //	app_io_get_i2c_exp_value()->BitName.gsm_pw_en = 0;
    //	app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 0;
    //
    //	//Swith Audio input relay
    //	app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 0;		//Default to AUX input
    //
    //	//Switch Ouput Codec - FM
    //	app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 0;		//Relay to Codec ouput
    //
    //	//PA enable
    //	app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;
    //
    //	//Input
    //	app_io_get_i2c_exp_value()->BitName.ISO_IN1 = 1;
    //	app_io_get_i2c_exp_value()->BitName.ISO_IN2 = 1;
    //	app_io_get_i2c_exp_value()->BitName.BTN_RESET = 1;
    //	app_io_get_i2c_exp_value()->BitName.BTN_SET = 1;
    //
    //	Int_t ioEx;
    //	ioEx.value = app_io_get_i2c_exp_value()->Value;
    //	err = pcf_i2c_write(I2C_NUM_0, PCF8575_I2C_ADDR, ioEx.bytes, 2);
    //	ESP_LOGI*TAG,  "[ 3 ] Init PCF8575: %s", err == ESP_OK ? "OK" : "ERR");
    //	vTaskDelay(1000 / portTICK_PERIOD_MS);
    //	/* ======================= End of PCF8575 ===================================*/
    // }

    // xSystem.Status.Speaker.Value = 255;
    /* Get Unique ID */
    uint8_t uid[6];
    err = esp_efuse_mac_get_default(uid);
    if (err == ESP_OK)
    {
        char mac_string[13];
        //		ESP_LOGI*TAG,  "ESP unique MAC: %02X:%02X:%02X:%02X:%02X:%02X", uid[0],
        //			uid[1], uid[2], uid[3],
        //			uid[4], uid[5]);

        DEBUG_INFO("[ZIG] MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", 
                uid[0], uid[1], uid[2], 
                uid[3], uid[4], uid[5]);
        sprintf (mac_string, "%02X%02X%02X%02X%02X%02X", uid[0], uid[1], uid[2], 
                uid[3], uid[4], uid[5]);
#warning "mac string haven't write into flash "
        DEBUG_ERROR("[ZIG] MAC: %s\r\n", mac_string);
        // app_flash_set_imei(mac_string);
    }
    else
    {
        DEBUG_WARN("[ZIG] MAC: ERROR\r\n");
    }
    //===================================================================================================//
    if (app_io_get_hardware_version() == 3) // Khởi tạo UART giao tiếp STM32/GD32 trước để điều khiển IO
    {
        /**
         * @brief uart handle task: Giao tiếp UART module FM
         */
        app_sntp_start();
        app_io_init_min_protocol();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        app_io_set_default_value();
    }
    /* ======================End of Khởi tạo GPIO mở rộng ======================*/
    network_initialize(&info, network_event_cb);

    app_mqtt_initialize();


    /**
     * @brief Tạo task quản lý hoạt động của main
     */
    xTaskCreate(main_manager_task, "main_manager_task", 5 * 1024, NULL, 5, NULL);

    if (app_io_get_hardware_version() == 2)
    { // Khởi tạo UART giao tiếp FM module sau khi đã khởi tạo xong audio codec
        /**
         * @brief uart handle task: Giao tiếp UART module FM
         */
        app_io_expander_uart_initialize();
        xTaskCreate(app_io_exp_uart_task, "fm_uart_task", 3 * 1024, NULL, 5, NULL);
    }

    app_io_allow_process_fm_data();

    /* Task http stream */
    live_streaming_down_task(NULL);
}


