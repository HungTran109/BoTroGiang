#include "app_audio.h"
#include "app_flash.h"
#include "esp_log.h"
#include "wav_encoder.h"
#include "opus_encoder.h"
#include "opus_decoder.h"
#include "aac_decoder.h"
#include "esp_peripherals.h"
#include "sdkconfig.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_common.h"
#include "board.h"
#include "version_ctrl.h"
#include "app_io.h"
#include "periph_button.h"
#include <string.h>
#include "i2s_stream.h"
#include "http_stream.h"
#include "app_mqtt.h"
#include "esp_http_client.h"
#include "main.h"
#include "pcf8575.h"
#include "app_debug.h"

// static const char *TAG = "app_audio";

// static esp_periph_handle_t m_button_handle;
static esp_periph_set_handle_t m_periph_set_handle;
audio_event_iface_handle_t m_evt;
static audio_board_handle_t m_board_handle = NULL;
static uint8_t m_output_vol = 70;

/* Variables for audio streamer */
static audio_pipeline_handle_t m_pipeline;
static audio_element_handle_t m_http_stream_reader, m_i2s_stream_writer;
static audio_element_handle_t m_opus_decoder;
static uint8_t m_auto_restart_stream_retries_number = 0;
static app_audio_codec_mode_t m_codec_mode;


// typedef enum {
//     AEL_STATE_NONE          = 0,
//     AEL_STATE_INIT          = 1,
//     AEL_STATE_INITIALIZING  = 2,
//     AEL_STATE_RUNNING       = 3,
//     AEL_STATE_PAUSED        = 4,
//     AEL_STATE_STOPPED       = 5,
//     AEL_STATE_FINISHED      = 6,
//     AEL_STATE_ERROR         = 7
// } audio_element_state_t;

static const char *m_ael_state_des[] = 
{
    "NONE",
    "INIT",
    "INIT",//INITIALIZING",
    "RUNNING",
    "PAUSED",
    "INIT", //"FINISH", // "STOP",
    "INIT", //"FINISH",
    "AUE",      // ERROR
    "NA"
};

static char m_http_stream_url[APP_AUDIO_HTTP_URL_SIZE] = {0};
static char m_alternative_stream_url[APP_FLASH_MASTER_TOTAL][APP_AUDIO_HTTP_URL_SIZE] = {0};
static bool m_is_http_reader_stopped = true;
static uint8_t m_streaming_step = APP_AUDIO_STREAM_NOT_INIT;
static char m_last_url_link_404[256];
static uint32_t m_delay_force_turn_off_pa_after_404 = 0;
static bool m_stream_url_404 = false;

char *app_audio_get_last_link_404(void)
{
    return m_last_url_link_404;
}

uint32_t app_audio_get_force_turn_off_pa_remain_time()
{
    return m_delay_force_turn_off_pa_after_404;
}

app_audio_codec_mode_t app_audio_get_codec_mode(void)
{
    return m_codec_mode;
}

bool app_audio_is_http_reader_stopped(void)
{
    return m_is_http_reader_stopped;
}

uint8_t app_audio_get_streaming_logic_step(void)
{
    return m_streaming_step;
}

void app_audio_set_streaming_logic_step(uint8_t step)
{
    m_streaming_step = step;
}

void app_audio_set_http_stream_stopped_flag(void)
{
    m_is_http_reader_stopped = true;
    m_streaming_step = APP_AUDIO_STREAM_STOPPED;
    slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false); /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
}   

void app_audio_set_invalid_link(void)
{
    sprintf(m_http_stream_url, 
            "%s%s", 
            app_flash_get_http_stream_header(), 
            APP_FLASH_INVALID_LINK);
}

char *app_audio_get_stream_url(void)
{
    return m_http_stream_url;
}

char *app_audio_get_alt_stream_url(int master)
{
    if (master >= APP_FLASH_MASTER_TOTAL)
    {
        return APP_FLASH_INVALID_LINK;
    }
    return &m_alternative_stream_url[master][0];
}

void app_audio_set_alt_stream_url(int master, char *url)
{
    if (master < APP_FLASH_MASTER_TOTAL)
    {
        strncpy(&m_alternative_stream_url[master][0], url, APP_AUDIO_HTTP_URL_SIZE);
    }
}

void app_audio_set_stream_retry(uint8_t times)
{
    m_auto_restart_stream_retries_number = times;
}

uint8_t app_audio_get_reset_stream_retry_number(void)
{
    return m_auto_restart_stream_retries_number;
}

int _http_stream_event_handle(http_stream_event_msg_t *msg)
{
    if (msg->event_id == HTTP_STREAM_RESOLVE_ALL_TRACKS)
    {
        DEBUG_INFO("HTTP_STREAM_RESOLVE_ALL_TRACKS\r\n");
        return ESP_OK;
    }

    if (msg->event_id == HTTP_STREAM_FINISH_TRACK)
    {
        DEBUG_INFO("FINISH_TRACK\r\n");

        /** Sự kiện gọi khi stream hết dữ liệu live stream hoặc hết file trên server, hoặc đang stream thì master tắt nguồn */

        slave_on_hls_finish_track_cb();

        return http_stream_next_track(msg->el);
    }
    if (msg->event_id == HTTP_STREAM_FINISH_PLAYLIST)
    {
        DEBUG_INFO("HTTP_STREAM_FINISH_PLAYLIST");
        return http_stream_restart(msg->el);
    }

    /*!< The event handler will be called when HTTP Client is receiving data
     * If the fucntion return the value (-1: ESP_FAIL), HTTP Client will be stopped
     * If the fucntion return the value > 0, HTTP Stream will ignore the read function
     * If the fucntion return the value = 0, HTTP Stream continue read data from HTTP Server
     */
    if (msg->event_id == HTTP_STREAM_ON_RESPONSE)
    {
        m_stream_url_404 = false;
        slave_increase_stream_data_downloaded(msg->buffer_len);
        memset(m_last_url_link_404, 0, sizeof(m_last_url_link_404));
        // Reset 404 timeout
        if (m_delay_force_turn_off_pa_after_404 > 1) 
        {
            m_delay_force_turn_off_pa_after_404 = 1;
        }
    }
    else if (msg->event_id == HTTP_STREAM_FINISH_ON_404_NOT_FOUND)
    {
        m_stream_url_404 = true;
        DEBUG_WARN("%s 404 not found\r\n", m_http_stream_url);
    }
    return ESP_OK;
}

void app_audio_reset_force_turn_off_pa_when_link_404_not_found()
{
    if (m_delay_force_turn_off_pa_after_404 > 1) // for task safe
    {
        m_delay_force_turn_off_pa_after_404 = 1;
    }
}


audio_element_state_t app_audio_get_opus_state(void)
{
    return audio_element_get_state(m_opus_decoder);
}


bool app_audio_is_opus_running(void)
{
    bool retval = false;
    if (m_opus_decoder && audio_element_get_state(m_opus_decoder) == AEL_STATE_RUNNING)
    {
        retval = true;
    }
    return retval;
}

audio_element_state_t app_audio_get_i2s_state(void)
{
    return audio_element_get_state(m_i2s_stream_writer);
}

bool app_audio_is_i2s_running(void)
{
    bool retval = false;
    if (m_i2s_stream_writer && audio_element_get_state(m_i2s_stream_writer) == AEL_STATE_RUNNING)
    {
        retval = true;
    }
    return retval;
}

bool app_audio_board_init(void)
{
    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    m_periph_set_handle = esp_periph_set_init(&periph_cfg);

    m_board_handle = audio_board_init(0);
    return m_board_handle ? true : false;
}

uint8_t app_audio_is_http_audio_stream_running(void)
{
    uint8_t retval = 0;
    if (m_i2s_stream_writer && m_opus_decoder && m_http_stream_reader) 
    {
        audio_element_state_t el_http_state = audio_element_get_state(m_http_stream_reader);
        if (el_http_state == AEL_STATE_RUNNING)
        {
            retval = 1;
        }
    }
    return retval;
}


void app_audio_hal_i2c_master_write(uint32_t addr, uint8_t *data, uint32_t size)
{
    audio_hal_i2c_master_write(m_board_handle->audio_hal, addr, data, size);
}

uint8_t app_audio_get_current_output_vol(void)
{
    return m_output_vol;
}

void app_audio_set_output_volume(uint8_t vol)
{
    m_output_vol = vol;
}

audio_element_state_t app_audio_get_http_state(void)
{
    return audio_element_get_state(m_http_stream_reader);
}


static bool m_codec_error = false;
bool app_audio_is_codec_error(void)
{
    return m_codec_error;
}

uint8_t app_audio_change_codec_vol(uint8_t set_vol)
{
    // Set volume cho audio codec
    // 1. Đọc lại giá trị volume hiện tại, nếu không khác nhiều so với volume set -> không set nữa
    int get_vol = 0;
    esp_err_t err = audio_hal_get_volume(m_board_handle->audio_hal, &get_vol);
    DEBUG_VERBOSE("Get volume: %u : %d", get_vol, err);

    if (err == 0)
    {
        // DEBUG_INFO("[ZIG] AUDIO_CODEC: OK, VOLUME: %d\r\n", get_vol);
    }
    else
    {
        // DEBUG_INFO("[ZIG] AUDIO_CODEC: ERROR\r\n");
        m_codec_error = true;
    }

    if (abs(set_vol - get_vol) <= 3)
    {
        DEBUG_VERBOSE("curVol is same setVol, don't change: %u-%u", get_vol, set_vol);
        get_vol = set_vol;
        m_output_vol = get_vol & 0xFF;
        return get_vol;
    }

    // 2. Nếu volume hiện tại != set_vol -> set mức mới
    err = audio_hal_set_volume(m_board_handle->audio_hal, set_vol);
    DEBUG_VERBOSE("Set volume codec to %u : %d", set_vol, err);

    // Read volume again
    get_vol = 0;
    err = audio_hal_get_volume(m_board_handle->audio_hal, &get_vol);
    DEBUG_VERBOSE("Get volume again: %u : %d", get_vol, err);

    if (abs(set_vol - get_vol) <= 3)
    {
        get_vol = set_vol;
    }
    m_output_vol = get_vol & 0xFF;
    return m_output_vol;
}

void app_audio_simple_terminate_pipeline(void)
{
    DEBUG_INFO("Reset ringbuffer\r\n");
    audio_pipeline_reset_ringbuffer(m_pipeline);
    audio_pipeline_terminate(m_pipeline);
}

void app_audio_run_new_url(char *uri)
{
    DEBUG_INFO("Run new url %s\r\n", uri);
    audio_element_set_uri(m_http_stream_reader, uri);

    audio_element_reset_state(m_i2s_stream_writer);
    audio_element_reset_state(m_opus_decoder);
    audio_element_reset_state(m_http_stream_reader);

    audio_pipeline_reset_ringbuffer(m_pipeline);
    audio_pipeline_reset_elements(m_pipeline);
    audio_pipeline_run(m_pipeline);

}

void app_audio_pause(void)
{
   audio_pipeline_pause(m_pipeline); 
}

void app_audio_run_pipeline(void)
{
   audio_pipeline_run(m_pipeline); 
}


void app_audio_complete_terminate(void)
{
    audio_pipeline_stop(m_pipeline);
    audio_pipeline_wait_for_stop(m_pipeline);
    audio_element_reset_state(m_opus_decoder);
    audio_element_reset_state(m_i2s_stream_writer);
    audio_element_reset_state(m_http_stream_reader);
    audio_pipeline_reset_ringbuffer(m_pipeline);
    audio_pipeline_reset_items_state(m_pipeline);
    audio_pipeline_terminate(m_pipeline);
}

void app_audio_stop_pipeline(void)
{
    audio_pipeline_stop(m_pipeline);
    audio_pipeline_wait_for_stop(m_pipeline);
    audio_element_reset_state(m_opus_decoder);
    audio_element_reset_state(m_i2s_stream_writer);
    audio_element_reset_state(m_http_stream_reader);
    audio_pipeline_reset_ringbuffer(m_pipeline);
    audio_pipeline_reset_items_state(m_pipeline);
}


/**
 * Chuyển sang chạy chế độ thu từ FM
 *	- Codec về chế độ internet
 *	- Relay input chuyển về line AUX
 *	- Relay output chuyển về FM output
 *	- Enable PA
 */
void app_audio_change_to_local_fm_mode(void)
{
    // Change codec mode to LINE2 - CODEC -> OFF MIC passthrough
    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

    app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 0;  // Switch Relay to AUX input
    app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 1; // Switch Relay to FM output
    if (!app_io_is_speaker_error())
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_ON;   // Enable PA
        app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_STM_PA_ON;       // Enable PA
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;   // Disable PA
        app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_STM_PA_OFF;       // Disable PA
    }

    // STM32
    app_io_get_mcu_exp_value()->BitName.sw_mic_aux = APP_IO_STM_AUX_ON; // Switch Relay to AUX input
    app_io_get_mcu_exp_value()->BitName.sw_codec_fm = APP_IO_STM_FM_ON; // Switch Relay to FM output

    if (app_io_get_hardware_version() == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (app_io_get_hardware_version() == 3)
    {
        app_io_mcu_update_io(app_io_get_mcu_exp_value());
    }
}


/**
 * Chuyển sang chạy chế độ thu từ LINE AUX
 *	- Codec về chế độ APP_AUDIO_OPERATION_MODE_LINE
 *	- Relay input chuyển về line AUX
 *	- Relay output chuyển về Codec output
 *	- Enable PA
 */
void app_audio_change_to_local_line_in_mode(void)
{
    // Change codec mode to LINE1 - MIC -> MIC passthrough
    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_LINE);

    // Set input adc channel for codec: LINE2: AUX
    audio_hal_set_input(m_board_handle->audio_hal, APP_AUDIO_LINE_IN_AUX);

    app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 0;  // Switch Relay to AUX input
    app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 0; // Switch Relay to Codec output
    if (!app_io_is_speaker_error())
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_ON;   // Enable PA
        app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_STM_PA_ON;          // Enable PA
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;   // Disable PA
        app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_STM_PA_OFF;          // Disable PA
    }

    // STM32
    app_io_get_mcu_exp_value()->BitName.sw_mic_aux = APP_IO_STM_AUX_ON;    // Switch Relay to AUX input
    app_io_get_mcu_exp_value()->BitName.sw_codec_fm = APP_IO_STM_CODEC_ON; // Switch Relay to Codec output
    
    if (app_io_get_hardware_version() == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (app_io_get_hardware_version() == 3)
    {
        app_io_mcu_update_io(app_io_get_mcu_exp_value());
    }
}


/**
 * Chuyển sang chạy chế độ thu từ MIC
 *	- Codec về chế độ APP_AUDIO_OPERATION_MODE_MIC
 *	- Relay input chuyển về line AUX
 *	- Relay output chuyển về FM output
 *	- Enable PA
 */
void app_audio_change_to_local_mic_mode(void)
{
    // Change codec mode to LINE1 - MIC -> MIC passthrough
    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_MIC);

    // Set input adc channel for codec: LINE1: MIC
    audio_hal_set_input(m_board_handle->audio_hal, APP_AUDIO_LINE_IN_MIC);

    app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 1;  // Switch Relay to MIC input
    app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 0; // Switch Relay to Codec output
    if (!app_io_is_speaker_error())
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_ON;   // Enable PA
        app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_STM_PA_ON;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;   // Disable PA
        app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_STM_PA_OFF;
    }

    // STM32
    app_io_get_mcu_exp_value()->BitName.sw_mic_aux = APP_IO_STM_MIC_ON;    // Switch Relay to AUX input
    app_io_get_mcu_exp_value()->BitName.sw_codec_fm = APP_IO_STM_CODEC_ON; // Switch Relay to codec output

    if (app_io_get_hardware_version() == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (app_io_get_hardware_version() == 3)
    {
        app_io_mcu_update_io(app_io_get_mcu_exp_value());
    }
}

/******************************************************************************************/
/**
 * @brief :	Chuyển chế độ làm việc của Audio codec: DECODE vs LINE_IN
 */
void app_audio_change_codec_mode(app_audio_operation_mode_t mode)
{
    esp_err_t err;

    /* Chuyển sang dùng MIC/LINE passthu -> Stop mode CODEC_DECODE, start mode LINE_IN */
    if (mode == APP_AUDIO_OPERATION_MODE_MIC || mode == APP_AUDIO_OPERATION_MODE_LINE)
    {
        err = audio_hal_ctrl_codec(m_board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_STOP);
        DEBUG_VERBOSE("Stop 'AUDIO_HAL_CODEC_MODE_DECODE' %s\r\n", err == ESP_OK ? "OK" : "ERR");

        err = audio_hal_ctrl_codec(m_board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
        DEBUG_VERBOSE("Start 'AUDIO_HAL_CODEC_MODE_LINE_IN' %s\r\n", err == ESP_OK ? "OK" : "ERR");

        if (err == ESP_OK)
        {
            m_codec_mode = APP_AUDIO_CODEC_MODE_LINE_IN;
        }
    }
    else if (mode == APP_AUDIO_OPERATION_MODE_INTERNET)
    {
        err = audio_hal_ctrl_codec(m_board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_STOP);
        DEBUG_VERBOSE("Stop 'AUDIO_HAL_CODEC_MODE_LINE_IN' %s\r\n", err == ESP_OK ? "OK" : "ERR");

        err = audio_hal_ctrl_codec(m_board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
        DEBUG_VERBOSE("Start 'AUDIO_HAL_CODEC_MODE_DECODE\r\n' %s", err == ESP_OK ? "OK" : "ERR");

        if (err == ESP_OK)
        {
            m_codec_mode = APP_AUDIO_CODEC_MODE_DECODE;
        }
    }
}


uint8_t app_audio_test_mode_change_codec_vol(uint8_t setVolume)
{
    // Set volume cho audio codec
    int get_vol = 0;
    // 1. Đọc lại giá trị volume hiện tại, nếu không khác nhiều so với volume set -> không set nữa
    esp_err_t err = audio_hal_get_volume(m_board_handle->audio_hal, &get_vol);
    DEBUG_INFO("Test mode get volume: %u : %d", get_vol, err);

    if (err == 0)
    {
        DEBUG_VERBOSE("[ZIG] AUDIO_CODEC: OK, VOLUME: %d\r\n", get_vol);
    }
    else
    {
        DEBUG_WARN("[ZIG] AUDIO_CODEC: ERROR\r\n");
        m_codec_error = true;
    }

    if (abs(setVolume - get_vol) <= 3)
    {
        DEBUG_INFO("curVol is same setVol, don't change: %u-%u", get_vol, setVolume);
        get_vol = setVolume;
        return get_vol;
    }

    // 2. Nếu volume hiện tại != setVolume -> set mức mới
    err = audio_hal_set_volume(m_board_handle->audio_hal, setVolume);
    DEBUG_VERBOSE("Set volume codec to %u : %d", setVolume, err);

    // Read volume again
    get_vol = 0;
    err = audio_hal_get_volume(m_board_handle->audio_hal, &get_vol);
    DEBUG_VERBOSE("Get volume again: %u : %d", get_vol, err);

    if (abs(setVolume - get_vol) <= 3)
    {
        get_vol = setVolume;
    }
    get_vol = get_vol & 0xFF;
    app_flash_set_volume(get_vol);
    return get_vol;
}


/**
 * Chuyển sang chạy chế độ thu từ INTERNET
 *	- Codec về chế độ MODE_DECODE
 *	- Relay output chuyển về INTERNET output
 *	- Enable PA
 */
void app_audio_change_to_stop_mode(void)
{
    // Change codec mode to DECODE
    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

    // Change codec volume to mute
    app_audio_change_codec_vol(0);

    // Change relay output and PA enable
    app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 0;  // Switch Relay to AUX input
    app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 0; // Switch Relay to Codec output
    app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;  // Disable PA

    // STM32
    app_io_get_mcu_exp_value()->BitName.sw_mic_aux = APP_IO_STM_AUX_ON;    // Switch Relay to AUX input
    app_io_get_mcu_exp_value()->BitName.sw_codec_fm = APP_IO_STM_CODEC_ON; // Switch Relay to Codec output
    app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_STM_PA_OFF;         // Disable PA

    if (app_io_get_hardware_version() == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (app_io_get_hardware_version() == 3)
    {
        app_io_mcu_update_io(app_io_get_mcu_exp_value());
    }
}

/**
 * Chuyển sang chạy chế độ thu từ INTERNET
 *	- Codec về chế độ MODE_DECODE
 *	- Relay output chuyển về INTERNET output
 *	- Enable PA
 */
void app_audio_change_codec_to_internet_mode(void)
{
    // Change codec mode to DECODE
    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

    // Change codec volume to setting volume
    app_audio_change_codec_vol(app_flash_get_volume());

    // Change relay output and PA enable
    app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 0; // Switch Relay to Codec output
    app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_ON;   // Enable PA

    app_io_get_mcu_exp_value()->BitName.sw_codec_fm = APP_IO_STM_CODEC_ON; // Switch Relay to Codec output
    app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_PA_ON;              // Enable PA
    if (!app_io_is_speaker_error())
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;   // Disable PA
        app_io_get_mcu_exp_value()->BitName.en_pa = APP_IO_PA_OFF;   // Disable PA
    }

    if (app_io_get_hardware_version() == 2)
    {
        Int_t exOut;
        exOut.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, exOut.bytes, 2);
    }
    else if (app_io_get_hardware_version() == 3)
    {
        app_io_mcu_update_io(app_io_get_mcu_exp_value());
    }
}

void app_audio_restart_pipeline(char *url)
{
    DEBUG_INFO("Re-start pipeline\r\n");
    DEBUG_VERBOSE("%s\r\n", url);
    audio_element_set_uri(m_http_stream_reader, url);

    audio_element_reset_state(m_i2s_stream_writer);
    audio_element_reset_state(m_opus_decoder);
    audio_element_reset_state(m_http_stream_reader);

    audio_pipeline_reset_ringbuffer(m_pipeline);
    audio_pipeline_reset_elements(m_pipeline);
    audio_pipeline_run(m_pipeline);
}

const char *app_audio_get_http_state_description(void)
{
    audio_element_state_t el_http_state = audio_element_get_state(m_http_stream_reader);
    const char *state_str = m_ael_state_des[el_http_state];
    return state_str;
}

bool app_audio_wait_for_event(uint32_t timeout_ms)
{
    /* Handle event interface messages from pipeline to set music info and to advance to the next song
         * Ngồi lắng nghe có sự kiện thì mới chạy tiếp!
         */
    static uint32_t m_last_tick;
    uint32_t now = xTaskGetTickCount();
    if (now - m_last_tick >= (uint32_t)1000)
    {
        if (m_delay_force_turn_off_pa_after_404)
        {
            m_delay_force_turn_off_pa_after_404--;
            if (m_delay_force_turn_off_pa_after_404 == 0)
            {
                // mqtt_publish_message("DBG", "Timeout force turn off PA is over");
                if (app_audio_is_http_audio_stream_running())
                {
                    app_io_control_pa(APP_IO_PA_ON);
                }
            }
            else
            {
                app_io_control_pa(APP_IO_PA_OFF);
                if (m_delay_force_turn_off_pa_after_404 % 60 == 0)       // giam so ban tin debug
                {
                    // mqtt_publish_message("DBG", "Timeout force turn off PA : %u", 
                    //                         m_delay_force_turn_off_pa_after_404);
                }
            }
        }
        m_last_tick = now;
    }

    audio_event_iface_msg_t msg;
    esp_err_t ret = audio_event_iface_listen(m_evt, &msg, timeout_ms);
    if (ret != ESP_OK)
    {
        return false;
    }

    /* ================================ Xử lý sự kiện từ audio element ================================= */
    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT
        && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO
        && (msg.source == (void *)m_opus_decoder))
    {
        audio_element_info_t music_info = {0};

        if (msg.source == (void *)m_opus_decoder)
        {
            audio_element_getinfo(m_opus_decoder, &music_info);
            mqtt_publish_message("DBG", "Music info from opus, sample_rates=%d, bits=%d, ch=%d",
                                    music_info.sample_rates, music_info.bits, music_info.channels);
        }
        audio_element_setinfo(m_i2s_stream_writer, &music_info);
        i2s_stream_set_clk(m_i2s_stream_writer, 
                            music_info.sample_rates, 
                            music_info.bits, 
                            music_info.channels);
        return true;
    }

    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT)
    {
        audio_element_state_t el_state;
        audio_element_status_t el_status;

        /* Các sự kiện đến từ source: m_i2s_stream_writer */
        if (msg.source == (void *)m_i2s_stream_writer)
        {
            DEBUG_VERBOSE("AEL event, source: i2s_wr, cmd: %d\r\n", msg.cmd);

            switch (msg.cmd)
            {
            case AEL_MSG_CMD_NONE: /* 0 */
                break;
            // case AEL_MSG_CMD_ERROR:	/* 1 */
            // 	break;
            case AEL_MSG_CMD_FINISH: /* 2 */
            case AEL_MSG_CMD_STOP: /* 3 */
            case AEL_MSG_CMD_PAUSE: /* 4 */
            case AEL_MSG_CMD_RESUME: /* 5 */
            case AEL_MSG_CMD_DESTROY: /* 6 */
                break;
            case AEL_MSG_CMD_REPORT_STATUS: /* 8 */
                el_state = audio_element_get_state(m_i2s_stream_writer);
                el_status = (int)msg.data;

                DEBUG_INFO("i2s_wr, state: %d, status: %d\r\n", el_state, el_status);
                switch (el_state)
                {
                case AEL_STATE_NONE: // 0
                case AEL_STATE_INIT: // 1
                    break;
                case AEL_STATE_RUNNING: // 2
                    DEBUG_INFO("i2s: RUNNING\r\n");
                    break;
                case AEL_STATE_PAUSED: // 3
                    break;
                case AEL_STATE_STOPPED: // 4 -> Sau lệnh "audio_pipeline_terminate()"
                    DEBUG_INFO("i2s_wr: STOPPED\r\n");
                    slave_set_timeout_turn_off_opto_output(15);
                    m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                    break;
                case AEL_STATE_FINISHED: // 5
                    m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                    break;
                case AEL_STATE_ERROR: // 6
                    break;
                default:
                    break;
                }
                break;
            case AEL_MSG_CMD_REPORT_MUSIC_INFO: /* 9 */
            case AEL_MSG_CMD_REPORT_CODEC_FMT: /* 10 */
            case AEL_MSG_CMD_REPORT_POSITION: /* 11 */
            default:
                break;
            }
        }

        /* Các sự kiện đến từ source: opus_decoder */
        if (msg.source == (void *)m_opus_decoder)
        {
            DEBUG_INFO("AEL event, source: decoder, cmd: %d\r\n", msg.cmd);

            switch (msg.cmd)
            {
            case AEL_MSG_CMD_REPORT_STATUS: /* 8 */
                el_state = audio_element_get_state(msg.source);
                el_status = (int)msg.data;
                DEBUG_INFO("decoder, state: %d, status: %d\r\n", el_state, el_status);
                switch (el_state)
                {
                case AEL_STATE_NONE: // 0
                case AEL_STATE_INIT: // 1
                case AEL_STATE_PAUSED: // 3
                    break;
                case AEL_STATE_RUNNING: // 2
                    DEBUG_WARN("decoder: RUNNING\r\n");
                    break;

                case AEL_STATE_STOPPED: // 4 -> Sau lệnh "audio_pipeline_terminate()"
                    DEBUG_WARN("decoder: STOPPED\r\n");
                    slave_set_timeout_turn_off_opto_output(15);
                    m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                    break;
                case AEL_STATE_FINISHED: // 5
                    m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                    break;
                case AEL_STATE_ERROR: // 6
                {
                    DEBUG_WARN("decoder AEL_STATE_ERROR\r\n");
                    m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                    break;
                }
                default:
                    break;
                }
                break;
            case AEL_MSG_CMD_NONE: /* 0 */
            // case AEL_MSG_CMD_ERROR:	/* 1 */
            // 	break;
            case AEL_MSG_CMD_FINISH: /* 2 */
            case AEL_MSG_CMD_STOP: /* 3 */
            case AEL_MSG_CMD_PAUSE: /* 4 */
            case AEL_MSG_CMD_RESUME: /* 5 */
            case AEL_MSG_CMD_DESTROY: /* 6 */
            case AEL_MSG_CMD_REPORT_MUSIC_INFO: /* 9 */
            case AEL_MSG_CMD_REPORT_CODEC_FMT: /* 10 */
            case AEL_MSG_CMD_REPORT_POSITION: /* 11 */
            default:
                break;
            }
        }

        /* Các sự kiện đến từ source: m_http_stream_reader */
        if (msg.source == (void *)m_http_stream_reader)
        {
            DEBUG_INFO("AEL event, src: http_rd, cmd: %d\r\n", msg.cmd);
            el_state = app_audio_get_http_state();
            el_status = (int)msg.data;

            DEBUG_INFO("state: %d, status: %d\r\n", el_state, el_status);
            switch (el_state)
            {
            case AEL_STATE_NONE: // 0
                break;
            case AEL_STATE_INIT: // 1
                DEBUG_VERBOSE("Http reader: INIT\r\n");
                break;
            case AEL_STATE_INITIALIZING: // 2
                DEBUG_VERBOSE("Http reader: initializing\r\n");
                break;
            case AEL_STATE_RUNNING: // 3
                DEBUG_VERBOSE("Http: RUNNING\r\n");
                m_is_http_reader_stopped = false;
                m_auto_restart_stream_retries_number = 0; /** Đã chạy stream được thì reset số lần retry */
                slave_set_timeout_number_of_waiting_master_streaming(3);

                app_io_control_led_stream(APP_AUDIO_STREAM_RUNNING);
                slave_allow_http_element_monitor_downloaded_data_in_streaming_state(true);

                /** Nếu đang cấu hình chạy chế độ FM/MIC/LINE thì chuyển về chế độ INTERNET để LED7 hiển thị đúng */
                app_flash_set_operate_mode(APP_AUDIO_OPERATION_MODE_INTERNET);
                app_io_set_current_fm_module(APP_AUDIO_OPERATION_MODE_BUSY);

                /** Chuyển codec về mode DECODE nếu trước đó bị chuyển sang mode LINE_IN (do module FM/lệnh từ web) */
                if (m_codec_mode != APP_AUDIO_CODEC_MODE_DECODE)
                {
                    app_audio_change_codec_to_internet_mode();
                }
                /* Điều khiển bật loa ngoài khi stream thực sự chạy */
                if (!app_io_is_switch_codec_on())
                {
                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);
                    if (app_audio_get_force_turn_off_pa_remain_time() == 0)
                    {
                        app_io_control_pa(APP_IO_PA_ON);
                    }
                }
                /* Test: Clear timeout tắt relay ngoài (Case: cấu hình stream url mới -> tắt stream đang chạy -> chạy stream mới ) */
                slave_set_timeout_turn_off_opto_output(0);
                break;
            case AEL_STATE_PAUSED: // 4
                DEBUG_WARN("m_http_stream_reader: PAUSED\r\n");
                break;
            case AEL_STATE_STOPPED: // 5 -> Sau lệnh "audio_pipeline_terminate()"
                DEBUG_WARN("m_http_stream_reader: STOPPED\r\n");
                m_is_http_reader_stopped = true;
                slave_set_timeout_turn_off_opto_output(15);
                m_streaming_step = APP_AUDIO_STREAM_STOPPED;

                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false);
                break;

            // HTTP_STREAM: No more data,errno:0, total_bytes:2287681
            // HTTP_STREAM: there are no track
            //=> Terminate pipeline
            case AEL_STATE_FINISHED: // 6
                DEBUG_WARN("m_http_stream_reader: FINISHED\r\n");
                // m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false);
                break;

            case AEL_STATE_ERROR: // 7 -> Sau lệnh "Kill Source" từ Server
            {
                DEBUG_WARN("m_http_stream_reader: ERROR, stream link %s\r\n", m_stream_url_404 ? "not found" : "found");
                audio_element_reset_state(m_http_stream_reader);
                m_is_http_reader_stopped = true;
                if (m_stream_url_404)
                {
                    snprintf(m_last_url_link_404, 256, "Link %s 404", m_http_stream_url);

                    if (!strstr(m_last_url_link_404, APP_FLASH_INVALID_LINK))
                    {
                        m_delay_force_turn_off_pa_after_404 = 90;  // sec
                        mqtt_publish_message("DBG", m_last_url_link_404);
                    }
                    else
                    {
                        if (m_delay_force_turn_off_pa_after_404 > 1) 
                        {
                            m_delay_force_turn_off_pa_after_404 = 1;
                        }
                    }

                    app_flash_write_u8(APP_FLASH_CURRENT_MASTER_KEY, 0);
                    app_flash_set_last_streaming_url(APP_FLASH_INVALID_LINK);
                    // if (slave_get_timeout_turn_off_opto_output() == 0)
                    {
                        slave_set_timeout_turn_off_opto_output(15);
                    }
                    slave_reset_delay_turn_on_relay_process_on_air();
                    m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                    slave_set_received_running_state_timeout(0);
                    app_io_control_pa(APP_IO_PA_OFF);
                }
                else
                {
                    DEBUG_WARN("AEL_STATE_ERROR : restart retries num %d\r\n", m_auto_restart_stream_retries_number);
                    if (m_auto_restart_stream_retries_number == 0)
                    {
                        m_auto_restart_stream_retries_number = 3;
                        m_streaming_step = APP_AUDIO_STREAM_STOPPED; //APP_AUDIO_STREAM_RESTART;
                        snprintf(m_last_url_link_404, 256, "Restart stream link %s", m_http_stream_url);
                        mqtt_publish_message("DBG", m_last_url_link_404);
                    }
                    else
                    {
                        m_streaming_step = APP_AUDIO_STREAM_STOPPED;
                        snprintf(m_last_url_link_404, 256, "%s", "No more retry");
                        app_io_control_pa(APP_IO_PA_OFF);
                        slave_process_stop_onair();
                        mqtt_publish_message("DBG", m_last_url_link_404);
                    }
                    memset(m_last_url_link_404, 0, sizeof(m_last_url_link_404));
                }
                app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false);
            }
                break;
            default:
                break;
            }

            switch (msg.cmd)
            {
            case AEL_MSG_CMD_NONE: /* 0 */
            // case AEL_MSG_CMD_ERROR:	/* 1 */
            // 	break;
            case AEL_MSG_CMD_FINISH: /* 2 */
            case AEL_MSG_CMD_STOP: /* 3 */
            case AEL_MSG_CMD_PAUSE: /* 4 */
            case AEL_MSG_CMD_RESUME: /* 5 */
            case AEL_MSG_CMD_DESTROY: /* 6 */
                break;
            case AEL_MSG_CMD_REPORT_STATUS: /* 8 */
                /* Các trạng thái http_element */
                switch (el_status)
                {
                case AEL_STATUS_NONE: // 0
                    break;
                case AEL_STATUS_ERROR_OPEN: // 1 : HTTP_CLIENT: Connection failed, sock < 0
                    DEBUG_ERROR("http element AEL_STATUS_ERROR_OPEN\r\n");
                    m_stream_url_404 = true;
                    break;
                case AEL_STATUS_ERROR_INPUT: // 2
                case AEL_STATUS_ERROR_PROCESS: // 3
                case AEL_STATUS_ERROR_OUTPUT: // 4
                case AEL_STATUS_ERROR_CLOSE: // 5
                case AEL_STATUS_ERROR_UNKNOWN: // 7
                case AEL_STATUS_INPUT_DONE: // 8
                case AEL_STATUS_INPUT_BUFFERING: // 9
                case AEL_STATUS_OUTPUT_DONE: // 10
                case AEL_STATUS_OUTPUT_BUFFERING: // 11
                case AEL_STATUS_STATE_RUNNING: // 12
                case AEL_STATUS_STATE_PAUSED: // 13
                case AEL_STATUS_STATE_STOPPED: // 14
                    break;
                case AEL_STATUS_STATE_FINISHED: // 15
                    /* In previous versions, audio_pipeline_terminal() was called here. It will close all the elememnt task and when use
                        * the pipeline next time, all the tasks should be restart again. It speed too much time when we switch to another music.
                        * So we use another method to acheive this as below.
                        */

                    // Tắt Relay (nếu được bật ở chế độ LINE IN)
                    //  app_io_opto_control_all(APP_IO_OPTO_OFF);

                    /* Case: Đang stream thì bị FINISHED, có thể do các nguyên nhân:
                        * -> Bên phát dừng phát (do đường truyền, mất nguồn đột ngột...) -> mất link
                        * -> Bên thu bị dừng stream do đường truyền kém, mất mạng...
                        * => Nếu không xử lý gì đến khi bên phát stream trở lại -> bên thu ko start stream lại được!!!
                        * => Thử restart stream ngay khi FINISHED, nếu k được -> terminated
                        * => Chạy audio_pipeline_run() tại đây làm treo CPU0 luôn!!! -> phải chuyển về APP_AUDIO_STREAM_RESTART
                        */
                    // DEBUG_WARN("http stream reader is FINISHED, try to restart stream...\r\n");
                    // mqtt_publish_message("DBG", "Stream finish no more data");
                    m_auto_restart_stream_retries_number = 0;
                    
                    // MUST to use 2 these commands!
                    vTaskDelay(10000 / portTICK_RATE_MS);
                    slave_process_stop_onair();

                    /* Điều khiển tắt loa ngoài khi stream kết thúc, ko tat luon ma delay 5s de flush not buffer audio i2s */
                    // app_io_control_pa(APP_IO_PA_OFF);
                    // audio_pipeline_change_state(m_pipeline, AEL_STATE_INIT);
                    // vTaskDelay(5000 / portTICK_RATE_MS);
                    // m_streaming_step = APP_AUDIO_STREAM_RESTART;
                    app_audio_pause();
                    app_audio_complete_terminate();

                    app_audio_set_streaming_logic_step(APP_AUDIO_STREAM_STOPPED);
                    slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false); /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
                    app_io_control_led_stream(APP_AUDIO_STREAM_STOPPED);
                    // Set PA off
                    app_io_control_pa(APP_IO_PA_OFF);
                    app_audio_set_invalid_link();
                    app_flash_set_last_streaming_url(APP_FLASH_INVALID_LINK);
                    // mqtt_publish_message("DBG", "stream finish, auto stop streaming");
                    return true;
                    // break;

                case AEL_STATUS_MOUNTED: // 16
                case AEL_STATUS_UNMOUNTED: // 17
                    break;
                default:
                    break;
                }
                break;
            case AEL_MSG_CMD_REPORT_MUSIC_INFO: /* 9 */
            case AEL_MSG_CMD_REPORT_CODEC_FMT: /* 10 */
            case AEL_MSG_CMD_REPORT_POSITION: /* 11 */
                break;
            default:
                break;
            }
        }
    }

    /* Restart stream when the first pipeline element (m_http_stream_reader in this case) receives stop event (caused by reading errors) */
    if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT 
        && msg.source == (void *)m_http_stream_reader 
        && msg.cmd == AEL_MSG_CMD_REPORT_STATUS 
        && (int)msg.data == AEL_STATUS_ERROR_OPEN)
    {
        if (m_stream_url_404 || strstr(m_http_stream_url, APP_FLASH_INVALID_LINK))
        {
            DEBUG_WARN("Stream link not found\r\n");
            m_stream_url_404 = false;
            m_auto_restart_stream_retries_number = 0;
        }
        DEBUG_WARN("http_stream_reader is 'AEL_STATUS_ERROR_OPEN'. Restart stream %d times\r\n", 
                    m_auto_restart_stream_retries_number);

        /* Retry start stream 1 số lần, không được thì terminate pipieline */
        if (m_auto_restart_stream_retries_number > 0)
        {
            m_auto_restart_stream_retries_number--;
            app_audio_stop_pipeline();
            // Auto restart pipeline after several seconds
            slave_set_auto_restart_stream_timeout(5);
            return true;
        }
        else
        {
            DEBUG_WARN("Over. TERMINATE Pipeline...!\r\n");
            app_flash_set_current_streaming_master(APP_FLASH_MASTER_TOTAL);
            app_flash_set_last_streaming_url(APP_FLASH_INVALID_LINK);
            slave_set_auto_restart_stream_timeout(0);
            app_audio_complete_terminate();
            /** Nếu tự terminate thì không giám sát lưu lượng stream nữa */
            slave_allow_http_element_monitor_downloaded_data_in_streaming_state(false); 
        }
    }
    return true;
}

void app_audio_start(void)
{
    DEBUG_INFO("[ 1 ] Start audio codec chip\r\n");
    audio_hal_ctrl_codec(m_board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START); // Default: mode DECODE!
    m_codec_mode = APP_AUDIO_CODEC_MODE_DECODE;

    /* Set mode/volume as config value */
    // err = audio_hal_set_volume(m_board_handle->audio_hal, app_flash_get_volume());
    // DEBUG_INFO("Set volume codec to %u : %d", app_flash_get_volume(), err);

    // //Read volume again
    // int get_vol = 0;
    // err = audio_hal_get_volume(m_board_handle->audio_hal, &get_vol);
    // DEBUG_INFO("Get volume gain: %u : %d", get_vol, err);
    // if(abs(app_flash_get_volume() - get_vol) <= 3) {
    // get_vol = app_flash_get_volume();
    // }
    // xSystem.Status.Volume = get_vol & 0xFF;

    // 1.7.1: Call luôn hàm set volume
    app_audio_change_codec_vol(app_flash_get_volume());

    /* Change codec mode */
    switch (app_flash_get_operate_mode())
    {
    case APP_AUDIO_OPERATION_MODE_MIC:
        app_audio_change_to_local_mic_mode();
        break;
    case APP_AUDIO_OPERATION_MODE_LINE:
        app_audio_change_to_local_line_in_mode();
        break;
    case APP_AUDIO_OPERATION_MODE_FM:
        app_audio_change_to_local_fm_mode();
        break;
    default:
        break;
    }

    DEBUG_VERBOSE("[2.0] Create audio pipeline for playback\r\n");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    m_pipeline = audio_pipeline_init(&pipeline_cfg);

    DEBUG_VERBOSE("[2.1] Create http stream to read data\r\n");
    http_stream_cfg_t http_cfg = HTTP_STREAM_CFG_DEFAULT();
    http_cfg.event_handle = _http_stream_event_handle;
    http_cfg.type = AUDIO_STREAM_READER;
    http_cfg.out_rb_size = 48*1024;
    http_cfg.enable_playlist_parser = true;
    m_http_stream_reader = http_stream_init(&http_cfg);

    DEBUG_VERBOSE("[2.2] Create i2s stream to write data to codec chip\r\n");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.out_rb_size = 128*1024;
    m_i2s_stream_writer = i2s_stream_init(&i2s_cfg);
    DEBUG_VERBOSE("[2.3] Create opus decoder to decode opus codec\r\n");
    opus_decoder_cfg_t opus_cfg = DEFAULT_OPUS_DECODER_CONFIG();
    m_opus_decoder = decoder_opus_init(&opus_cfg);

    DEBUG_VERBOSE("[2.4] Register all elements to audio pipeline\r\n");
    audio_pipeline_register(m_pipeline, m_http_stream_reader, "http");
    audio_pipeline_register(m_pipeline, m_i2s_stream_writer, "i2s");
    audio_pipeline_register(m_pipeline, m_opus_decoder, "opus");

    // default is opus
    audio_pipeline_link(m_pipeline, (const char *[]){"http", "opus", "i2s"}, 3);

    DEBUG_VERBOSE("[ 3 ] Set up  event listener\r\n");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    m_evt = audio_event_iface_init(&evt_cfg);

    DEBUG_VERBOSE("[3.1] Listening event from all elements of pipeline\r\n");
    audio_pipeline_set_listener(m_pipeline, m_evt);

    DEBUG_VERBOSE("[3.2] Listening event from peripherals\r\n");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(m_periph_set_handle), m_evt);

    DEBUG_VERBOSE("stream url at boot %s\r\n", m_http_stream_url);
    if (strstr(m_http_stream_url, APP_FLASH_INVALID_LINK))
        m_auto_restart_stream_retries_number = 1;
    else
    {
        m_auto_restart_stream_retries_number = 2;
    }
}

