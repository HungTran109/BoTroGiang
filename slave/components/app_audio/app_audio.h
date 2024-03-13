#ifndef APP_AUDIO_H
#define APP_AUDIO_H

#include <stdint.h>
#include <stdbool.h>

#include "audio_element.h"
//#include "a2dp_stream.h"

#define APP_AUDIO_HTTP_URL_SIZE 156
typedef enum
{
    APP_AUDIO_STREAM_NOT_INIT = 0,
    APP_AUDIO_STREAM_START,
    APP_AUDIO_STREAM_RUNNING,
    APP_AUDIO_STREAM_PAUSE,
    APP_AUDIO_STREAM_POST_REQUEST,
    APP_AUDIO_STREAM_STOPPED,
    APP_AUDIO_STREAM_RESTART,
    APP_AUDIO_STREAM_FORCE_TERMINATE,
    APP_AUDIO_STREAM_NOT_ALLOWED
} app_audio_stream_state_t;

typedef enum
{
    APP_AUDIO_OPERATION_MODE_INTERNET = 1,
    APP_AUDIO_OPERATION_MODE_FM,
    APP_AUDIO_OPERATION_MODE_MIC,
    APP_AUDIO_OPERATION_MODE_NO_OPERATION,
    APP_AUDIO_OPERATION_MODE_LINE,
    APP_AUDIO_OPERATION_MODE_IDLE,

    /* Chế độ streaming internet nhưng cho chọn đầu vào từ MIC hoặc LINE */
    APP_AUDIO_OPERATION_MODE_INTERNET_MIC,
    APP_AUDIO_OPERATION_MODE_INTERNET_LINE,
    APP_AUDIO_OPERATION_MODE_BUSY
} app_audio_operation_mode_t;


typedef enum
{
    APP_AUDIO_CODEC_MODE_DECODE,
    APP_AUDIO_CODEC_MODE_LINE_IN
} app_audio_codec_mode_t;


/**
typedef enum {
    AUDIO_HAL_ADC_INPUT_LINE1 = 0x00,  
    AUDIO_HAL_ADC_INPUT_LINE2,       
    AUDIO_HAL_ADC_INPUT_ALL,           
    AUDIO_HAL_ADC_INPUT_DIFFERENCE,    
} audio_hal_adc_input_t;
 */
typedef enum
{
    APP_AUDIO_LINE_IN_MIC = 0x00, // AUDIO_HAL_ADC_INPUT_LINE1,
    APP_AUDIO_LINE_IN_AUX = 0x01, // AUDIO_HAL_ADC_INPUT_LINE2
} app_audio_line_in_type_t;

#define SAVE_FILE_RATE      44100
#define SAVE_FILE_CHANNEL   2
#define SAVE_FILE_BITS      16

#define PLAYBACK_RATE       48000
#define PLAYBACK_CHANNEL    2
#define PLAYBACK_BITS       16
/**
 * @brief       Check if http stream is running
 * @retval      1 HTTP stream is running
 *              0 HTTP Stream is not running
 */
uint8_t app_audio_is_http_audio_stream_running(void);


/**
 * @brief       Check if i2s stream is running
 * @retval      TRUE I2S stream is running
 *              FALSE I2S Stream is not running
 */
bool app_audio_is_i2s_running(void);

/**
 * @brief       Init board handle
 * @retval      TRUE Board init succeeded
 *              FLASE Board init failed
 */
bool app_audio_board_init(void);

/**
 * @brief       Set output DAC volume
 * @param[in]   volume DAC volume
 * @retval      Volume after set
 */
uint8_t app_audio_change_codec_vol(uint8_t volume);

/**
 * @brief       Get output DAC volume
 * @retval      Output volume
 */
uint8_t app_audio_get_current_output_vol(void);

/**
 * @brief       Terminate audio pipeline
 */
void app_audio_simple_terminate_pipeline(void);

/**
 * @brief       Force terminate pipeline and wait for pipeline stop
 */
void app_audio_complete_terminate(void);

/**
 * @brief       Force stop pipeline and wait for pipeline stop
 */
void app_audio_stop_pipeline(void);

/**
 * @brief       Terminate audio pipeline
 * @param[in]   uri New uri
 */
void app_audio_run_new_url(char *uri);

/**
 * @brief       Pause pipeline
 */
void app_audio_pause(void);

/**
 * @brief       Run pipeline
 */
void app_audio_run_pipeline(void);

/**
 * @brief       Switch codec mode ENCODE or LINE_IN
 * @param[in]   mode Codec mode
 */
void app_audio_change_codec_mode(app_audio_operation_mode_t mode);

/**
 * @brief       Set audio volume in test mode
 * @param[in]   setVolume Audio vol
 */
uint8_t app_audio_test_mode_change_codec_vol(uint8_t setVolume);

/**
 * @brief       Switch slave to stop mode
 * @note        Chuyển sang chạy chế độ thu từ INTERNET
 *	            Codec về chế độ MODE_DECODE
 *              Relay output chuyển về INTERNET output
 *              Disable PA
 */
void app_audio_change_to_stop_mode(void);

/**
 * @brief       Switch slave to input MIC
 * @note        Codec về chế độ APP_AUDIO_OPERATION_MODE_MIC
 *              Relay input chuyển về line AUX
 *              Relay output chuyển về FM output
 *              Enable PA
 */
void app_audio_change_to_local_mic_mode(void);

/**
 * @brief       Switch slave to input FM
 * @note        Codec về chế độ internet
 *              Relay input chuyển về line AUX
 *              Relay output chuyển về FM output
 *              Enable PA
 */
void app_audio_change_to_local_fm_mode(void);

/**
 * @brief       Chuyển sang chạy chế độ thu từ LINE AUX
 * @note        Codec về chế độ APP_AUDIO_OPERATION_MODE_LINE
 *	            Relay input chuyển về line AUX
 *	            Relay output chuyển về Codec output
 *	            Enable PA
 */
void app_audio_change_to_local_line_in_mode(void);

/**
 * @brief       Restart pipeline with new url
 * @param[in]   New http url
 */
void app_audio_restart_pipeline(char *url);

/**
 * @brief       Get audio state description in string
 */
const char *app_audio_get_http_state_description(void);

/**
 * @brief       Get OPUS state
 * @retval      OPUS stream state
 */
audio_element_state_t app_audio_get_opus_state(void);

/**
 * @brief       Check if opus stream is running
 * @retval      TRUE OPUS stream is running
 *              FALSE OPUS Stream is not running
 */
bool app_audio_is_opus_running(void);

/**
 * @brief       Get I2S state
 * @retval      I2S stream state
 */
audio_element_state_t app_audio_get_i2s_state(void);

/**
 * @brief       Get HTTP state
 * @retval      HTTP stream state
 */
audio_element_state_t app_audio_get_http_state(void);

/**
 * @brief       Get http stream url
 * @retval      HTTP stream url
 */
char *app_audio_get_stream_url(void);

/**
 * @brief       Set invalid URL
 */
void app_audio_set_invalid_link(void);

/**
 * @brief       Get ALT http stream url
 * @retval      ALT HTTP stream url
 */
char *app_audio_get_alt_stream_url(int master);

/**
 * @brief       Set ALT http stream url
 * @param[in]   ALT HTTP stream url
 */
void app_audio_set_alt_stream_url(int master, char *url);

/**
 * @brief       Wait for new audio event
 * @param[in]   timeout_ms Max wait time is ms
 */
bool app_audio_wait_for_event(uint32_t timeout_ms);

/**
 * @brief       Chuyển sang chạy chế độ thu từ INTERNET
 * @note        Codec về chế độ MODE_DECODE
 *	            Relay output chuyển về INTERNET output
 *	            Enable PA
 */
void app_audio_change_codec_to_internet_mode(void);

/**
 * @brief       Set stream retry number
 */
void app_audio_set_stream_retry(uint8_t times);


/**
 * @brief       Get stream retry number
 */
uint8_t app_audio_get_reset_stream_retry_number(void);

/**
 * @brief       Start audio component
 */
void app_audio_start(void);

/**
 * @brief       Start a2dp sink
 */
void start_pipeline_a2dp_sink_stream(void);

/**
 * @brief       Check if http stream reader is stopped
 */
bool app_audio_is_http_reader_stopped(void);

/**
 * @brief       Reset http stream reader stopped flag
 */
void app_audio_set_http_stream_stopped_flag(void);

/**
 * @brief       Get audio streaming state machine
 */
uint8_t app_audio_get_streaming_logic_step(void);

/**
 * @brief       Set audio streaming state machine
 * @param[in]   state New stream state
 */
void app_audio_set_streaming_logic_step(uint8_t step);

/**
 * @brief       Audio board i2c write cmd
 * @param[in]   addr I2C slave address
 * @param[in]   data Pointer to data
 * @param[in]   size Len of data
 */
void app_audio_hal_i2c_master_write(uint32_t addr, uint8_t *data, uint32_t size);

/**
 * @brief       Get audio volume
 * @retval      volume New volume
 */
uint8_t app_audio_get_current_output_vol(void);

/**
 * @brief       Set audio volume
 * @param[in]   volume New volume
 */
void app_audio_set_volume(uint8_t volume);

/**
 * @brief       Check if codec error
 * @retval      TRUE codec error
 *              FALSE codec ok
 */
bool app_audio_is_codec_error(void);


/**
 * @brief       get audio codec mode
 * @retval      Codec mode
 */
app_audio_codec_mode_t app_audio_get_codec_mode(void);

/**
 * @brief       Get the last link 404 not found
 */
char *app_audio_get_last_link_404(void);

/**
 * @brief       Get time out remaining force turn off PA
 */
uint32_t app_audio_get_force_turn_off_pa_remain_time(void);


/**
 * @brief       Reset timeout force turn of OA
 */
void app_audio_reset_force_turn_off_pa_when_link_404_not_found(void);

/**
 * @brief Start audio pipeline from sdcard
 * 
 */
void start_sdcard_pipeline (void);

#endif /* APP_AUDIO_H */
