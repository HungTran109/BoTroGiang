#ifndef APP_IO_H
#define APP_IO_H

#include <stdint.h>
#include <stdbool.h>
#include "app_audio.h"
#include "driver/gpio.h"
#include "sdcard.h"
#include "periph_sdcard.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "driver/sdmmc_host.h"
#endif

//Điều khiển IO STM32 Expander
#define APP_IO_STM_OPTO_ON 1
#define APP_IO_STM_OPTO_OFF 0

#define APP_IO_STM_PA_ON 1
#define APP_IO_STM_PA_OFF 0

#define APP_IO_STM_FM_ON 1
#define APP_IO_STM_CODEC_ON 0

#define APP_IO_STM_MIC_ON 1
#define APP_IO_STM_AUX_ON 0

#define APP_IO_STM_LED_ON 0
#define APP_IO_STM_LED_OFF 1

#define APP_IO_STM_ISO_OUT_ON 1
#define APP_IO_STM_ISO_OUT_OFF 0

#define APP_IO_STM_GSM_PWR_ON 1
#define APP_IO_STM_GSM_PWR_OFF 0

#define APP_IO_STM_GSM_PWR_KEY_LOW 1
#define APP_IO_STM_GSM_PWR_KEY_HI 0

//Điều khiển IO Expander PCF
#define APP_IO_OPTO_ON 1
#define APP_IO_OPTO_OFF 0

#define APP_IO_PA_ON 1
#define APP_IO_PA_OFF 0

#define APP_IO_OUT_FM_ON 1
#define APP_IO_OUT_CODEC_ON 0

#define APP_IO_IOEX_LED_ON 0
#define APP_IO_IOEX_LED_OFF 1

#define APP_IO_ISOOUT_ON 1
#define APP_IO_ISOOUT_OFF 0

#define APP_IO_BUTTON_PRESSED 0
// // Nối nút bấm ngoài điều khiển Start/Stop stream
#define APP_IO_BUTTON_ON_AIR_NUM GPIO_NUM_36
#define APP_IO_MIC_DETECT_NUM GPIO_NUM_39

#define SD_CS_GPIO_PIN      14
#define SD_MOSI_GPIO_PIN    13
#define SD_MISO_GPIO_PIN    12
#define BUTTON_INPUT        34
#define GPIO_OUTPUT_SEL ((1ULL << SD_CS_GPIO_PIN) | (1ULL << SD_MOSI_GPIO_PIN))
#define GPIO_INPUT_SEL ((1ULL << BUTTON_INPUT))

typedef enum
{
    APP_IO_MCU_PROTOCOL_STATE_UNKNOWN,
    APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT,
    APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL
} app_io_protocol_type_t;

typedef union
{
    struct
    {
        uint16_t ISO_IN1 : 1;
        uint16_t ISO_IN2 : 1;
        uint16_t BTN_RESET : 1;
        uint16_t BTN_SET : 1;
        uint16_t NA1 : 1;
        uint16_t sw_mic_aux : 1;
        uint16_t sw_codec_fm : 1;
        uint16_t en_pa : 1;

        uint16_t ISO_OUT1 : 1;
        uint16_t ISO_OUT2 : 1;
        uint16_t LEDAUX_RED : 1;
        uint16_t LEDAUX_BLUE : 1;
        uint16_t LEDMIC_RED : 1;
        uint16_t LEDMIC_BLUE : 1;
        uint16_t GSM_PWR_KEY : 1;
        uint16_t gsm_pw_en : 1;
    } __attribute__((packed)) BitName;
    uint16_t Value;
} __attribute__((packed)) app_io_i2c_expander_t;

typedef union
{
    struct
    {
        uint16_t LED_BLE : 1;
        uint16_t LED_AUX : 1;
        uint16_t LED_SDCARD : 1;
        uint16_t LED_DEBUG : 1;
        uint16_t LED_INTERNET : 1;
        uint16_t LED_STREAM : 1;
        uint16_t EN_PA : 1;
        uint16_t NA : 9;
    } __attribute__((packed)) BitName;
    uint16_t Value;
} __attribute__((packed)) app_io_i2c_t;


typedef union
{
    struct
    {
        uint32_t sw_codec_fm : 1;
        uint32_t sw_mic_aux : 1;
        uint32_t en_pa : 1;
        uint32_t gsm_pw_en : 1;

        uint32_t GSM_PWR_KEY : 1;
        uint32_t GSM_RESET : 1;
        uint32_t ISO_OUT1 : 1;
        uint32_t ISO_OUT2 : 1;

        uint32_t LED1_RED : 1;
        uint32_t LED1_BLUE : 1;
        uint32_t LED2_RED : 1;
        uint32_t LED2_BLUE : 1;
        uint32_t audio_class : 2;
        uint32_t NA : 18;
    } __attribute__((packed)) BitName;
    uint32_t Value;
} __attribute__((packed)) app_io_mcu_expander_t;

typedef union 
{
    struct 
    {
        uint8_t IO1 : 1;
        uint8_t IO2 : 1;
        uint8_t IO3 : 1;
        uint8_t IO4 : 1;
        uint8_t IO5 : 1;
        uint8_t IO6 : 1;
        uint8_t IO7 : 1;
        uint8_t IO8 : 1;
    } Name;
    uint8_t Value;
} __attribute__((packed)) app_io_struct_t;

// typedef union 
// {
// 	struct SpkInput 
//     {
// 		uint8_t IN1 : 1;
// 		uint8_t IN2 : 1;
// 		uint8_t IN3 : 1;
// 		uint8_t IN4 : 1;
// 	} Name;
// 	uint8_t Value;
// } __attribute__((packed))  app_io_speaker_detect_state_t;

// typedef app_io_iso_input_t app_io_speaker_detect_state_t;

typedef union
{
    struct
    {
        uint8_t input1 : 1;
        uint8_t input2 : 1;
        uint8_t input3 : 1;
        uint8_t input4 : 1;
        uint8_t reserve : 4;
    } name;
    uint8_t value;
} __attribute__((packed)) app_io_iso_input_t;

typedef struct
{
    uint8_t mode;       // ref app_audio_operation_mode_t
    uint8_t vol;
    uint32_t snr;
    uint32_t rssi;
    uint32_t dbm;
    float gps_lat;
    float gps_long;
    uint32_t freq;
    uint8_t fw_version;
    uint8_t hw_version;
} __attribute__((packed)) app_io_fm_ping_msg_t;

typedef struct
{
    uint8_t local_vol;
    app_io_iso_input_t iso_input;
    uint32_t vin;
    uint8_t test_mode;
    uint8_t fw_version;
    uint8_t hw_version;
} app_io_esp32_ping_msg_t;

typedef struct
{
    uint8_t mode;
    uint8_t server_state;
    uint8_t interface;
    uint32_t current_freq;      // 0xFFFFFFF on invalid
    uint32_t freq0;
    uint32_t freq1;
    uint32_t freq2;
    uint8_t volume;             // 0xFF on invalid
    uint8_t remember_local_state;
    uint8_t csq;
    uint8_t band;
    uint8_t temp;
    uint8_t fm_rds_state;
} __attribute__((packed)) app_io_esp32_to_fm_frame_t;

typedef union
{
    struct {
        uint32_t mode_p46 : 1;
        uint32_t mode_p57 : 1;
        uint32_t mode_p60 : 1;
        uint32_t mode_p24 : 1;
        uint32_t mode_p42 : 1;
        uint32_t mode_p27 : 1;
        uint32_t mode_p35 : 1;
        uint32_t mode_p32 : 1;
        uint32_t mode_p36 : 1;
        uint32_t mode_p61 : 1;
        uint32_t mode_p62 : 1;
        uint32_t mode_p47 : 1;
        uint32_t mode_unuse : 4;
        
        uint32_t value_p46 : 1;
        uint32_t value_p57 : 1;
        uint32_t value_p60 : 1;
        uint32_t value_p24 : 1;
        uint32_t value_p42 : 1;
        uint32_t value_p27 : 1;
        uint32_t value_p35 : 1;
        uint32_t value_p32 : 1;
        uint32_t value_p36 : 1;
        uint32_t value_p61 : 1;
        uint32_t value_p62 : 1;
        uint32_t value_p47 : 1;
        uint32_t value_unuse : 4;
    } __attribute__((packed)) name;
    uint32_t val;
} __attribute__((packed)) app_io_ex_info_t;

//***************** SD CARD DEFINE ****************/
#define MOUNT_POINT "/sdcard"
// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:
#define USE_SPI_MODE    
// ESP32-S2 and ESP32-C3 doesn't have an SD Host peripheral, always use SPI:
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
#ifndef USE_SPI_MODE
#define USE_SPI_MODE
#endif // USE_SPI_MODE
// on ESP32-S2, DMA channel must be the same as host id
#if CONFIG_IDF_TARGET_ESP32S2
#define SPI_DMA_CHAN    host.slot
#elif CONFIG_IDF_TARGET_ESP32C3
#define SPI_DMA_CHAN    SPI_DMA_CH_AUTO
#endif //CONFIG_IDF_TARGET_ESP32S2
#endif //CONFIG_IDF_TARGET_ESP32S2
// DMA channel to be used by the SPI peripheral
#ifndef SPI_DMA_CHAN
#define SPI_DMA_CHAN    1
#endif //SPI_DMA_CHAN
// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.
#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  2
#define PIN_NUM_CS   14
#elif CONFIG_IDF_TARGET_ESP32C3
#define PIN_NUM_MISO 18
#define PIN_NUM_MOSI 9
#define PIN_NUM_CLK  8
#define PIN_NUM_CS   19
#endif //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#endif //USE_SPI_MODE
//************************************************/

/**
 * @brief           Get io expander i2c value
 * @retval          Expander value
 */
app_io_i2c_expander_t *app_io_get_i2c_exp_value(void);

/**
 * @brief           Get io expander mcu value
 * @retval          Expander value
 */
app_io_mcu_expander_t *app_io_get_mcu_exp_value(void);

/**
 * @brief           Get PA level
 * @retval          PA level
 */
uint32_t app_io_get_pa_state(void);

/**
 * @brief           Check if PA is off
 * @retval          TRUE PA is off
 *                  FALSE PA is on
 */
bool app_io_is_pa_off(void);

/**
 * @brief           Check if iso relay is off
 * @retval          TRUE ISO relay is off
 *                  FALSE ISO relay is on
 */
bool app_io_is_iso_relays_off(void);

/**
 * @brief           Check led WiFi is on or not
 * @retval          true LED WiFi is on
 *                  false led WiFi is off
 */
bool app_io_is_led_wifi_on(void);

/**
 * @brief           Control led wifi state
 * @param[in]       state : Wifi state
 */
void app_io_control_led_wifi_state(uint8_t wifi_state);

/**
 * @brief           Control led ETH state
 * @param[in]       state : ETH state
 */
void app_io_control_led_eth_state(uint8_t state);

/**
 * @brief           Check led ETH is on or not
 * @retval          true LED ETH is on
 *                  false led ETH is off
 */
bool app_io_is_led_eth_on(void);

/**
 * @brief           Check led 4G is on or not
 * @retval          true LED 4G is on
 *                  false led 4G is off
 */
bool app_io_is_led_4g_on(void);

/**
 * @brief           Control led GSM indicator
 * @param           state : New gsm led state
 */
void app_io_control_led_4g_state(uint8_t state);

/**
 * @brief           Commit current stm32 IO value
 * @param[in]       gpio Pointer to io value
 */
void app_io_mcu_update_io(app_io_mcu_expander_t *gpio);

/**
 * @brief           Init min protocol
 */
void app_io_init_min_protocol(void);


/*
 * @brief           EXP uart task
 */
void app_io_exp_uart_task(void *arg);

/*
 * @brief           Allow app_io_exp_uart_task process FM data
 */
void app_io_allow_process_fm_data(void);

/**
 * @brief           Control led stream state-machine
 * @param[in]       state : Stream state
 */
void app_io_control_led_stream(uint8_t state);

/**
 * @brief           Check led streaming is on or not
 * @retval          true LED streaming is on
 *                  false led streaming is off
 */
bool app_io_is_led_streaming_on(void);

/**
 * @brief           Initialize uart communication with GD32 MCU
 */
uint32_t app_io_expander_uart_initialize(void);

/**
 * @brief           Control PA
 */
void app_io_control_pa(uint8_t state);

/**
 * @brief           Get hardware verion
 * @retval          hardware version
 */
uint8_t app_io_get_hardware_version(void);

/**
 * @brief           Set hardware verion
 * @param[in]       hardware version
 */
void app_io_set_hardware_version(uint8_t version);

/**
 * @brief           Control all opto output
 * @param[in]       opto state
 */
void app_io_opto_control_all(uint8_t state);

/**
 * @brief           Get mic plug staus
 */
bool app_io_is_mic_plugged(void);

/**
 * @brief           Control relay audio output
 * @param[in]       mode Audio mode
 */
void app_io_select_relay_audio_input(app_audio_line_in_type_t mode);

/**
 * @brief           Control opto1 and save value to flash
 * @param[in]       state : Opto state
 */
void app_io_control_opto_output1(uint8_t state);

/**
 * @brief           Control opto2 and save value to flash
 * @param[in]       state : Opto state
 */
void app_io_control_opto_output2(uint8_t state);

/**
 * @brief           Set default IO value
 */
void app_io_set_default_value(void);

/**
 * @brief           Get GD32 protocol type
 * @retval          GD32 protocol type
 */
app_io_protocol_type_t app_io_get_gd32_protocol_method(void);

/**
 * @brief           Get FM protocol type
 * @retval          FM protocol type
 */
app_io_protocol_type_t app_io_get_fm_protocol_method(void);

/**
 * @brief           Initialize io driver
 */
void app_io_initialize(void);

/**
 * @brief           Control audio relay output
 */
void app_io_control_relay_audio_output(app_audio_operation_mode_t mode);

/**
 * @brief           Check if switch codec is on
 */
bool app_io_is_switch_codec_on(void);

/**
 * @brief           Set retry value
 */
void app_io_set_number_of_retries_counter(uint8_t counter);

/**
 * @brief           Control GSM power pin
 * @param[in]       ctrl : GSM power level
 */
void app_io_control_gsm_power_en(uint8_t ctrl);

/**
 * @brief           Control GSM power key
 * @param[in]       ctrl : GSM power key level
 */
void app_io_control_gsm_pwr_key(uint8_t ctrl);

/**
 * @brief           Get current IO value
 * @retval          IO value
 */
app_io_struct_t *app_io_get_io_value(void);

/**
 * @brief           Check if device is in test mode
 * @retval          TRUE device is in test mode
 *                  FALSE device is not in test mode
 */
bool app_io_is_in_test_mode(void);

/**
 * @brief           Get speaker detect value
 * @retval          Speaker detect value
 */
app_io_iso_input_t app_io_get_speaker_detect_value(void);

/**
 * @brief           Get current freq running
 * @retval          Current freq running in khz
 */
uint32_t app_io_get_current_fm_freq(void);

/**
 * @brief           Set current freq running
 * @retval          Current freq
 */
void app_io_set_current_fm_freq(uint32_t freq);


/**
 * @brief           Get current freq running at FM slave
 * @retval          Current freq
 */
uint32_t app_io_get_freq_at_slave(void);

/**
 * @brief           Get current freq SNR and RSSI
 * @param[in]       snr FM SNR
 * @param[in]       rssi FM RSSI
 */
void app_io_get_fm_snr_info(uint32_t *snr, uint32_t *rssi, uint32_t *dbm);

/**
 * @brief           Set current fm mode
 * @param[in]       Current freq mode
 */
void app_io_set_current_fm_module(uint8_t mode);

/**
 * @brief           Get current fm mode
 * @retval          Current freq mode
 */
uint8_t app_io_get_current_fm_module(void);

/**
 * @brief           Send a frame to GD32 on main board
 * @param[in]       id Frame id
 * @param[in]       data Pointer to raw data
 * @param[in]       size Payload length
 */
void app_io_send_custom_frame_to_gd32(uint8_t id, uint8_t* data, uint32_t size);

/**
 * @brief           Send a frame to FM board
 * @param[in]       id Frame id
 * @param[in]       data Pointer to raw data
 * @param[in]       size Payload length
 */
void app_io_send_custom_frame_to_fm(uint8_t id, uint8_t* data, uint32_t size);

/**
 * @brief           Get last location in string format
 * @retval          Last gps info
 */
char *app_io_get_last_gps_info(void);

/**
 * @brief           Get worker version
 * @retval          Worker version
 */
uint8_t app_io_get_worker_version(void);

/**
 * @brief           Check if speaker error (short, open)
 */
bool app_io_is_speaker_error(void);

/**
 * @brief           Get the last FM message
 */
app_io_fm_ping_msg_t *app_io_get_last_fm_msg(void);

/**
 * @brief           Get the last GD32 message
 */
app_io_esp32_ping_msg_t *app_io_get_last_gd32_msg(void);

/**
 * @brief           Get current Vin
 * @retval          Current vin in MV
 */
uint32_t app_io_get_current_vin(void);

/**
 * @brief           Set DTMF IO 1 value
 */
void app_io_set_dtmf_io1(uint8_t value);

/**
 * @brief           Set DTMF IO 2 value
 */
void app_io_set_dtmf_io2(uint8_t value);

/**
 * @brief           Start button on air ADC value
 */
void app_io_button_on_air_adc_start(void);

/**
 * @brief           ADC poll
 */
void app_io_adc_poll(void);

/**
 * @brief           Get temperature value
 */
uint8_t app_io_get_temperature(void);

/**
 * @brief           Get NTC voltage in mv
 */
uint32_t app_io_get_ntc_voltage(void);

/**
 * @brief           Get button on air level
 */
uint8_t app_io_get_btn_on_air_level(void);

/**
 * @brief           Check if vin lost
 */
uint8_t app_io_is_power_lost(void);
/**
 * @brief           Set led Internet state
 */
void set_led_internet_state (uint8_t state);
/**
 * @brief           Set EN_PA state
 */
void set_PA_state (uint8_t state);

void board_gpio_config (void);
esp_err_t SD_card_config (void);
esp_err_t i2c_app_io_set (app_io_i2c_t* io_i2c);
int app_i2c_config();
uint8_t get_button_state (void);
void set_button_state (uint8_t val);
#endif /* APP_IO_H */
