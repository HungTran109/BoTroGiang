#include "app_io.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "app_audio.h"
#include "network.h"
#include "driver/uart.h"
#include "app_flash.h"
#include "version_ctrl.h"
#include "min.h"
#include "min_id.h"
#include "string.h"
#include "esp_sntp.h"
#include "app_mqtt.h"
#include "app_ota.h"
#include "esp_log.h"
#include "app_audio.h"
#include "utilities.h"
#include "pcf8575.h"
#include "main.h"
#include "esp_modem.h"
#include "esp_modem_dte.h"
#include "pcf8575.h"
#include "lpf.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "app_debug.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "board_pins_config.h"
#include "app_audio.h"

#define ADC_NB_OF_CHANNEL 1
#define DEFAULT_VREF 1100

// UART giao tiep mach FM hoac STM32
#define FM_UART_BAUD_RATE 115200
#define FM_UART_RX_BUF_SIZE 256
#define FM_UART_TX_BUF_SIZE 256
// Các chân FM UART
#define FM_UART_PORT_NUM UART_NUM_2
#define FM_UART_RX_IO GPIO_NUM_34 // Input only
#define FM_UART_TX_IO GPIO_NUM_13
#define MCU_EXPANDER_CHIP_NAME_INDEX 0
#define MCU_FM_CHIP_NAME_INDEX 1
#define MCU_MAX_NAME_LEN 16
#define FM_RDS_MSG_SIZE 87

// #define DTMF_TIMEOUT1    7
// #define DTMF_TIMEOUT2    7

static const char *TAG = "app_io";
static SemaphoreHandle_t m_sem_protect_gd32_uart;
static app_io_protocol_type_t m_gd32_protocol_state = APP_IO_MCU_PROTOCOL_STATE_UNKNOWN;
static app_io_protocol_type_t m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_UNKNOWN;
static uint8_t *m_min_gd32_rx_buffer;
static uint8_t *m_min_fm_rx_buffer;
static min_context_t m_min_fm_context;
static min_context_t m_min_gd32_context;
static min_frame_cfg_t m_min_fm_setting = MIN_DEFAULT_CONFIG();
static min_frame_cfg_t m_min_esp32_setting = MIN_DEFAULT_CONFIG();
static uint8_t m_protocol_select;
static app_io_i2c_expander_t i2c_io_expander;
static app_io_mcu_expander_t mcu_expander;
static void on_gd32_frame_callback(void *context, min_msg_t *frame);
static bool gd32_uart_tx(void *ctx, uint8_t data);
static uint8_t m_hw_version = 3;
static void on_fm_frame_callback(void *context, min_msg_t *frame);
static uint8_t resend_mode_to_fm_module;
static app_io_struct_t m_io_control;	/* Điều khiển các IO vào ra */
static uint32_t m_is_in_test_mode;
static app_io_fm_ping_msg_t m_last_fm_msg;
static app_io_esp32_ping_msg_t m_last_gd_msg;
static void process_fm_uart_buffer(char *buffer, uint32_t len);
static uint32_t m_publish_fm_info = 0;
/* FM UART */
static QueueHandle_t fm_uart_queue;
// Receive buffer to collect incoming data
uint8_t fm_uart_rx_buf[FM_UART_RX_BUF_SIZE];
uint32_t fm_uart_rx_len;
static uint8_t m_fm_mode;
static char m_last_gps_info[32] = {"0.000,0.000"};
static uint32_t m_worker_version;
static uint32_t m_publish_ex_mcu_info;
static uint8_t m_mcu_chipname[2][MCU_MAX_NAME_LEN];
static uint32_t m_worker_last_vin;

static app_io_ex_info_t m_ioex_io;
static uint32_t delay_turn_on_dtmf1 = 0, delay_turn_on_dtmf2 = 0;
static uint32_t m_dtmf_state = 0;
static uint32_t m_dtmf_retires = 0;
static uint8_t m_vin_lost = 0;

static app_io_i2c_t m_i2c_control_io;

static button_state_t button_pin_state = BTN_IS_RELEASED;

static void process_dtmf_state(void)
{
    static uint32_t m_last_io2_state = -1;
    static uint32_t m_last_io1_state = -1;

    switch (m_dtmf_state)
    {
        case 0:
            if (mcu_expander.BitName.ISO_OUT2 == IO_OFF && m_last_io2_state == IO_ON)
            {
                delay_turn_on_dtmf2 = app_flash_get_dtmf_t2();
                DEBUG_WARN("turn DTMF2\r\n");
                // mqtt_publish_message("DBG", "DTMF2 ON");
                m_dtmf_state = 2;
            }

            if (mcu_expander.BitName.ISO_OUT1 == IO_ON && m_last_io1_state == IO_OFF)
            {
                delay_turn_on_dtmf1 = app_flash_get_dtmf_startup();
                DEBUG_WARN("Turn on DTMF1\r\n");
                // mqtt_publish_message("DBG", "Delay %ds for machine start", delay_turn_on_dtmf1);
                m_dtmf_state = 3;
            }

            m_last_io2_state = mcu_expander.BitName.ISO_OUT2;
            m_last_io1_state = mcu_expander.BitName.ISO_OUT1;
            if (m_dtmf_state)
            {
                m_dtmf_retires = app_flash_get_dtmf_retires();
                DEBUG_INFO("DTMF retries : %d\r\n", m_dtmf_retires);
            }
            break;
        
        case 1:
            if (delay_turn_on_dtmf1)
            {
                delay_turn_on_dtmf1--;
                if (delay_turn_on_dtmf1 > app_flash_get_dtmf_t1()/2)
                {
                    app_io_set_dtmf_io1(1);
                }
                else
                {
                    app_io_set_dtmf_io1(0);
                }
                if (delay_turn_on_dtmf1 == 0)
                {
                    // mqtt_publish_message("DBG", "DTMF1 OFF");
                    app_io_set_dtmf_io1(0);
                    if (m_dtmf_retires > 0)
                    {
                        m_dtmf_retires--;
                        if (m_dtmf_retires == 0)
                        {
                            m_dtmf_state = 0;
                        }
                        else
                        {
                            delay_turn_on_dtmf1 = app_flash_get_dtmf_t2();
                        }
                    }
                    else
                    {
                        m_dtmf_state = 0;
                    }
                }
            }
            else
            {
                m_dtmf_state = 0;
                app_io_set_dtmf_io1(0);
                DEBUG_INFO("Restore DTMF state\r\n");
            }
            break;

        case 3:
            if (delay_turn_on_dtmf1)
            {
                delay_turn_on_dtmf1--;
                if (delay_turn_on_dtmf1 == 0)
                {
                    m_dtmf_state = 1;
                    delay_turn_on_dtmf1 = app_flash_get_dtmf_t1();
                    // mqtt_publish_message("DBG", "Start DTMF state", delay_turn_on_dtmf1);
                }
            }
            else
            {
                m_dtmf_state = 1;
                delay_turn_on_dtmf1 = app_flash_get_dtmf_t1();
            }
            break;

        case 2:
            if (delay_turn_on_dtmf2)
            {
                delay_turn_on_dtmf2--;
                if (delay_turn_on_dtmf2 > app_flash_get_dtmf_t2()/2)
                {
                    app_io_set_dtmf_io2(1);
                }
                else
                {
                    app_io_set_dtmf_io2(0);
                }
                if (delay_turn_on_dtmf2 == 0)
                {
                    // mqtt_publish_message("DBG", "DTMF2 OFF");
                    app_io_set_dtmf_io2(0);
                    if (m_dtmf_retires > 0)
                    {
                        m_dtmf_retires--;
                        if (m_dtmf_retires == 0)
                        {
                            m_dtmf_state = 0;
                            DEBUG_INFO("Restore DTMF state\r\n");
                        }
                        else
                        {
                            delay_turn_on_dtmf2 = app_flash_get_dtmf_t2();
                        }
                    }
                    else
                    {
                        m_dtmf_state = 0;
                        DEBUG_INFO("Restore DTMF state\r\n");
                    }
                }
            }
            else
            {
                m_dtmf_state = 0;
                app_io_set_dtmf_io2(0);
                DEBUG_INFO("Restore DTMF state\r\n");
            }
            break;

        default:
            break;
    }
}

static app_io_iso_input_t m_speaker_detect_state = 
{
    .value =  255,
};

uint8_t app_io_get_worker_version(void)
{
    return m_worker_version;
}

char *app_io_get_last_gps_info(void)
{
    return m_last_gps_info;
}

void app_io_set_dtmf_io1(uint8_t value)
{
    if (value)
    {
        m_ioex_io.name.value_p46 = 1;
    }
    else
    {
        m_ioex_io.name.value_p46 = 0;
    }
}

void app_io_set_dtmf_io2(uint8_t value)
{
    if (value)
    {
        m_ioex_io.name.value_p57 = 1;
    }
    else
    {
        m_ioex_io.name.value_p57 = 0;
    }
}

static const adc_channel_t m_adc_channels[ADC_NB_OF_CHANNEL] = {ADC_CHANNEL_0};
static const adc_bits_width_t m_adc_width = ADC_WIDTH_BIT_12;
static const adc_unit_t m_adc_unit = ADC_UNIT_1;
static lpf_data_t m_vtemp;        // Low pass filter
static const adc_atten_t m_adc_atten = ADC_ATTEN_DB_11;
static esp_adc_cal_characteristics_t *m_adc_chars;

void app_io_button_on_air_adc_start(void)
{
        /* Init ADC */
    adc1_config_width(m_adc_width);
    for (uint32_t i = 0; i < 1; i++)
    {
        adc1_config_channel_atten(m_adc_channels[i], m_adc_atten);
    }

    // Characterize ADC
    m_adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(m_adc_unit, m_adc_atten, m_adc_width, DEFAULT_VREF, m_adc_chars);
    (void)val_type;
}

static bool convert_temperature(uint32_t vtemp_mv, uint32_t vbat_mv, int32_t *result)
{
    #define HW_RESISTOR_SERIES_NTC 10000 //10K Ohm
    /* This is thermistor dependent ` and it should be in the datasheet, or refer to the
    article for how to calculate it using the Beta equation.
    I had to do this, but I would try to get a thermistor with a known
    beta if you want to avoid empirical calculations. */
    #define BETA 3470.0f //B25/B75

    /* This is also needed for the conversion equation as "typical" room temperature
    is needed as an input. */
    #define ROOM_TEMP 298.15f // room temperature in Kelvin

    /* Thermistors will have a typical resistance at room temperature so write this 
    down here. Again, needed for conversion equations. */
    #define RESISTOR_ROOM_TEMP 10000.0f //Resistance in Ohms @ 25oC	330k/470k

    bool retval = true;
    float vtemp_float;
    float vbat_float;
    float r_ntc;
    float temp ;
    
    if(vtemp_mv == vbat_mv
        || vtemp_mv == 0)
    {
        retval = false;
        goto end;
    }

    //NTC ADC: Vntc1 = Vntc2 = Vin
    //Tinh dien ap NTC
    vtemp_float = vtemp_mv / 1000.0f;
    vbat_float = vbat_mv / 1000.0f;

    //Caculate NTC resistor : Vntc = Vin*Rs/(Rs + Rntc) -> Rntc = Rs(Vin/Vntc - 1)
    r_ntc = HW_RESISTOR_SERIES_NTC*(vbat_float/vtemp_float - 1);

    if (r_ntc <= 0)
    {
        retval = false;
        goto end;
    }

    temp = (BETA * ROOM_TEMP) / (BETA + (ROOM_TEMP * log(r_ntc / RESISTOR_ROOM_TEMP))) - 273.15f;

    if (temp < 0)
    {
        retval = false;
        goto end;
    }

    *result = (int32_t)temp;
end:
    return retval;
}

static int32_t m_temperature = 0;
static int32_t m_btn_on_air_level = 1;

void app_io_adc_poll()
{
    static uint32_t m_first_time = 1;
    uint32_t adc_reading = 0;
    uint32_t voltage;

    // Get vtemp
    adc_reading = adc1_get_raw((adc1_channel_t)m_adc_channels[0]);
    voltage = esp_adc_cal_raw_to_voltage(adc_reading, m_adc_chars);
    if (m_first_time)
    {
        m_vtemp.gain = 5; // New value is no more than 5% old value
        m_vtemp.estimate_value = voltage;
    }
    else    // Apply low pass filter
    {
        int32_t tmp = voltage;
        lpf_update_estimate(&m_vtemp, &tmp);
    }

    convert_temperature(3300-m_vtemp.estimate_value, 3300, &m_temperature);

    if (m_vtemp.estimate_value < 500)
    {
        m_btn_on_air_level = 0;
    }
    else
    {
        m_btn_on_air_level = 1;
    }
    m_first_time = 0;
}

uint8_t app_io_get_temperature()
{
    if (m_temperature > 100 || m_temperature < 0)
    {
        m_temperature = 0;
    }
    return m_temperature;
}

uint8_t app_io_is_power_lost()
{
    return m_vin_lost;
}

uint32_t app_io_get_ntc_voltage()
{
    return m_vtemp.estimate_value;
}

uint8_t app_io_get_btn_on_air_level()
{
    return m_btn_on_air_level;
}

bool app_io_is_speaker_error(void)
{
    uint8_t class = app_flash_speaker_audio_class_get();
    if (class == APP_FLASH_AUDIO_CLASS_NONE)
    {
        return false;
    }
    uint32_t spk_err = 0;
    static uint32_t spk_err_counter = 0;
    if (class == APP_FLASH_AUDIO_CLASS_AB)
    {
        spk_err = ((m_speaker_detect_state.value == 1)
                        || (m_speaker_detect_state.value == 6)
                        || (m_speaker_detect_state.value == 13)
                        || (m_speaker_detect_state.value == 14));
    }
    else    // class D
    {
        /**
         *              IN1    IN2    IN3    IN4
         *    Open       0      1      1      0
         *    Short      1      0      0      0
         *    Normal     0      0      0      0
         */
        spk_err = ((m_speaker_detect_state.value == 6)
                        || (m_speaker_detect_state.value == 8));
        // if (spk_err == 0 && m_speaker_detect_state.value)
        // {
        //     m_speaker_detect_state.value = 15;
        // }
    }
    if (spk_err)
    {
        spk_err_counter++;
    }
    else
    {
        spk_err_counter = 0;
    }

    return (spk_err && spk_err_counter > 5);
}


app_io_fm_ping_msg_t *app_io_get_last_fm_msg(void)
{
    return &m_last_fm_msg;
}

app_io_esp32_ping_msg_t *app_io_get_last_gd32_msg(void)
{
    return &m_last_gd_msg;
}



void app_io_set_current_fm_module(uint8_t mode)
{
    m_fm_mode = mode;
}

uint8_t app_io_get_current_fm_module(void)
{
    return m_fm_mode;
}

uint32_t app_io_get_current_fm_freq(void)
{
    return app_flash_get_current_fm_freq();
}

void app_io_set_current_fm_freq(uint32_t freq)
{
    app_flash_set_current_fm_freq(freq);
}

static uint32_t m_freq_at_fm_slave;
uint32_t app_io_get_freq_at_slave(void)
{
    return m_freq_at_fm_slave;
}

void app_io_get_fm_snr_info(uint32_t *snr, uint32_t *rssi, uint32_t *dbm)
{
    *snr = m_last_fm_msg.snr;
    *rssi = m_last_fm_msg.rssi; 
    *dbm = m_last_fm_msg.dbm;
}

app_io_iso_input_t app_io_get_speaker_detect_value(void)
{
    return m_speaker_detect_state;
}

bool app_io_is_in_test_mode(void)
{
    return m_is_in_test_mode ? true : false;
}

app_io_struct_t *app_io_get_io_value(void)
{
    return &m_io_control;
}

void app_io_set_number_of_retries_counter(uint8_t counter)
{
    resend_mode_to_fm_module = counter;
}

app_io_i2c_expander_t *app_io_get_i2c_exp_value(void)
{
    return &i2c_io_expander;
}
app_io_i2c_t *app_io_get_i2c_control_io_value(void)
{
    return &m_i2c_control_io;
}
app_io_mcu_expander_t *app_io_get_mcu_exp_value(void)
{
    return &mcu_expander;
}

app_io_protocol_type_t app_io_get_gd32_protocol_method(void)
{
    return m_gd32_protocol_state;
}

app_io_protocol_type_t app_io_get_fm_protocol_method(void)
{
    return m_fm_protocol_state;
}


uint32_t app_io_get_pa_state(void)
{
#ifdef BOARD_HW_VER_BOTROGIANG_A00
    return m_i2c_control_io.BitName.EN_PA;
#else
    return i2c_io_expander.BitName.en_pa;
#endif
}

bool app_io_is_pa_off(void)
{
#ifdef BOARD_HW_VER_BOTROGIANG_A00
    if (m_i2c_control_io.BitName.EN_PA != APP_IO_PA_ON)
    return true;
#else
    if (m_hw_version == 2)
    {
        if (i2c_io_expander.BitName.en_pa != APP_IO_PA_ON)
            return true;
    }
    else if (m_hw_version == 3)
    {
        if (mcu_expander.BitName.en_pa != APP_IO_STM_PA_ON)
            return true;
    }
#endif
    return false;
}

bool app_io_is_iso_relays_off(void)
{
    if (m_hw_version == 2)
    {
        if (i2c_io_expander.BitName.ISO_OUT1 == APP_IO_OPTO_OFF 
            || i2c_io_expander.BitName.ISO_OUT2 == APP_IO_OPTO_OFF)
            return true;
    }
    else if (m_hw_version == 3)
    {
        if (mcu_expander.BitName.ISO_OUT1 == APP_IO_STM_ISO_OUT_OFF 
            || mcu_expander.BitName.ISO_OUT2 == APP_IO_STM_ISO_OUT_OFF)
            return true;
    }
    return false;
}


bool app_io_is_led_4g_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE != APP_IO_IOEX_LED_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (mcu_expander.BitName.LED1_BLUE != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

// LED1 BLUE
void app_io_control_led_4g_state(uint8_t state)
{
    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_ON;

        // LED WiFi, ETH Off
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;

        // STM32
        mcu_expander.BitName.LED1_BLUE = APP_IO_STM_LED_ON;

        // LED WiFi, ETH Off
        mcu_expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
        mcu_expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;
        mcu_expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

bool app_io_is_led_wifi_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDAUX_RED != APP_IO_IOEX_LED_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (mcu_expander.BitName.LED1_RED != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

// LED2 RED
void app_io_control_led_eth_state(uint8_t state)
{
    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_ON;

        // LED PPP, WiFi Off
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;

        // STM32
        mcu_expander.BitName.LED2_RED = APP_IO_STM_LED_ON;

        // LED PPP, WiFi Off
        mcu_expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
        mcu_expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
        mcu_expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

// LED1 RED
void app_io_control_led_wifi_state(uint8_t state)
{
    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_ON;

        // LED ETH, PPP Off
        app_io_get_i2c_exp_value()->BitName.LEDMIC_RED = APP_IO_IOEX_LED_OFF;
        app_io_get_i2c_exp_value()->BitName.LEDAUX_BLUE = APP_IO_IOEX_LED_OFF;

        // STM32
        mcu_expander.BitName.LED1_RED = APP_IO_STM_LED_ON;

        // LED ETH, PPP Off
        mcu_expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;
        mcu_expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.LEDAUX_RED = APP_IO_IOEX_LED_OFF;
        mcu_expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

bool app_io_is_led_eth_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDMIC_RED != APP_IO_IOEX_LED_ON)
            return false;
    }
    else
    {
        if (mcu_expander.BitName.LED2_RED != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

void app_io_control_opto_output1(uint8_t state)
{
    if (state == APP_IO_OPTO_ON)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_ON;
        mcu_expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_ON;

        // Reset delay timeout relay1
        slave_reset_delay_turn_off_relay1_when_stop_stream();
        m_io_control.Name.IO1 = IO_ON;
    }
    else if (state == APP_IO_OPTO_OFF)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_OFF;
        mcu_expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_OFF;
        m_io_control.Name.IO1 = IO_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }

    // Lưu lại trạng thái output Opto
    app_flash_write_u8(APP_FLASH_IO_STATE_KEY, m_io_control.Value);
}

void app_io_control_opto_output2(uint8_t state)
{
    if (state == APP_IO_OPTO_ON)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_ON;
        mcu_expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_ON;

        // Reset delay timeout relay2
        slave_reset_delay_turn_off_relay2_when_stop_stream();
        m_io_control.Name.IO2 = IO_ON;
    }
    else if (state == APP_IO_OPTO_OFF)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_OFF;
        mcu_expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_OFF;
        m_io_control.Name.IO2 = IO_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }

    // Lưu lại trạng thái output Opto
    app_flash_write_u8(APP_FLASH_IO_STATE_KEY, m_io_control.Value);
}

void app_io_control_gsm_power_en(uint8_t ctrl)
{
    if (ctrl)
    {
        i2c_io_expander.BitName.gsm_pw_en = 1;                        // Power ON Vbat +4V2
        mcu_expander.BitName.gsm_pw_en = APP_IO_STM_GSM_PWR_ON; // Power ON Vbat +4V2
    }
    else
    {
        i2c_io_expander.BitName.gsm_pw_en = 0;                         // Power OFF Vbat +4V2
        mcu_expander.BitName.gsm_pw_en = APP_IO_STM_GSM_PWR_OFF; // Power OFF Vbat +4V2
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = i2c_io_expander.Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

void app_io_control_gsm_pwr_key(uint8_t ctrl)
{
    if (ctrl)
    {
        app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 1;            // PowerKey LOW
        mcu_expander.BitName.GSM_PWR_KEY = APP_IO_STM_GSM_PWR_KEY_LOW; // PowerKey LOW
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.GSM_PWR_KEY = 0;           // PowerKey HIGH
        mcu_expander.BitName.GSM_PWR_KEY = APP_IO_STM_GSM_PWR_KEY_HI; // PowerKey HIGH
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

void app_io_select_relay_audio_input(app_audio_line_in_type_t mode)
{
    DEBUG_INFO("Switch relay input to: %s\r\n", mode == APP_AUDIO_LINE_IN_AUX ? "AUX" : "MIC");

    if (mode == APP_AUDIO_LINE_IN_MIC)
    {
        app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 1;   // Switch Relay to MIC input
        mcu_expander.BitName.sw_mic_aux = APP_IO_STM_MIC_ON; // Switch Relay to MIC input
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.sw_mic_aux = 0;   // Switch Relay to AUX input
        mcu_expander.BitName.sw_mic_aux = APP_IO_STM_AUX_ON; // Switch Relay to AUX input
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

void app_io_control_relay_audio_output(app_audio_operation_mode_t mode)
{
    DEBUG_INFO("Switch relay output to: %s\r\n", mode == APP_AUDIO_OPERATION_MODE_FM ? "FM" : "CODEC");

    if (mode == APP_AUDIO_OPERATION_MODE_FM)
    {
        app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 1;  // Switch Relay to FM output
        mcu_expander.BitName.sw_codec_fm = APP_IO_STM_FM_ON; // Switch Relay to FM output
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.sw_codec_fm = 0;     // Switch Relay to codec output
        mcu_expander.BitName.sw_codec_fm = APP_IO_STM_CODEC_ON; // Switch Relay to codec output
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

/******************************************************************************************/
/**
 * @brief   : Khởi tạo UART cho module FM
 * @param   :
 * @retval  : ESP_OK or ESP_FAIL
 * @author  :
 * @created :
 */
uint32_t app_io_expander_uart_initialize(void)
{
    esp_err_t err;

    uart_config_t uart_config =
        {
            .baud_rate = FM_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    // Config UART with any gpio pins
    err = uart_param_config(FM_UART_PORT_NUM, &uart_config);
    DEBUG_VERBOSE("uart_param_config: %s\r\n", err == ESP_OK ? "OK" : "ERR");

    err += uart_set_pin(FM_UART_PORT_NUM, FM_UART_TX_IO, FM_UART_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    DEBUG_VERBOSE("uart_set_pin: %s\r\n", err == ESP_OK ? "OK" : "ERR");

    err += uart_driver_install(FM_UART_PORT_NUM, 2 * FM_UART_RX_BUF_SIZE, 2 * FM_UART_TX_BUF_SIZE, 3, &fm_uart_queue, 0);
    DEBUG_VERBOSE("uart_driver_install: %s\r\n", err == ESP_OK ? "OK" : "ERR");

    return err;
}

static app_io_iso_input_t m_iso_input;
static uint8_t m_delay_save_local_vol = 0;
static int8_t m_notify_power_lost_counter = -1;
static uint8_t m_send_imei_to_lcd = 3;

static void on_gd32_frame_callback(void *context, min_msg_t *frame)
{
    DEBUG_VERBOSE("On GD32 frame callback, id %u\r\n", frame->id);
    m_gd32_protocol_state = APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL;
    // if (m_protocol_select != m_gd32_protocol_state)
    // {
    //     m_protocol_select = m_gd32_protocol_state;
    //     app_flash_write_u8(APP_FLASH_IO_MAIN_GD32_PROTCOL_KEY, m_protocol_select);
    // }
    switch (frame->id)
    {
    case MIN_ID_PING:
    {
        app_io_esp32_ping_msg_t *msg = (app_io_esp32_ping_msg_t *)frame->payload;
        memcpy(&m_last_gd_msg, msg, sizeof(app_io_esp32_ping_msg_t));

        if (msg->vin > 12000)
        {
            if (m_vin_lost)
            {
                esp_restart();
            }
            m_vin_lost = 0;
            m_notify_power_lost_counter = 5;
        }
        if (msg->vin < 10000 && m_notify_power_lost_counter > 0)
        {
            m_vin_lost = 1;
            m_notify_power_lost_counter--;
            if (-1 == app_mqtt_send_pwr_lost(msg->vin))
            {
                m_notify_power_lost_counter++;
            }
            slave_set_mqtt_state_timeout(60); // Send heartbeat luon
        }
        

        if (m_iso_input.value != msg->iso_input.value)
        {
            m_iso_input.value = msg->iso_input.value;
            // DEBUG_INFO("Local vol %u, iso 0x%04X, vin %umV, version %d, out ISO1=%u, out ISO2=%u",
            //         msg->local_vol, msg->iso_input.value, msg->vin, msg->fw_version,
            //         mcu_expander.BitName.ISO_OUT1, mcu_expander.BitName.ISO_OUT2);

            // Test: print trạng thái (theo bảng trạng thái)
            if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
            {                
                if (app_io_get_i2c_exp_value()->BitName.en_pa == APP_IO_PA_OFF 
                    && mcu_expander.BitName.en_pa == APP_IO_STM_PA_OFF)
                {
                    m_speaker_detect_state.value = m_iso_input.value;
                    switch(m_iso_input.value) 
                    {
                        case 7:
                        case 15:
                            DEBUG_WARN("Audio class older version, does not support: %d\r\n", m_iso_input.value);
                            break;
                        //Class AB
                        case 12:
                            DEBUG_INFO("Class AB: Normal (%d)\r\n", m_iso_input.value);
                            break;
                        case 13:
                            ESP_LOGW(TAG, "Class AB: Short to GND (%d)\r\n", m_iso_input.value);
                            break;
                        case 14:
                            ESP_LOGW(TAG, "Class AB: Open circuit (%d)\r\n", m_iso_input.value);
                            break;
                        //Class D original version
                        case 0:
                            DEBUG_INFO("Class D: Normal (%d)\r\n", m_iso_input.value);
                            break;
                        case 1:
                            ESP_LOGW(TAG, "Class D: Short to GND (%d)\r\n", m_iso_input.value);
                            break;
                        case 6:
                            ESP_LOGW(TAG, "Class D: Open circuit (%d)\r\n", m_iso_input.value);
                            break;
                        
                        default:
                            ESP_LOGW(TAG, "Unknown input state: %d\r\n", m_iso_input.value);
                            break;
                    }
                }
            }
            else
            {
                // Class D v2 
                // Chi enable class D status khi ISO_OUT2 = 1 && PA off
                if (app_io_get_i2c_exp_value()->BitName.ISO_OUT2 == APP_IO_PA_ON 
                    && mcu_expander.BitName.en_pa == APP_IO_STM_PA_OFF)
                {
                    m_speaker_detect_state.value = m_iso_input.value;
                    switch (m_iso_input.value) 
                    {
                        //Class D new version
                        case 0:
                            DEBUG_INFO("Class D: Normal (%d)", m_iso_input.value);
                            break;
                        case 8:
                            ESP_LOGW(TAG, "Class D: Short load (%d)", m_iso_input.value);
                            break;
                        case 6:
                            ESP_LOGW(TAG, "Class D: Open circuit (%d)", m_iso_input.value);
                            break;
                        
                        default:
                            ESP_LOGW(TAG, "Unknown input state: %d", m_iso_input.value);
                            break;
                    }
                }
            }
        }
        app_io_mcu_update_io(&mcu_expander);

        if (m_publish_ex_mcu_info == 1)
        {
            m_publish_ex_mcu_info++;
            if (m_publish_ex_mcu_info >= 10 && app_mqtt_is_connected_to_server())
            {
                m_publish_ex_mcu_info = 0;
                mqtt_publish_message("DBG", "MCU IO %s, version %u, FM %s", 
                                        &m_mcu_chipname[MCU_EXPANDER_CHIP_NAME_INDEX][0], 
                                        m_worker_version,
                                        &m_mcu_chipname[MCU_FM_CHIP_NAME_INDEX][0]);
            }
        }
    }
    break;
    case MIN_ID_FORWARD:
    {
        uint8_t *ptr = frame->payload;
        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
        {
            for (uint32_t i = 0; i < frame->len; i++)
            {
                min_rx_feed(&m_min_fm_context, ptr + i, 1);
            }
        }

        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
        {
            process_fm_uart_buffer((char *)frame->payload, frame->len);
        }
    }
    break;
    case MIN_ID_RESET:
        {
            if (m_publish_ex_mcu_info == 0)
            {
                ESP_LOGE(TAG, "GD32 reset\r\n");
            }
            m_publish_ex_mcu_info = 1;
        }
            break;

    case MIN_ID_MCU_EXPANDER_REPORT_CHIP:
        snprintf((char*)&m_mcu_chipname[MCU_EXPANDER_CHIP_NAME_INDEX][0], 
                MCU_MAX_NAME_LEN, 
                "%s", 
                (char*)frame->payload);
        break;

    case MIN_ID_CTRL_EXT_GPIO:
    {
        static app_io_ex_info_t rx_io;
        app_io_ex_info_t *io_ptr = (app_io_ex_info_t*)frame->payload;
        uint16_t new_val = (io_ptr->val) >> 16;
        uint16_t old_val = (rx_io.val) >> 16;

        if (new_val != old_val)
        {
            ESP_LOGW(TAG, "Gpio changed from 0x%04X to 0x%04X", old_val, new_val);
        }

        memcpy(&rx_io, frame->payload, sizeof(rx_io));

        for (uint32_t i = 0; i < 12; i++)       // Hardware support maximum 12 GPIO
        {
            if (m_ioex_io.val & (1 << i))   // 1 = Mode input, 0 = output
            {
                int input_val = io_ptr->val & (1 << (i+16)) ? 1 : 0;
                if (input_val)
                {
                    m_ioex_io.val |= (1 << (i+16));
                }
                else
                {
                    m_ioex_io.val &= (~(1 << (i+16)));
                }
            }
        }
    }
        break;

    default:
        app_ota_on_slave_frame_callback(frame);
        break;
    }
}

typedef struct {
    i2c_config_t i2c_conf;   /*!<I2C bus parameters*/
    i2c_port_t i2c_port;     /*!<I2C port number */
} i2c_bus_t;
static i2c_bus_handle_t i2c_handle = NULL;
static esp_err_t __attribute__((unused)) i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_GD32_SLAVE_ADDRESS7 << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 5000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
int app_i2c_config()
{
    int i2c_master_port = I2C_NUM_1;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_io_initialize(void)
{
    if (!m_sem_protect_gd32_uart)
    {
        m_sem_protect_gd32_uart = xSemaphoreCreateMutex();
    }

    gpio_set_direction(APP_IO_MIC_DETECT_NUM, GPIO_MODE_DEF_INPUT);

    DEBUG_VERBOSE("[GPIO] User button: %d, MIC detect: %d", 
                gpio_get_level(APP_IO_BUTTON_ON_AIR_NUM),
                gpio_get_level(APP_IO_MIC_DETECT_NUM));

    /* ========================= End of Init 4G modem ===================== */
    /* =================== Init Peripheral ================================ */
    //Đọc logic chân button/IO36 trước xem có đang bấm ko (Input only IO không support pull-up/down -> cần trở pullup ngoài!
    gpio_set_direction(APP_IO_BUTTON_ON_AIR_NUM, GPIO_MODE_DEF_INPUT);
}

// i2c_bus_write_bytes(i2c_handle, slave_addr, &reg_add, sizeof(reg_add), &data, sizeof(data));
esp_err_t i2c_app_io_set (app_io_i2c_t* io_i2c)
{
    int ret = 0;
    // uint8_t data_h, data_l;
    // data_l = (uint8_t)(io_i2c->Value) & 0x00ff;
    // data_h = (uint8_t)(io_i2c->Value >> 8) & 0x00ff;
    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, I2C_GD32_SLAVE_ADDRESS7 << 1 | WRITE_BIT, ACK_CHECK_EN);
    // i2c_master_write(cmd, io_i2c, sizeof(app_io_i2c_t), ACK_VAL);
    // i2c_master_stop(cmd);
    // ret = i2c_master_cmd_begin(1, cmd, 1000 / portTICK_RATE_MS);
    // i2c_cmd_link_delete(cmd);
    
    // printf ("send expander io i2c\r\n");
    // ret = i2c_bus_write_bytes(i2c_handle, I2C_GD32_SLAVE_ADDRESS7, NULL, 0, (uint8_t*)io_i2c, sizeof (app_io_i2c_t));
    app_audio_hal_i2c_master_write(I2C_GD32_SLAVE_ADDRESS7 << 1, io_i2c, sizeof(app_io_i2c_t));
    // uint8_t data[2];
    // data[0] = (uint8_t)((io_i2c->Value) & 0x00ff);
    // data[1] = (uint8_t)((io_i2c->Value >> 8) & 0x00ff);
    // DEBUG_INFO ("DATA 0: %d\r\n DATA: 1:%d\r\n", data[0], data[1]);
    // ret = i2c_master_write_slave(I2C_NUM_0, data, sizeof (data));
    return ret;
}

void app_io_mcu_update_io(app_io_mcu_expander_t *gpio)
{
    if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
    {
        char stmMessage[256] = {0};
        uint32_t index = 0;

        /* Bản tin phản hồi module FM
         * 4G,GPIO=<value>,CRC=12345#
         */
        index = sprintf(stmMessage, "WORKER,GPIO=%d,", gpio->Value);
        if (m_is_in_test_mode)
        {
            if (app_mqtt_get_state() == APP_MQTT_CONNECTED)
            {
                index += sprintf(stmMessage + index, "%s", "SERVER(OK),");
            }
            else
            {
                index += sprintf(stmMessage + index, "%s", "SERVER(ERROR),");
            }

            index += sprintf(stmMessage + index, "GSM_IMEI(%s),", app_flash_get_imei());
            modem_dce_t *dce = slave_get_modem_dce();

            if (dce && strlen(dce->sim_imei) > 10)
            {
                index += sprintf(stmMessage + index, "SIM_IMEI(%s),", dce->sim_imei);
            }
            else
            {
                index += sprintf(stmMessage + index, "SIM_IMEI(%s),", "000");
            }

            if (network_is_eth_got_ip())
            {
                index += sprintf(stmMessage + index, "%s", "ETH(OK),");
            }
            else
            {
                index += sprintf(stmMessage + index, "%s", "ETH(ERROR),");
            }

            if (!app_audio_is_codec_error())
            {
                index += sprintf(stmMessage + index, "%s", "CODEC(OK),");
            }
            else
            {
                index += sprintf(stmMessage + index, "%s", "CODEC(ERROR),");
            }
        }

        /* CRC and tail */
        uint16_t crc16 = CalculateCRC16((uint8_t *)stmMessage, index);
        index += sprintf(&stmMessage[index], "CRC=%05u#", crc16);

        /* send to UART */
        xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
        uart_write_bytes(FM_UART_PORT_NUM, stmMessage, index);
        xSemaphoreGive(m_sem_protect_gd32_uart);
    }
    else
    {
        min_msg_t msg;
        msg.id = MIN_ID_CONTROL_GPIO;
        msg.len = sizeof(app_io_mcu_expander_t);
        msg.payload = gpio;
        xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
        min_send_frame(&m_min_gd32_context, &msg);
        xSemaphoreGive(m_sem_protect_gd32_uart);

        // Send control IO gpio
        msg.id = MIN_ID_CTRL_EXT_GPIO;
        msg.len = sizeof(m_ioex_io);
        msg.payload = &m_ioex_io;
        xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
        min_send_frame(&m_min_gd32_context, &msg);
        xSemaphoreGive(m_sem_protect_gd32_uart);


        if (m_is_in_test_mode)
        {
            min_jig_data_t jig_data;
            jig_data.jig_status.name.gsm = 0;
            if (strlen(app_flash_get_imei()) > 10)
            {
                jig_data.jig_status.name.gsm = 1;
                sprintf((char*)jig_data.gsm_imei, "%s", app_flash_get_imei());
            }
            else
            {
                sprintf((char*)jig_data.gsm_imei, "%s", "000");
            }

            modem_dce_t *dce = slave_get_modem_dce();
            if (dce && strlen(dce->sim_imei) > 10)
            {
                sprintf((char*)jig_data.sim_imei, "%s", dce->sim_imei);
            }
            else
            {
                sprintf((char*)jig_data.sim_imei, "%s", "000");
            }

            jig_data.jig_status.name.server = (app_mqtt_get_state() == APP_MQTT_CONNECTED) ? 1 : 0;
            jig_data.jig_status.name.eth = network_is_eth_got_ip() ? 1 : 0;
            jig_data.jig_status.name.codec = app_audio_is_codec_error() ? 0 : 1;

            min_msg_t msg;
            msg.id = MIN_ID_JIG_DATA;
            msg.len = sizeof(min_jig_data_t);
            msg.payload = &jig_data;
            xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
            min_send_frame(&m_min_gd32_context, &msg);
            xSemaphoreGive(m_sem_protect_gd32_uart);
        }
    }
}

void app_io_init_min_protocol()
{
    m_min_gd32_rx_buffer = malloc(MIN_MAX_PAYLOAD);
    m_min_fm_rx_buffer = malloc(MIN_MAX_PAYLOAD);

    m_min_fm_setting.get_ms = xTaskGetTickCount;
    m_min_fm_setting.last_rx_time = 0x00;
    /* Not using the rx_callback then*/
    m_min_fm_setting.rx_callback = on_fm_frame_callback;
    m_min_fm_setting.timeout_not_seen_rx = 5000;
    m_min_fm_setting.tx_byte = NULL;
    m_min_fm_setting.use_timeout_method = 1;
    // m_min_fm_context.cb = &m_min_fm_setting;
    m_min_fm_context.rx_frame_payload_buf = m_min_fm_rx_buffer;
    m_min_fm_context.callback = &m_min_fm_setting;
    min_init_context(&m_min_fm_context);

    m_min_esp32_setting.get_ms = xTaskGetTickCount;
    m_min_esp32_setting.last_rx_time = 0x00;
    /* Not using the rx_callback then*/
    m_min_esp32_setting.rx_callback = on_gd32_frame_callback;
    m_min_esp32_setting.timeout_not_seen_rx = 5000;
    m_min_esp32_setting.tx_byte = gd32_uart_tx;
    m_min_esp32_setting.use_timeout_method = 1;
    // m_min_esp32context.cb = &m_min_esp32setting;
    m_min_gd32_context.rx_frame_payload_buf = m_min_gd32_rx_buffer;
    m_min_gd32_context.callback = &m_min_esp32_setting;
    min_init_context(&m_min_gd32_context);

    app_io_expander_uart_initialize();
    xTaskCreate(app_io_exp_uart_task, "fm_uart_task", 5 * 1024, NULL, 5, NULL);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

void app_io_set_default_value(void)
{
    mcu_expander.Value = 0;

    mcu_expander.BitName.LED1_BLUE = APP_IO_STM_LED_OFF;
    mcu_expander.BitName.LED1_RED = APP_IO_STM_LED_OFF;
    mcu_expander.BitName.LED2_BLUE = APP_IO_STM_LED_OFF;
    mcu_expander.BitName.LED2_RED = APP_IO_STM_LED_OFF;

    /* Các chân ISO Ouput khởi tạo theo cấu hình cài đặt */
    /* Mặc định khởi tạo OFF, khi nào chạy stream hoặc MIC/LINE thì bật */
    mcu_expander.BitName.ISO_OUT1 = m_io_control.Name.IO1;
    mcu_expander.BitName.ISO_OUT2 = m_io_control.Name.IO2;

    // GSM OFF
    mcu_expander.BitName.gsm_pw_en = APP_IO_STM_GSM_PWR_OFF;
    mcu_expander.BitName.GSM_PWR_KEY = APP_IO_STM_GSM_PWR_KEY_HI;

    // Swith Audio input relay
    mcu_expander.BitName.sw_mic_aux = APP_IO_STM_AUX_ON; // Default to AUX input

    // Switch Ouput Codec - FM
    mcu_expander.BitName.sw_codec_fm = APP_IO_STM_CODEC_ON; // Relay to Codec ouput

    // PA enable
    mcu_expander.BitName.en_pa = APP_IO_STM_PA_OFF;

    // Send lệnh qua UART
    app_io_mcu_update_io(&mcu_expander);
}

bool app_io_is_led_streaming_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE != APP_IO_IOEX_LED_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (mcu_expander.BitName.LED2_BLUE != APP_IO_STM_LED_ON)
            return false;
    }
    return true;
}

// LED2 - BLUE
void app_io_control_led_stream(uint8_t state)
{
    if (state == APP_AUDIO_STREAM_RUNNING)
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_ON;
        mcu_expander.BitName.LED2_BLUE = APP_IO_STM_LED_ON;
    }
    else if (state == APP_AUDIO_STREAM_STOPPED)
    {
        app_io_get_i2c_exp_value()->BitName.LEDMIC_BLUE = APP_IO_IOEX_LED_OFF;
        mcu_expander.BitName.LED2_BLUE = APP_IO_STM_LED_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
}

uint32_t app_io_get_current_vin()
{
    return m_last_gd_msg.vin + m_worker_last_vin;
}

/*
 * Điều khiển mạch PA ngoài
 * 1: ON, 0: OFF
 */
void app_io_control_pa(uint8_t state)
{
    bool spk_err = app_io_is_speaker_error();

    // 1 so mach cu ko han NTC -> gia tri nhiet do se khong dung (>90 do C)
    bool force_turn_off_pa_when_volume_zero = false;
    if (app_flash_get_volume() == 0)
    {
        force_turn_off_pa_when_volume_zero = true;
    }
#ifdef BOARD_HW_VER_BOTROGIANG_A00
    if ((state && spk_err) || (m_temperature > 75 && m_temperature < 80) || force_turn_off_pa_when_volume_zero)
    {
        // Khi ho mach hoac ngan mach thi ko bat PA
        app_io_get_i2c_control_io_value()->BitName.EN_PA = APP_IO_PA_OFF;
        return;
    }

    ESP_LOGD(TAG, "Turn PA_EN: %s", state ? "ON" : "OFF");

    if (state)
    {
        app_io_get_i2c_control_io_value()->BitName.EN_PA = APP_IO_PA_ON;
    }
    else
    {
        app_io_get_i2c_control_io_value()->BitName.EN_PA = APP_IO_PA_OFF;
    }

    Int_t ex_out;
    ex_out.value = app_io_get_i2c_control_io_value()->Value;
    app_audio_hal_i2c_master_write(I2C_GD32_SLAVE_ADDRESS7 << 1, ex_out.bytes, sizeof(app_io_i2c_t));

#else
    if ((state && spk_err) || (m_temperature > 75 && m_temperature < 80) || force_turn_off_pa_when_volume_zero)
    {
        // Khi ho mach hoac ngan mach thi ko bat PA
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;
        mcu_expander.BitName.en_pa = APP_IO_STM_PA_OFF;
        return;
    }

    ESP_LOGD(TAG, "Turn PA_EN: %s", state ? "ON" : "OFF");

    if (state)
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_ON;
        mcu_expander.BitName.en_pa = APP_IO_STM_PA_ON;
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.en_pa = APP_IO_PA_OFF;
        mcu_expander.BitName.en_pa = APP_IO_STM_PA_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }
#endif
}

static bool m_fm_allowed = false;
void app_io_allow_process_fm_data(void)
{
    m_fm_allowed = true;
}

static void process_fm_uart_buffer(char *buffer, uint32_t size)
{
    /** 
     * Xử lý bản tin FM
     * FM|LOCAL_MODE=<FM/MIC/LINE/IDLE>|FREQ=<FREQ>|VOL=<70>|SNR=<>|RSSI=<>|GPS=<VD,KD>|CRC=12345#
     */
    char response[128];
    uint8_t index = 0;
    if (m_hw_version == 3)
    {
        // Chưa khởi tạo xong audio codec thì không nhận lệnh chuyển mode!
        if (!m_fm_allowed)
            return;
    }

    if (strstr((char *)buffer, "FM|") && strstr((char *)buffer, "#") && size > 10)
    {
        /* Check CRC: |CRC=12345# */
        char *crc = strstr((char *)buffer, "CRC=");
        if (crc)
        {
            uint16_t crc16 = GetNumberFromString(4, crc);

            /* Tinh CRC thuc cua chuoi: Tru 10 ki tu cuoi CRC=12345# */
            uint16_t crcCal = CalculateCRC16((uint8_t *)buffer, size - 10);

            if (crc16 != crcCal)
            {
                DEBUG_INFO("FM CRC failed: %u - %u", crc16, crcCal);
                return;
            }
        }
        else
        {
            return;
        }

        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
        {
            m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT;
        }
        
        /** Chỉ xét các chế độ đang chạy dưới module FM khi slave đang rảnh (chế độ INTERNET và không streaming) */
        if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
            && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !slave_at_least_one_master_is_streaming()))
        {
            if (strstr((char *)buffer, "MODE=LOCAL_FM"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_FM)
                {
                    DEBUG_INFO("Switch to Local FM mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_FM;

                    /* Change mode to FM */
                    app_audio_change_to_local_fm_mode();

                    // Tắt Relay (nếu được bật ở chế độ LINE IN)
                    DEBUG_INFO("call app_io_opto_control_all...\r\n");
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to Local FM mode");
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_MIC"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_MIC)
                {
                    DEBUG_INFO("Switch to Local MIC mode\r\n");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_MIC;

                    /* Change mode to MIC */
                    app_audio_change_to_local_mic_mode();

                    // Bật Relay cho chế độ MIC cho class AB
                    if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                    {
                        mqtt_publish_message("DBG", "CLass AB: Switch to Local MIC mode");
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                    }
                    else
                    {
                        mqtt_publish_message("DBG", "Class D: Switch to Local MIC mode");
                        app_io_opto_control_all(APP_IO_OPTO_OFF);
                    }
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_LINE"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_LINE)
                {
                    DEBUG_INFO("Switch to Local LINE mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_LINE;

                    /* Change mode to LINE */
                    app_audio_change_to_local_line_in_mode();

                    // Bật Relay cho chế độ LINE IN
                    if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                    {
                        mqtt_publish_message("DBG", "Class AB: Switch to Local LINE IN");
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                    }
                    else
                    {
                        mqtt_publish_message("DBG", "Class D: Switch to Local LINE IN");
                        app_io_opto_control_all(APP_IO_OPTO_OFF);
                    }
                }
            }
        }

          /* Volume */
        char *sVolume = strstr((char *)buffer, "VOL=");
        if (sVolume)
        {
            uint8_t vol = GetNumberFromString(4, sVolume);

            /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
            if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE) &&
                app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
            {
                if (abs(app_audio_get_current_output_vol() - vol) > 3)
                {
                    DEBUG_INFO("Change codec volume: %u -> %u\r\n", app_audio_get_current_output_vol(), vol);
                    app_audio_change_codec_vol(vol);
                }
            }
        }

        /* Clear trạng thái FM module trước đó */
        if (strstr((char *)buffer, "MODE=LOCAL_IDLE"))
        {
            /* Nếu trước đó đang thu FM hoặc chạy từ MIC/LINE IN -> tắt codec, ngắt Relay và PA */
            if (m_fm_mode == APP_AUDIO_OPERATION_MODE_FM 
                || m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC
                || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
            {
                // Chuyển codec về mode DECODE
                app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                // Switch Relay to Codec output
                app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                /* Nếu PA đang ON thì OFF */
                if (!app_io_is_pa_off())
                {
                    app_io_control_pa(APP_IO_PA_OFF);
                }

                // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                DEBUG_INFO("call app_io_opto_control_all...\r\n");
                if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                {
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }
                else
                {
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
            }
            m_fm_mode = APP_AUDIO_OPERATION_MODE_IDLE;
        }

        /* Các trạng thái FM */
        char *freq = strstr((char *)buffer, "FREQ=");
        if (freq)
        {
            m_freq_at_fm_slave = GetNumberFromString(5, freq);
        }

        /* Tọa độ GPS: GPS=<LAT>,<LNG>| */
        char *gps = strstr((char *)buffer, "GPS=");
        if (gps)
        {
            memset(m_last_gps_info, 0, sizeof(m_last_gps_info));
            CopyParameter(gps, m_last_gps_info, '=', '|');
        }

        /* Bản tin phản hồi module FM
         * 4G,MODE=<FM/4G/MIC/NONE/IDLE>,FREQ_RUN=<10270>,FREQ1=<>,FREQ2=<>,FREQ3=<>,VOL=<70>,THRESHOLD=<90>,CRC=12345#
         */
        index = sprintf(response, "%s", "4G,MODE=");

        /* Chế độ/ trạng thái hoạt động */
        switch (app_flash_get_operate_mode())
        {
        case APP_AUDIO_OPERATION_MODE_INTERNET:
            /* Nếu đang không stream internet thì là ở trạng thái IDLE */
            if (!app_audio_is_http_audio_stream_running() &&
                (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !slave_at_least_one_master_is_streaming() ||
                 slave_get_http_received_data_timeout() == 0)) /* Quá lâu không nhận được gói tin http */
            {
                if (resend_mode_to_fm_module)
                {
                    index += sprintf(&response[index], "INTERNET,");
                }
                else
                {
                    index += sprintf(&response[index], "IDLE,");
                }
            }
            else
            {
                index += sprintf(&response[index], "INTERNET,");
            }
            break;

        case APP_AUDIO_OPERATION_MODE_FM:
            index += sprintf(&response[index], "FM,FREQ_RUN=%u,", app_flash_get_current_fm_freq() / 10);
            break;
        case APP_AUDIO_OPERATION_MODE_MIC:
            index += sprintf(&response[index], "MIC,");
            break;

        case APP_AUDIO_OPERATION_MODE_NO_OPERATION:
            index += sprintf(&response[index], "NONE,");
            break;
        case APP_AUDIO_OPERATION_MODE_IDLE:
            index += sprintf(&response[index], "IDLE,");
            break;
            
        default:
            break;
        }

        if (app_mqtt_is_connected_to_server())
        {
            index += sprintf(&response[index], "%s%s,", "CONNECTED,", NET_IF_TAB[network_get_current_interface()]);
        }
        else
        {
            index += sprintf(&response[index],  "%s", "DISCONNECTED,");
        }

        /* Volume */
        index += sprintf(&response[index], "VOL=%u,", app_flash_get_volume());

        /* Tần số cấu hình Freq1,2,3 : Đơn vị dưới module FM là 10KHz -> /10 */
        index += sprintf(&response[index], "FREQ1=%u,FREQ2=%u,FREQ3=%u,", app_flash_get_fm_freq1() / 10,
                         app_flash_get_fm_freq2() / 10, app_flash_get_fm_freq3() / 10);

        /* CRC and tail */
        uint16_t crc16 = CalculateCRC16((uint8_t *)response, index);
        index += sprintf(&response[index], "CRC=%05u#", crc16);
        /* send to UART */
        if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
        {
            xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
            uart_write_bytes(FM_UART_PORT_NUM, response, index);
            xSemaphoreGive(m_sem_protect_gd32_uart);
        }
        else
        {
            min_msg_t msg;
            msg.id = MIN_ID_FORWARD;
            msg.len = index;
            msg.payload = response;
            xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
            min_send_frame(&m_min_gd32_context, &msg);
            xSemaphoreGive(m_sem_protect_gd32_uart);
        }
    }
}

static void process_gd32_new_data(char *buffer, uint32_t len)
{
    char response[128] = {0};
    uint16_t index = 0;

    if (m_hw_version == 3)
    {
        // Chưa khởi tạo xong audio codec thì không nhận lệnh chuyển mode!
        if (!m_fm_allowed)
            return;
    }

    /** Xử lý bản tin FM
     * FM|LOCAL_MODE=<FM/MIC/LINE/IDLE>|FREQ=<FREQ>|VOL=<70>|SNR=<>|RSSI=<>|GPS=<VD,KD>|CRC=12345#
     */
    ESP_LOGD(TAG, "%s\r\n", (char *)buffer);
    if (strstr((char *)buffer, "FM|") && strstr((char *)buffer, "#") && len > 10)
    {
        /* Check CRC: |CRC=12345# */
        char *crc = strstr((char *)buffer, "CRC=");
        if (crc)
        {
            uint16_t crc16 = GetNumberFromString(4, crc);

            /* Tinh CRC thuc cua chuoi: Tru 10 ki tu cuoi CRC=12345# */
            uint16_t crcCal = CalculateCRC16((uint8_t *)buffer, len - 10);

            if (crc16 != crcCal)
            {
                DEBUG_INFO("FM CRC failed: %u - %u\r\n", crc16, crcCal);
                return;
            }
        }
        else
        {
            return;
        }

        if (m_fm_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
        {
            m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT;
        }

        /** Chỉ xét các chế độ đang chạy dưới module FM khi slave đang rảnh (chế độ INTERNET và không streaming) */
        if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
            && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !slave_at_least_one_master_is_streaming()))
        {
            if (strstr((char *)buffer, "MODE=LOCAL_FM"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_FM)
                {
                    DEBUG_INFO("Switch to Local FM mode\r\n");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_FM;

                    /* Change mode to FM */
                    app_audio_change_to_local_fm_mode();

                    // Tắt Relay (nếu được bật ở chế độ LINE IN)
                    DEBUG_INFO("call app_io_opto_control_all...\r\n");
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to Local FM mode");
                    #warning "think about classD"
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_MIC"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_MIC)
                {
                    DEBUG_INFO("Switch to Local MIC mode\r\n");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_MIC;

                    /* Change mode to MIC */
                    app_audio_change_to_local_mic_mode();

                    // Bật Relay cho chế độ MIC
                    mqtt_publish_message("DBG", "app_io_opto_control_all(): Switch to Local MIC mode");
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
            }
            if (strstr((char *)buffer, "MODE=LOCAL_LINE"))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_LINE)
                {
                    DEBUG_INFO("Switch to Local LINE mode\r\n");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_LINE;
                    
                    /* Change mode to LINE */
                    app_audio_change_to_local_line_in_mode();

                    // Bật Relay cho chế độ LINE IN
                    if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                    {
                        mqtt_publish_message("DBG", "Class AB: Switch to Local MIC mode");
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                    }
                    else
                    {
                        mqtt_publish_message("DBG", "Class D: Switch to Local MIC mode");
                        app_io_opto_control_all(APP_IO_OPTO_OFF);
                    }
                }
            }
        }

        /* Volume */
        char *sVolume = strstr((char *)buffer, "VOL=");
        if (sVolume)
        {
            uint8_t vol = GetNumberFromString(4, sVolume);

            /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
            if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
                 && app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
            {
                if (abs(app_audio_get_current_output_vol() - vol) > 3)
                {
                    DEBUG_INFO("Change codec volume: %u -> %u\r\n", app_audio_get_current_output_vol(), vol);

                    app_audio_change_codec_vol(vol);
                }
            }
        }

        /* Clear trạng thái FM module trước đó */
        if (strstr((char *)buffer, "MODE=LOCAL_IDLE"))
        {
            /* Nếu trước đó đang thu FM hoặc chạy từ MIC/LINE IN -> tắt codec, ngắt Relay và PA */
            if (m_fm_mode == APP_AUDIO_OPERATION_MODE_FM 
                || m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC
                 ||m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
            {
                // Chuyển codec về mode DECODE
                app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                // Switch Relay to Codec output
                app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                /* Nếu PA đang ON thì OFF */
                if (!app_io_is_pa_off())
                {
                    app_io_control_pa(APP_IO_PA_OFF);
                }

                // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                DEBUG_INFO("call app_io_opto_control_all...\r\n");
                if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                {
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }
                else
                {
                    app_io_opto_control_all(APP_IO_OPTO_ON);
                }
            }
            m_fm_mode = APP_AUDIO_OPERATION_MODE_IDLE;
        }

        /* Các trạng thái FM */
        char *freq = strstr((char *)buffer, "FREQ=");
        if (freq)
        {
            m_freq_at_fm_slave = GetNumberFromString(5, freq);
        }
        
        char *SNR = strstr((char *)buffer, "SNR=");
        if (SNR)
        {
            m_last_fm_msg.snr = GetNumberFromString(4, SNR);
        }
        char *RSSI = strstr((char *)buffer, "RSSI=");
        if (RSSI)
        {
            m_last_fm_msg.rssi = GetNumberFromString(5, RSSI);
        }
        char *dBm = strstr((char *)buffer, "dBm=");
        if (dBm)
        {
            m_last_fm_msg.dbm = GetNumberFromString(4, dBm);
        }

        /* Tọa độ GPS: GPS=<LAT>,<LNG>| */
        char *gps = strstr((char *)buffer, "GPS=");
        if (gps)
        {
            memset(m_last_gps_info, 0, sizeof(m_last_gps_info));
            CopyParameter(gps, m_last_gps_info, '=', '|');
        }

        /* Bản tin phản hồi module FM
         * 4G,MODE=<FM/4G/MIC/NONE/IDLE>,FREQ_RUN=<10270>,FREQ1=<>,FREQ2=<>,FREQ3=<>,VOL=<70>,THRESHOLD=<90>,CRC=12345#
         */
        index = sprintf(response, "4G,MODE=");
        int remote_vol = app_flash_get_volume();

        /* Chế độ/ trạng thái hoạt động */
        switch (app_flash_get_operate_mode())
        {
        case APP_AUDIO_OPERATION_MODE_INTERNET:
            /* Nếu đang không stream internet thì là ở trạng thái IDLE */
            // audio_element_state_t el_i2s_state = audio_element_get_state(i2s_stream_writer);
            // audio_element_state_t el_opus_state = audio_element_get_state(opus_decoder);
            if (!app_audio_is_http_audio_stream_running()
                 && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !slave_at_least_one_master_is_streaming() ||
                 slave_get_http_received_data_timeout() == 0)) /* Quá lâu không nhận được gói tin http */
            {
                if (resend_mode_to_fm_module)
                {
                    index += sprintf(&response[index], "INTERNET,");
                }
                else
                {
                    index += sprintf(&response[index], "IDLE,");
                }
            }
            else
            {
                index += sprintf(&response[index], "INTERNET,");
            }

            break;
        case APP_AUDIO_OPERATION_MODE_FM:
            index += sprintf(&response[index], "FM,FREQ_RUN=%u,", app_flash_get_current_fm_freq() / 10);
            remote_vol = 100;
            break;
        case APP_AUDIO_OPERATION_MODE_MIC:
            index += sprintf(&response[index], "MIC,");
            break;
        case APP_AUDIO_OPERATION_MODE_NO_OPERATION:
            index += sprintf(&response[index], "NONE,");
            break;
        case APP_AUDIO_OPERATION_MODE_IDLE:
            index += sprintf(&response[index], "IDLE,");
            break;
        default:
            break;
        }

        /* Tần số cấu hình Freq1,2,3 : Đơn vị dưới module FM là 10KHz -> /10 */
        index += sprintf(&response[index], "FREQ1=%u,FREQ2=%u,FREQ3=%u,", app_flash_get_fm_freq1() / 10,
                         app_flash_get_fm_freq2() / 10, app_flash_get_fm_freq3() / 10);

        /* Volume */
        index += sprintf(&response[index], "VOL=%u,", remote_vol);

        /* CRC and tail */
        uint16_t crc16 = CalculateCRC16((uint8_t *)response, index);
        index += sprintf(&response[index], "CRC=%05u#", crc16);

        if (resend_mode_to_fm_module)
            resend_mode_to_fm_module--;
        xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
        /* send to UART */
        uart_write_bytes(FM_UART_PORT_NUM, response, index);
        xSemaphoreGive(m_sem_protect_gd32_uart);
    }
    /* Xử lý bản tin từ IO_EXT MCU STM32/GD32 */
    else if (strstr((char *)buffer, "WORKER|") && strstr((char *)buffer, "#") && len > 10)
    {
        /* Check CRC: |CRC=12345# */
        char *crc = strstr((char *)buffer, "CRC=");
        if (crc)
        {
            uint16_t crc16 = GetNumberFromString(4, crc);

            /* Tinh CRC thuc cua chuoi: Tru 10 ki tu cuoi CRC=12345# */
            uint16_t crcCal = CalculateCRC16((uint8_t *)buffer, len - 10);

            if (crc16 != crcCal)
            {
                DEBUG_WARN("GD32 CRC failed: %u - %u\r\n", crc16, crcCal);
                return;
            }
        }
        else
        {
            return;
        }

        // Trạng thái input detect speaker: chỉ detect đúng khi en_pa đang off
        char *isoInput = strstr((char *)buffer, "ISOINPUT=");
        if (isoInput != NULL)
        {
            uint8_t spkInput = GetNumberFromString(9, isoInput);

            // Test: print trạng thái (theo bảng trạng thái)
            /*switch(spkInput) {
                case 7:
                case 15:
                    DEBUG_INFO("Older version, does not support: %d", spkInput);
                    break;
                //Class AB
                case 12:
                    DEBUG_INFO("Class AB: Normal (%d)", spkInput);
                    break;
                case 13:
                    DEBUG_INFO("Class AB: Short to GND (%d)", spkInput);
                    break;
                case 14:
                    DEBUG_INFO("Class AB: Open circuit (%d)", spkInput);
                    break;
                //Class D
                case 0:
                    DEBUG_INFO("Class D: Normal (%d)", spkInput);
                    break;
                case 1:
                    DEBUG_INFO("Class D: Short to GND (%d)", spkInput);
                    break;
                case 6:
                    DEBUG_INFO("Class D: Open circuit (%d)", spkInput);
                    break;
                default:
                    DEBUG_INFO("Unknow input state: %d", spkInput);
                    break;
            } */

            uint8_t class = app_flash_speaker_audio_class_get();
            // Only get speaker state if device is not streaming
            if (class == APP_FLASH_AUDIO_CLASS_AB)
            {
                // Class AB : get speaker status when PA off
                if (i2c_io_expander.BitName.en_pa == APP_IO_PA_OFF && mcu_expander.BitName.en_pa == APP_IO_STM_PA_OFF)
                    m_speaker_detect_state.value = spkInput;
            }
            else if (class == APP_FLASH_AUDIO_CLASS_D)
            {
                // Class D : get speaker status when ISO output 1 & 2 off
                if (i2c_io_expander.BitName.ISO_OUT1 == APP_IO_OPTO_OFF && mcu_expander.BitName.ISO_OUT2 == APP_IO_OPTO_OFF)
                    m_speaker_detect_state.value = spkInput;
            }
            // else
            // DEBUG_INFO("en_pa is ON, don't detect speaker state: %d", spkInput);
        }
        else
        {
            m_speaker_detect_state.value = 255;
        }

        if (!m_is_in_test_mode)
        {
            // if (strstr((char *)buffer, "TESTMODE=1|"))
            // {
            //     slave_set_audio_switch_audio_in_test_mode();
            //     m_is_in_test_mode = 1;
            //     sprintf(app_audio_get_stream_url(), "%s%s", app_flash_get_http_stream_header(), "863674040993444");
            //     // Auto enable speaker_detect mode
            //     app_flash_write_u8(APP_FLASH_SPK_CLASS, 1);
            //     app_flash_speaker_audio_class_set(1);
            //     app_audio_test_mode_change_codec_vol(100);
            // }
        }
        // Version mạch MCU
        char *versionWorker = strstr((char *)buffer, "VERSION=");
        if (versionWorker != NULL)
        {
            m_worker_version = GetNumberFromString(8, versionWorker);
        }
        else
        {
            m_worker_version = 0;
        }

        char *vin = strstr((char *)buffer, "VIN=");
        if (vin != NULL)
        {
            m_worker_last_vin = GetNumberFromString(8, vin);
        }
        else
        {
            m_worker_last_vin = 0;
        }

        m_gd32_protocol_state = APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT;
        // Phản hồi lệnh điều khiển GPIO nếu giá trị gpio dưới STM32 khác ESP32
        app_io_mcu_update_io(&mcu_expander);
    }
}

void app_io_exp_uart_task(void *arg)
{
    uart_event_t event;

    DEBUG_INFO("\t\t--- app_io_exp_uart_task is running ---\r\n");

    while (1)
    {
        // Loop will continually block (i.ez. wait) on event messages from the event queue
        if (xQueueReceive(fm_uart_queue, (void *)&event, (portTickType)50))
        {
            // Handle received event
            if (event.type == UART_DATA)
            {
                fm_uart_rx_len = 0;
                memset(fm_uart_rx_buf, 0, FM_UART_RX_BUF_SIZE);

                uint16_t uart_received_length = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(FM_UART_PORT_NUM, (size_t *)&uart_received_length));
                fm_uart_rx_len = uart_read_bytes(FM_UART_PORT_NUM, fm_uart_rx_buf, uart_received_length, 100);

                /* Process data */
                if (fm_uart_rx_len > 0)
                {
                    if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT)
                    {
                        // ESP_LOGW(TAG, "Feed1 %u bytes\r\n", fm_uart_rx_len);
                        for (uint32_t i = 0; i < fm_uart_rx_len; i++)
                        {
                            min_rx_feed(&m_min_gd32_context, fm_uart_rx_buf + i, 1);
                        }
                    }

                    if (m_gd32_protocol_state != APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
                    {
                        process_gd32_new_data((char*)fm_uart_rx_buf, fm_uart_rx_len);
                        if (m_gd32_protocol_state == APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT && m_min_gd32_rx_buffer)
                        {
                            ESP_LOGW(TAG, "GD32 main used string protocol");
                            free(m_min_gd32_rx_buffer);
                            m_min_gd32_rx_buffer = NULL;
                        }
                    }
                    else
                    {
                        min_timeout_poll(&m_min_gd32_context);
                    }
                }
            }
        }
        else
        {
            app_io_adc_poll();
        }
        if (m_gd32_protocol_state == APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL)
        {
            min_timeout_poll(&m_min_gd32_context);
        }

        if (m_fm_protocol_state == APP_IO_MCU_PROTOCOL_STATE_STRING_FORMAT && m_min_fm_rx_buffer)
        {
            ESP_LOGD(TAG, "FM module used string protocol\r\n");
            free(m_min_fm_rx_buffer);
            m_min_fm_rx_buffer = NULL;
        }

        static uint32_t m_last_tick = 0;
        if (xTaskGetTickCount() - m_last_tick >= (uint32_t)1000)
        {
            m_last_tick = xTaskGetTickCount();
            process_dtmf_state();
            static uint32_t debug_cnt = 0;
            if (debug_cnt++ == 120)
            {
                debug_cnt = 0;
                DEBUG_VERBOSE("Temp %d, vol %d, level %d\r\n", 
                            app_io_get_temperature(), 
                            app_io_get_ntc_voltage(), 
                            app_io_get_btn_on_air_level());
            }
        }
    }

    DEBUG_VERBOSE("\t\t--- app_io_exp_uart_task is stopped ---\r\n");
    vTaskDelete(NULL);
}

uint8_t app_io_get_hardware_version(void)
{
    return m_hw_version;
}

void app_io_set_hardware_version(uint8_t version)
{
    m_hw_version = version;
}

bool app_io_is_mic_plugged(void)
{
    return !gpio_get_level(APP_IO_MIC_DETECT_NUM);
}

/**
 * Điều khiển ON/OFF cả 2 OptoOutput cho relay ngoài
 */
void app_io_opto_control_all(uint8_t state)
{
    if (state == APP_IO_OPTO_ON)
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_ON;
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_ON;

        // STM32
        mcu_expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_ON;
        mcu_expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_ON;

        m_io_control.Name.IO1 = IO_ON;
        m_io_control.Name.IO2 = IO_ON;

        // Reset delay timeout all
        slave_reset_delay_turn_off_relay1_when_stop_stream();
        slave_reset_delay_turn_off_relay2_when_stop_stream();
    }
    else
    {
        app_io_get_i2c_exp_value()->BitName.ISO_OUT1 = APP_IO_ISOOUT_OFF;
        app_io_get_i2c_exp_value()->BitName.ISO_OUT2 = APP_IO_ISOOUT_OFF;

        // STM32
        mcu_expander.BitName.ISO_OUT1 = APP_IO_STM_ISO_OUT_OFF;
        mcu_expander.BitName.ISO_OUT2 = APP_IO_STM_ISO_OUT_OFF;

        m_io_control.Name.IO1 = IO_OFF;
        m_io_control.Name.IO2 = IO_OFF;
    }

    if (m_hw_version == 2)
    {
        Int_t ex_out;
        ex_out.value = app_io_get_i2c_exp_value()->Value;
        app_audio_hal_i2c_master_write(PCF8575_I2C_ADDR, ex_out.bytes, 2);
    }
    else if (m_hw_version == 3)
    {
        app_io_mcu_update_io(&mcu_expander);
    }

    // Lưu lại trạng thái output Opto
    app_flash_write_u8(APP_FLASH_IO_STATE_KEY, m_io_control.Value);
}


bool app_io_is_switch_codec_on(void)
{
    if (m_hw_version == 2)
    {
        if (app_io_get_i2c_exp_value()->BitName.sw_codec_fm != APP_IO_OUT_CODEC_ON)
            return false;
    }
    else if (m_hw_version == 3)
    {
        if (mcu_expander.BitName.sw_codec_fm != APP_IO_STM_CODEC_ON)
            return false;
    }
    return true;
}

static void on_fm_frame_callback(void *context, min_msg_t *frame)
{
    m_fm_protocol_state = APP_IO_MCU_PROTOCOL_STATE_MIN_PROTOCOL;
    ESP_LOGD(TAG, "ON FM frame callback id %d", frame->id);
    switch (frame->id)
    {
        case MIN_ID_PING:
        case MIN_ID_FORWARD:
        {
            app_io_fm_ping_msg_t *msg = (app_io_fm_ping_msg_t*)frame->payload;
            if (msg->vol != m_last_fm_msg.vol)
            {
                m_delay_save_local_vol = 10;
            }
            if (m_delay_save_local_vol)
            {
                m_delay_save_local_vol--;
                if (m_delay_save_local_vol == 0)
                {
                    // app_flash_set_local_volume(msg->vol);
                    ESP_LOGD(TAG, "Save local vol %u\r\n", msg->vol);
                }
            }

            memcpy(&m_last_fm_msg, msg, sizeof(app_io_fm_ping_msg_t));
            
            uint8_t mode = APP_AUDIO_OPERATION_MODE_NO_OPERATION;
            uint32_t freq = 0xFFFFFFFF;
            if (app_audio_is_http_audio_stream_running())
            {
                mode = APP_AUDIO_OPERATION_MODE_INTERNET;
            }
            else
            {
                mode = app_flash_get_operate_mode();
                freq = app_flash_get_current_fm_freq() / 10;
            }
            
            m_worker_version = m_last_fm_msg.fw_version;
            /** Chỉ xét các chế độ đang chạy dưới module FM khi slave đang rảnh (chế độ INTERNET và không streaming) */
            if (app_flash_get_operate_mode() == APP_AUDIO_OPERATION_MODE_INTERNET 
                && (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !slave_at_least_one_master_is_streaming()))
            {
                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_FM
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_FM)
                {
                    DEBUG_INFO("Switch to Local FM mode\r\n");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_FM;

                    /* Change mode to FM */
                    app_audio_change_to_local_fm_mode();
                    mqtt_publish_message("DBG", "Switch to Local FM mode");
                    #warning "Think about class D"
                    app_io_opto_control_all(APP_IO_OPTO_OFF);
                }

                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_MIC
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_MIC)
                {
                    DEBUG_INFO("Switch to Local MIC mode");

                    m_fm_mode = APP_AUDIO_OPERATION_MODE_MIC;

                    /* Change mode to MIC */
                    app_audio_change_to_local_mic_mode();
                    mqtt_publish_message("DBG", "Switch to Local MIC mode");
                    if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                    {
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                    }
                    else
                    {
                        app_io_opto_control_all(APP_IO_OPTO_OFF);
                    }
                }

                if (m_fm_mode != APP_AUDIO_OPERATION_MODE_LINE
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_LINE)
                    {
                        DEBUG_INFO("Switch to Local LINE mode\r\n");

                        m_fm_mode = APP_AUDIO_OPERATION_MODE_LINE;
                        
                        /* Change mode to LINE */
                        app_audio_change_to_local_line_in_mode();
                        // Bật Relay cho chế độ LINE IN
                        if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                        {
                            app_io_opto_control_all(APP_IO_OPTO_ON);
                            mqtt_publish_message("DBG", "Class AB switch to Local LINE mode");
                        }
                        else
                        {
                            app_io_opto_control_all(APP_IO_OPTO_OFF);
                            mqtt_publish_message("DBG", "Class D switch to Local LINE mode");
                        }
                        app_audio_change_codec_vol(m_last_fm_msg.vol);
                    }

                /* Clear trạng thái FM module trước đó khi chuyển về trạng thái IDLE */
                if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_FM 
                    || m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC
                    || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
                    && m_last_fm_msg.mode == APP_AUDIO_OPERATION_MODE_IDLE)
                {
                    DEBUG_INFO("Switch to IDLE mode\r\n");
                    // Chuyển codec về mode DECODE
                    app_audio_change_codec_mode(APP_AUDIO_OPERATION_MODE_INTERNET);

                    // Switch Relay to Codec output
                    app_io_control_relay_audio_output(APP_AUDIO_OPERATION_MODE_INTERNET);

                    /* Nếu PA đang ON thì OFF */
                    if (!app_io_is_pa_off())
                    {
                        app_io_control_pa(APP_IO_PA_OFF);
                    }

                    // Tắt Relay (nếu được bật ở chế độ LINE/MIC)
                    DEBUG_INFO("call app_io_opto_control_all...\r\n");
                    if (app_flash_speaker_audio_class_get() != APP_FLASH_AUDIO_CLASS_D)
                    {
                        app_io_opto_control_all(APP_IO_OPTO_OFF);
                    }
                    else
                    {
                        app_io_opto_control_all(APP_IO_OPTO_ON);
                    }
                    m_fm_mode = APP_AUDIO_OPERATION_MODE_IDLE;
                }

                /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
                if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE)
                     && app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
                {
                    if (abs(app_audio_get_current_output_vol() - m_last_fm_msg.vol) > 3)
                    {
                        DEBUG_INFO("Change codec volume: %u -> %u\r\n", app_audio_get_current_output_vol(), m_last_fm_msg.vol);
                        app_audio_change_codec_vol(m_last_fm_msg.vol);
                    }
                }
                
                sprintf(m_last_gps_info, "%.6f,%.6f", m_last_fm_msg.gps_lat, m_last_fm_msg.gps_long);

                /* Chế độ/ trạng thái hoạt động */
                switch (app_flash_get_operate_mode())
                {
                    case APP_AUDIO_OPERATION_MODE_INTERNET:
                        /* Nếu đang không stream internet thì là ở trạng thái IDLE */
                        if (!app_audio_is_http_audio_stream_running() &&
                            (m_fm_mode != APP_AUDIO_OPERATION_MODE_BUSY || !slave_at_least_one_master_is_streaming()
                            || slave_get_http_received_data_timeout() == 0)) /* Quá lâu không nhận được gói tin http */
                        {
                            if (resend_mode_to_fm_module)
                            {
                                resend_mode_to_fm_module--;
                                mode = APP_AUDIO_OPERATION_MODE_INTERNET;
                            }
                            else
                            {
                                mode = APP_AUDIO_OPERATION_MODE_IDLE;
                            }
                        }
                        else
                        {
                            mode = APP_AUDIO_OPERATION_MODE_IDLE;
                        }
                        break;

                    case APP_AUDIO_OPERATION_MODE_FM:
                        freq = app_flash_get_current_fm_freq() / 10;
                        mode = APP_AUDIO_OPERATION_MODE_FM;
                        break;
                    case APP_AUDIO_OPERATION_MODE_MIC:
                        mode = APP_AUDIO_OPERATION_MODE_MIC;
                        break;

                    case APP_AUDIO_OPERATION_MODE_NO_OPERATION:
                        mode = APP_AUDIO_OPERATION_MODE_NO_OPERATION;
                        break;

                    case APP_AUDIO_OPERATION_MODE_IDLE:
                        mode = APP_AUDIO_OPERATION_MODE_IDLE;
                        break;
                        
                    default:
                        // mode = APP_AUDIO_OPERATION_MODE_NO_OPERATION;
                        break;
                }

                m_freq_at_fm_slave = m_last_fm_msg.freq;

                // DEBUG_INFO("mode %u, vol %u, (%.6f-%.6f), freq %uKhz, ISO[1-2] = [%u-%u], input %u\r\n", 
                //         m_last_fm_msg.mode,
                //         m_last_fm_msg.vol,
                //         m_last_fm_msg.gps_lat,
                //         m_last_fm_msg.gps_long,
                //         m_last_fm_msg.freq/1000,
                //         mcu_expander.BitName.ISO_OUT1, mcu_expander.BitName.ISO_OUT2,
                //         app_io_get_speaker_detect_value().value);

                // // forward to gd32 in main
                // reply.id = MIN_ID_FORWARD;
                // reply.payload = buffer;
                // reply.len = size;
                // min_send_frame(&m_min_gd32_context, &reply);
            }
            else
            {
                DEBUG_VERBOSE("In streaming\r\n");
            }
            
            /* Nếu ở chế độ Local MIC/LINE thì cho phép điều chỉnh volume từ núm vặn */
            if ((m_fm_mode == APP_AUDIO_OPERATION_MODE_MIC || m_fm_mode == APP_AUDIO_OPERATION_MODE_LINE) 
               && app_audio_get_codec_mode() == APP_AUDIO_CODEC_MODE_LINE_IN) /* Chỉ cho thay đổi volume codec khi đang ở mode LINE_IN */
            {
                if (abs(app_audio_get_current_output_vol() - m_last_fm_msg.vol) > 3)
                {
                    DEBUG_INFO("Change codec volume: %u -> %u\r\n", app_audio_get_current_output_vol(), m_last_fm_msg.vol);
                    app_audio_change_codec_vol(m_last_fm_msg.vol);
                }
            }

            if (m_publish_fm_info == 1)
            {
                m_publish_fm_info++;
                if (m_publish_fm_info >= 5 && app_mqtt_is_connected_to_server())
                {
                    m_publish_fm_info = 0;
                    mqtt_publish_message("DBG", "FM reset, version %u-%u", 
                                                m_last_fm_msg.fw_version, 
                                                m_last_fm_msg.hw_version);
                }
            }

            static app_io_esp32_to_fm_frame_t frame;
            uint8_t mqtt_state = app_mqtt_get_state();
            frame.remember_local_state = 0;
            frame.current_freq = freq;
            frame.mode = mode;
            frame.server_state = mqtt_state;
            frame.interface = network_get_current_interface();;
            frame.freq0 = app_flash_get_fm_freq1() / 10;
            frame.freq1 = app_flash_get_fm_freq2() / 10;
            frame.freq2 = app_flash_get_fm_freq3() / 10;
            frame.volume = app_flash_get_volume();
            frame.csq = slave_get_gsm_csq();
            frame.band = slave_get_gsm_band();
            frame.temp = m_temperature;
            if (frame.volume == 0)
            {
                ESP_LOGE(TAG, "Set volume FM = 0");
            }
            
            
            static uint8_t buffer[MIN_MAX_PAYLOAD];
            uint32_t size = 0;


            min_msg_t reply;
            reply.id = MIN_ID_PING;
            reply.payload = (uint8_t*)&frame;
            reply.len = sizeof(frame);
            min_build_raw_frame_output(&reply, buffer, &size);
            reply.id = MIN_ID_FORWARD;
            reply.payload = buffer;
            reply.len = size;
            min_send_frame(&m_min_gd32_context, &reply);
            // ESP_LOGD(TAG, "mode %u, vol %u, gps (%.6f-%.6f), freq %uKhz, interface %d", 
            //         m_last_fm_msg.mode,
            //         m_last_fm_msg.vol,
            //         m_last_fm_msg.gps_lat,
            //         m_last_fm_msg.gps_long,
            //         m_last_fm_msg.freq/1000,
            //         frame.interface);
            if (app_flash_enable_test_ab())
            {
                min_msg_t reply;
                uint8_t error = app_io_is_speaker_error() ? 1 :  0;
                if (error == 0)
                {
                    if (m_speaker_detect_state.value == 15 
                        || m_speaker_detect_state.value == 7)
                    {
                        error = m_speaker_detect_state.value;
                    }
                }
                if (error)
                {
                    error = m_speaker_detect_state.value;
                }
                reply.id = MIN_ID_AUDIO_DETECT_STATUS;
                reply.payload = &error;
                reply.len = 1;
                
                min_build_raw_frame_output(&reply, buffer, &size);
                reply.id = MIN_ID_FORWARD;
                reply.payload = buffer;
                reply.len = size;
                min_send_frame(&m_min_gd32_context, &reply);
            }

            // Send imei to LCD
            if (m_send_imei_to_lcd > 0)
            {
                m_send_imei_to_lcd--;
                min_msg_t reply;
                reply.id = MIN_ID_FM_SHOW_IMEI;
                reply.payload = app_flash_get_imei();
                reply.len = 15;
                
                min_build_raw_frame_output(&reply, buffer, &size);
                reply.id = MIN_ID_FORWARD;
                reply.payload = buffer;
                reply.len = size;
                min_send_frame(&m_min_gd32_context, &reply);
            }
        }
            break;
        case MIN_ID_RESET:
        {
            ESP_LOGE(TAG, "FM reset\r\n");
            m_publish_fm_info = 1;
        }
            break;

        case MIN_ID_MCU_EXPANDER_REPORT_CHIP:
        {
            DEBUG_INFO("Report chip\r\n");
            snprintf((char*)&m_mcu_chipname[MCU_FM_CHIP_NAME_INDEX][0], 
                    MCU_MAX_NAME_LEN, 
                    "%s", (char*)frame->payload);
        }
        break;

        default:
            if (frame->len == FM_RDS_MSG_SIZE)
            {
                // DEBUG_DUMP(frame->payload, FM_RDS_MSG_SIZE, "RDS");
            }
            else
            {
                DEBUG_INFO("Unknown min id %d\r\n", frame->id);
            }
            // app_ota_on_slave_frame_callback(frame);
            break;
    }
}

void app_io_send_custom_frame_to_gd32(uint8_t id, uint8_t* data, uint32_t size)
{
    min_msg_t msg;
    msg.id = id;
    msg.len = size;
    msg.payload = data;
    xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
    min_send_frame(&m_min_gd32_context, &msg);
    xSemaphoreGive(m_sem_protect_gd32_uart);
}

void app_io_send_custom_frame_to_fm(uint8_t id, uint8_t* data, uint32_t size)
{
    min_msg_t reply;
    uint32_t out_size = 0;
    reply.id = id;
    reply.payload = (uint8_t*)data;
    reply.len = size;

    xSemaphoreTake(m_sem_protect_gd32_uart, portMAX_DELAY);
    static uint8_t buffer[MIN_MAX_PAYLOAD];
    min_build_raw_frame_output(&reply, buffer, &out_size);
    ESP_LOGD(TAG, "Send min frame size %u", out_size);

    // forward to gd32 in main
    reply.id = MIN_ID_FORWARD;
    reply.payload = buffer;
    reply.len = out_size;

    min_send_frame(&m_min_gd32_context, &reply);
    xSemaphoreGive(m_sem_protect_gd32_uart);
}

static bool gd32_uart_tx(void *ctx, uint8_t data)
{
    if (ESP_OK != uart_write_bytes(FM_UART_PORT_NUM, (uint8_t*)&data, 1))
    {
        return false;
    }
    return true;
}

// void set_led_internet_state (uint8_t state)
// {
//     esp_err_t ret = ESP_OK;
//     if (state)
//     {
//         m_hw_output_state.BitName.LED_INTERNET = 1;
//     }
//     else
//     {
//         m_hw_output_state.BitName.LED_INTERNET = 0;
//     }
//     ret = i2c_app_io_set (&m_hw_output_state);
//     if (ret != ESP_OK)
//     {
//         DEBUG_ERROR ("SEND IO STATE THROUGH I2C FAIL:%x\r\n", ret);
//     }
// }

// void set_PA_state (uint8_t state)
// {
//     esp_err_t ret = ESP_OK;
//     if (state)
//     {
//         m_hw_output_state.BitName.EN_PA = 1;
//     }
//     else
//     {
//         m_hw_output_state.BitName.EN_PA = 0;
//     }
//     ret = i2c_app_io_set (&m_hw_output_state);
//     if (ret != ESP_OK)
//     {
//         DEBUG_ERROR ("SEND IO STATE THROUGH I2C FAIL:%x\r\n", ret);
//     }
// }
// void set_led_debug_state (uint8_t state)
// {
//     esp_err_t ret = ESP_OK;
//     if (state)
//     {
//         m_hw_output_state.BitName.LED_DEBUG = 1;
//     }
//     else
//     {
//         m_hw_output_state.BitName.LED_DEBUG = 0;
//     }
//     ret = i2c_app_io_set (&m_hw_output_state);
//     if (ret != ESP_OK)
//     {
//         DEBUG_ERROR ("SEND IO STATE THROUGH I2C FAIL:%x\r\n", ret);
//     }
// }

// void set_led_state (app_io_i2c_t* info_to_send)
// {
//     esp_err_t ret = ESP_OK;
//     ret = i2c_app_io_set (info_to_send);
//     if (ret != ESP_OK)
//     {
//         DEBUG_ERROR ("SEND IO STATE THROUGH I2C FAIL:%x\r\n", ret);
//     }
// }

uint8_t get_button_state (void)
{
    return button_pin_state;
}
void set_button_state (uint8_t val)
{
    button_pin_state = val;
}
void read_button_task (void* arg)
{
    //DEBUG_WARN ("READ BUTTON TASK IS CREATED\r\n");
    static uint32_t cnt = 0; 
    static uint32_t m_last_tick = 0;
    while (1)
    {
        if (cnt++ > 200)
        {
            cnt = 0; 
            //DEBUG_WARN ("READ BUTTON TASK IS running\r\n");
        }
        if (!gpio_get_level(BUTTON_INPUT) && get_button_state() == BTN_IS_RELEASED)
        {
            if (m_last_tick == 0) m_last_tick = xTaskGetTickCount();
            if (xTaskGetTickCount() - m_last_tick >= (uint32_t)100)
            {
                m_last_tick = xTaskGetTickCount();
                DEBUG_WARN ("BUTTON IS PRESSED!\r\n");
                //restart_pipeline ();
                set_button_state(BTN_IS_PRESSED);
            }
        }
        else if (gpio_get_level(BUTTON_INPUT) == 1)
        {
            m_last_tick = xTaskGetTickCount();
            set_button_state(BTN_IS_RELEASED);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }   
}

void board_gpio_config (void)
{
    gpio_config_t io_config = {};
    //out put no pull
    io_config.pin_bit_mask = GPIO_INPUT_SEL;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.intr_type = GPIO_INTR_DISABLE;
    io_config.pull_down_en = 0;
    io_config.pull_up_en = 1;
    gpio_config (&io_config);
    xTaskCreate(read_button_task, "read_button_task", 4*1024, NULL, 2, NULL);
    // SD_card_config();
}
esp_err_t SD_card_config (void)
{
    // if (mode != SD_MODE_1_LINE) {
    //     ESP_LOGE(TAG, "current board only support 1-line SD mode!");
    //     return ESP_FAIL;
    // }
    // esp_periph_set_handle_t set;
    // periph_sdcard_cfg_t sdcard_cfg = {
    //     .root = "/sdcard",
    //     .card_detect_pin = -1,//get_sdcard_intr_gpio(), // GPIO_NUM_34
    //     .mode = SD_MODE_SPI
    // };
    // esp_periph_handle_t sdcard_handle = periph_sdcard_init(&sdcard_cfg);
    // esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    // esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    // esp_err_t ret = esp_periph_start(set, sdcard_handle);
    // int retry_time = 5;
    // bool mount_flag = false;
    // while (retry_time --) {
    //     if (periph_sdcard_is_mounted(sdcard_handle)) {
    //         mount_flag = true;
    //         break;
    //     } else {
    //         vTaskDelay(500 / portTICK_PERIOD_MS);
    //     }
    // }
    // if (mount_flag == false) {
    //     ESP_LOGE(TAG, "Sdcard mount failed");
    //     return ESP_FAIL;
    // }
    // return ESP_OK;
     esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");
    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
#ifndef USE_SPI_MODE
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // To use 1-line SD mode, uncomment the following line:
    // slot_config.width = 1;
    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2, GPIO_PULLUP_ONLY);    // D0, needed in 4- and 1-line modes
    gpio_set_pull_mode(4, GPIO_PULLUP_ONLY);    // D1, needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3, needed in 4- and 1-line modes
    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
#else
    ESP_LOGI(TAG, "Using SPI peripheral");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    //host.max_freq_khz = 1000;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }
    gpio_config_t io_config = {};
    //out put no pull
    // io_config.pin_bit_mask = GPIO_OUTPUT_SEL;
    // io_config.mode = GPIO_MODE_OUTPUT;
    // io_config.intr_type = GPIO_INTR_DISABLE;
    // io_config.pull_down_en = 0;
    // io_config.pull_up_en = 1;
    // gpio_config (&io_config);
    // io_config.pin_bit_mask = GPIO_INPUT_SEL;
    // io_config.mode = GPIO_MODE_INPUT;
    // io_config.intr_type = GPIO_INTR_DISABLE;
    // io_config.pull_down_en = 0;
    // io_config.pull_up_en = 0;
    // gpio_config (&io_config);
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
#endif //USE_SPI_MODE
        if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE *f = fopen(MOUNT_POINT"/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello %s!\n", card->cid.name);
    fclose(f);
    ESP_LOGI(TAG, "File written");
    // Check if destination file exists before renaming
    struct stat st;
    if (stat(MOUNT_POINT"/foo.txt", &st) == 0) {
        // Delete it if it exists
        unlink(MOUNT_POINT"/foo.txt");
    }
    // Rename original file
    ESP_LOGI(TAG, "Renaming file");
    if (rename(MOUNT_POINT"/hello.txt", MOUNT_POINT"/foo.txt") != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }
    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen(MOUNT_POINT"/foo.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);
    // // All done, unmount partition and disable SDMMC or SPI peripheral
    // esp_vfs_fat_sdcard_unmount(mount_point, card);
    // ESP_LOGI(TAG, "Card unmounted");
#ifdef USE_SPI_MODE
    //deinitialize the bus after all devices are removed
    // spi_bus_free(host.slot);
#endif
}