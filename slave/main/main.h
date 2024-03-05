#ifndef MAIN_H
#define MAIN_H

#include "stdint.h"
#include "stdbool.h"

/**
 * @brief       Get modem dce
 */
void *master_get_modem_dce(void);


/**
 * @brief       Get CSQ
 * @retval      CSQ value, 0 or 99 on error
 */
uint8_t slave_get_gsm_csq(void);

/**
 * @brief       Get GSM band
 */
uint8_t slave_get_gsm_band(void);

/**
 * @brief       Software reset system
 * @param[in]   reason Reset reason
 */
void system_software_reset(uint8_t reason);

/**
 * @brief       Get modem dce
 */
void *slave_get_modem_dce(void);

/**
 * @brief       Set delay prepare stream value
 * @param[in]   value Timeout value
 */
void slave_set_delay_turn_on_relay_prepare_stream(uint8_t value);

/**
 * @brief       Get delay prepare stream value
 */
uint8_t slave_get_delay_turn_on_relay_prepare_stream(void);

/**
 * @brief       Reset delay prepare stream value
 */
void slave_reset_delay_turn_on_relay_process_on_air(void);

/**
 * @brief       Report slave info immediately
 */
void slave_report_info_now(void);

/**
 * @brief       Get stream time in 1 session
 */
uint32_t slave_get_stream_time(void);

/**
 * @brief       Get stream time in 1 day
 */
uint32_t slave_get_stream_time_in_day(void);

/**
 * @brief       Increase number of stream bytes received
 */
void slave_increase_stream_data_downloaded(uint32_t number_of_bytes);

/**
 * @brief       Reset total stream bytes received
 */
void slave_reset_stream_monitor_data(void);

/**
 * @brief       Set slave auto restart stream timeout
 */
void slave_set_auto_restart_stream_timeout(uint8_t timeout);


/**
 * @brief       Get software reset reason
 */
uint8_t system_get_software_reset_reason(void);

/**
 * @brief       Clear software reset reason
 */
void system_clear_software_reset_reason(void);

/**
 * @brief       Get HTTP finish track timeout
 */
uint8_t slave_get_http_finish_track_timeout(void);

/**
 * @brief       Set HTTP finish track timeout
 */
void slave_set_http_finish_track_timeout(uint8_t timeout);

/**
 * @brief       Set mqtt state timeout
 */
void slave_set_mqtt_state_timeout(uint8_t timeout);

/**
 * @brief       Set timeout remain until device enter streaming state
 */
void slave_set_received_running_state_timeout(uint32_t timeout);

/**
 * @brief       Get timeout remain until device enter streaming state
 */
uint8_t slave_get_received_running_state_timeout(void);

void slave_set_timeout_number_of_waiting_master_streaming(uint8_t counter);

/**
 * @brief       Get number of master is streaming
 */
uint8_t slave_get_number_of_second_timeout_waiting_master_streaming(void);

/**
 * @brief       Slave process stop on air
 */
void slave_process_stop_onair(void);

/**
 * @brief       Send timeout on start stream
 */
void slave_set_start_stream_command_timeout(uint8_t timeout);

/**
 * @brief       Get timeout on start stream
 */
uint8_t slave_get_start_stream_command_timeout(void);

/**
 * @brief       Allow monitor stream speed state
 */
void slave_set_http_element_in_streaming_state(bool state);

/**
 * @brief       Get number of bytes stream downloaded
 */
uint32_t slave_get_total_streaming_received(void);

/**
 * @brief       Reset number of bytes stream downloaded
 */
void slave_reset_total_streaming_received(void);

/**
 * @brief       Set last stream data = total stream data
 */
void slave_update_last_stream_data(void);

/**
 * @brief       Reset delay timeout turn off relay 1
 */
void slave_reset_delay_turn_off_relay1_when_stop_stream(void);

/**
 * @brief       Get delay timeout turn off relay 1
 */
uint32_t slave_get_delay_turn_off_relay1_remain_time(void);

/**
 * @brief       Reset delay timeout turn off relay 2
 */
void slave_reset_delay_turn_off_relay2_when_stop_stream(void);

/**
 * @brief       Get delay timeout turn off relay 2
 */
uint32_t slave_get_delay_turn_off_relay2_remain_time(void);

/**
 * @brief       Reset delay timeout turn off relay 2
 */
void slave_reset_delay_turn_off_relay2_when_stop_stream(void);

/**
 * @brief       Auto switch audio in test mode
 */
void slave_set_audio_switch_audio_in_test_mode(void);

/**
 * @brief       Get timeout remaining time to turn off opto
 */
uint8_t slave_get_timeout_turn_off_opto_output(void);


/**
 * @brief       Set timeout remaining time to turn off opto
 */
void slave_set_timeout_turn_off_opto_output(uint8_t timeout);

/**
 * @brief       Enable/disable monitor data downloaded in stream mode
 */
void slave_allow_http_element_monitor_downloaded_data_in_streaming_state(bool state);

/**
 * @brief       Reset counter timeout need for http stop
 */
void slave_reset_counter_wait_for_http_element_stop(void);

/**
 * @brief       Get current http received data timeout
 */
uint8_t slave_get_http_received_data_timeout(void);

/**
 * @brief       Get 4G IP address
 */
char *slave_get_ppp_ip(void);

/**
 * @brief       Check if at least 1 master is streaming
 * @retval      TRUE At least 1 master is streaming
 *              No master is streaming
 */
bool slave_at_least_one_master_is_streaming(void);

/**
 * @brief       Terminate stream
 */
void slave_terminate_stream(void);

/**
 * @brief       Restart stream
 */
void slave_restart_stream(char *url);

/**
 * @brief       On http finish track callback
 */
void slave_on_hls_finish_track_cb();

void change_mode_cmd(char *mode);

void do_factory_reset(void);

#endif /* MAIN_H */
