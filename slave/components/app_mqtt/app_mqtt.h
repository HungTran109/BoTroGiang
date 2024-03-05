#ifndef APP_MQTT_H
#define APP_MQTT_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>


#define APP_MQTT_KEEP_ALIVE_INTERVAL 120
#define NETWORK_ERROR_TIMEOUT_SEC 120 // 180
/*
 * Master sub topic config: vs/sub/<Master IMEI>
 * Master pub topic command: vs/pub/<Master IMEI>
 * Master pub topic info: vs/pub/<Master IMEI>/info
 */
#define APP_MQTT_MASTER_PUB_TOPIC_HEADER "vs/pub/"
#define APP_MQTT_MASTER_SUB_TOPIC_HEADER "vs/sub/"

/*
 * Slave sub topic config: vs/sub/<Slave IMEI>
 * Slave sub topic master: vs/pub/<Master IMEI>
 * Slave pub topic: vs/pub/<Slave IMEI>
 */
#define SLAVE_PUB_TOPIC_HEADER "vs/pub/"
#define SLAVE_SUB_TOPIC_CONF_HEADER "vs/sub/" /* sub config from user */
#define SLAVE_SUB_TOPIC_CMD_HEADER "vs/pub/"  /* sub command from master */

typedef enum
{
    APP_MQTT_DISCONNECTED = 0,
    APP_MQTT_CONNECTING,
    APP_MQTT_CONNECTED
} app_mqtt_state_t;



typedef struct
{
    char *topic;
    char *payload;
    bool need_free;
} mqtt_queue_stream_event_t;



typedef struct
{
    uint8_t firmware_version;
    uint8_t hardware_version;
    uint16_t vin;
    uint16_t v5v;
    uint16_t vgsm;
    uint8_t output_control;     // PA|RL1|RL2
    uint16_t input_state;
} mcu_info_t;

typedef struct
{
    // @SerializedName("C")
    uint8_t audio_class;		//loại class mạch PA
    // @SerializedName("D")
    uint8_t detect_value;	//Giá trị tín hiệu detect
    // @SerializedName("E")
    uint8_t err_state;		//Trạng thái lỗi: 1 - lỗi, 0 - bình thường
} speaker_info_t;

//fmInfo: Thông tin trạng thái mạch FM
typedef struct  
{
    // // @SerializedName("F")
    uint32_t freq;	//Tần số thu hiện tại (x10 = Freq thực tế)
    // // @SerializedName("S")
    int snr;		//SNR
    // // @SerializedName("R")
    int rssi;	//RSSI
    // // @SerializedName("D")
    int dBm;		//Rx dBm
} fm_info_t;

typedef union
{
    struct
    {
        uint32_t spk_open : 1;
        uint32_t spk_short : 1;
        uint32_t pwr_lost : 1;
        uint32_t sim_err : 1;
        uint32_t vol_err : 1;
        uint32_t reserve : 27;
    } name;
    uint32_t err;
} __attribute__((packed)) sys_error_code_t;

typedef struct 
{
   // // @SerializedName("Sn")
   char *serial;
   // // @SerializedName("Type")
   uint8_t role;                   //Loại bộ thu (0)/phát (1)
   // // @SerializedName("Ip")
   char ip[48];                  //IP của node
//    // // @SerializedName("Id")
//    public String nodeId;              //deviceId
//    // // @SerializedName("Name")
//    public String nodeName;            //deviceName
    //Trạng thái join room meeting
   // // @SerializedName("Stmt")
   char streaming_master[128];     //Imei của master đang stream, ko stream thì empty
   // // @SerializedName("Stlk")
   char streaming_link[156];       //Link đang streaming, nếu đang ko stream thì empty
   // // @SerializedName("Stst")
   uint8_t stream_state;            //Trạng thái streaming
   // // @SerializedName("Mic")
   bool mic_on;            //Trạng thái Mic On/Off
   // // @SerializedName("Spk")
   bool spk_on;            //Trạng thái Loa meeting On/Off
   // // @SerializedName("Vl2")
   uint8_t vol_music;               // m lượng loa phát nhạc (%)
//    // // @SerializedName("Ver")
    char app_version[14];          //Phiên bản app
//    // // @SerializedName("Hw")
//    char hardware_version[12];               //Phiên bản phần cứng

    // // @SerializedName("A")
    uint8_t temp_air;             //air temperature (oC, < 0 là không xác định)
    // // @SerializedName("N")
    char network[136];	     //Thông tin mạng đang sử dụng
    // // @SerializedName("P")
    uint32_t play_time_of_day;       //Tổng thời lượng phát trong ngày (giây)
    // // @SerializedName("O")
    uint8_t operating_mode;       //Chế độ hoạt động
    // // @SerializedName("L")
    char gps_lat[12];       //Thông tin location (nullable)
    char gps_long[12];
    // // @SerializedName("F")
    fm_info_t fm_info;       //Thông tin FM (nullable)
    // // @SerializedName("M")
    mcu_info_t mcu_info;       //Thông tin từ MCU
    // // @SerializedName("S")
    speaker_info_t speaker;       //Thông tin speaker
    // // @SerializedName("Mp")
    uint8_t mic_plug_state;       //Trạng thái cắm Micro: 1 - Cắm, 0 - Không cắm
    char device_name[128];
    sys_error_code_t sys_err;
} ping_message_t;

/**
 * @brief       Init audio msq to ensure thread safety
 */
void mqtt_queue_streaming_init(void);

/**
 * @brief       Put a msg to queue
 * @param       event Pointer to msg
 */
void mqtt_queue_streaming_put(mqtt_queue_stream_event_t *event);

/**
 * @brief       Get a msg from queue
 * @retval      Pointer to msg, null on empty
 */
mqtt_queue_stream_event_t *mqtt_queue_streaming_get(void);

/**
 * @brief       Send all config to svr
 */
void app_mqtt_send_all_config_after_reset(void);

/**
 * @brief           Get server connection status
 * @retval          TRUE device is connected to server
 *                  FALSE device is not connected to server
 */
bool app_mqtt_is_connected_to_server(void);

static const char NET_IF_TAB[4][5] = 
{
    "NA",
    "WIFI",
    "ETH",
    "4G",
};

/**
 * @brief           Send mqtt message
 * @param[in]       Message header
 * @param[in]       msg Data
 * @retval          Message id, -1 on error
 */
int mqtt_publish_message(char *header, const char *format, ...);

// /**
//  * @brief           Send mqtt debug
//  * @param[in]       format Data format
//  * @retval          Message id, -1 on error
//  */
// int mqtt_publish_dbg_message(const char *format, ...);

/**
 * @brief           Start mqtt task
 */
void app_mqtt_initialize(void);

/**
 * @brief           Subscribe topic
 * @param[in]       imei GSM imei
 */
void app_mqtt_send_subscribe_config_topic(char *imei);

/**
 * @brief           Subscribe command topic
 */
uint8_t app_mqtt_send_subscribe_command_topic(uint8_t master);

/**
 * @brief           Check if client is subscribed all topic
 * @retval          TRUE Client is subscribed
 *                  FALSE Client is not subscribed
 */
bool app_mqtt_is_subscribed(void);

/**
 * @brief           Start mqtt task
 * @retval          mqtt error code
 */
esp_err_t app_mqtt_start(void);

/**
 * @brief           Publish reset message
 */
void app_mqtt_send_reset_message(void);

/**
 * @brief           Get MQTT state
 * @retval          MQTT state
 */
app_mqtt_state_t app_mqtt_get_state(void);

/**
 * @brief           Set MQTT state
 * @param[in]       MQTT state
 */
void app_mqtt_set_state(app_mqtt_state_t state);

/**
 * @brief           Check if device is subscribed to a master
 */
bool app_mqtt_is_master_subscribed(uint8_t master_index);

/**
 * @brief           Publish slave info to server
 */
void app_mqtt_publish_slave_info(void);

/**
 * @brief           Check if we need re-init mqtt client
 */
bool app_mqtt_need_restart_mqtt(void);

/**
 * @brief           Clear restart mqtt flag
 */
void app_mqtt_need_clear_restart_mqtt_flag(void);

/**
 * @brief           Close mqtt client
 */
void app_mqtt_close(void);

void app_mqtt_set_streaming_master(char *master);
void app_mqtt_write_wifi_config(char *data);
int app_mqtt_send_pwr_lost(uint32_t vin_mv);
int app_mqtt_publish_ttn_debug_msg(const char *format, ...);

#endif /* APP_MQTT_H */
