#ifndef __APP_UHF__
#define __APP_UHF__
#include "main.h"
#include <stdbool.h>
#include "board_def.h"

#ifdef BOARD_HW_HAS_UHF
#include "kt0656m.h"
//-----------------------------------------------------------------------------
// 功能及参数定义
//-----------------------------------------------------------------------------
#ifdef     TWOCHANNEL
    #define channelNum 2
#else 
    #define channelNum 1
#endif

#define channelA 0
#define channelB 1

#ifdef DIVERSITY
    #ifdef TWOCHANNEL
           #define chipNumb 4
        #define chipAM 0
        #define chipAS 1
        #define chipBM 2
        #define chipBS 3
       #else
         #define chipNumb 2
        #define chipAM 0
        #define chipAS 1

        //显示的时候根据display_ascii_string(1,2+8*(chipSel>=chipBM),"H");来决
        //定显示的位置，如果是一路的话，让chipBM为一个比较大的数，使得一直显示在左边
        #define chipBM 8
       #endif
#else
    #ifdef TWOCHANNEL
           #define chipNumb 2
        #define chipAM 0
        #define chipBM 1
       #else
         #define chipNumb 1
        #define chipAM 0

        //显示的时候根据display_ascii_string(1,2+8*(chipSel>=chipBM),"H");来决
        //定显示的位置，如果是一路的话，让chipBM为一个比较大的数，使得一直显示在左边
        #define chipBM 8 
       #endif
#endif

//设备地址定义
#define CS4398AddrA 0x98
#define    CS4398AddrB    0x9A

#define saveAddress   0x6A00 //把频点及音效等保存在flash里面的地址

//在此保存了数据0x88,每次上电的时候先读取此数据，看是否是0x88,如果是，再load 0x6A00处
//的数据，否则就不再load
#define saveAddressSpec   0x6A00+0x00A0        


//-----------------------------------------------------------------------------
//通道参数定义
//-----------------------------------------------------------------------------
#define CH_A    0
#define CH_B    1

//菜单状态定义
enum statusName{normal,selectSet,setEcho,setEqualizer,setExciter};

typedef enum
{
    UHF_State_Init, /*Init*/
    UHF_State_Unpair, /*Pair */
    UHF_State_Normal, /*Paired but not working*/
    UHF_State_EnterPair, /*Enter Pair mode -> looking for suitable RSSI and SNR*/
    UHF_State_Pair_Done, /*Get apropriate frequency value from the host*/
    UHF_State_Pair_Timeout, /*After certain seconds from Enter pair mode time -> device cannot get any fesible frequency -> indicate the timeout*/
    UHF_State_Streaming /*Receiving Data from master*/
}device_state_t;



typedef struct
{
	uint8_t seekState;
	uint16_t delay_time;
	uint8_t repeat_time;
	uint8_t confirm_cnt;
	uint8_t confirm2_cnt;
    uint16_t privateCode;     // Private Code
	uint8_t MatchSave;        // Private Code Save Flag
	uint8_t seek_mute;        // Set to 1 during seeking stations, set to 0 after seek is complete
	int8_t CurrentChanNum;
	uint8_t read_bpsk_cnt;
	uint8_t cSnr;
	uint8_t cRssi;
	uint8_t match_delay_cnt;
}StateMachine,*pStateMachine;

#define IS_VALID_TUNE	0x01
#define READ_SNR	0x02
#define SEARCH_NEXT_FREQ	0x03
#define READ_RSSI	0x04
#define READ_PILOT	0x05
#define READ_BPSK	0x06
#define WAIT_PLL_DONE	0x07
#define GET_RSSI_500K	0x08
#define GET_RSSI_250K_L	0x09
#define GET_RSSI_250K_R	0x0A
#define SCAN_OVER  0x0B
#define SEARCH_PREVIOUS_FREQ 0x0C
#define SEARCH_START 0x0D
#define SEEK_MATCH_FREQ	0x0E
#define SET_MATCH	0x0F
#define validTuneHold 0x10
#define WAIT_VALID_PLL_DONE 0x11

#define VENDOR_ID	0xA5

#define TIMER0_RELOAD_HIGH (65536-250)/256
#define	TIMER0_RELOAD_LOW	(65536-250)%256

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------


void KT_MicRX_Init (void); // KT_MicRX初始化
void KT_MicRX_Next_Fre (void); // KT_MicRX加100KHz
void KT_MicRX_Previous_Fre (void); // KT_MicRX减100KHz

void KT_MicRX_PowerDown_Detecter (void);
void KT_MicRX_Batter_Detecter (void);
void LCD_Refresh (void);
void DEMO_Init (void);
void Save_Freq (void);
void Load_Freq (void);
void KT_FreqSearch(void);
void KT_FreqSearchB(void);
uint8_t isValidTune(void);
void saveFreqInfo(void);
void displayFreq(void);
void voltageDisplay(void);
void mutePointChange(void);
void antChange(void);
uint8_t SNR_Display(void);
void BATTERY_Display(void);
void setMenu(void);
void keyWork(uint8_t keyName);
void freshMenu(void);
void MenuAdd(void);
void MenuDec(void);
void MenuSelect(void);

void freshSetEqualizer(void);
void freshSetExciter(void);
void loadChannelInfo(void);
void toNormal(void);

void KT_MicRX_Automute_Pilot(void);
void KT_MicRX_Previous_searchFre (bool stepNum);
void KT_MicRX_Next_searchFre (bool stepNum);
void CS4398Init(uint8_t address);
void dacInit(void);
void KT_MicRX_Automute_Pilot_SW(uint8_t pilotSw);
void xtalDisp(void);
void batCodeDisp(void);
void displayMenu(void);
void MSDisplay(void);

uint8_t CheckChannelNum(void);
void Seek_Freq_FastTune(long Freq);
void Seek_NextFreq_FastTune(bool stepNum,bool fast_flag);
void Timer0_Init(void);
uint8_t MuteSW(void);
void StateMachineRefresh(bool chan_num);
void SeekFreqStateMachine(bool chan_num);

/*TueTD: place for custom function*/
/******************************************************************************
 * @brief : apply initialise sequence in FAQ of datasheet - the demo code does the same, if it is't working please debug it
 * @author	:	TueTD
 * @created	:	20/08/2022
 * @version	:
 * @reviewer:
******************************************************************************/
void app_uhf_init(void);
/******************************************************************************
 * @brief : Doin normal task of UHF. Monitoring RSSI, SNR, and search for new frequency if in pair state
 * @author	:	TueTD
 * @created	:	20/08/2022
 * @version	:
 * @reviewer:
******************************************************************************/
void app_uhf_task(void);
/******************************************************************************
 * @brief : find frequency has the lowest RSSI
 * @author	:	TueTD
 * @created	:	20/08/2022
 * @version	:
 * @reviewer:
******************************************************************************/
void app_uhf_find_pair_frequency(void);
/******************************************************************************
 * @brief : Change the device state of UHD
 * @params[in] : next device state
 * @author	:	TueTD
 * @created	:	20/08/2022
 * @version	:
 * @reviewer:
******************************************************************************/
void uhf_pair_change_state(device_state_t next_state);
/******************************************************************************
 * @brief : Turn off howling suppression of UHD
 * @params[in] : next device state
 * @author	:	ThucND
 * @created	:	09/10/2023
 * @version	:
 * @reviewer:
******************************************************************************/
void turn_off_howling_suppression (void);
/******************************************************************************
 * @brief : Turn on howling suppression of UHD
 * @params[in] : next device state
 * @author	:	ThucND
 * @created	:	09/10/2023
 * @version	:
 * @reviewer:
******************************************************************************/
void turn_on_howling_suppression (void);
/******************************************************************************
 * @brief : Set Frequency for UHF
 * @params[in] : Freq
 * @author	:	Hungtd
 * @created	:	05/01/2024
 * @version	:
 * @reviewer:
******************************************************************************/
void app_uhf_tune(uint32_t Freq);

#endif //#if BOARD_HW_HAS_UHF == 1
#endif

