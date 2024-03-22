//*****************************************************************************
// File Name: main.c of UHF code example
// Function:  KT Wireless Mic Receiver Products Demoboard
//*****************************************************************************
// Revision History
// Version Date        Description
// V1.0    2012-08-28  Initial draft
// V1.7    2017-02-10  Removed KT_WirelessMicRx_CheckAUXCH function
//                  Modified KT_MicRX_Batter_Detecter function to read battery level information
//                  in correspondence with the transmitter (KT0646M) program location
// V1.4N   2017-04-01 Display corresponding RSSI, SNR, pilot, and BPSK values based on whether
//                  it's the main or secondary path (Main.c)
//                  Pulled down then up the chipen on power-up (Main.c)
//                  Increased I2C speed (I2C.c)
//                  Updated version number according to the latest naming convention
// V1.5    2017-04-21 Continuously read pilot and SNR values in the main loop and decide whether to mute based on the latest values (Main.c)
// V1.6    2017-06-05 Corrected register address for reading battery voltage from 0x029 to 0x0249
//                  Moved rfIntCtl(), pilotMuteRefresh(), and snrMuteRefresh() to the driver file (Main.c)
// V1.7    2017-06-28 Added BATTERY_Display function to display the receiver's battery voltage (Main.c)
// V1.8    2017-09-18 Added code controlled by macros for channel scanning functionality
//*****************************************************************************

/*TueTD:
I will modify and do the code as the minium requirement for this
Some part of code is not necessary but i will keep it unless it is usefull in the future
*/
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "kt0656m.h"
//#include <intrins.h>
#include <stdio.h>
//#include "I2C.h"
#include "main.h"
#include "math.h"
#include "app_uhf.h"
#include "app_debug.h"
//#include "app_ir_rx.h"
//#include "gpio.h"
//#include "sys_ctx.h"
//#include "internal_flash.h"
//#include "app_led.h"
//#include "iwdg.h"
#define debug

#ifdef debug

#endif

#define BOARD_HW_HAS_UHF

#ifdef BOARD_HW_HAS_UHF
#define Delay_ms(x)                     vTaskDelay(x / portTICK_PERIOD_MS);
//void displayChanFreq(uint8_t channel);

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
//uint8_t bChannel;


extern uint8_t MorSSelect;

enum statusName currentStatus = normal; //Lcd显示的功能状态
uint8_t chipSel; //芯片选择指示，表示当前选择的哪颗芯片
uint8_t currentChannel; //通道指示

uint8_t Flag_AUTOMUTE[chipNumb]; //MUTE状态标识
uint8_t Flag_PKGSYNC[chipNumb]; //包同步状态标识
uint8_t Flag_PILOT[chipNumb]; //导频状态标识

uint8_t Key_UP_flag = 0; //按键长按快速往上加标志状态
uint8_t Key_DOWN_flag = 0; //按键长按快速往下减标志状态

uint8_t setNum=0; // 设置菜单之显示选择

uint8_t echoStatus=0; //菜单显示之echo状态指示
uint8_t echoSel=0; //echo菜单显示选择

uint8_t equalierStatus=0; //菜单显示之equalier状态指示
uint8_t equalierSel=0; //equalier菜单显示选择

uint8_t exciterStatus=0; //菜单显示之exciter状态指示
uint8_t exciterSel=0; //exciter菜单显示选择

//通道A 频率及音效信息表
soundEffect soundA = 
{
    740150
    ,0,0,0,1,1,10,23,{12,12,12,12,12,12,12,12,12,12,12,12,12,12,12},1,1
};

#ifdef TWOCHANNEL
//通道B 频率及音效信息表
soundEffect data soundB = 
{
    755150,0,0,0,1,1,10,23,{12,12,12,12,12,12,12,12,12,12,12,12,12,12,12},1,1
};
#endif

//均衡器频段划分，中心频率常量表
float equalizerFreq[15] = 
{
    25.0,40.0,63.0,100.0,160.0,250.0,400.0,630.0,1.0,1.6,2.5,4.0,6.3,10.0,16.0
};

//通道信息表指针变量
pSoundEffect pChangeSound;

//临时存储缓冲区
uint8_t strTemp[20];
/*
static StateMachine MyPairStateMachine =
{
    READ_RSSI,0,3,0,0,0,0,0,0,0,0,0,	
};
*/
StateMachine StateMachineA = 
{
	READ_RSSI,0,3,0,0,0,0,0,0,0,0,0,	
};

#ifdef TWOCHANNEL
StateMachine idata StateMachineB = 
{
	READ_RSSI,0,3,0,0,0,0,0,0,0,0,0,	
};
#endif
StateMachine *pState;
//pStateMachine data pState;
uint16_t refreshTimer=0;
uint16_t timerCounter=0;
device_state_t m_device_state;


/*static uint32_t m_valid_frequency = 0;*/
//static void uhf_monitor_streaming(void);
//static void uhf_enter_pair_mode(void);


//void freqSearch(uint32_t startFreq, uint32_t stopFreq)
//{
//    /*Find the highest frequency and compare to the threshold value*/
//    DEBUG_INFO("Find pairing frequency\r\n");
//	uint32_t freqTemp, rssiMinFreq, rssiMaxFreq;
//	uint8_t rssiMin = 0xff, rssiMax = 0, rssi = 0;
//	for(freqTemp = startFreq; freqTemp < stopFreq; freqTemp += 250)
//	{   
//        HAL_IWDG_Refresh(&hiwdg);
//        sys_ctx()->UHF_Methods.FastTune(freqTemp);
//		rssi = sys_ctx()->UHF_Methods.GetRssi();
//        DEBUG_INFO("Frequency: %d, rssi:%d\r\n", freqTemp, rssi);
//		if(rssi < rssiMin)
//		{
//			rssiMin = rssi;
//			rssiMinFreq = freqTemp;
//		}
//		if(rssi > rssiMax)
//		{
//			rssiMax = rssi;
//			rssiMaxFreq = freqTemp;
//		}
//	}
//    /*Find and compare to the lowest*/
//    if(1) /*Place the guard condition here*/
//    {
//        
//        sys_ctx()->uhf_chip_status.Frequency = rssiMaxFreq;
//        DEBUG_INFO("Found pairing frequency: %d\r\n", sys_ctx()->uhf_chip_status.Frequency);
//        InternalFlash_WriteConfig();
//        sys_ctx()->UHF_Methods.Tune(rssiMaxFreq);
//        uhf_pair_change_state(UHF_State_Pair_Done);
//    }
//    else
//    {
//        uhf_pair_change_state(UHF_State_Pair_Timeout);
//    }
//    DEBUG_INFO("%s max_RSSI: %ld - min_RSSI: %ld\r\n", __FUNCTION__, rssiMaxFreq, rssiMinFreq);
//}
//void uhf_pair_change_state(device_state_t next_state)
//{
//    m_device_state = next_state;
//    sys_ctx()->uhf_chip_status.ChipState = next_state;
//}
//device_state_t uhf_pair_get_state(void)
//{
//    return NULL;
//   return  sys_ctx()->uhf_chip_status.ChipState;
//}
void app_uhf_init()
{
    uint32_t uhf_start_init_ms = sys_get_10ms(); 
    /*Turn on chip power*/
    //board_hw_enable_uhf_rx_power(true);
    //Delay_ms(300);
    /*TODO: Load previous config*/
    //KT0656M_LowLevel_Init();
    //delay_ms(100);
    // Pre Init
    uhf_start_init_ms = sys_get_10ms();
    while(KT_WirelessMicRx_PreInit() == 0)
    {
        if (sys_get_10ms() - uhf_start_init_ms >= 30)
        {
            DEBUG_ERROR(" Uhf Timeout\r\n");
            return;
        }
    }
    // Init
    uhf_start_init_ms = sys_get_10ms();
    while(KT_WirelessMicRx_Init() == 0)
    {
        if (sys_get_10ms() - uhf_start_init_ms >= 30)
        {
            DEBUG_ERROR(" Timeout\r\n");
            return;
        }
    }
            DEBUG_ERROR(" DONE\r\n");
    KT_WirelessMicRx_Tune(772500);    //772500 MHz
    KT_WirelessMicRx_SetVolume(100);
    
//    if(sys_ctx()->uhf_chip_status.Frequency != 0xFFFFFFFF)
//    {
//        DEBUG_INFO("Found frequency in the flash\r\n");
//        sys_ctx()->UHF_Methods.Tune(sys_ctx()->uhf_chip_status.Frequency);
//        DEBUG_INFO("Change to Frequency: %u\r\n", sys_ctx()->uhf_chip_status.Frequency);
//        uhf_pair_change_state(UHF_State_Streaming);
//        sys_ctx()->UHF_Methods.Volume(sys_ctx()->uhf_chip_status.Volume);
//        DEBUG_INFO("Set volume: %u\r\n", sys_ctx()->uhf_chip_status.Volume);
//    }
//    else
//    {
//        uhf_pair_change_state(UHF_State_Unpair);
//    }
//    sys_ctx()->UHF_Methods.Volume(sys_ctx()->uhf_chip_status.Volume);
    return ;
}

void turn_off_howling_suppression (void)
{
    uint8_t regx;
    regx=KT_Bus_Read(0x026f,chipSel);
    KT_Bus_Write( 0x026f, (regx & 0x00) & (~((SQUEAL_ELIM_EN << 5) | (SQEUAL_DET_EN << 3))),chipSel); //SQUEAL_ELIM_EN == 0
}

void turn_on_howling_suppression (void)
{
    uint8_t regx;
    regx=KT_Bus_Read(0x026f,chipSel);
    KT_Bus_Write( 0x026f, (regx & 0x00) & ((SQUEAL_ELIM_EN << 5) | (SQEUAL_DET_EN << 3)),chipSel); //SQUEAL_ELIM_EN == 0
}

void app_uhf_tune(uint32_t Freq)
{
    // 470MHz~960MHz
    if (Freq >= 470000 && Freq <= 960000) 
    {
        if (Freq != (uint32_t)KT_WirelessMicRx_GetFrequency())
        {
            KT_WirelessMicRx_SetVolume(0);
            Delay_ms(10);
            DEBUG_INFO("UHF Tunning to %u\r\n", Freq);
            KT_WirelessMicRx_Tune(Freq);
            KT_WirelessMicRx_SetVolume(100);
        }
    }
}

void app_uhf_task(void)
{
#ifdef BOARD_HW_HAS_UHF
    static uint16_t m_app_uhf_tick = 0;
    static uint32_t m_last_time_read_uhf = 0;
    static uint32_t m_uhf_wait_for_init = 0;
    static uint8_t uhf_state = UHF_State_Init;
    switch (uhf_state)
    {
        case UHF_State_Init:
            if (sys_get_10ms() - m_uhf_wait_for_init >= (uint32_t)1000)
            {
                m_uhf_wait_for_init = sys_get_10ms();
                app_uhf_init();
                uhf_state = UHF_State_Streaming;
            }
            else
            {
                DEBUG_WARN("tick %u\r\n", sys_get_10ms());
            }
            break;
        case UHF_State_Streaming:
            if (m_app_uhf_tick++ > 100)
            {
                // Patchchip regularly
                m_app_uhf_tick = 0;
                //KT_WirelessMicRx_PatchChip(0);
            }
            if (sys_get_10ms() - m_last_time_read_uhf >= (uint32_t) 300)
            {
                m_last_time_read_uhf = sys_get_10ms();

                // scan for ir data -> get frequency
//                uint32_t ir_data = get_ir_data();
//                if (ir_data)
//                {
//                    //DEBUG_WARN("IR DATA: %u\r\n", ir_data);
//                    app_uhf_tune(ir_data);
//                }

                //uint8_t snr = KT_WirelessMicRx_GetSNR();
                uint8_t rssi = KT_WirelessMicRx_GetRSSI();
                //uint32_t freq = KT_WirelessMicRx_GetFrequency();
                //DEBUG_WARN("rssi:%d, snr:%d, freq:%u\r\n", rssi, snr, freq);
                DEBUG_WARN("rssi:%d \r\n", rssi);
            }

        break;
        default:
        break;
    }
#endif     
   
}

void rfIntCtlChip(uint8_t chip)
{
	uint8_t regx;
	if(!(0x7f & KT_Bus_Read(0x005b, chip)))
	{
		regx = KT_Bus_Read(0x0053, chip);
		KT_Bus_Write(0x0053, regx &~ 0x40, chip);
	}
	else
	{
		regx=KT_Bus_Read(0x0053, chip);
		KT_Bus_Write(0x0053, regx | 0x40, chip);
	}
}

void pilotMuteRefreshChip(uint8_t chip)
{
	uint8_t regx;
	chipSel = chip;
	if(((0x80 & KT_Bus_Read(0x0100, chipSel)) == 0x80) && ((0x80 & KT_Bus_Read(0x0209, chipSel)) == 0x00))
    {
        regx=KT_Bus_Read(0x0087,chipSel);
        KT_Bus_Write(0x0087,(regx | 0x04),chipSel);
    }
	else
    {
		regx=KT_Bus_Read(0x0087,chipSel);
        KT_Bus_Write(0x0087,(regx &~0x04),chipSel);
    }
}

void snrMuteRefreshChip(uint8_t chip)
{
	uint8_t regx;
	chipSel = chip;
	if(((KT_Bus_Read(0x0100, chipSel) & 0x20) == 0x20) && (KT_Bus_Read(0x020D, chipSel) <= AUTOMUTE_SNR_LOWTH))
    {
        regx = KT_Bus_Read(0x0087,chipSel);
        KT_Bus_Write(0x0087, (regx | 0x02), chipSel);
    }
	else if((KT_Bus_Read(0x020D,chipSel) >= AUTOMUTE_SNR_HIGHTH) || ((KT_Bus_Read(0x0100,chipSel)&0x20) == 0x00))
    {
		regx=KT_Bus_Read(0x0087, chipSel);
        KT_Bus_Write(0x0087, (regx&~0x02), chipSel);
    }
	else
	{
	}
}

void KT_WirelessMicRx_PatchChip(uint8_t chip)
{
	rfIntCtlChip(chip);
	pilotMuteRefreshChip(chip);
	snrMuteRefreshChip(chip);
}

/**
 * @brief Find the frequency which has the lowest RSSI and SNR value
 * @param[in]: Index - Pin Number 
 * @param[in]: Event - Event Handle type 
 * @param[in]: p_args - Pointer to arguments need for handling event
 */
//void app_uhf_find_pair_frequency(void)
//{

//    uhf_enter_pair_mode();
//    /*This never can happen because we in loop when pairing -> I just want to make sure*/
//    if(uhf_pair_get_state() == UHF_State_EnterPair)
//    {
//        DEBUG_WARN("Already in pair mode\r\n");
//    }
//    else
//    {
//        return;
//    }
//    static uint32_t m_pair_timeout = 30000;
//   if(m_pair_timeout)
//   {
//       m_pair_timeout--;
//       if(m_pair_timeout == 0)
//       {
//           /**/
//           uhf_pair_change_state(UHF_State_Pair_Timeout);
//           pState->seekState = SEARCH_START;
//           pState->seek_mute = 0;
//           pState->confirm2_cnt = 0;
//           pState->confirm_cnt = 0;
//       }
//   }
//#if(!USE_SEARCH_FREQUENCY)
//    freqSearch(750000, 800000);
//#else
//    uint8_t bpsk_data_ah, bpsk_data_al, bpsk_data_b;
//    uint8_t chip_sel_save = chipSel;
//    /*Save the global variable*/
//    pSoundEffect pChaneSoundSave = pChangeSound;
//    uint8_t currentChannelSave = currentChannel;
//    
//    pChangeSound = &soundA; /*We will adjust and calib it if we have the pcb and test sound*/
//    currentChannel = channelA;
//    chipSel = chipAM;
//    pState = &MyPairStateMachine;
//    /*Tick for pair state machine*/
//   pState->delay_time++;

//      
//   switch(pState->seekState)
//   {
//       case SEARCH_START:
//           if(pState->seek_mute == 0)
//           {
//               m_pair_timeout = 30000; /*30 seconds for pair*/
//               pState->seek_mute = 1; /*Transfer state to seek*/
//               KT_WirelessMicRx_Volume(0);  /*Mute the device when seeking frequency*/
//           }
//           pState->seekState = SEARCH_NEXT_FREQ;
//           break;
//       case SEARCH_NEXT_FREQ:
//           Seek_NextFreq_FastTune(0, 1);
//           pState->delay_time = 0;
//           pState->seekState = WAIT_PLL_DONE;
//           break;
//		case WAIT_PLL_DONE:
//			if(KT_Bus_Read(0x0061,chipSel) & 0x01)
//			{
//				pState->seekState = READ_RSSI; 
//				pState->delay_time = 5;	 //让其直接读RSSI
//				pState->confirm_cnt = 0;
//			}
//			break;
//		case READ_RSSI:		
//			if(pState->delay_time > 5)
//			{			
//				pState->delay_time = 0;
//				pState->cRssi = KT_WirelessMicRx_GetRSSI();
//				if(pState->cRssi > 0x30)
//				{
//					rfIntCtlChip(0);        /*We have */
//					pState->seekState = READ_SNR;
//					pState->confirm_cnt=0;
//				}
//				else 
//				{
//					if(++pState->confirm_cnt >= (pState->repeat_time - 1))
//					{
//						pState->seekState = SEARCH_START;
//					}
//				}
//			}
//			break;
//		case READ_SNR:		
//			if(pState->delay_time > 10)
//			{			
//				pState->delay_time = 0;
//				pState->cSnr = KT_WirelessMicRx_GetSNR();
//				if(pState->cSnr > 0x58) /*i dont know why we have this value*/
//				{						
//					pState->seekState = READ_PILOT;
//					pState->confirm_cnt=0;
//				}
//				else 
//				{
//					if(++pState->confirm_cnt >= pState->repeat_time)
//					{
//						pState->seekState = SEARCH_START;
//					}
//				}
//			}
//			break;	
//		case READ_PILOT:
//			if(pState->delay_time >=40)
//			{
//				pState->delay_time = 0;
//				if((KT_Bus_Read(0x0209,chipSel) & 0x80) == 0x80)
//				{
//					pState->seekState = READ_BPSK;
//					pState->read_bpsk_cnt =0;
//					pState->confirm_cnt=0;
//				}									
//				else
//				{
//					if(++pState->confirm_cnt >= pState->repeat_time)
//					{					
//						pState->seekState = SEARCH_START;
//					}
//				}						
//			}
//			break;
//		case READ_BPSK:
//			if(pState->delay_time >= 100)
//			{				
//				pState->delay_time = 0;
//				if(KT_Bus_Read(0x0209, 0) & 0x40)
//				{					
//					bpsk_data_ah = KT_Bus_Read(0x0248, 0);
//					bpsk_data_al = KT_Bus_Read(0x0249, 0);
//					bpsk_data_b = (KT_Bus_Read(0x024A, 0) << 8) | KT_Bus_Read(0x024B, 0);
//					pState->CurrentChanNum = CheckChannelNum();
//                    sys_ctx()->uhf_chip_status.CurrentChannelNumber = CheckChannelNum();
//					if((bpsk_data_ah == VENDOR_ID) && (bpsk_data_al == pState->CurrentChanNum)/*&& ((bpsk_data_b == pState->privateCode)*/ /*|| (!pState->MatchSave)*/)
//					{
//						pState->seek_mute = 0;
//						KT_WirelessMicRx_Tune(pChangeSound->Memery_Frequency);
//						KT_WirelessMicRx_Volume(PRESET_VOL);
//		
//						sys_ctx()->uhf_chip_status.Frequency = pChangeSound->Memery_Frequency;
//                        InternalFlash_WriteConfig();
//                        uhf_pair_change_state(UHF_State_Pair_Done);
//                        m_pair_timeout = 0;
//						pState->read_bpsk_cnt = 0;
//						pState->seekState = SEARCH_START /*Get desire frequency*/;
//					}
//					else
//					{
//						if(++pState->read_bpsk_cnt >= 10)
//						{								
//							pState->seekState = SEARCH_START;
//                            
//						}
//					}							
//					break;	
//				}
//				else
//				{
//					if(++pState->read_bpsk_cnt >= 10)
//					{								
//						pState->seekState = SEARCH_START;
//					}
//				}	
//			}
//			break;
//            default:
//                break;
//   }
//#endif
//}
#endif //#if BOARD_HW_HAS_UHF == 1
