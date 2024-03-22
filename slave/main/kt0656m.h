///****************************************************************************
//  File Name: KT_WirelessMicRxdrv.h
//  Function:  KT Wireless Mic Receiver Products Driver For Customer
///****************************************************************************
//        Revision History
//  Version Date        Description
//  V0.1    2016-07-13  For KT0656M
//  V0.2    2017-02-10  Standardized and organized; corrected the bit-shifting error in the KT_WirelessMicRx_Init function's auto_mute control.
//  V0.3    2017-02-17  Added KT_WirelessMicRx_CheckAUXCH function to monitor auxiliary channel status changes.
//  V0.4    2017-03-27  Added operations in the tune function to address temperature drift issues.
//  V1.3    2017-04-01  When using antenna diversity, display whether the reception is from the main or secondary antenna. Read main or secondary RSSI, SNR, pilot, and BPSK values. Updated version numbering according to the latest naming conventions.
//  V1.4    2017-04-01  Added initialization to modify values of registers 0x1f and 0x0d.
//  V1.5    2017-05-24  In the initialization function, changed comp_tc from 1 to 3, set vtr_momitor_en to 1 for enabling, and configured ref_vtr_vth_sel as 1.
//  V1.6    2017-06-05  Added KT_WirelessMicRx_Patch function to fix certain bugs. Modified some macro definitions and added crystal oscillator selection program similar to KT0616M.
//  V1.7    2017-06-28  Added AUTOMUTE_SNR_LOWTH and AUTOMUTE_SNR_HIGHTH macro definitions. Commented out PLL_UNLOCK_EN=1 in initialization, as enabling it could cause occasional freezing. Reset ifadc during crystal oscillator switching. Added battery voltage measurement function KT_WirelessMicRx_BatteryMeter_Read.
//  V1.8    2017-08-25  Modified values of AUTOMUTE_SNR_LOWTH and AUTOMUTE_SNR_HIGHTH from 0x58 and 0x60 to 0x78 and 0x80. Added macro definitions for search function. Maximum configuration for echo delay is set to 23 (197ms).
//	   
//-----------------------------------------------------------------------------

// Includes
//-----------------------------------------------------------------------------
#ifndef __KT_RX__
#define __KT_RX__
#include "main.h"
#include <stdbool.h>
#include "board_def.h"
#include "esp_err.h"

//-----------------------------------------------------------------------------
// 功能及参数定义
//-----------------------------------------------------------------------------

#ifdef BOARD_HW_HAS_UHF
#define KT0656M
//#define KT0655M

//#define XTAL_DUAL
#define XTAL_24M_ONLY
//#define XTAL_24P576M_ONLY

#ifdef	 XTAL_DUAL
	//#define OLD_SEL_XTAL_MODE 			  //KT0616M选择晶体的程序
	#define NEW_SEL_XTAL_MODE 				  //KT0656M选择晶体的程序
#endif

//#define AUTO_SEARCH_FREQ	 //自动搜台程序


#ifdef  XTAL_24P576M_ONLY
    #define XTAL_SEL1  1 //0:24MHz xtal 1:24.576MHz xtal
    #define XTAL_SEL2  0 //0:24MHz xtal 1:24.576MHz xtal
#else
    #define XTAL_SEL1  0 //0:24MHz xtal 1:24.576MHz xtal
    #define XTAL_SEL2  1 //0:24MHz xtal 1:24.576MHz xtal
#endif

#define XTAL_24M_FREQ     0
#define XTAL_24P576M_FREQ 1

#define HLSI_INV          0 //0: (HLSI); 1: inv(HLSI)

#define DCDC_EN           1 //DCDC使能位：0：关闭；1：使能。

#define AUTOMUTE_EN //undefine to disable automute
#define SQUEAL_EN //undefine to disable squeal
#define I2S_EN


#define MUTE              1
#define UNMUTE            0

#define FAST_RSSI_MUTE_EN       1     //1:enable 0:disable
#define FAST_RSSI_PD_MUTE_EN    1     //1:enable 0:disable
#define SUPER_RSSI_MUTE_EN      1     //1:enable 0:disable

#define AUTOMUTE_SNR_LOWTH	   0x64
#define AUTOMUTE_SNR_HIGHTH	   0x76


//I2C地址的bit4根据芯片ADDR决定，高为1，低为0. I2C地址的bit3根据主还是从决定：主为1，从为0
#define KTWirelessMicRxw_addressAM 0x72//0x7A
#define KTWirelessMicRxr_addressAM 0x73//0x7B

#define KTWirelessMicRxw_addressAS 0x7a//0x72
#define KTWirelessMicRxr_addressAS 0x7b//0x73

#define KTWirelessMicRxw_addressBM 0x62//0x6A
#define KTWirelessMicRxr_addressBM 0x63//0x6B

#define KTWirelessMicRxw_addressBS 0x6a
#define KTWirelessMicRxr_addressBS 0x6b

#define KTWirelessMicRxw_addressOTP 0x62
#define KTWirelessMicRxr_addressOTP 0x63


#define INIT_FAIL_TH        3

//频点范围及步进
#define    BAND_TOP_CHA            754850//727550//754850//662500
#define    BAND_BOTTOM_CHA         740150//710450//740150
#define    BAND_TOP_CHB            769850//743750//769850
#define    BAND_BOTTOM_CHB         755150//728150//755150
#define    BAND_STEP               300

//电池电压检测
#define    BATTERY_MAX             0x7FF
#define    BATTERY_HIGHTH          0x500
#define    BATTERY_MIDDLETH        0x4C0
#define    BATTERY_LOWTH           0x4A0

#ifdef KT0655M
	#define LINEIN_AGC_DIS	 0 			//0：使用自动调整增益的功能 1：不使用自动调整功能
	#define COMPEN_GAIN	   1  			//0：补偿后总增益为0dB 1：补偿后总体增益为6dB 2：补偿后总体增益为12dB 3：补偿后总体增益为18dB
	#define PGA_GAIN_SEL 3 				// 2'b00：-6dB 2'b01：0dB 2'b10：6dB	2'b11：12dB
	#define	SLNC_MUTE_TIME	0x13		//
	#define SLNC_MUTE_DIS  1			//0：使能Silence Mute功能 1：关闭Silence Mute功能。
	#define	SLNC_MUTE_LOW_LEVEL	0x04
	#define SLNC_MUTE_HIGH_LEVEL 0x06
	
	#define ALC_DIS	1					//0：开启 1：不开启。
	#define ALC_VMAX  0x7f				//
    #define ALC_SOFTKNEE  1				//
#endif

//REG0x0200
#define ADJUST_GAIN			  1		   //0:75KHz 1:50KHz 2:37.5KHz 3:25KHz 4:20KHz 5:120KHz 6:100KHz

//REG0x0241
#define BPSK_NEW_MODE         1//0//1        //1:new mode  0:old mode
#define AUXDATA_EN            1        //

#define CARRY_NUM_TH          4        // 3'b000:10 3'b001:12  3'b010:14 3'b011:16 3'b100:18 3'b101:20  3'b110:22 3'b111:24
#define AUX_CARRY_NUM         3        //3'b000 32 3'b001 28	3'b010 24  3'b011 22 3'b100 20 3'b101 18  3'b110 16	 3'b111 12

//REG0x026F
#define SQUEAL_ELIM_EN        1    //0: disable;    1: ensable		bit5
#define SQEUAL_DET_EN         3	   //0:关闭 1:保留 2:保留 3:开启	bit4:3
#define FNOTCH_NOTRESP_TH     1    //0: 1/4;    1: 1/2
#define N_OCTAVE              3//0    //0: 1/5;    1: 1/10;    2: 1/20;    3:1/80

//REG0x0270
#define FFT_R_TH            15    //8// 0: 0;        15: 30
#define FRAME_NUM_TH        0    //4    //

//REG0x0271
#define    PMAX_HITH          14  //8
#define    PMAX_LOWTH         12  //6
#define    AFC_RNG		  	  2//3	   //2'b00: +/-20kHz;2'b01: +/-40kHz;2'b10: +/-60kHz;2'b11: +/-90kHz;

//REG0x0272
#define    FDIFF_HITH         7//15
#define    FDIFF_LOWTH        1//7

#define ECHO_EN         1
#define ECHO_DIS        0

#define ECHO_STRU         1    //0: 全通; 1: 梳状
#define ECHO_GAIN_DOWN    0    //0: -13dB; 1: -10dB; 2: -7dB;
#define ECHO_GAIN_UP      7    //0: 0dB; 1: 1.9dB; 2: 3.5dB; 3: 5.5dB; 4: 7dB; 5: 9.4dB; 6: 10.9dB; 7: 13.1dB;
#define ECHO_RATIO        10    //0~25: 0~25/32 ECHO反馈比例
#define ECHO_DELAY        23    //0~23:    22ms~197ms ECHO信号延时

#define EXCITER_EN        1
#define EXCITER_DIS       0

#define EXCITER_TUNE    2    //0: 600Hz; 1: 1KHz; 2: 2KHz; 3: 3.8KHz; 激励起始频率
#define EXCITER_DRIVE   0    //0: 0dB; 1: 3.5dB; 2: 6dB; 3: 9dB; 4: 12dB; 5: 15dB; 激励剩余增益
#define EXCITER_ODD     1    //0~6:    负无穷~0dB 奇次激励衰减量
#define EXCITER_EVEN    1    //0~6:    负无穷~0dB 偶次激励衰减量

#define EQ_EN           1
#define EQ_DIS          0

#define EQ_25Hz         0
#define EQ_40Hz         1
#define EQ_63Hz         2
#define EQ_100Hz        3
#define EQ_160Hz        4
#define EQ_250Hz        5
#define EQ_400Hz        6
#define EQ_630Hz        7
#define EQ_1KHz         8
#define EQ_1K6Hz        9
#define EQ_2K5Hz        10
#define EQ_4KHz         11
#define EQ_6K3Hz        12
#define EQ_10KHz        13
#define EQ_16KHz        14

#define EQ_Neg12dB       0
#define EQ_Neg11dB       1
#define EQ_Neg10dB       2
#define EQ_Neg9dB        3
#define EQ_Neg8dB        4
#define EQ_Neg7dB        5
#define EQ_Neg6dB        6
#define EQ_Neg5dB        7
#define EQ_Neg4dB        8
#define EQ_Neg3dB        9
#define EQ_Neg2dB        10
#define EQ_Neg1dB        11
#define EQ_Pos0dB        12
#define EQ_Pos1dB        13
#define EQ_Pos2dB        14
#define EQ_Pos3dB        15
#define EQ_Pos4dB        16
#define EQ_Pos5dB        17
#define EQ_Pos6dB        18
#define EQ_Pos7dB        19
#define EQ_Pos8dB        20
#define EQ_Pos9dB        21
#define EQ_Pos10dB       22
#define EQ_Pos11dB       23
#define EQ_Pos12dB       24

#define PRESET_VOL       31 //0: mute; 31: maximum volume

#define  PKGSYNC         1
#define  NON_PKGSYNC     0


#define REG_ANA_CFG             0x0012f
#define REG_ANA_CFG38           0x002Df
#define REG_CHAN_CFG0           0x0045f
#define REG_CHAN_CFG1           0x0046f
#define REG_CHAN_CFG2           0x0047f
#define REG_SAI_SLAVE_CFG       0x0050f
#define REG_SAI_MASTER_CFG      0x0051f
#define REG_INTERFACE_EN        0x0052f
#define REG_INT_FLAG            0x0059f

#define REG_PLL_FSM_CTRL5       0x0061f
#define REG_POWER_STA_CTRL1     0x007Ff
#define REG_AUTOMUTE_FSM_STA    0x0088f
#define REG_ADC_CHAN0_HB        0x00C0f
#define REG_ADC_CHAN0_LB        0x00C1f


#define REG_SW_CFG0             0x0100f
#define REG_SW_CFG8             0x0108f
#define REG_SW_CFG14            0x010Ef


#define REG_MANUFACTURER_ID0    0x0192f
#define REG_MANUFACTURER_ID1    0x0193f

#define REG_TOP_CFG0            0x0200f
#define REG_TOP_CFG1            0x0201f
#define REG_TOP_STATUS0         0x0209f
#define REG_TOP_STATUS1         0x020Af
#define REG_TOP_STATUS3         0x020Cf
#define REG_TOP_STATUS4         0x020Df

#define REG_AFC_CFG0            0x0217f
#define REG_AFC_CFG1            0x0218f
#define REG_AFC_STATUS0         0x0219f
#define REG_AFC_STATUS1         0x021Af

#define REG_ANTDIV_CFG1         0x021Cf
#define REG_AUDIOLINK_CFG       0x0224f

#define REG_AUTOMUTE_CFG0       0x0225f

#define REG_BPSK_CFG0           0x0241f

#define REG_BURST_DATAH         0x0246f
#define REG_BURST_DATAL         0x0247f

#define REG_AUX_DATA_AH         0x0248f
#define REG_AUX_DATA_AL         0x0249f
#define REG_AUX_DATA_BH         0x024Af
#define REG_AUX_DATA_BL         0x024Bf
#define REG_AUX_DATA_CH         0x024Cf
#define REG_AUX_DATA_CL         0x024Df
#define REG_AUX_DATA_DH         0x024Ef
#define REG_AUX_DATA_DL         0x024Ff

#define REG_DE_EXP_CFG          0x0256f

#define REG_EQ_CFG0             0x0257f
#define REG_EQ_CFG1             0x0258f
#define REG_EQ_CFG2             0x0259f
#define REG_EQ_CFG3             0x025Af
#define REG_EQ_CFG4             0x025Bf
#define REG_EQ_CFG5             0x025Cf
#define REG_EQ_CFG6             0x025Df
#define REG_EQ_CFG7             0x025Ef
#define REG_EQ_CFG8             0x025Ff
#define REG_EQ_CFG9             0x0260f
#define REG_EQ_CFG10            0x0261f
#define REG_EQ_CFG11            0x0262f
#define REG_EQ_CFG12            0x0263f
#define REG_EQ_CFG13            0x0264f
#define REG_EQ_CFG14            0x0265f

#define REG_ECHO_CFG0           0x0266f
#define REG_ECHO_CFG1           0x0267f
#define REG_ECHO_CFG2           0x0268f

#define REG_EXCITER_CFG0        0x0269f
#define REG_EXCITER_CFG1        0x026Af
#define REG_SQUEAL_CFG0         0x026Ff

typedef struct
{
    int32_t Memery_Frequency;
    uint8_t echoFlag; //1:ON 0:OFF
    uint8_t equalierFlag; //1:ON 0:OFF
    uint8_t exciterFlag; //1:ON 0:OFF
    uint8_t diversityFlag; //1:ON 0:OFF
    uint8_t pilotFlag; //1:ON 0:OFF
    uint8_t echoDepth;
    uint8_t echoDelay;
    uint8_t equalier16Sel[15];
    uint8_t exciterOdd;
    uint8_t exciterEven;
}soundEffect,*pSoundEffect;

typedef uint8_t (*p_preinit)(void);
typedef bool (*p_init)(void);
typedef void (*p_tune)(long Freq);
typedef bool (*p_volume)(uint8_t cVolume);
typedef void (*p_sw_echo)(uint8_t cEcho_En);
typedef void (*p_set_echo)(uint8_t cEcho_Ratio, uint8_t cEcho_Delay);
typedef void (*p_sw_exciter)(uint8_t cExciter_En);
typedef void (*p_set_exciter)(uint8_t cExciter_Odd, uint8_t cExciter_Even);
typedef void (*p_sw_equalizer)(uint8_t cEqualizer_En);
typedef void (*p_set_equalizer)(uint8_t cEqualizer_Frq, uint8_t cEqualizer_Gain);
typedef void (*p_sw_diversity)(uint8_t diversity_En);
typedef uint8_t (*p_get_af)(void);
typedef uint8_t (*p_get_rssi)(void);
typedef uint8_t (*p_get_fast_rssi)(void);
typedef uint8_t (*p_automute)(void);
typedef void (*p_check_auxch)(void);
typedef uint8_t (*p_check_pilot)(void);
typedef uint8_t (*p_get_snr)(void);
typedef void (*p_select_ms)(void);
typedef void (*p_sai_init)(void);
typedef void (*p_fast_tune)(long Freq);
typedef void (*p_patch)(void);
typedef uint16_t (*p_battery_read)(void);
typedef void (*p_patch_chip)(uint8_t chipsel);






typedef struct
{
    soundEffect UHF_SoundEffect;
    uint8_t rssi;
    uint8_t snr;
    p_preinit   PreInit;
    p_init      Init;
    p_tune     Tune;
    p_volume    Volume;
    p_sw_echo   SwEcho;
    p_set_echo  SetEcho;
    p_sw_exciter    SwExciter;
    p_set_exciter   SetExciter;
    p_sw_equalizer  SwEqualizer;
    p_set_equalizer  SetEqualizer;
    p_sw_diversity  SwDiverity;
    p_get_af        GetAF;
    p_get_rssi      GetRssi;
    p_get_fast_rssi GetFastRssi;
    p_automute  Automute;
    p_check_auxch   CheckAuxCH;
    p_check_pilot   CheckPilot;
    p_get_snr   GetSnr;
    p_select_ms SelectMS;
    p_sai_init  SaiInit;
    p_fast_tune FastTune;
    p_patch Patch;
    p_battery_read  BatteryMeterRead;
    p_patch_chip    PatchChip;
    
}kt_uhf_transmit;

extern pSoundEffect pChangeSound;
extern soundEffect soundA;

#ifdef TWOCHANNEL
extern soundEffect soundB;
#endif

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void KT_Bus_Write(uint16_t Register_Address, uint8_t Byte_Data,uint8_t chipNum);
uint8_t KT_Bus_Read(uint16_t Register_Address,uint8_t chipNum);

void KT0656M_LowLevel_Init(void);
uint8_t KT_WirelessMicRx_PreInit(void);
bool KT_WirelessMicRx_Init(void);
void KT_WirelessMicRx_Tune(long Freq); //in KHz
//bool KT_WirelessMicRx_Volume(uint8_t cVolume);
uint8_t KT_WirelessMicRx_GetAF(void);
uint8_t KT_WirelessMicRx_GetRSSI(void);
uint8_t KT_WirelessMicRx_GetFastRSSI(void);
uint8_t KT_WirelessMicRx_GetSNR(void);
long KT_WirelessMicRx_GetFrequency(void); //in KHz
void KT_WirelessMicRx_SAIInit(void);
void KT_WirelessMicRx_FastTune(long Freq);
void KT_WirelessMicRx_PatchChip(uint8_t chip);
bool KT_WirelessMicRx_SetVolume(uint8_t cVolume);


void Pll_Band_Cali(uint8_t CLl, uint8_t CLh);
void PLL_Reset(void);
void rfIntCtl(void);

extern kt_uhf_transmit UHF_Transmit;

#endif // BOARD_HW_HAS_UHF == 1
#endif

