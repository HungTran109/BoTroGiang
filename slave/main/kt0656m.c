#include "kt0656m.h"
#include <math.h>
#include "app_uhf.h"
#include "main.h"
#include "app_debug.h"
#include <string.h>
#include <stdlib.h>
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "i2c_bus.h"
#include "es8388.h"

#define USE_SOFT_I2C 0
#if(!USE_SOFT_I2C)

#define UHF_I2C I2C0
#endif

#define BOARD_HW_UHF_SCL_PIN  -1
#define BOARD_HW_UHF_SDA_PIN  -1

#ifdef BOARD_HW_HAS_UHF

#if(DEBUG_I2C_FM)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
 
/**
  * @brief  Retargets the C library printf function.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  SEGGER_RTT_Write(0, (uint8_t *)&ch, 1);
  return ch;
}
#define DEBUG_I2C(format_, ...)  DEBUG_RAW(format_, ##__VA_ARGS__)
#else
#define DEBUG_I2C(format_, ...) (void)(0)
#endif


// #define BIT0 0x01
// #define BIT1 0x02
// #define BIT2 0x04
// #define BIT3 0x08
// #define BIT4 0x10
// #define BIT5 0x20
// #define BIT6 0x40
// #define BIT7 0x80

#define KT0656M_LONG_TIMEOUT 3000
#define DIRECT_ACCESS 1
#define I2C_ADDR_DIRECT_ACCESS 0x12

#define UHF_TIMEOUT ((uint32_t) 1)
#define LL_GPIO_SetOutputPin(x)      gpio_set_level(x, 1)
#define LL_GPIO_ResetOutputPin(x)    gpio_set_level(x, 0)
#define LL_GPIO_IsInputPinSet           gpio_input_bit_get
#define Delay_ms(x)                     vTaskDelay(x / portTICK_PERIOD_MS);
#define UHF_SDA_SET_MODE_INPUT()        gpio_set_direction(BOARD_HW_UHF_SDA_PIN, GPIO_MODE_INPUT);
#define UHF_SDA_SET_MODE_OUTPUT()       gpio_set_direction(BOARD_HW_UHF_SDA_PIN, GPIO_MODE_OUTPUT);
                                        //gpio_output_options_set(BOARD_HW_UHF_SDA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BOARD_HW_UHF_SDA_PIN);

/*****************************************************************************
 * Typedef Implementation
 *****************************************************************************/

typedef union {
    struct
    {
        uint8_t lowByte;
        uint8_t highByte;
    } refined;
    uint16_t raw;
} word16_to_bytes;
/*****************************************************************************
 * Private variables
 *****************************************************************************/

static volatile uint32_t KT0656M_Timeout = 3000;
static long KT0656M_Frequency = 0;
/*****************************************************************************
 * Implementation
 *****************************************************************************/
//*****************************************************************************
//  File Name: I2C.c
//  Function:  KT Wireless Mic Transmitter Products Demoboard I2C Function Define
//*****************************************************************************

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
bool Ack_Flag=0; // I2C Ack Flag

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//函 数 名：I2C_Delay
//功能描述：I2C延时
//函数说明：
//全局变量：无
//输    入：无
//返    回：无
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
void I2C_Delay(void)
{
//    UINT8 i;
    
   for(int i=0;i<=100;i++)
    {
        //__nop();
    }
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Start
//功能描述：I2C数据帧开始
//函数说明：
//全局变量：无
//输    入：无
//返    回：无
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
void I2C_Start(void)
{
    // I2C_Delay();
    // I2C_Delay();
    // //SDA = 1;
    // LL_GPIO_SetOutputPin( BOARD_HW_UHF_SDA_PIN);
    // I2C_Delay();
    // I2C_Delay();
    // //SCL = 1;
    // LL_GPIO_SetOutputPin( BOARD_HW_UHF_SCL_PIN);
    // I2C_Delay();
    // I2C_Delay();
    // //SDA = 0;
    // LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SDA_PIN);
    // I2C_Delay();
    // I2C_Delay();
    // //SCL = 0;
    // LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
    // I2C_Delay();
    // I2C_Delay();
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Senddata
//功能描述：I2C发送数据
//函数说明：
//全局变量：无
//输    入：uchar senddata
//返    回：无
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
void I2C_Senddata(uint8_t senddata)
{
    // uint8_t i;
    
    // for (i=0;i<8;i++)
    // {    
    //     I2C_Delay();
    //     if ((senddata & 0x80) != 0x80)
    //         //SDA = 0;
    //         LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SDA_PIN);
    //     else 
    //         //SDA = 1;
    //         LL_GPIO_SetOutputPin( BOARD_HW_UHF_SDA_PIN);
    //     senddata = senddata << 1;
    //     I2C_Delay();
    //     //SCL = 1;
    //     LL_GPIO_SetOutputPin( BOARD_HW_UHF_SCL_PIN);
    //     I2C_Delay();
    //     //SCL = 0;
    //     LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
    // }
    // I2C_Delay();
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Receivedata
//功能描述：I2C接收数据
//函数说明：
//全局变量：无
//输    入：无
//返    回：uchar receivedata
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
uint8_t I2C_Receivedata(void)
{
    // uint8_t i,temp,receivedata=0;
    
    // for (i = 0; i < 8; i++)
    // {
    //     I2C_Delay();
    //     //SCL = 1;
    //     LL_GPIO_SetOutputPin( BOARD_HW_UHF_SCL_PIN);
    //     UHF_SDA_SET_MODE_INPUT();
    //     I2C_Delay();
        
    //     temp = LL_GPIO_IsInputPinSet( BOARD_HW_UHF_SDA_PIN);
    //     UHF_SDA_SET_MODE_OUTPUT();
    //     //SCL = 0;
    //     LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
    //     receivedata = receivedata | temp;
    //     if (i < 7)
    //     {
    //         receivedata = receivedata << 1;
    //     }
    // }
    // I2C_Delay();
    // return(receivedata);    
    return 0;
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Ack
//功能描述：I2C_Ack
//函数说明：
//全局变量：Ack_Flag
//输    入：无
//返    回：无
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
void I2C_Ack(void)
{
    // //SDA = 1;
    // LL_GPIO_SetOutputPin( BOARD_HW_UHF_SDA_PIN);
    // I2C_Delay();
    // I2C_Delay();
    // //SCL = 1;
    // LL_GPIO_SetOutputPin( BOARD_HW_UHF_SCL_PIN);
    // UHF_SDA_SET_MODE_INPUT();
    // I2C_Delay();
    // Ack_Flag = LL_GPIO_IsInputPinSet( BOARD_HW_UHF_SDA_PIN);
    // UHF_SDA_SET_MODE_OUTPUT();
    // //SCL = 0;
    // LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
    // I2C_Delay();
    // I2C_Delay();
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Stop
//功能描述：I2C数据帧结束
//函数说明：
//全局变量：无
//输    入：无
//返    回：无
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
void I2C_Stop(void)
{
//     //SCL = 0;
//     LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
//     I2C_Delay();
//     I2C_Delay();
//    // SDA = 0;
    
//     LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SDA_PIN);
//     I2C_Delay();
//     I2C_Delay();
//     //SCL = 1;
//     LL_GPIO_SetOutputPin( BOARD_HW_UHF_SCL_PIN);
//     I2C_Delay();I2C_Delay();
//     //SDA = 1;
//     LL_GPIO_SetOutputPin( BOARD_HW_UHF_SDA_PIN);
//     I2C_Delay();I2C_Delay();
}

/**
  * @brief  Initializes the I2C source clock and IOs used to drive the LM75
  * @param  None
  * @retval None
  */
void KT0656M_LowLevel_Init(void)
{
// #if (!USE_SOFT_I2C)
//     /* enable GPIOB clock */
//     rcu_periph_clock_enable(RCU_GPIOB);
//     /* enable KT0935_I2C clock */
//     rcu_periph_clock_enable(RCU_I2C0);
//     /* connect PB8 to I2C0_SCL */
//     gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_8);
//     /* connect PB7 to I2C0_SDA */
//     gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_7);
//     /* configure GPIO pins of KT0935_I2C */
//     gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8);
//     gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_8);
//     gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_7);
//     gpio_output_options_set(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_7);

//     /* I2C clock configure */
//     i2c_clock_config(UHF_I2C, 100000, I2C_DTCY_2);
//     /* I2C address configure */
//     i2c_mode_addr_config(UHF_I2C, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0x01);
//     /* enable KT0935_I2C */
//     i2c_enable(UHF_I2C);
//     /* enable acknowledge */
//     i2c_ack_config(UHF_I2C, I2C_ACK_ENABLE);
// #endif
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Byte_Write
//功能描述：I2C按Byte写操作
//函数说明：寄存器地址范围为16位
//全局变量：无
//输    入：uchar device_address,UINT16 reg_add,uchar writedata
//返    回：无
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
void I2CS_Byte_Write(uint8_t device_address, uint16_t reg_add, uint16_t writedata)
{
    uint8_t add_H,add_L;
    
    add_H = reg_add>>8;
    add_L = reg_add & 0x00FF;

    I2C_Start();
    I2C_Senddata(device_address & 0xFE);
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(add_H);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Senddata(add_L);
            I2C_Ack();
            if (Ack_Flag == 0)
            {
                I2C_Senddata(writedata);
                I2C_Ack();
            }
            else
                //SCL = 0;
                LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
        }
        else
           // SCL = 0;        
            LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
    }
    else
        //SCL = 0;
        LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
    I2C_Stop();    
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Byte_Read
//功能描述：I2C按Byte读操作
//函数说明：寄存器地址范围为16位
//全局变量：无
//输    入：uchar device_address,UINT16 reg_add
//返    回：正确：uchar readdata    错误：0x00
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
uint8_t I2CS_Byte_Read(uint8_t device_address, uint16_t reg_add)
{
    uint8_t readdata;
    uint8_t add_H,add_L;
    
    add_H = reg_add>>8;
    add_L = reg_add & 0x00FF;
    I2C_Start();
    I2C_Senddata(device_address & 0xFE);
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(add_H);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Senddata(add_L);
            I2C_Ack();
            if (Ack_Flag == 0)
            {
                I2C_Start();
                I2C_Senddata(device_address | 0x01);
                I2C_Ack();
                if (Ack_Flag == 0)
                {
                    // SDA pin is high Z
                    readdata = I2C_Receivedata();
                    I2C_Ack();
                }
                else
                {
                    //SCL = 0;
                    LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
                    return(0x00);
                }
            }
            else
            {
                //SCL = 0;
                LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
                 return(0x00);
            }
        }
        else
        {
            //SCL = 0;
            LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
             return(0x00);
        }            
    }
    else
    {
       // SCL = 0;
        LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
        return(0x00);
    }

    I2C_Stop();  
    return(readdata);
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Byte_Write
//功能描述：I2C按Byte写操作
//函数说明：
//全局变量：无
//输    入：uchar device_address,UINT8 reg_add,uchar writedata
//返    回：无
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
void I2S_Byte_Write(uint8_t device_address, uint8_t reg_add, uint8_t writedata)
{
    I2C_Start();
    I2C_Senddata(device_address & 0xFE);
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(reg_add);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Senddata(writedata);
            I2C_Ack();
        }
        else
            //SCL = 0;        
            LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);        
    }
    else
        //SCL = 0;
            LL_GPIO_ResetOutputPin( BOARD_HW_UHF_SCL_PIN);
    I2C_Stop();    
}

//-----------------------------------------------------------------------------
//函 数 名：I2C_Byte_Read
//功能描述：I2C按Byte读操作
//函数说明：
//全局变量：无
//输    入：uchar device_address,UINT8 reg_add
//返    回：正确：uchar readdata    错误：0x00
//设 计 者：PAE                     时间：2012-08-01
//修 改 者：                        时间：
//版    本：V1.0
//-----------------------------------------------------------------------------
uint8_t I2S_Byte_Read(uint8_t device_address, uint8_t reg_add)
{
    uint8_t readdata;
    I2C_Start();
    I2C_Senddata(device_address & 0xFE);
    I2C_Ack();
    if (Ack_Flag == 0)
    {
        I2C_Senddata(reg_add);
        I2C_Ack();
        if (Ack_Flag == 0)
        {
            I2C_Start();
            I2C_Senddata(device_address | 0x01);
            I2C_Ack();
            if (Ack_Flag == 0)
            {
                // SDA pin is high Z
                readdata = I2C_Receivedata();
                I2C_Ack();
                I2C_Stop();    
                return(readdata);
            }
            else
            {
                I2C_Stop();
                return(0x00);
            }
        }
        else
        {
            I2C_Stop();
             return(0x00);
        }            
    }
    else
    {
        I2C_Stop();
        return(0x00);
    }
}

 /*****************************************************************************
 * Global variables
 *****************************************************************************/

extern uint8_t chipSel;
extern uint8_t Flag_PKGSYNC[chipNumb]; //包同步状态标识
uint8_t MorSSelect = 1;//有天线分集时 1:主 0:从
  /*****************************************************************************
 * Function  ariables
 *****************************************************************************/
 void SwapValue(uint8_t *a, uint8_t *b) 
{
   uint8_t t = *a;
   *a = *b;
   *b = t;
}
#define DIRECT_ACCESS 1
esp_err_t I2C_Byte_Write(uint8_t device_addr, uint16_t reg_add, uint8_t data)
{
#if(!USE_SOFT_I2C)
    i2c_bus_handle_t i2c_handle = es8388_get_i2c_handle();
    return i2c_bus_write_bytes_uhf(i2c_handle, device_addr, reg_add, sizeof(reg_add), &data, sizeof(data));
#endif
    return 0;
}
esp_err_t I2C_Byte_Read(uint8_t device_addr, uint16_t reg_add, uint8_t *p_receive_buff)
{
#if(!USE_SOFT_I2C)
    i2c_bus_handle_t i2c_handle = es8388_get_i2c_handle();
    return i2c_bus_read_bytes_uhf(i2c_handle, device_addr, reg_add, sizeof(reg_add), p_receive_buff, 1);
#endif
}
void KT_Bus_Write(uint16_t Register_Address, uint8_t Byte_Data, uint8_t chipNum)
{
    switch(chipNum)
    {
#ifdef DIVERSITY
    #ifdef TWOCHANNEL
        case 0: 
            I2C_Byte_Write(KTWirelessMicRxw_addressAM, KTWirelessMicRxw_addressAM,Register_Address, Byte_Data);
            break;
        case 1: 
            I2C_Byte_Write(KTWirelessMicRxw_addressAS, Register_Address, Byte_Data);
            break;
        case 2: 
            I2C_Byte_Write(KTWirelessMicRxw_addressBM,Register_Address,Byte_Data);
            break;
        case 3: 
            I2C_Byte_Write(KTWirelessMicRxw_addressBS,Register_Address,Byte_Data);
            break;
    #else
        case 0: 
            I2C_Byte_Write(KTWirelessMicRxw_addressAM,Register_Address,Byte_Data);
            break;
        case 1: 
            I2C_Byte_Write(KTWirelessMicRxw_addressAS,Register_Address,Byte_Data);
            break;
    #endif
#else
    #ifdef TWOCHANNEL
        case 0: 
            I2C_Byte_Write(KTWirelessMicRxw_addressAM,Register_Address,Byte_Data);
            break;
        case 1: 
            I2C_Byte_Write(KTWirelessMicRxw_addressBM,Register_Address,Byte_Data);
            break;
       #else
        case 0: 
#if(!USE_SOFT_I2C)
            I2C_Byte_Write(KTWirelessMicRxw_addressBM, Register_Address, Byte_Data);
#else
            I2CS_Byte_Write(KTWirelessMicRxw_addressBM, Register_Address, Byte_Data);
#endif
            break;
       #endif
#endif
    }
}


uint8_t KT_Bus_Read(uint16_t Register_Address, uint8_t chipNum)
{
    uint8_t read_data = 0;
    switch(chipNum)
    {
#ifdef DIVERSITY
    #ifdef TWOCHANNEL
        case 0: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressAM, Register_Address, &read_data) ;
            return read_data;
        }
            break;
        case 1: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressAS, Register_Address, &read_data);
            return read_data;
        }
            break;
        case 2: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressBM, Register_Address, &read_data);
            return read_data;
        }
            break;
        case 3: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressBS, Register_Address, &read_data);
            return read_data;
        }
            break;
    #else
        case 0: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressAM, Register_Address, &read_data);
            return read_data;
        }
            break;
           
        case 1: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressAS, Register_Address, &read_data);
            return read_data;
        }
            break;
       #endif
#else
    #ifdef TWOCHANNEL
        case 0: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressAM, Register_Address, &read_data);
            return read_data;
            break;
        }
        case 1: 
        {
            I2C_Byte_Read(KTWirelessMicRxr_addressBM, Register_Address, &read_data);
            return read_data;
        }
            break;
       #else
        case 0: 
        {
#if(!USE_SOFT_I2C)
            I2C_Byte_Read(KTWirelessMicRxw_addressBM, Register_Address, &read_data);
#else
            read_data = I2CS_Byte_Read(KTWirelessMicRxw_addressBM, Register_Address);
#endif
            return read_data;
        }
            //break;
       #endif
#endif
		default:
//            while(1)
//            {
                DEBUG_WARN("Invalid chipNum in %s %s\r\n", __FILE__, __FUNCTION__);
                return ESP_FAIL;
//            }
            //break;
    }
}






//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_PreInit
// Function Description: Chip initialization program
// Function Note: Check if the chip is powered up correctly and if the I2C bus is working fine
// Global Variables: INIT_FAIL_TH
// Input: None
// Return: Correct: 1, ESP_FAIL: 0
// Designer: Zhou Dongfeng           Date: 2016-07-12
// Modifier:                        Date:
// Version: V0.1    For KT0656M
// Translate: TueTD
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicRx_PreInit(void)              
{
    uint8_t regx;
    uint8_t i;
    for (i = 0; i < INIT_FAIL_TH; i++)
    {
        regx = KT_Bus_Read(0x0192, chipSel);        //Read Manufactory ID
        DEBUG_INFO("%d\r\n", regx);
        if (regx == 0x4B)
        {
            regx = KT_Bus_Read(0x0193, chipSel);
            if(regx == 0x54)
            {
                #ifdef KT0656M
                KT_WirelessMicRx_SetVolume(0);
                regx = KT_Bus_Read(0x002d, chipSel); //dcdc
                KT_Bus_Write(0x002d, (regx & ~0x04) | (DCDC_EN << 2), chipSel); // Turn on the DC DC bit -
                #endif
                return(1);
            }

        }
    }
    return(0);
}

//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_Init
// Function Description: Chip initialization program
// Function Note:
// Global Variables: None
// Input: None
// Return: Correct: 1, ESP_FAIL: 0
// Designer: Zhou Dongfeng           Date: 2016-07-13
// Modifier: WSL                     Date: 2017-02-10
// Version: V0.1    For KT0656M
// Version: V0.2    Corrected shifting ESP_FAIL in auto_mute control
// Version: V1.4    Added modification of values in registers 0x1f and 0x0d during initialization
// Version: V1.5    In the initialization function, changed comp_tc from 1 to 3, enabled vtr_momitor_en, and set ref_vtr_vth_sel to 1
//-----------------------------------------------------------------------------
bool KT_WirelessMicRx_Init(void)
{
    uint32_t uhf_start_ms = sys_get_10ms();
    uint8_t regx;
    while(!(KT_Bus_Read(0x007F, chipSel) & 0x01))
    {
        if (sys_get_10ms() - uhf_start_ms >= 3*UHF_TIMEOUT)
        {
            DEBUG_ERROR("KT_WirelessMicRx_Init timeout\r\n");
            return false;
        }
    }
    ; //power_on finish
    
	regx = KT_Bus_Read(0x010f, chipSel);//FIRMWARE_VERSION
	if(regx!=0x11)
		return 0;

	regx = KT_Bus_Read(REG_SW_CFG8, chipSel);          
    KT_Bus_Write(REG_SW_CFG8, (regx & 0x1f) | (XTAL_SEL1<<7) | (XTAL_SEL2<<6) | (HLSI_INV<<5), chipSel); //hlsi

	regx = KT_Bus_Read(0x001f, chipSel); 
	KT_Bus_Write(0x001f, (regx&0x1f) | 0x40, chipSel); //0.4V

	KT_Bus_Write(0x000d, 0xcf, chipSel); //6mA

	//可通过0x16<5:0>的变化体现，注意:由于由软件完成该过程，改变此值后要重新tune台
	KT_Bus_Write(0x0109, 0x10, chipSel); //pll noise增加3db

    /*Consider add auto link*/
   // KT_Bus_Write(0x0224, 0x20, chipSel); //pll noise增加3db
#ifdef AUTOMUTE_EN

    KT_Bus_Write(0x0082,0x89,chipSel); //fast_rssi_mute_th=0x89
	//auto_mute controlAUTOMUTE_EN
    regx = KT_Bus_Read(0x0081,chipSel);
    KT_Bus_Write(0x0081,(regx & 0x8f) | (FAST_RSSI_MUTE_EN<<6) |
                 (FAST_RSSI_PD_MUTE_EN<<5) | (SUPER_RSSI_MUTE_EN<<4), chipSel); 

    KT_Bus_Write(0x0079,AUTOMUTE_SNR_LOWTH,chipSel); 
    KT_Bus_Write(0x007a,AUTOMUTE_SNR_HIGHTH,chipSel); 

	regx = KT_Bus_Read(0x0201,chipSel);
    KT_Bus_Write(0x0201,regx|0x80,chipSel); //NBW=1

    regx = KT_Bus_Read(0x0100,chipSel);
    KT_Bus_Write(0x0100,(regx & 0xDF) | (1<<5),chipSel); //automute_snr_en=1

    
    regx = KT_Bus_Read(0x0100,chipSel);
#ifdef TWOCHANNEL
    if(chipSel<chipBM)	//A 通道的主或者从
#endif
    {
        if((soundA.pilotFlag & 0x01)==1)
        {
            KT_Bus_Write(0x0100, regx | 0x80, chipSel);    //开导频检测          
        }
        else
        {
            KT_Bus_Write(0x0100, regx&0x7f, chipSel);   //关导频检测           
        }
    }
#ifdef TWOCHANNEL
    else
    {
        if((soundB.pilotFlag&0x01)==1)
        {
            KT_Bus_Write(0x0100,regx|0x80,chipSel);              
        }
        else
        {
            KT_Bus_Write(0x0100,regx&0x7f,chipSel);              
        }
    }
#endif

//    regx=KT_Bus_Read(0x0053,chipSel);
//    KT_Bus_Write(0x0053,regx|0x10,chipSel); //BPSK_PKG_SYN_INT_EN=1
#endif

#ifdef SQUEAL_EN
    regx=KT_Bus_Read(0x026f,chipSel);
    KT_Bus_Write( 0x026f, (regx & 0x00) | (SQUEAL_ELIM_EN << 5) |(SQEUAL_DET_EN << 3) | (FNOTCH_NOTRESP_TH << 2) | (N_OCTAVE << 0),chipSel);
    regx=KT_Bus_Read(0x0270,chipSel);
    KT_Bus_Write( 0x0270, (regx & 0x00) | (FFT_R_TH << 4) | (FRAME_NUM_TH << 0),chipSel);
    regx=KT_Bus_Read(0x0271,chipSel);
    KT_Bus_Write( 0x0271, (regx & 0x00) | (PMAX_HITH << 4) | (PMAX_LOWTH << 0),chipSel);
    regx=KT_Bus_Read(0x0272,chipSel);
    KT_Bus_Write( 0x0272, (regx & 0x00) | (FDIFF_HITH << 4) | (FDIFF_LOWTH << 0),chipSel);
#endif

    //config for bpsk
    regx = KT_Bus_Read(REG_BPSK_CFG0, chipSel);          
    KT_Bus_Write(REG_BPSK_CFG0, (regx & 0x3f) | (AUXDATA_EN<<7) | (BPSK_NEW_MODE<<6), chipSel); //bpsk enable bpsk new mode
    regx = KT_Bus_Read(0x243, chipSel);          
    KT_Bus_Write(0x243, (regx & 0x88) | (AUX_CARRY_NUM<<4) | CARRY_NUM_TH, chipSel); //载波个数为22 载波判决门限18 
    regx = KT_Bus_Read(0x245, chipSel);          
    KT_Bus_Write(0x245, 0x22, chipSel); //解析过程中连续正确和错误（可以不连续）的数据包数都为2

    regx = KT_Bus_Read(REG_TOP_CFG0, chipSel);    /*Channel gain bandwidth*/         
    KT_Bus_Write(REG_TOP_CFG0, (regx & 0x8f) | (ADJUST_GAIN << 4), chipSel); //adjust gain =50kHz

    regx = KT_Bus_Read(REG_AUTOMUTE_CFG0, chipSel); //DC_NOTCH_MUTE_EN=1
    KT_Bus_Write(REG_AUTOMUTE_CFG0, regx & 0xf1, chipSel);
//	KT_Bus_Write(0x0225,regx&0xf0,chipSel);
                                                
	regx = KT_Bus_Read(0x010e, chipSel);		  	 //AFC CTRL FSM使能控制位disable
	KT_Bus_Write(0x010e, regx&0xfe, chipSel);

	regx=KT_Bus_Read(0x0087, chipSel);			//SOFT_AFC_MUTE写为0
	KT_Bus_Write(0x0087,regx&~0x08,chipSel);

    regx = KT_Bus_Read(0x0217,chipSel); //
    KT_Bus_Write(0x0217,(regx&0x3f)|(AFC_RNG<<6)|0x01,chipSel);	// +/-60kHz;

    KT_Bus_Write(0x0218,0x02,chipSel); //afc_en=1  AFC_FROZEN=0
//	KT_Bus_Write(0x0218,0x00,chipSel); //afc_en=0   

    regx = KT_Bus_Read(0x0256,chipSel); //comp_tc=1
    KT_Bus_Write(0x0256,(regx&0x8f)|0x10,chipSel); 

	regx = KT_Bus_Read(0x002d,chipSel); //vtr_momitor_en=1
    KT_Bus_Write(0x002d,regx|0x08,chipSel); 

	regx = KT_Bus_Read(0x0010,chipSel); //ref_vtr_vth_sel=1
    KT_Bus_Write(0x0010,regx|0x80,chipSel);


	#ifdef DIVERSITY
		#ifdef TWOCHANNEL
		if((chipSel==chipAS)||(chipSel==chipBS))
		#else
		if(chipSel==chipAS)
		#endif
		{
			regx=KT_Bus_Read(0x0202,chipSel);
			KT_Bus_Write(0x0202,regx|0x04,chipSel);	  //SIGINV_  HLSI=1
			regx=KT_Bus_Read(0x0255,chipSel);
			KT_Bus_Write(0x0255,regx|0x10,chipSel);	  //AUDIO_INV=1
		}
	#endif

	regx=KT_Bus_Read(0x0030, chipSel);
	KT_Bus_Write(0x0030,regx & ~BIT7, chipSel);//LO_LOCK_DET_PD=0

//	regx = KT_Bus_Read(0x0133,chipSel);          
//    KT_Bus_Write(0x0133,(regx|0x20),chipSel); //PLL_UNLOCK_EN=1

    regx = KT_Bus_Read(0x0133, chipSel);          
    KT_Bus_Write(0x0133, (regx|0x40), chipSel); //dll_rst_en

//	regx = KT_Bus_Read(0x010e,chipSel);          
//    KT_Bus_Write(0x010e,(regx|0x06),chipSel); //打开watch dog

	regx = KT_Bus_Read(0x010e, chipSel); //天线分集软件状态机需要关闭（用硬件的）
    KT_Bus_Write(0x010e, regx&0xf7, chipSel);
	
	//AUDIO_SEL 输出合并信号，需要在天线分集软件状态机需要关闭后在写，因为软件开时会自动改写
	regx = KT_Bus_Read(0x021c, chipSel); 
    KT_Bus_Write(0x021c, regx&0xcf, chipSel);

	regx = KT_Bus_Read(0x0108, chipSel);          
    KT_Bus_Write(0x0108, (regx|0x10), chipSel); //LOBAND_CALI_SCAN_EN=1 for fasttune
    uhf_start_ms = sys_get_10ms();
	while(1)
	{
		Delay_ms(10);
		if(0==(0x10 & KT_Bus_Read(0x0108, chipSel)))
            break;
        if (sys_get_10ms() - uhf_start_ms >= 200*UHF_TIMEOUT)
        {
            DEBUG_ERROR("KT_WirelessMicRx_Init timeout\r\n");
            return(0);
        }
	}
	#ifdef KT0655M
	regx=KT_Bus_Read(0x0331,chipSel);
	KT_Bus_Write(0x0331,(regx & 0xfe)|LINEIN_AGC_DIS,chipSel);//LINEIN_AGC_DIS

	regx=KT_Bus_Read(0x0333,chipSel);
	KT_Bus_Write(0x0333,(regx & ~0x30)|(COMPEN_GAIN<<4),chipSel);//COMPEN_GAIN

	regx=KT_Bus_Read(0x0334,chipSel);
	KT_Bus_Write(0x0334,(regx & ~0x18)|(PGA_GAIN_SEL<<3),chipSel);//PGA_GAIN_SEL

	regx=KT_Bus_Read(0x0336,chipSel);
	KT_Bus_Write(0x0336,(regx & 0xc0)|(SLNC_MUTE_TIME<<1)|SLNC_MUTE_DIS,chipSel);//SLNC_MUTE_DIS  SLNC_MUTE_TIME

	KT_Bus_Write(0x0337,(SLNC_MUTE_LOW_LEVEL<<4)|SLNC_MUTE_HIGH_LEVEL,chipSel);//SLNC_MUTE_LEVEL

	KT_Bus_Write(0x0339,(ALC_DIS<<7)|ALC_VMAX,chipSel);	 //	ALC_DIS ALC_VMAX

	regx=KT_Bus_Read(0x033a,chipSel);
	KT_Bus_Write(0x033a,(regx & ~0x08)|(ALC_SOFTKNEE<<3),chipSel);//ALC_SOFTKNEE
	#endif
    
    return(1);
}
//-----------------------------------------------------------------------------
// TueTD modify divide input by 3
//-----------------------------------------------------------------------------
static bool KT_WirelessMicRx_Volume(uint8_t cVolume);
bool KT_WirelessMicRx_SetVolume(uint8_t cVolume)
{
    cVolume = cVolume / 3;
    if(cVolume >= 31)
    {
        cVolume = 31;
    }
    KT_WirelessMicRx_Volume(cVolume);
    return (1);
}
//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_Volume
// Function Description: Adjusts the output volume
// Function Notes: The range of cVolume is 0-31, with a total of 32 levels, where 0 is mute
// Global Variables:
// Input: cVolume
// Return: Correct: 1                ESP_FAIL: 0
// Designer: Zhou Dongfeng           Date: 2016-07-12
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------
static bool KT_WirelessMicRx_Volume(uint8_t cVolume)
{
    uint8_t regx;

    regx = KT_Bus_Read(0x0201,chipSel);
    KT_Bus_Write(0x0201, (regx & 0xE0) | (cVolume),chipSel);

    return(1);
}

long KT_WirelessMicRx_GetFrequency(void)
{
    return KT0656M_Frequency;
}

//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_Tune
// Function Description: Receiver frequency setting function
// Function Note: Input the receiving frequency in KHz units
// Global Variables: None
// Input: Freq (receiving frequency in KHz units)
// Return: None
// Designer: Zhou Dongfeng           Date: 2016-07-12
// Modifier:                        Date:
// Version: V0.1    For KT0656M
// Version: V1.6    Added a program to select a crystal based on KT0616M
// Version: V1.7    Turn off PLL unlock interrupt before switching crystal, then turn it back on after tuning
//-----------------------------------------------------------------------------
void KT_WirelessMicRx_Tune(long Freq)
{
    uint32_t uhf_start_ms = sys_get_10ms();
    KT0656M_Frequency = Freq;
    
    uint8_t Freq_H,Freq_M,Freq_L,regx;
	uint8_t LO_LOCK_DET_PD_SAVE;
	uint8_t state;
    
//	regx = KT_Bus_Read(0x0133,chipSel);
//	KT_Bus_Write(0x0133,(regx&~0x60),chipSel); //dll_rst_en=0  I_PLL_UNLOCK_EN=0  
//	regx=KT_Bus_Read(0x0054,chipSel);
//	KT_Bus_Write(0x0054,regx&~0x18,chipSel);//PLL失锁中断 dis

#ifdef XTAL_DUAL
	#ifdef NEW_SEL_XTAL_MODE
    	caclXtal(Freq);//select xtal
	#else
		oldCaclXtal(Freq);
	#endif
#endif
    
	regx = KT_Bus_Read(0x0108, chipSel);
    KT_Bus_Write(0x0108, regx & ~BIT3, chipSel); //SCAN_MODE=0;

	Freq = Freq & 0x000FFFFF;

    Freq_H = ( Freq >> 12 );
    Freq_M = ( (Freq & 0x00000FFF) >> 4 );
    Freq_L = ( (Freq & 0x0000000F) << 4 );


    /*DEBUG*/
    KT_Bus_Write(0x0045, Freq_H, chipSel);                
    KT_Bus_Write(0x0046, Freq_M, chipSel);
    
    //KT_Bus_Write(0x0045, 0xA2, chipSel); 
    //KT_Bus_Write(0x0046, 0xE8, chipSel);

    regx = KT_Bus_Read(0x0047, chipSel);
    KT_Bus_Write(0x0047, (regx & 0x0F) | Freq_L , chipSel);
    // DEBUG
    //KT_Bus_Write(0x0047, 0x40, chipSel);

	regx=KT_Bus_Read(0x0053,chipSel);
	KT_Bus_Write(0x0053,regx&~0x40,chipSel);  //rfamp_int_en=0
    //KT_Bus_Write(0x0053, 0x81, chipSel);
    regx=KT_Bus_Read(0x0047, chipSel);
    //KT_Bus_Write(0x0047, 0x40);
    KT_Bus_Write(0x0047,regx | BIT0, chipSel); //chan_valid=1;
    //KT_Bus_Write(0x0047, 0x41, chipSel);
	//因为chan_valid=1以后，触发tune中断PLL done先写为0，完成后写1，如果不delay 1ms,可能PLL done读出为1的时候
	//内部cpu程序还没有把PLL done写成0，所以外部mcu和内部cpu可能会同时操作一些内部DSP等，导致死机，所以需要delay
	Delay_ms(10); 
    
    uhf_start_ms = sys_get_10ms();
    while (!(KT_Bus_Read(0x0061, chipSel) & 0x01))
    {
        if (sys_get_10ms() - uhf_start_ms >= 3*UHF_TIMEOUT)
        {
            //DEBUG_ERROR("KT_WirelessMicRx_Tune timeout\r\n");
            return;
        }
    }        //PLL done
	regx = KT_Bus_Read(0x0042, chipSel);
	KT_Bus_Write(0x0042,regx | BIT2, chipSel);//S_DSP_RST

	regx = KT_Bus_Read(0x0042, chipSel);
	KT_Bus_Write(0x0042, regx | BIT4, chipSel);//S_PLL_SDM_RST			

	regx = KT_Bus_Read(0x0030,chipSel);
	LO_LOCK_DET_PD_SAVE = regx&0x80;
	KT_Bus_Write(0x0030,regx | BIT7,chipSel);//LO_LOCK_DET_PD=1
   
	regx = KT_Bus_Read(0x0017, chipSel); //double+16MHz/V locoarse_var_sel
    state = regx & 0x07;
    if(state >= 3)
    {
        state = 7;                                
    }
    else
    {
        state = (state<<1) + 3;
    }
    regx=(regx & 0xf8)|state;                                          
    KT_Bus_Write(0x0017, regx,chipSel); //write locoarse/lofine_var_sel

	Pll_Band_Cali(0, 255);

	PLL_Reset();

	regx = KT_Bus_Read(0x0042,chipSel);
	KT_Bus_Write(0x0042,regx & ~BIT4,chipSel);//C_PLL_SDM_RST

	regx=KT_Bus_Read(0x0042,chipSel);
	KT_Bus_Write(0x0042,regx & ~BIT2,chipSel);//C_DSP_RST

	Delay_ms(10);

	regx=KT_Bus_Read(0x0030,chipSel);
	KT_Bus_Write(0x0030,(regx & ~BIT7)|LO_LOCK_DET_PD_SAVE,chipSel);//LO_LOCK_DET_PD recovery

    KT_WirelessMicRx_SAIInit();

	if(KT_WirelessMicRx_GetSNR() > AUTOMUTE_SNR_LOWTH)
	{
		regx = KT_Bus_Read(0x0087,chipSel);			//SOFT_SNR_MUTE写为0
		KT_Bus_Write(0x0087,regx&~0x02,chipSel);
	}
}

//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_FastTune
// Function Description: Receiver frequency setting function
// Function Notes: Input the receiving frequency in KHz units
// Global Variables: None
// Input: Freq (Receiving frequency in KHz units)
// Return: None
// Designer: Zhou Dongfeng           Date: 2016-07-12
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------
void KT_WirelessMicRx_FastTune(long Freq)
{
//     KT0656M_Frequency = Freq;
    
//     uint8_t Freq_H,Freq_M,Freq_L,regx;
    
// #ifdef XTAL_DUAL
// 	#ifdef NEW_SEL_XTAL_MODE
//     	caclXtal(Freq);//select xtal
// 	#else
// 		oldCaclXtal(Freq);
// 	#endif
// #endif
    
// 	regx=KT_Bus_Read(0x0108,chipSel);
//     KT_Bus_Write(0x0108,regx | BIT3,chipSel); //SCAN_MODE=1;

// 	Freq = Freq & 0x000FFFFF;

//     Freq_H = ( Freq >> 12 );
//     Freq_M = ( (Freq & 0x00000FFF) >> 4 );
//     Freq_L = ( (Freq & 0x0000000F) << 4 );

//     KT_Bus_Write(0x0045,Freq_H,chipSel);                
//     KT_Bus_Write(0x0046,Freq_M,chipSel);

//     regx=KT_Bus_Read(0x0047,chipSel);
//     KT_Bus_Write(0x0047,(regx & 0x0F) | Freq_L ,chipSel);

// 	regx=KT_Bus_Read(0x0053,chipSel);
// 	KT_Bus_Write(0x0053,regx&~0x40,chipSel);  //rfamp_int_en=0

//     regx=KT_Bus_Read(0x0047,chipSel);
//     KT_Bus_Write(0x0047,regx | BIT0,chipSel); //chan_valid=1;

// 	Delay_ms(1); 
//     while (!(KT_Bus_Read(0x0061,chipSel) & 0x01)); //PLL done
 
//     //KT_WirelessMicRx_SAIInit();

// 	if(KT_WirelessMicRx_GetSNR() > AUTOMUTE_SNR_LOWTH)
// 	{
// 		regx=KT_Bus_Read(0x0087,chipSel);			//SOFT_SNR_MUTE写为0
// 		KT_Bus_Write(0x0087,regx&~0x02,chipSel);
// 	}	
}


//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_GetAF
// Function Description: Obtain audio amplitude
// Function Note:
// Global Variables: None
// Input: None
// Return: Audio signal strength
// Designer: Zhou Dongfeng           Date: 2016-07-13
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicRx_GetAF(void)
{
    uint16_t regx;
    regx = KT_Bus_Read(0x0209,chipSel);
    return( regx & 0x0F );
}

//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_GetRSSI
// Function Description: Obtain RF signal strength
// Function Note:
// Global Variables: None
// Input: None
// Return: RF signal strength
// Designer: Zhou Dongfeng           Date: 2016-07-13
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicRx_GetRSSI(void)
{
    uint8_t cRssi;
    //if(MorSSelect)
	//{
    	cRssi = KT_Bus_Read(0x020C,chipSel);
	//}
	//else
	//{
		//cRssi = KT_Bus_Read(0x0221,chipSel);
	//}
    return( cRssi );
}

//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_GetFastRSSI
// Function Description: Obtain RF signal strength
// Function Notes:
// Global Variables:
// Input: None
// Return: RF signal strength
// Designer: Zhou Dongfeng           Date: 2016-07-13
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicRx_GetFastRSSI(void)
{
    uint8_t cRssi;
    if(MorSSelect)
	{
    	cRssi = KT_Bus_Read(0x020A,chipSel);
	}
	else
	{
		cRssi = KT_Bus_Read(0x0221,chipSel);
	}
    return (cRssi);
}
//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_GetSNR
// Function Description: Obtain SNR
// Function Notes:
// Global Variables:
// Input: None
// Return: SNR value
// Designer: Zhou Dongfeng           Date: 2016-07-13
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------
uint8_t KT_WirelessMicRx_GetSNR(void)
{
    uint8_t snr;
    //if(MorSSelect)
	//{
    	snr = KT_Bus_Read(0x020D,chipSel);
	//}
	//else
	//{
    	//snr = KT_Bus_Read(0x0223 ,chipSel);
	//}
    return( snr );
}

//-----------------------------------------------------------------------------
// Function Name: Pll_Band_Cali
// Function Description: Newly added function to address temperature drift issues, called during the tune process
// Function Notes:
// Global Variables:
// Input: LO_VCO_BAND_SEL starting value and maximum value
// Return: None
// Designer: Wu Jinfeng              Date: 2017-04-01
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------

void Pll_Band_Cali(uint8_t CLl, uint8_t CLh)
{
    uint32_t uhf_start_ms = sys_get_10ms();   
    
    int Tmp[2], I_VCOCNT_RES_2, I_VCOFREQ_REF;
	uint8_t regx,loVcoBandSelRead;

	regx=KT_Bus_Read(0x005c,chipSel);
    if(!(regx&BIT6))
    {
        return;
    }
    regx=KT_Bus_Read(0x0061,chipSel);
	regx&=0xc7;
	regx|=0x28;					//O_VCOCNT_WIN(5);
    KT_Bus_Write(0x0061, regx ,chipSel);

	regx=KT_Bus_Read(0x001c,chipSel); //S_LO_VCO_BAND_CALI_EN;
    KT_Bus_Write(0x001c, regx|BIT6 ,chipSel);

	regx=KT_Bus_Read(0x001a,chipSel);
	regx&=0xc0;
	regx|=0x20;					//O_LO_MMD_MC(32);
    KT_Bus_Write(0x001a, regx, chipSel);

	regx=KT_Bus_Read(0x0061, chipSel); //S_LOMC_SEL;
    KT_Bus_Write(0x0061, regx | BIT1, chipSel);

    regx=KT_Bus_Read(0x0016,chipSel); //S_LO_DIV128_EN;
    KT_Bus_Write(0x0016, regx | BIT7, chipSel);
    
    
//    Delay_ms(1);
	loVcoBandSelRead = KT_Bus_Read(0x001e,chipSel);
	if(loVcoBandSelRead<5)
	{
		KT_Bus_Write(0x001e, CLl ,chipSel);//R_LO_VCO_BAND_SEL = CLl;
	}
	else
	{
		KT_Bus_Write(0x001e, loVcoBandSelRead-5 ,chipSel);//R_LO_VCO_BAND_SEL = CLl;
	}    

	regx=KT_Bus_Read(0x0042,chipSel); //C_VCOCNT_RST;
    KT_Bus_Write(0x0042, regx&~BIT5 ,chipSel);

	regx=KT_Bus_Read(0x0061,chipSel);
    KT_Bus_Write(0x0061, regx|BIT6 ,chipSel);  //S_VCOCNT_START;
    
    
    uhf_start_ms = sys_get_10ms();
    while(!(BIT7&KT_Bus_Read(0x0064,chipSel)))
    {
        if (sys_get_10ms() - uhf_start_ms >= 3*UHF_TIMEOUT)
        {
//            DEBUG_ERROR("UHF Pll_Band_Cali Timeout\r\n");
            return;
        }
    }//while(!I_VCOCNT_RDY);

	I_VCOCNT_RES_2=(0x0f & KT_Bus_Read(0x0064,chipSel));
	I_VCOCNT_RES_2<<=8;
	I_VCOCNT_RES_2|=KT_Bus_Read(0x0065,chipSel);
	I_VCOCNT_RES_2<<=8;
	I_VCOCNT_RES_2|=KT_Bus_Read(0x0066,chipSel);
	I_VCOCNT_RES_2>>=2;

	I_VCOFREQ_REF=(0x03 & KT_Bus_Read(0x0067,chipSel));
	I_VCOFREQ_REF<<=8;
	I_VCOFREQ_REF|=KT_Bus_Read(0x0068,chipSel);
	I_VCOFREQ_REF<<=8;
	I_VCOFREQ_REF|=KT_Bus_Read(0x0069,chipSel);

    Tmp[1] = I_VCOCNT_RES_2 - I_VCOFREQ_REF;

	regx=KT_Bus_Read(0x0042,chipSel); //S_VCOCNT_RST
    KT_Bus_Write(0x0042, regx|BIT5 ,chipSel);
    
    uint32_t uhf_begin_ms = sys_get_10ms(); 
    while(1)
    {      
        regx=KT_Bus_Read(0x001e,chipSel); //R_LO_VCO_BAND_SEL++
    	KT_Bus_Write(0x001e, ++regx ,chipSel);

		regx=KT_Bus_Read(0x0042,chipSel); //C_VCOCNT_RST;
    	KT_Bus_Write(0x0042, regx&~BIT5 ,chipSel);

		regx=KT_Bus_Read(0x0061,chipSel);
    	KT_Bus_Write(0x0061, regx|BIT6 ,chipSel);  //S_VCOCNT_START;
        
        uhf_start_ms = sys_get_10ms();
        while(!(BIT7&KT_Bus_Read(0x0064,chipSel)))
        {
            if (sys_get_10ms() - uhf_start_ms >= 2*UHF_TIMEOUT)
            {
//                DEBUG_ERROR("UHF Pll_Band_Cali Timeout\r\n");
                return;
            }
        }//while(!I_VCOCNT_RDY);
        Tmp[0] = labs(Tmp[1]);
		I_VCOCNT_RES_2=(0x0f & KT_Bus_Read(0x0064,chipSel));
		I_VCOCNT_RES_2<<=8;
		I_VCOCNT_RES_2|=KT_Bus_Read(0x0065,chipSel);
		I_VCOCNT_RES_2<<=8;
		I_VCOCNT_RES_2|=KT_Bus_Read(0x0066,chipSel);
		I_VCOCNT_RES_2>>=2;
	
		I_VCOFREQ_REF=(0x03 & KT_Bus_Read(0x0067,chipSel));
		I_VCOFREQ_REF<<=8;
		I_VCOFREQ_REF|=KT_Bus_Read(0x0068,chipSel);
		I_VCOFREQ_REF<<=8;
		I_VCOFREQ_REF|=KT_Bus_Read(0x0069,chipSel);
        Tmp[1] = I_VCOCNT_RES_2 - I_VCOFREQ_REF;
        regx=KT_Bus_Read(0x0042,chipSel); //S_VCOCNT_RST
    	KT_Bus_Write(0x0042, regx|BIT5 ,chipSel);

        if (sys_get_10ms() - uhf_begin_ms >= 5*UHF_TIMEOUT)
        {
//            DEBUG_ERROR("UHF Pll_Band_Cali Timeout\r\n");
            return;
        }
        if((labs(Tmp[1]) >= Tmp[0]) && (Tmp[1] <= 0))
        {
            regx=KT_Bus_Read(0x001e,chipSel); //R_LO_VCO_BAND_SEL--
    		KT_Bus_Write(0x001e, --regx ,chipSel);
            break;
        }
        else if(CLh != KT_Bus_Read(0x001e,chipSel))
        {
            continue;
        }
        else
        {
            break;
        }
    }
    regx=KT_Bus_Read(0x001c,chipSel); //C_LO_VCO_BAND_CALI_EN
    KT_Bus_Write(0x001c, regx&~BIT6 ,chipSel);

	regx=KT_Bus_Read(0x0016,chipSel); //C_LO_DIV128_EN;
    KT_Bus_Write(0x0016, regx&~BIT7 ,chipSel);

	regx=KT_Bus_Read(0x0061,chipSel); //C_LOMC_SEL;
    KT_Bus_Write(0x0061, regx&~BIT1 ,chipSel);
    
    return;
}

//-----------------------------------------------------------------------------
// Function Name: PLL_Reset
// Function Description: Newly added function to address temperature drift issues, called during the tune process
// Function Notes:
// Global Variables:
// Input: None
// Return: None
// Designer: Wu Jinfeng              Date: 2017-04-01
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------

void PLL_Reset(void)
{
    uint8_t Tmp;
    uint8_t VrefSel;
	uint8_t regx,regy;
    
    if(BIT3&KT_Bus_Read(0x00133,chipSel))	  //I_PLL_RESET_EN
    {
        regx=KT_Bus_Read(0x0016,chipSel);  //Tmp = I_LO_KVCO_CALI_EN;
		Tmp = (regx>>6)&BIT0;

        VrefSel = 0x07&KT_Bus_Read(0x0016,chipSel);//I_LO_KVCO_COARSE_VREF_SEL;

		regy=KT_Bus_Read(0x0133,chipSel);
		regy&=0x07;

		regx=KT_Bus_Read(0x0018,chipSel);
		regx&=~0x07;
		regx|=regy;
		KT_Bus_Write(0x0018, regx ,chipSel);  //O_LO_KVCO_COARSE_VREF_SEL(I_LO_KVCO_COARSE_VREF_SEL_RST);
		
		regx=KT_Bus_Read(0x0016,chipSel);
		KT_Bus_Write(0x0016, regx|BIT6 ,chipSel); 	//S_LO_KVCO_CALI_EN;      
        
        Delay_ms(10);

        if(Tmp)
        {
            regx=KT_Bus_Read(0x0016,chipSel);
			KT_Bus_Write(0x0016, regx|BIT6 ,chipSel); 	//S_LO_KVCO_CALI_EN; 
        }
        else
        {
            regx=KT_Bus_Read(0x0016,chipSel);
			KT_Bus_Write(0x0016, regx&~BIT6 ,chipSel); 	//C_LO_KVCO_CALI_EN; 
        }

		regx=KT_Bus_Read(0x0018,chipSel);
		KT_Bus_Write(0x0018, (regx&0xf8)|VrefSel ,chipSel);  //O_LO_KVCO_COARSE_VREF_SEL(VrefSel);        
    }
}

//-----------------------------------------------------------------------------
// Function Name: selectMS
// Function Description: Determines whether the main or secondary antenna is used for diversity based on the value of fast RSSI
// Function Notes:
// Global Variables:
// Input: None
// Return: None
// Designer: Wu Jinfeng              Date: 2017-04-01
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------
void KT_WirelessMicRx_SelectMS(void)
{
	#ifdef DIVERSITY
	uint8_t fastRssiSlave,fastRssiMaster;
	fastRssiMaster=KT_Bus_Read(0x020a,chipSel);
	fastRssiSlave=KT_Bus_Read(0x0222,chipSel);
	if(fastRssiMaster>=fastRssiSlave)
	{
		MorSSelect=1;
	}
	else
	{
		MorSSelect=0;
	}
	#else
		MorSSelect=1;
	#endif

}

void Seek_Freq_FastTune(long Freq)
{
    uint8_t Freq_H,Freq_M,Freq_L,regx;
	regx=KT_Bus_Read(0x0053,chipSel);
	KT_Bus_Write(0x0053,regx&~0x40,chipSel );  //rfamp_int_en=0  
#ifdef XTAL_DUAL
	#ifdef NEW_SEL_XTAL_MODE
    	caclXtal(Freq);//select xtal
	#else
		oldCaclXtal(Freq);
	#endif
#endif 
	regx=KT_Bus_Read(0x0108,chipSel);
    KT_Bus_Write(0x0108,regx | BIT3,chipSel); //SCAN_MODE=1;

	Freq = Freq & 0x000FFFFF;

    Freq_H = ( Freq >> 12 );
    Freq_M = ( (Freq & 0x00000FFF) >> 4 );
    Freq_L = ( (Freq & 0x0000000F) << 4 );

    KT_Bus_Write(0x0045,Freq_H,chipSel);                
    KT_Bus_Write(0x0046,Freq_M,chipSel);

    regx=KT_Bus_Read(0x0047,chipSel);
    KT_Bus_Write(0x0047,(regx & 0x0F) | Freq_L ,chipSel);

    regx=KT_Bus_Read(0x0047,chipSel);
    KT_Bus_Write(0x0047,regx | BIT0,chipSel); //chan_valid=1;
	Delay_ms(1); 
}

#ifdef XTAL_DUAL
//-----------------------------------------------------------------------------
//函 数 名：selXtal
//功能描述：Xtal选择
//函数说明：
//全局变量：
//输    入：选择24MHz: 0            选择24.576MHz: 1
//返    回：无
//设 计 者：Zhou Dongfeng           时间：2016-07-13                                
//修 改 者：                        时间：                                
//版    本：V0.1    For KT0656M 
//版    本：V1.7    在切换晶振的过程中，把ifadc也rst一下                                         
//-----------------------------------------------------------------------------
static void selXtal(bit xtalSel)
{
     uint8_t regx,state;
     
     xtalSel ^= XTAL_SEL1;
     regx = KT_Bus_Read(0x0012,chipSel);
     if(((regx&0x80)>>7)!=xtalSel) //当前的选择是xtal1
     {
         if(xtalSel==1)
        {
            KT_Bus_Write(0x0012,regx|0x80,chipSel);
        }
        else
        {
            KT_Bus_Write(0x0012,regx&~0x80,chipSel);
        }
        do
        {
            regx=KT_Bus_Read(0x00a0,chipSel);
        }while(!(regx&0x02));
		regx=KT_Bus_Read(0x00a0,chipSel);
        KT_Bus_Write(0x00a0,regx|0x02,chipSel);

        regx = KT_Bus_Read(0x0015,chipSel); //rst dll
        KT_Bus_Write(0x0015,regx|0x40,chipSel);
		regx = KT_Bus_Read(0x002a,chipSel); //ifadc rst
        KT_Bus_Write(0x002a,regx|0x80,chipSel);
		Delay_ms(10);
        regx = KT_Bus_Read(0x0015,chipSel);      
        KT_Bus_Write(0x0015,regx&~0x40,chipSel);
        do
        {
            regx = KT_Bus_Read(0x0180,chipSel);     
            state=(regx&0x08)>>3;
        }while(!state);
		regx = KT_Bus_Read(0x002a,chipSel); //ifadc rst
        KT_Bus_Write(0x002a,regx&~0x80,chipSel);

//		regx=KT_Bus_Read(0x0042,chipSel);	//dsp_rst
//		KT_Bus_Write(0x0042,regx | 0x04,chipSel);
//		Delay_ms(1);
//		regx=KT_Bus_Read(0x0042,chipSel);
//		KT_Bus_Write(0x0042,regx &~ 0x04,chipSel); 
     }
}
#ifdef NEW_SEL_XTAL_MODE
//-----------------------------------------------------------------------------
//函 数 名：caclFreqFrac
//功能描述：根据频点和晶振的频率来计算参数
//函数说明：
//全局变量：
//输    入：int Freq,int xtal_freq
//返    回：UINT32的参数值
//设 计 者：Zhou Dongfeng           时间：2016-07-13                                
//修 改 者：                        时间：                                
//版    本：V0.1    For KT0656M                    
//-----------------------------------------------------------------------------
static UINT32 caclFreqFrac(int Freq,int xtal_freq)
{
    UINT32 temp,tempMin=xtal_freq;
    uint8_t i;
    
    for(i=1;i<4;i++)
    {
        temp = ((xtal_freq/(2*i))-abs(((Freq%(xtal_freq/i))-(xtal_freq/(2*i)))))*i;
        if(temp<tempMin)
        {
            tempMin=temp;
        }
    }
    i=8;
    temp = ((xtal_freq/(2*i))-abs(((Freq%(xtal_freq/i))-(xtal_freq/(2*i)))))*i;
    if(temp<tempMin)
    {
        tempMin=temp;
    }
    return(tempMin*(3072000/xtal_freq));
}

//使用24MHz晶振的频点
UINT32 code use24M[26] = 
{   
    490500,492000,516000,541500,556000,565500,566000,590000,614000,615000,639000,651250,688000,
    688500,712000,712250,712500,722500,736500,760500,762000,787500,810000,811500,835500,859500
};

//使用24.56MHz晶振的频点
UINT32 code use24576M[14] = 
{
    7500,9000,10000,10500,12000,13500,14000,15000,16000,16500,18000,19500,20000,22000
};


//-----------------------------------------------------------------------------
//函 数 名：caclXtal
//功能描述：根据频点选择要使用的晶体
//函数说明：
//全局变量：
//输    入：int Freq
//返    回：无
//设 计 者：Zhou Dongfeng           时间：2016-07-13                                
//修 改 者：                        时间：                                
//版    本：V0.1    For KT0656M                    
//-----------------------------------------------------------------------------
static void caclXtal(int Freq)
{
    uint8_t use24M_flag=0,use24576M_flag=0,i;
    UINT32 state_tmp,chan_frac_temp0,chan_frac_temp1;
    
    for(i=0;i<26;i++)
    {
        if(Freq==use24M[i])    
        {
            use24M_flag=1;
            break;
        }
    }
    state_tmp = Freq%24000;
    for(i=0;i<14;i++)
    {
        if(state_tmp==use24576M[i])
        {
            use24576M_flag=1;
            break;
        }
    }
    if (use24M_flag)
    {
        selXtal(XTAL_24M_FREQ);
    }
    else if(use24576M_flag)
    {
        selXtal(XTAL_24P576M_FREQ);
    }
    else 
    {    
        chan_frac_temp0= caclFreqFrac(Freq,24000);
        chan_frac_temp1= caclFreqFrac(Freq,24576);
        if(chan_frac_temp0>chan_frac_temp1)
        {
            selXtal(XTAL_24M_FREQ);    
        }
        else
        {
            selXtal(XTAL_24P576M_FREQ);
        }
    }
}
#endif

#ifdef OLD_SEL_XTAL_MODE
//------------------------------------------------------------------------------------
//函 数 名：KT_WirelessMicRx_Calc_ChanReg									
//功能描述：晶体的频率控制字计算												
//函数说明：输入以KHz为单位的VCO震荡频率;											
//			计算结果存在*chan_ptr,*chan_frac_ptr,*chan_frac_msb_ptr中				
//调用函数：KT_Bus_Read()、KT_Bus_Write()、Delay_ms()								
//全局变量：																		
//输    入：Freq （输入以KHz为单位的VCO频率）										
//返    回：正确：1	错误：0															
//设 计 者：YANG Pei					时间：2012-04-19							
//修 改 者：KANG Hekai					时间：2013-03-29							
//版    本：V1.1																	
//			V1.7	修改余数<40或大于xtal-40的bug																	
//------------------------------------------------------------------------------------
bool KT_WirelessMicRx_Calc_ChanReg(int Freq, uint16_t *chan_ptr, int16_t *chan_frac_ptr, uint8_t *chan_frac_msb_ptr, uint16_t xtal_freq)
{
	*chan_ptr = Freq / xtal_freq;
	Freq = Freq % xtal_freq; 
	*chan_frac_ptr = (Freq << 16) / xtal_freq;
	if ((Freq <= 40) && (Freq >= 0))
	{
		*chan_frac_ptr = 0xffff;
		*chan_frac_msb_ptr =3;
	}
	else if ((Freq < xtal_freq ) && (Freq >= xtal_freq - 40))
	{
		(*chan_ptr)++; 
		*chan_frac_ptr = 0xffff;
		*chan_frac_msb_ptr =3;
	}
	else if ((Freq >= (xtal_freq / 2 - 40)) && (Freq <= (xtal_freq / 2 + 40)))
	{
		*chan_frac_ptr = 0x7FFF;
		*chan_frac_msb_ptr = 0;
	}
	else if (Freq > (xtal_freq >> 1))
	{
		(*chan_ptr)++; 
		*chan_frac_msb_ptr = 3;
	}
	else	
	{
		*chan_frac_msb_ptr = 0;
	}
	return(1);
}

//-----------------------------------------------------------------------------
//函 数 名：oldCaclXtal
//功能描述：根据频点选择要使用的晶体(跟KT0616M选择的晶体的方法一致)
//函数说明：
//全局变量：
//输    入：int Freq
//返    回：无
//设 计 者：wu jinfeng              时间：2017-06-05                                
//修 改 者：                        时间：                                
//版    本：V0.1    新增加函数                   
//-----------------------------------------------------------------------------
static void oldCaclXtal(int Freq)
{
	uint16_t chan0,chan1;
	int16_t chan_frac0,chan_frac1;
	uint8_t chan_frac_msb0,chan_frac_msb1;
	int16_t mod0,mod1,mod2,mod3;
	Freq = Freq << 1;
	KT_WirelessMicRx_Calc_ChanReg(Freq, &chan0, &chan_frac0, &chan_frac_msb0,24000);
	KT_WirelessMicRx_Calc_ChanReg(Freq, &chan1, &chan_frac1, &chan_frac_msb1,24576);
	mod0 = chan_frac0;
	mod1 = chan_frac1;
	mod2 = chan_frac0 << 1;
	mod3 = chan_frac1 << 1;
	if(mod0 < 0) 
		mod0 = ~mod0;			 //mod0=abs(mod0);
	if(mod1 < 0)
		mod1 = ~mod1;			 //mod1=abs(mod1);
	if(mod2 < 0)
		mod2 = ~mod2;			 //mod2=abs(mod2);
	if(mod3 < 0)
		mod3 = ~mod3;			 //mod3=abs(mod3);
	if(mod2 < mod0)
		mod0 = mod2;
	if(mod3 < mod1)
		mod1 = mod3;
	if(mod0 < mod1)
	{
		selXtal(XTAL_24P576M_FREQ);
	}
	else
	{
		selXtal(XTAL_24M_FREQ); 
	}
}
#endif
#endif

//I2S 主右从左 可以


//-----------------------------------------------------------------------------
// Function Name: KT_WirelessMicRx_SAIInit
// Function Description: SAI working initialization program (Serial Audio Interface)
// Function Notes:
// Global Variables:
// Input: int Freq
// Return: None
// Designer: Zhou Dongfeng           Date: 2016-07-13
// Modifier:                        Date:
// Version: V0.1    For KT0656M
//-----------------------------------------------------------------------------

void KT_WirelessMicRx_SAIInit(void)
{
#ifdef I2S_EN
    uint8_t regx;
#ifdef DIVERSITY
    if(chipSel>chipAS) //B通道
#else
    if(chipSel>chipAM) //B通道
#endif
    {
        KT_Bus_Write(0x0050, 0x20,chipSel); //stereo_mono_sel=1 LR_SEL=0   //1 left  0 right
        KT_Bus_Write(0x0051, 0x10,chipSel); //默认情况下，mclk=12m,mclk/lrclk=128,i2s标准，对应dac double模式
        KT_Bus_Write(0x0052, 0x02,chipSel); //i2s_master_en= 1
    }
    else
    {            
        KT_Bus_Write(0x0052,0x01,chipSel); //i2s_slave_en = 1
        KT_Bus_Write(0x004d,0x01,chipSel); //自动估算，快速模式，phase=0,unlock_tw=0
        regx = KT_Bus_Read(0x004d,chipSel);
        KT_Bus_Write(0x004d,regx|0x80,chipSel); //unlock_tw_cfg_rdy = 1
        KT_Bus_Write(0x0050,0x36,chipSel); //立体声，左声道，i2s标准以及24bit data
        regx = KT_Bus_Read(0x0050,chipSel);
        KT_Bus_Write(0x0050,regx|0x40,chipSel); //i2s_slave_sync_en=1
        regx = KT_Bus_Read(0x0052,chipSel);
        KT_Bus_Write(0x0052,regx|0x08,chipSel); //i2ss_pad_sdout_oen=1    
    }

#endif
    return;
}

#endif // BOARD_HW_HAS_UHF 
