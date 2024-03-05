#ifndef __DATA_DEFINE_H__
#define __DATA_DEFINE_H__

#include <stdint.h>
#include <ctype.h>
#include <math.h>
#include "esp_system.h"

#define __FIRMWARE_VERSION__ "SL1.9.8.4"

// //typedef union
// //{
// //    float value;
// //    uint8_t bytes[4];
// //} Float_t;

// typedef union
// {
//     uint32_t value;
//     uint8_t bytes[4];
// } Long_t;

typedef union
{
    uint16_t value;
    uint8_t bytes[2];
} Int_t;

// typedef struct
// {
//     uint8_t Year;
//     uint8_t Month;
//     uint8_t Day;
//     uint8_t Hour;
//     uint8_t Minute;
//     uint8_t Second;
// } DateTime_t;

// typedef struct
// {
//     uint8_t Sub[4];
//     uint16_t Port;
// } IPStruct_t;


typedef enum {
    IO_OFF = 0,
    IO_ON = 1
} IOState_e;


// typedef enum {
// 	NO_STREAM  = 0,
// 	LIVE_MASTER = 1,
// 	SCHEDULE_LINK = 2,
// 	LAPTOP = 3,
// 	MOBILE = 4
// } MasterStreamType_t;

// typedef struct
// {
// 	/* Phiên bản phần cứng */
// 	uint8_t hwVersion;
// } Status_t;

// typedef struct
// {
// 	// Parameters_t Parameters;	
// 	Status_t Status;
// }System_t;

#endif // __DATA_DEFINE_H__