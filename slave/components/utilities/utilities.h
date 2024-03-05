#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include "version_ctrl.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

/**
 * @brief  Free memory
 *
 * @param  ptr  Memory pointer
 */
#define LOLA_FREE(ptr) { \
        if(ptr) { \
            free(ptr); \
            ptr = NULL; \
        } \
    } while(0)
    

//void GetIPAddressFromString(char *Buffer, IPStruct_t *DiaChiIP);
uint8_t CopyParameter(char* BufferSource, char* BufferDes, char FindCharBegin, char FindCharEnd);
int16_t FindIndexOfChar(char CharToFind, char *BufferToFind);
uint8_t ValidSecureCode(char* BufferData);
uint32_t GetNumberFromString(uint16_t BeginAddress, char* Buffer);
uint32_t GetHexNumberFromString(uint16_t BeginAddress, char* Buffer);
uint16_t CalculateCheckSum(uint8_t* Buffer, uint16_t BeginAddress, uint16_t Length);
uint16_t CalculateCRC16(uint8_t *DataIn, uint8_t NbByte);
//DateTime_t GetTimeFromBuffer(char* Buffer, uint8_t Begin);
void PrintHexStr(uint8_t *Buffer, uint16_t Length, char *UserStr);
char* ConvBufferToStr(uint8_t *Buffer, uint8_t Length);
uint32_t CalcABS(uint32_t Value1, uint32_t Value2);
bool IsASCIIString(char* str);
bool IsDigitString(char* str);

#endif // __UTILITIES_H__

