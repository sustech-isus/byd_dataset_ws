#ifndef _CANPROCESS_H
#define _CANPROCESS_H

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
//´íÎóÂë
#define ERR_CAN_PROCESS_OK                                 0
#define ERR_CAN_PROCESS_INIT                             -1
//#define ERR_CAN_PROCESS_CREATE_BUF                -2
#define ERR_CAN_PROCESS_INVALID_PARAM          -3
#define ERR_CAN_PROCESS_CLOSE                          -4
#define ERR_CAN_PROCESS_WRITE                          -5

#define CAN_PROCESS_DATA_LEN                             8

int CanInit();
int CanClose();
int CanWrite(uint32_t nSendId, uint8_t* nData, uint8_t nDataLen);
int CanPutData(uint32_t nId, uint64_t nTime, uint8_t* pData);

#endif
