/**
This file was generated by wechat public account: embedded_sw.

CRC Info:
Name                 Polynomial Initial    FinalXor   InputReflected ResultReflected
CRC16_MODBUS         0x8005     0xFFFF     0x0000     true           true
*/
#ifndef _CRC16_MODBUS_H_
#define _CRC16_MODBUS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    unsigned short crc16_modbus(unsigned short crc, const unsigned char *buf, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif // _CRC16_MODBUS_H_
