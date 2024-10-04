/*
 * @Author: sunjiacheng sunjiacheng@wyindustry.com
 * @Date: 2023-12-27 17:05:18
 * @LastEditors: sunjiacheng sunjiacheng@wyindustry.com
 * @LastEditTime: 2023-12-28 13:26:06
 * @FilePath: \GD32E10x_station_board\Template\flash.h
 * @Description:
 *
 * Copyright (c) 2023 by CalmCar, All Rights Reserved.
 */
#ifndef FLASH_H
#define FLASH_H

#include "main.h"

// indicate updating
#define OTA_GOTO_BOOT_FLAG 0x11223344

// indicate valid firmware
#define OTA_GOTO_JUMP_FLAG 0x32547697

// location 16K
#define FLASH_BASE_ADDR 0x8000000
#define MAIN_PROGRAM_ADDR (FLASH_BASE_ADDR + 16 * 0x400)

#define TEST_RESULT_CNT 8 // seet to an integral multiple of 4 bytes, otherwise data may losed

#ifdef __cplusplus
extern "C"
{
#endif

    int32_t Internal_Flash_Program(uint32_t addr_offset, uint8_t *data, uint32_t len);
    uint16_t Get_Flash_Size(void);
    uint32_t Get_Flag_Addr(void);
    void MCU_Reset(void);
    uint32_t Read_OTA_Size(void);
    uint32_t Read_OTA_Flag(void);
    int32_t Save_OTA_Flag(uint32_t fw_size);
    int32_t Clear_OTA_Flag(void);
    int32_t Read_Test_Result(uint8_t *test_result);
    int32_t Save_Test_Result(uint8_t *test_result);
    int32_t Clear_Test_Result(void);
    int32_t Erase_All_Page(void);
    int32_t Check_Blank(uint32_t addr, uint32_t len);
    int32_t Check_Match(uint32_t addr, uint8_t *data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */
