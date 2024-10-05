#include "flash.h"

int32_t Internal_Flash_Erase_Page(uint32_t page_addr)
{
  HAL_StatusTypeDef result;
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t pageError;

  eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  eraseInit.Banks = FLASH_BANK_1;
  eraseInit.PageAddress = page_addr;
  eraseInit.NbPages = 1;

  HAL_FLASH_Unlock();
  result = HAL_FLASHEx_Erase(&eraseInit, &pageError);
  HAL_FLASH_Lock();

  if (result == HAL_OK)
    return 0;
  else
    return -1;
}

int32_t Internal_Flash_Program_64(uint32_t addr, uint64_t data)
{
  HAL_StatusTypeDef result;

  HAL_FLASH_Unlock();
  result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data);
  HAL_FLASH_Lock();

  if (result == HAL_OK)
    return 0;
  else
    return -1;
}

int32_t Internal_Flash_Program_32(uint32_t addr, uint8_t *data, uint32_t len)
{
  uint32_t w_data;
  HAL_StatusTypeDef result;

  HAL_FLASH_Unlock();
  for (uint32_t i = 0; i < len; i += 4)
  {
    w_data = *((uint32_t *)(&data[i]));
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + i, w_data);
    if (result != HAL_OK)
      return -1;
  }
  HAL_FLASH_Lock();

  return 0;
}

int32_t Internal_Flash_Program(uint32_t addr_offset, uint8_t *data, uint32_t len)
{
  uint32_t w_data;
  HAL_StatusTypeDef result;

  HAL_FLASH_Unlock();
  for (uint32_t i = 0; i < len; i += 4)
  {
    w_data = *((uint32_t *)(&data[i]));
    result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MAIN_PROGRAM_ADDR + addr_offset + i, w_data);
    if (result != HAL_OK)
      return -1;
  }
  HAL_FLASH_Lock();

  return 0;
}

uint16_t Get_Flash_Size(void) // Kbytes
{
  uint16_t flash_size = *(uint16_t *)(0x1FFFF7E0);

  return flash_size;
}

uint32_t Get_Flag_Addr(void)
{
  uint16_t flash_size = *(uint16_t *)(0x1FFFF7E0);
  // last 1K page
  uint32_t ota_flag_addr = FLASH_BASE_ADDR + (flash_size - 1) * 0x400;

  // debug
  return ota_flag_addr;
}

void MCU_Reset(void)
{
  NVIC_SystemReset();
}

uint32_t Read_OTA_Size(void)
{
  // last 1K page
  uint32_t ota_flag_addr = Get_Flag_Addr();

  return (*(uint32_t *)(ota_flag_addr + 4));
}

uint32_t Read_OTA_Flag(void)
{
  // last 1K page
  uint32_t ota_flag_addr = Get_Flag_Addr();

  return (*(uint32_t *)ota_flag_addr);
}

int32_t Save_OTA_Flag(uint32_t fw_size)
{
  // last 1K page
  uint32_t ota_flag_addr = Get_Flag_Addr();
  uint64_t data = ((uint64_t)fw_size << 32) | OTA_GOTO_BOOT_FLAG;
  uint8_t nvm_data[NVM_DATA_CNT] = {0};

  for (uint8_t i = 0; i < NVM_DATA_CNT; i++)
  {
    nvm_data[i] = *(uint8_t *)(ota_flag_addr + 8 + i);
  }

  if (Internal_Flash_Erase_Page(ota_flag_addr))
    return -1;
  if (Internal_Flash_Program_64(ota_flag_addr, data))
    return -2;
  if (Internal_Flash_Program_32(ota_flag_addr + 8, nvm_data, NVM_DATA_CNT))
    return -3;
  if (*(uint32_t *)ota_flag_addr != OTA_GOTO_BOOT_FLAG)
    return -4;

  return 0;
}

int32_t Clear_OTA_Flag(void)
{
  // last 1K page
  uint32_t ota_flag_addr = Get_Flag_Addr();
  uint32_t fw_size = Read_OTA_Size();
  uint64_t data = ((uint64_t)fw_size << 32) | OTA_GOTO_JUMP_FLAG;
  uint8_t nvm_data[NVM_DATA_CNT] = {0};

  for (uint8_t i = 0; i < NVM_DATA_CNT; i++)
  {
    nvm_data[i] = *(uint8_t *)(ota_flag_addr + 8 + i);
  }

  if (Read_OTA_Flag() == OTA_GOTO_JUMP_FLAG)
    return 0;
  if (Internal_Flash_Erase_Page(ota_flag_addr))
    return -1;
  if (Internal_Flash_Program_64(ota_flag_addr, data))
    return -2;
  if (Internal_Flash_Program_32(ota_flag_addr + 8, nvm_data, NVM_DATA_CNT))
    return -3;
  if (*(uint32_t *)ota_flag_addr != OTA_GOTO_JUMP_FLAG)
    return -4;

  return 0;
}

int32_t Read_Nvm_Data(uint8_t *nvm_data)
{
  // last 1K page
  uint32_t ota_flag_addr = Get_Flag_Addr();

  for (uint8_t i = 0; i < NVM_DATA_CNT; i++)
  {
    nvm_data[i] = *(uint8_t *)(ota_flag_addr + 8 + i);
  }

  return 0;
}

int32_t Save_Nvm_Data(uint8_t *nvm_data)
{
  // last 1K page
  uint32_t ota_flag_addr = Get_Flag_Addr();
  uint32_t fw_size = Read_OTA_Size();
  uint32_t ota_flag = *(uint32_t *)ota_flag_addr;
  uint64_t data = ((uint64_t)fw_size << 32) | ota_flag;

  if (Read_OTA_Flag() == OTA_GOTO_JUMP_FLAG)
    return 0;
  if (Internal_Flash_Erase_Page(ota_flag_addr))
    return -1;
  if (Internal_Flash_Program_64(ota_flag_addr, data))
    return -2;
  if (Internal_Flash_Program_32(ota_flag_addr + 8, nvm_data, NVM_DATA_CNT))
    return -3;
  if (*(uint32_t *)ota_flag_addr != ota_flag)
    return -4;

  return 0;
}

int32_t Clear_Nvm_Data(void)
{
  // last 1K page
  uint32_t ota_flag_addr = Get_Flag_Addr();
  uint32_t fw_size = Read_OTA_Size();
  uint32_t ota_flag = *(uint32_t *)ota_flag_addr;
  uint64_t data = ((uint64_t)fw_size << 32) | ota_flag;

  if (Read_OTA_Flag() == OTA_GOTO_JUMP_FLAG)
    return 0;
  if (Internal_Flash_Erase_Page(ota_flag_addr))
    return -1;
  if (Internal_Flash_Program_64(ota_flag_addr, data))
    return -2;
  if (*(uint32_t *)ota_flag_addr != ota_flag)
    return -3;

  return 0;
}

int32_t Erase_All_Page(void)
{
  uint16_t flash_size = *(uint16_t *)(0x1FFFF7E0);
  int32_t res;

  for (uint32_t i = 0; i < flash_size - 16 - 1; i++)
  {
    res = Internal_Flash_Erase_Page(FLASH_BASE + (16 + i) * 0x400);
    if (res)
    {
      return res;
    }
  }

  return 0;
}

// return 0 - not blank
//        1 - blank
int32_t Check_Blank(uint32_t addr, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    if (*((uint8_t *)(addr + i)) != 0xFF)
      return 0;
  }

  return 1;
}

// check flash and memory data match
// return 0 - not match
//        1 - match
int32_t Check_Match(uint32_t addr, uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0; i < len; i++)
  {
    if (*((uint8_t *)(addr + i)) != data[i])
      return 0;
  }

  return 1;
}
