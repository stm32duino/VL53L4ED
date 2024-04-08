/**
******************************************************************************
* @file    platform.cpp
* @author  STMicroelectronics
******************************************************************************
* @attention
*
* Copyright (c) 2024 STMicroelectronics.
* All rights reserved.
*
* This software is licensed under terms that can be found in the LICENSE file
* in the root directory of this software component.
* If no LICENSE file comes with this software, it is provided AS-IS.
*
******************************************************************************
*/

#include "vl53l4ed_class.h"

/* Private functions prototypes */

uint8_t VL53L4ED::VL53L4ED_WrByte(uint16_t index, uint8_t data)
{
  uint8_t status = VL53L4ED_ERROR_NONE;

  status = _I2CWrite(index, &data, 1);

  return status;
}

uint8_t VL53L4ED::VL53L4ED_WrWord(uint16_t index, uint16_t data)
{
  uint8_t status = VL53L4ED_ERROR_NONE;
  uint8_t buffer[2] = {0, 0};

  buffer[0] = data >> 8;
  buffer[1] = data & 0x00FF;

  status = _I2CWrite(index, buffer, 2);

  return status;
}

uint8_t VL53L4ED::VL53L4ED_WrDWord(uint16_t index, uint32_t data)
{
  uint8_t status = VL53L4ED_ERROR_NONE;
  uint8_t buffer[4] = {0, 0, 0, 0};

  buffer[0] = (data >> 24) & 0xFF;
  buffer[1] = (data >> 16) & 0xFF;
  buffer[2] = (data >> 8)  & 0xFF;
  buffer[3] = (data >> 0) & 0xFF;

  status = _I2CWrite(index, buffer, 4);

  return status;
}

uint8_t VL53L4ED::VL53L4ED_RdByte(uint16_t index, uint8_t *data)
{
  uint8_t status = VL53L4ED_ERROR_NONE;

  status = _I2CRead(index, data, 1);

  return status;
}

uint8_t VL53L4ED::VL53L4ED_RdWord(uint16_t index, uint16_t *data)
{
  uint8_t status = VL53L4ED_ERROR_NONE;
  uint8_t buffer[2] = {0, 0};

  status = _I2CRead(index, buffer, 2);
  if (!status) {
    *data = ((uint16_t)buffer[0] << 8) + (uint16_t)buffer[1];
  }

  return status;
}

uint8_t VL53L4ED::VL53L4ED_RdDWord(uint16_t index, uint32_t *data)
{
  uint8_t status = VL53L4ED_ERROR_NONE;
  uint8_t buffer[4] = {0, 0, 0, 0};

  status = _I2CRead(index, buffer, 4);
  if (!status) {
    *data = ((uint32_t)buffer[0] << 24) + ((uint32_t)buffer[1] << 16) + ((uint32_t)buffer[2] << 8) + (uint32_t)buffer[3];
  }

  return status;
}

uint8_t VL53L4ED::WaitMs(uint32_t TimeMs)
{
  delay(TimeMs);
  return 0;
}

