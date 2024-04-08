/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
 * @file  vl53l4ed_api.c
 * @brief Functions implementation
 */

#include <string.h>
#include <math.h>
#include "vl53l4ed_class.h"
#include "vl53l4ed_config.h"

VL53L4ED_Error VL53L4ED::VL53L4ED_GetSWVersion(
  VL53L4ED_Version_t *p_Version)
{
  VL53L4ED_Error Status = VL53L4ED_ERROR_NONE;

  p_Version->major = VL53L4ED_IMPLEMENTATION_VER_MAJOR;
  p_Version->minor = VL53L4ED_IMPLEMENTATION_VER_MINOR;
  p_Version->build = VL53L4ED_IMPLEMENTATION_VER_BUILD;
  p_Version->revision = VL53L4ED_IMPLEMENTATION_VER_REVISION;
  return Status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SetI2CAddress(
  uint8_t new_address)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_WrByte(VL53L4ED_I2C_SLAVE__DEVICE_ADDRESS,
                            (uint8_t)(new_address >> (uint8_t)1));

  //if(status == VL53L4ED_ERROR_NONE)
  //{
  address = new_address;
  //}
  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetSensorId(
  uint16_t *p_id)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_RdWord(VL53L4ED_IDENTIFICATION__MODEL_ID, p_id);

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SensorInit()
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint8_t Addr, tmp;
  uint8_t continue_loop = 1;
  uint16_t i = 0;

  do {
    status |= VL53L4ED_RdByte(
                VL53L4ED_FIRMWARE__SYSTEM_STATUS, &tmp);

    if (tmp == (uint8_t)0x3) { /* Sensor booted */
      continue_loop = (uint8_t)0;
    } else if (i < (uint16_t)1000) {  /* Wait for boot */
      i++;
    } else { /* Timeout 1000ms reached */
      continue_loop = (uint8_t)0;
      status |= (uint8_t)VL53L4ED_ERROR_TIMEOUT;
    }
    WaitMs(1);
  } while (continue_loop == (uint8_t)1);

  /* Load default configuration */
  for (Addr = (uint8_t)0x2D; Addr <= (uint8_t)0x87; Addr++) {
    status |= VL53L4ED_WrByte(Addr,
                              VL53L4ED_DEFAULT_CONFIGURATION[
                             Addr - (uint8_t)0x2D]);
  }

  /* Start VHV */
  status |= VL53L4ED_WrByte(VL53L4ED_SYSTEM_START, (uint8_t)0x40);
  i  = (uint8_t)0;
  continue_loop = (uint8_t)1;
  do {
    status |= VL53L4ED_CheckForDataReady(&tmp);
    if (tmp == (uint8_t)1) { /* Data ready */
      continue_loop = (uint8_t)0;
    } else if (i < (uint16_t)1000) {  /* Wait for answer */
      i++;
    } else { /* Timeout 1000ms reached */
      continue_loop = (uint8_t)0;
      status |= (uint8_t)VL53L4ED_ERROR_TIMEOUT;
    }
    WaitMs(1);
  } while (continue_loop == (uint8_t)1);

  status |= VL53L4ED_ClearInterrupt();
  status |= VL53L4ED_StopRanging();
  status |= VL53L4ED_WrByte(
              VL53L4ED_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
              (uint8_t)0x09);
  status |= VL53L4ED_WrByte(0x0B, (uint8_t)0);
  status |= VL53L4ED_WrWord(0x0024, 0x500);

  status |= VL53L4ED_SetRangeTiming(100, 0);

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_ClearInterrupt(
)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_WrByte(VL53L4ED_SYSTEM__INTERRUPT_CLEAR, 0x01);
  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_StartRanging()
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint32_t tmp;

  status |= VL53L4ED_RdDWord(VL53L4ED_INTERMEASUREMENT_MS, &tmp);

  /* Sensor runs in continuous mode */
  if (tmp == (uint32_t)0) {
    status |= VL53L4ED_WrByte(VL53L4ED_SYSTEM_START, 0x21);
  }
  /* Sensor runs in autonomous mode */
  else {
    status |= VL53L4ED_WrByte(VL53L4ED_SYSTEM_START, 0x40);
  }

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_StopRanging(
)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_WrByte(VL53L4ED_SYSTEM_START, 0x00);
  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_CheckForDataReady(
  uint8_t *p_is_data_ready)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint8_t temp;
  uint8_t int_pol;

  status |= VL53L4ED_RdByte(VL53L4ED_GPIO_HV_MUX__CTRL, &temp);
  temp = temp & (uint8_t)0x10;
  temp = temp >> 4;

  if (temp == (uint8_t)1) {
    int_pol = (uint8_t)0;
  } else {
    int_pol = (uint8_t)1;
  }

  status |= VL53L4ED_RdByte(VL53L4ED_GPIO__TIO_HV_STATUS, &temp);

  if ((temp & (uint8_t)1) == int_pol) {
    *p_is_data_ready = (uint8_t)1;
  } else {
    *p_is_data_ready = (uint8_t)0;
  }

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SetRangeTiming(
  uint32_t timing_budget_ms,
  uint32_t inter_measurement_ms)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint16_t clock_pll, osc_frequency, ms_byte;
  uint32_t macro_period_us = 0, timing_budget_us = 0, ls_byte, tmp;
  float_t inter_measurement_factor = (float_t)1.055;

  status |= VL53L4ED_RdWord(0x0006, &osc_frequency);
  if (osc_frequency != (uint16_t)0) {
    timing_budget_us = timing_budget_ms * (uint32_t)1000;
    macro_period_us = (uint32_t)((uint32_t)2304 *
                                 ((uint32_t)0x40000000 / (uint32_t)osc_frequency)) >> 6;
  } else {
    status |= (uint8_t)VL53L4ED_ERROR_INVALID_ARGUMENT;
  }

  /* Timing budget check validity */
  if ((timing_budget_ms < (uint32_t)10)
      || (timing_budget_ms > (uint32_t)200) || (status != (uint8_t)0)) {
    status |= VL53L4ED_ERROR_INVALID_ARGUMENT;
  }
  /* Sensor runs in continuous mode */
  else if (inter_measurement_ms == (uint32_t)0) {
    status |= VL53L4ED_WrDWord(VL53L4ED_INTERMEASUREMENT_MS, 0);
    timing_budget_us -= (uint32_t)2500;
  }
  /* Sensor runs in autonomous low power mode */
  else if (inter_measurement_ms > timing_budget_ms) {
    status |= VL53L4ED_RdWord(
                VL53L4ED_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
    clock_pll = clock_pll & (uint16_t)0x3FF;
    inter_measurement_factor = inter_measurement_factor
                               * (float_t)inter_measurement_ms
                               * (float_t)clock_pll;
    status |= VL53L4ED_WrDWord(VL53L4ED_INTERMEASUREMENT_MS,
                               (uint32_t)inter_measurement_factor);

    timing_budget_us -= (uint32_t)4300;
    timing_budget_us /= (uint32_t)2;

  }
  /* Invalid case */
  else {
    status |= (uint8_t)VL53L4ED_ERROR_INVALID_ARGUMENT;
  }

  if (status != (uint8_t)VL53L4ED_ERROR_INVALID_ARGUMENT) {
    ms_byte = 0;
    timing_budget_us = timing_budget_us << 12;
    tmp = macro_period_us * (uint32_t)16;
    ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6))
              - (uint32_t)1;

    while ((ls_byte & 0xFFFFFF00U) > 0U) {
      ls_byte = ls_byte >> 1;
      ms_byte++;
    }
    ms_byte = (uint16_t)(ms_byte << 8)
              + (uint16_t)(ls_byte & (uint32_t)0xFF);
    status |= VL53L4ED_WrWord(VL53L4ED_RANGE_CONFIG_A, ms_byte);

    ms_byte = 0;
    tmp = macro_period_us * (uint32_t)12;
    ls_byte = ((timing_budget_us + ((tmp >> 6) >> 1)) / (tmp >> 6))
              - (uint32_t)1;

    while ((ls_byte & 0xFFFFFF00U) > 0U) {
      ls_byte = ls_byte >> 1;
      ms_byte++;
    }
    ms_byte = (uint16_t)(ms_byte << 8)
              + (uint16_t)(ls_byte & (uint32_t)0xFF);
    status |= VL53L4ED_WrWord(VL53L4ED_RANGE_CONFIG_B, ms_byte);
  }

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetRangeTiming(
  uint32_t *p_timing_budget_ms,
  uint32_t *p_inter_measurement_ms)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint16_t osc_frequency = 1, range_config_macrop_high, clock_pll = 1;
  uint32_t tmp, ls_byte, ms_byte, macro_period_us;
  float_t clock_pll_factor = (float_t)1.065;

  /* Get InterMeasurement */
  status |= VL53L4ED_RdDWord(VL53L4ED_INTERMEASUREMENT_MS, &tmp);
  status |= VL53L4ED_RdWord(
              VL53L4ED_RESULT__OSC_CALIBRATE_VAL, &clock_pll);
  clock_pll = clock_pll & (uint16_t)0x3FF;
  clock_pll_factor = clock_pll_factor * (float_t)clock_pll;
  clock_pll = (uint16_t)clock_pll_factor;
  *p_inter_measurement_ms = (uint16_t)(tmp / (uint32_t)clock_pll);

  /* Get TimingBudget */
  status |= VL53L4ED_RdWord(0x0006, &osc_frequency);
  status |= VL53L4ED_RdWord(VL53L4ED_RANGE_CONFIG_A,
                            &range_config_macrop_high);

  macro_period_us = (uint32_t)((uint32_t)2304 * ((uint32_t)0x40000000
                                                 / (uint32_t)osc_frequency)) >> 6;
  ls_byte = (range_config_macrop_high & (uint32_t)0x00FF) << 4;
  ms_byte = (range_config_macrop_high & (uint32_t)0xFF00) >> 8;
  ms_byte = (uint32_t)0x04 - (ms_byte - (uint32_t)1) - (uint32_t)1;

  macro_period_us = macro_period_us * (uint32_t)16;
  *p_timing_budget_ms = (((ls_byte + (uint32_t)1) * (macro_period_us >> 6))
                         - ((macro_period_us >> 6) >> 1)) >> 12;

  if (ms_byte < (uint8_t)12) {
    *p_timing_budget_ms = (uint32_t)(*p_timing_budget_ms
                                     >> (uint8_t)ms_byte);
  }

  /* Mode continuous */
  if (tmp == (uint32_t)0) {
    *p_timing_budget_ms += (uint32_t)2500;
  }
  /* Mode autonomous */
  else {
    *p_timing_budget_ms *= (uint32_t)2;
    *p_timing_budget_ms += (uint32_t)4300;
  }

  *p_timing_budget_ms = *p_timing_budget_ms / (uint32_t)1000;

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetResult(
  VL53L4ED_ResultsData_t *p_result)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint16_t temp_16;
  uint8_t temp_8;
  uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3,
                             0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
                             255, 255, 11, 12
                           };

  status |= VL53L4ED_RdByte(VL53L4ED_RESULT__RANGE_STATUS,
                            &temp_8);
  temp_8 = temp_8 & (uint8_t)0x1F;
  if (temp_8 < (uint8_t)24) {
    temp_8 = status_rtn[temp_8];
  }
  p_result->range_status = temp_8;

  status |= VL53L4ED_RdWord(VL53L4ED_RESULT__SPAD_NB,
                            &temp_16);
  p_result->number_of_spad = temp_16 / (uint16_t) 256;

  status |= VL53L4ED_RdWord(VL53L4ED_RESULT__SIGNAL_RATE,
                            &temp_16);
  p_result->signal_rate_kcps = temp_16 * (uint16_t) 8;

  status |= VL53L4ED_RdWord(VL53L4ED_RESULT__AMBIENT_RATE,
                            &temp_16);
  p_result->ambient_rate_kcps = temp_16 * (uint16_t) 8;

  status |= VL53L4ED_RdWord(VL53L4ED_RESULT__SIGMA,
                            &temp_16);
  p_result->sigma_mm = temp_16 / (uint16_t) 4;

  status |= VL53L4ED_RdWord(VL53L4ED_RESULT__DISTANCE,
                            &temp_16);
  p_result->distance_mm = temp_16;

  p_result->signal_per_spad_kcps = p_result->signal_rate_kcps
                                   / p_result->number_of_spad;
  p_result->ambient_per_spad_kcps = p_result->ambient_rate_kcps
                                    / p_result->number_of_spad;

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SetOffset(
  int16_t OffsetValueInMm)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint16_t temp;

  temp = (uint16_t)((uint16_t)OffsetValueInMm * (uint16_t)4);

  status |= VL53L4ED_WrWord(VL53L4ED_RANGE_OFFSET_MM, temp);
  status |= VL53L4ED_WrWord(VL53L4ED_INNER_OFFSET_MM, (uint8_t)0x0);
  status |= VL53L4ED_WrWord(VL53L4ED_OUTER_OFFSET_MM, (uint8_t)0x0);
  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetOffset(
  int16_t *p_offset)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint16_t temp;

  status |= VL53L4ED_RdWord(VL53L4ED_RANGE_OFFSET_MM, &temp);

  temp = temp << 3;
  temp = temp >> 5;
  *p_offset = (int16_t)(temp);

  if (*p_offset > 1024) {
    *p_offset = *p_offset - 2048;
  }

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SetXtalk(
  uint16_t XtalkValueKcps)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_WrWord(
              VL53L4ED_XTALK_X_PLANE_GRADIENT_KCPS, 0x0000);
  status |= VL53L4ED_WrWord(
              VL53L4ED_XTALK_Y_PLANE_GRADIENT_KCPS, 0x0000);
  status |= VL53L4ED_WrWord(
              VL53L4ED_XTALK_PLANE_OFFSET_KCPS,
              (XtalkValueKcps << 9));

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetXtalk(
  uint16_t *p_xtalk_kcps)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  float_t tmp_xtalk;

  status |= VL53L4ED_RdWord(
              VL53L4ED_XTALK_PLANE_OFFSET_KCPS, p_xtalk_kcps);

  tmp_xtalk = (float_t) * p_xtalk_kcps / (float_t)512.0;
  *p_xtalk_kcps = (uint16_t)(round(tmp_xtalk));

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SetDetectionThresholds(
  uint16_t distance_low_mm,
  uint16_t distance_high_mm,
  uint8_t window)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_WrByte(VL53L4ED_SYSTEM__INTERRUPT, window);
  status |= VL53L4ED_WrWord(VL53L4ED_THRESH_HIGH, distance_high_mm);
  status |= VL53L4ED_WrWord(VL53L4ED_THRESH_LOW, distance_low_mm);
  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetDetectionThresholds(
  uint16_t *p_distance_low_mm,
  uint16_t *p_distance_high_mm,
  uint8_t *p_window)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_RdWord(VL53L4ED_THRESH_HIGH, p_distance_high_mm);
  status |= VL53L4ED_RdWord(VL53L4ED_THRESH_LOW, p_distance_low_mm);
  status |= VL53L4ED_RdByte(VL53L4ED_SYSTEM__INTERRUPT, p_window);
  *p_window = (*p_window & (uint8_t)0x7);

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SetSignalThreshold(
  uint16_t signal_kcps)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status |= VL53L4ED_WrWord(
              VL53L4ED_MIN_COUNT_RATE_RTN_LIMIT_MCPS, signal_kcps >> 3);
  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetSignalThreshold(
  uint16_t  *p_signal_kcps)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint16_t tmp = 0;

  status |= VL53L4ED_RdWord(
              VL53L4ED_MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
  *p_signal_kcps = tmp << 3;

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_SetSigmaThreshold(
  uint16_t  sigma_mm)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  if (sigma_mm > (uint16_t)((uint16_t)0xFFFF >> 2)) {
    status |= (uint8_t)VL53L4ED_ERROR_INVALID_ARGUMENT;
  } else {
    status |= VL53L4ED_WrWord(
                VL53L4ED_RANGE_CONFIG__SIGMA_THRESH, sigma_mm << 2);
  }

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_GetSigmaThreshold(
  uint16_t  *p_sigma_mm)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;

  status += VL53L4ED_RdWord(
              VL53L4ED_RANGE_CONFIG__SIGMA_THRESH, p_sigma_mm);
  *p_sigma_mm = *p_sigma_mm >> 2;

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_StartTemperatureUpdate(
)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint8_t tmp = 0, continue_loop = 1;
  uint16_t i = 0;

  status |= VL53L4ED_WrByte(
              VL53L4ED_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, (uint8_t)0x81);
  status |= VL53L4ED_WrByte(0x0B, (uint8_t)0x92);
  status |= VL53L4ED_WrByte(VL53L4ED_SYSTEM_START, (uint8_t)0x40);

  do {
    status |= VL53L4ED_CheckForDataReady(&tmp);
    if (tmp == (uint8_t)1) { /* Data ready */
      continue_loop = (uint8_t)0;
    } else if (i < (uint16_t)1000) {  /* Wait for answer */
      i++;
    } else { /* Timeout 1000ms reached */
      continue_loop = (uint8_t)0;
      status = (uint8_t)VL53L4ED_ERROR_TIMEOUT;
    }
    WaitMs(1);
  } while (continue_loop == (uint8_t)1);

  status |= VL53L4ED_ClearInterrupt();
  status |= VL53L4ED_StopRanging();

  status += VL53L4ED_WrByte(
              VL53L4ED_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
  status += VL53L4ED_WrByte(0x0B, 0);
  return status;
}
