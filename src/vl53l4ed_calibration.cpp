/**
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

/**
 * @file  vl53l4ed_calibration.c
 * @brief Calibration functions implementation
 */

#include <math.h>
#include "vl53l4ed_class.h"

VL53L4ED_Error VL53L4ED::VL53L4ED_CalibrateOffset(
  int16_t TargetDistInMm,
  int16_t *p_measured_offset_mm,
  int16_t nb_samples)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint8_t i, tmp, continue_loop;
  uint16_t j, tmpOff;
  int16_t AvgDistance = 0;
  VL53L4ED_ResultsData_t results;

  if (((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
      || ((TargetDistInMm < (int16_t)10)
          || (TargetDistInMm > (int16_t)1000))) {
    status |= (uint8_t)VL53L4ED_ERROR_INVALID_ARGUMENT;
  } else {
    status |= VL53L4ED_WrWord(VL53L4ED_RANGE_OFFSET_MM, 0x0);
    status |= VL53L4ED_WrWord(VL53L4ED_INNER_OFFSET_MM, 0x0);
    status |= VL53L4ED_WrWord(VL53L4ED_OUTER_OFFSET_MM, 0x0);

    /* Device heat loop (10 samples) */
    status |= VL53L4ED_StartRanging();
    for (i = 0; i < (uint8_t)10; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4ED_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4ED_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);
      status |= VL53L4ED_GetResult(&results);
      status |= VL53L4ED_ClearInterrupt();
    }
    status |= VL53L4ED_StopRanging();

    /* Device ranging */
    status |= VL53L4ED_StartRanging();
    for (i = 0; i < (uint8_t)nb_samples; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4ED_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4ED_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);

      status |= VL53L4ED_GetResult(&results);
      status |= VL53L4ED_ClearInterrupt();
      AvgDistance += (int16_t)results.distance_mm;
    }

    status |= VL53L4ED_StopRanging();
    AvgDistance = AvgDistance / nb_samples;
    *p_measured_offset_mm = (int16_t)TargetDistInMm - AvgDistance;
    tmpOff = (uint16_t) * p_measured_offset_mm * (uint16_t)4;
    status |= VL53L4ED_WrWord(VL53L4ED_RANGE_OFFSET_MM, tmpOff);
  }

  return status;
}

VL53L4ED_Error VL53L4ED::VL53L4ED_CalibrateXtalk(
  int16_t TargetDistInMm,
  uint16_t *p_measured_xtalk_kcps,
  int16_t nb_samples)
{
  VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
  uint8_t i, tmp, continue_loop;
  float_t AverageSignal = (float_t)0.0;
  float_t AvgDistance = (float_t)0.0;
  float_t AverageSpadNb = (float_t)0.0;
  float_t TargetDistance = (float_t)TargetDistInMm;
  float_t tmp_xtalk, CounterNbSamples = (float_t)0.0;
  VL53L4ED_ResultsData_t results;

  uint16_t calXtalk, j;

  *p_measured_xtalk_kcps = 0;
  if (((nb_samples < (int16_t)5) || (nb_samples > (int16_t)255))
      || ((TargetDistInMm < (int16_t)10)
          || (TargetDistInMm > (int16_t)5000))) {
    status |= (uint8_t)VL53L4ED_ERROR_INVALID_ARGUMENT;
  } else {
    /* Disable Xtalk compensation */
    status |= VL53L4ED_WrWord(
                VL53L4ED_XTALK_PLANE_OFFSET_KCPS, *p_measured_xtalk_kcps);

    /* Device heat loop (10 samples) */
    status |= VL53L4ED_StartRanging();
    for (i = 0; i < (uint8_t)10; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4ED_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4ED_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);
      status |= VL53L4ED_GetResult(&results);
      status |= VL53L4ED_ClearInterrupt();
    }
    status |= VL53L4ED_StopRanging();

    /* Device ranging loop */
    status |= VL53L4ED_StartRanging();
    for (i = 0; i < (uint8_t)nb_samples; i++) {
      tmp = (uint8_t)0;
      j = (uint16_t)0;
      continue_loop = (uint8_t)1;
      do {
        status |= VL53L4ED_CheckForDataReady(&tmp);
        if (tmp == (uint8_t)1) { /* Data ready */
          continue_loop = (uint8_t)0;
        } else if (j < (uint16_t)5000) { /* Wait for answer*/
          j++;
        } else { /* Timeout 5000ms reached */
          continue_loop = (uint8_t)0;
          status |= (uint8_t)VL53L4ED_ERROR_TIMEOUT;
        }
        WaitMs(1);
      } while (continue_loop == (uint8_t)1);

      status |= VL53L4ED_GetResult(&results);
      status |= VL53L4ED_ClearInterrupt();

      /* Discard invalid measurements and first frame */
      if ((results.range_status == (uint8_t)0)
          && (i > (uint8_t)0)) {
        AvgDistance += (float_t)results.distance_mm;
        AverageSpadNb += (float_t)results.number_of_spad;
        AverageSignal += (float_t)results.signal_rate_kcps;
        CounterNbSamples++;
      }
    }
    status |= VL53L4ED_StopRanging();

    if (CounterNbSamples == (float_t)0.0) {
      status = VL53L4ED_ERROR_XTALK_FAILED;
    } else {
      AvgDistance /= CounterNbSamples;
      AverageSpadNb /= CounterNbSamples;
      AverageSignal /= CounterNbSamples;

      tmp_xtalk = (float_t)1.0 - (AvgDistance / TargetDistance);
      tmp_xtalk *= (AverageSignal / AverageSpadNb);

      /* 127kcps is the max Xtalk value (65536/512) */
      if (tmp_xtalk > (float_t)127.0) {
        status = VL53L4ED_ERROR_XTALK_FAILED;
      } else {
        *p_measured_xtalk_kcps = (uint16_t)(round(tmp_xtalk));

        /* Send data to firmware */
        calXtalk = (uint16_t)(tmp_xtalk * (float_t)512.0);
        status |= VL53L4ED_WrWord(VL53L4ED_XTALK_PLANE_OFFSET_KCPS, calXtalk);
      }
    }
  }

  return status;
}
