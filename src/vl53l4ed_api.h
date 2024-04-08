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

#ifndef VL53L4ED_API_H_
#define VL53L4ED_API_H_

/**
 *  @brief Driver version
 */

#define VL53L4ED_IMPLEMENTATION_VER_MAJOR       1
#define VL53L4ED_IMPLEMENTATION_VER_MINOR       0
#define VL53L4ED_IMPLEMENTATION_VER_BUILD       0
#define VL53L4ED_IMPLEMENTATION_VER_REVISION    0

/**
 *  @brief Driver error type
 */

typedef uint8_t VL53L4ED_Error;

#define VL53L4ED_ERROR_NONE         ((uint8_t)0U)
#define VL53L4ED_ERROR_XTALK_FAILED     ((uint8_t)253U)
#define VL53L4ED_ERROR_INVALID_ARGUMENT   ((uint8_t)254U)
#define VL53L4ED_ERROR_TIMEOUT        ((uint8_t)255U)

/**
 *  @brief Inner Macro for API. Not for user, only for development.
 */

#define VL53L4ED_SOFT_RESET     ((uint16_t)0x0000))
#define VL53L4ED_I2C_SLAVE__DEVICE_ADDRESS      ((uint16_t)0x0001)
#define VL53L4ED_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND  ((uint16_t)0x0008)
#define VL53L4ED_XTALK_PLANE_OFFSET_KCPS ((uint16_t)0x0016)
#define VL53L4ED_XTALK_X_PLANE_GRADIENT_KCPS     ((uint16_t)0x0018)
#define VL53L4ED_XTALK_Y_PLANE_GRADIENT_KCPS     ((uint16_t)0x001A)
#define VL53L4ED_RANGE_OFFSET_MM     ((uint16_t)0x001E)
#define VL53L4ED_INNER_OFFSET_MM     ((uint16_t)0x0020)
#define VL53L4ED_OUTER_OFFSET_MM     ((uint16_t)0x0022)
#define VL53L4ED_GPIO_HV_MUX__CTRL      ((uint16_t)0x0030)
#define VL53L4ED_GPIO__TIO_HV_STATUS    ((uint16_t)0x0031)
#define VL53L4ED_SYSTEM__INTERRUPT  ((uint16_t)0x0046)
#define VL53L4ED_RANGE_CONFIG_A     ((uint16_t)0x005E)
#define VL53L4ED_RANGE_CONFIG_B      ((uint16_t)0x0061)
#define VL53L4ED_RANGE_CONFIG__SIGMA_THRESH     ((uint16_t)0x0064)
#define VL53L4ED_MIN_COUNT_RATE_RTN_LIMIT_MCPS    ((uint16_t)0x0066)
#define VL53L4ED_INTERMEASUREMENT_MS ((uint16_t)0x006C)
#define VL53L4ED_THRESH_HIGH    ((uint16_t)0x0072)
#define VL53L4ED_THRESH_LOW     ((uint16_t)0x0074)
#define VL53L4ED_SYSTEM__INTERRUPT_CLEAR        ((uint16_t)0x0086)
#define VL53L4ED_SYSTEM_START     ((uint16_t)0x0087)
#define VL53L4ED_RESULT__RANGE_STATUS   ((uint16_t)0x0089)
#define VL53L4ED_RESULT__SPAD_NB   ((uint16_t)0x008C)
#define VL53L4ED_RESULT__SIGNAL_RATE   ((uint16_t)0x008E)
#define VL53L4ED_RESULT__AMBIENT_RATE   ((uint16_t)0x0090)
#define VL53L4ED_RESULT__SIGMA   ((uint16_t)0x0092)
#define VL53L4ED_RESULT__DISTANCE   ((uint16_t)0x0096)

#define VL53L4ED_RESULT__OSC_CALIBRATE_VAL      ((uint16_t)0x00DE)
#define VL53L4ED_FIRMWARE__SYSTEM_STATUS        ((uint16_t)0x00E5)
#define VL53L4ED_IDENTIFICATION__MODEL_ID       ((uint16_t)0x010F)

/**
 * @brief
 *
 */

typedef struct {
  uint8_t      major;    /*!< major number */
  uint8_t      minor;    /*!< minor number */
  uint8_t      build;    /*!< build number */
  uint32_t     revision; /*!< revision number */
} VL53L4ED_Version_t;

/**
 *  @brief Packed reading results type
 */

typedef struct {

  /* Status of measurements. If the status is equal to 0, the data are valid*/
  uint8_t range_status;
  /* Measured distance in millimeters */
  uint16_t distance_mm;
  /* Ambient noise in kcps */
  uint16_t ambient_rate_kcps;
  /* Ambient noise in kcps/SPAD */
  uint16_t ambient_per_spad_kcps;
  /* Measured signal of the target in kcps */
  uint16_t signal_rate_kcps;
  /* Measured signal of the target in kcps/SPAD */
  uint16_t signal_per_spad_kcps;
  /* Number of SPADs enabled */
  uint16_t number_of_spad;
  /* Estimated measurements std deviation in mm */
  uint16_t sigma_mm;
} VL53L4ED_ResultsData_t;

#endif  //VL53L4ED_API_H_
