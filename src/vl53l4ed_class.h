/**
 ******************************************************************************
 * @file    vl53l4ed_class.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    01 April 2024
 * @brief   Abstract Class for VL53L4ED sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef __VL53L4ED_CLASS_H
#define __VL53L4ED_CLASS_H

/* Includes ------------------------------------------------------------------*/
#include "Arduino.h"
#include "Wire.h"
#include "vl53l4ed_api.h"

#ifndef DEFAULT_I2C_BUFFER_LEN
  #ifdef BUFFER_LENGTH
    #define DEFAULT_I2C_BUFFER_LEN  BUFFER_LENGTH
  #else
    #define DEFAULT_I2C_BUFFER_LEN  32
  #endif
#endif

/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53L4ED sensor component
 */
class VL53L4ED {
  public:
    /** Constructor
     * @param[in] i2c device I2C to be used for communication
     * @param[in] xshut_pin pin
     */
    VL53L4ED(TwoWire *i2c, int xshut_pin = -1)
    {
      address = 0x52;
      dev_i2c = i2c;
      xshut = xshut_pin;
    }

    /** Destructor
     */
    virtual ~VL53L4ED() {}

    virtual int begin()
    {
      if (xshut >= 0) {
        pinMode(xshut, OUTPUT);
        digitalWrite(xshut, LOW);
      }
      return 0;
    }

    virtual int end()
    {
      if (xshut >= 0) {
        pinMode(xshut, INPUT);
      }
      return 0;
    }

    /*** Interface Methods ***/
    /*** High level API ***/
    /**
     * @brief       PowerOn the sensor
     * @return      void
     */
    virtual void VL53L4ED_On(void)
    {
      if (xshut >= 0) {
        digitalWrite(xshut, HIGH);
      }
      delay(10);
    }

    /**
     * @brief       PowerOff the sensor
     * @return      void
     */
    virtual void VL53L4ED_Off(void)
    {
      if (xshut >= 0) {
        digitalWrite(xshut, LOW);
      }
      delay(10);
    }

    /**
     * @brief  Initialize the sensor
     * @return (uint8_t) status : 0 if init_sensor is OK.
     */
    VL53L4ED_Error InitSensor(uint8_t newAddr = 0x52U)
    {
      VL53L4ED_Error status = VL53L4ED_ERROR_NONE;
      uint16_t sensor_id;
      VL53L4ED_Off();
      VL53L4ED_On();
      if (newAddr != address) {
        status = VL53L4ED_SetI2CAddress(newAddr);
      }

      if (status != VL53L4ED_ERROR_NONE) {
        return VL53L4ED_ERROR_TIMEOUT;
      }


      status = VL53L4ED_GetSensorId(&sensor_id);

      if (status != VL53L4ED_ERROR_NONE || (sensor_id != 0xecaa)) {

        return VL53L4ED_ERROR_TIMEOUT;
      }

      status = VL53L4ED_SensorInit();

      if (status != VL53L4ED_ERROR_NONE) {
        return VL53L4ED_ERROR_TIMEOUT;
      }

      return VL53L4ED_ERROR_NONE;
    }

    /* vl53l4ed_api.h */

    /**
     * @brief This function programs the software driver version.
     * @param (VL53L4ED_Version_t) pVersion : Pointer of structure, containing the
     * software version.
     * @return (VL53L4ED_ERROR) status : 0 if SW version is OK.
     */

    VL53L4ED_Error VL53L4ED_GetSWVersion(VL53L4ED_Version_t *pVersion);

    /**
     * @brief This function sets a new I2C address to a sensor. It can be used for
     * example when multiple sensors share the same I2C bus.
     * @param (uint8_t) new_address : New I2C address.
     * @return (VL53L4ED_ERROR) status : 0 if I2C address has been correctly
     * programmed.
     */

    VL53L4ED_Error VL53L4ED_SetI2CAddress(uint8_t new_address);

    /**
     * @brief This function is used to get the sensor id of VL53L4ED. The sensor id
     * should be 0xEBAA.
     * @param (uint16_t) *p_id : Sensor id.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_GetSensorId(uint16_t *p_id);

    /**
     * @brief This function is used to initialize the sensor.
     * @return (VL53L4ED_ERROR) status : 0 if init is OK.
     */

    VL53L4ED_Error VL53L4ED_SensorInit();

    /**
     * @brief This function clears the interrupt. It needs to be called after a
     * ranging data reading to arm the interrupt for the next data ready event.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_ClearInterrupt();

    /**
     * @brief This function starts a ranging session. The ranging operation is
     * continuous. The clear interrupt has to be done after each get data to allow
     * the interrupt to raise when the next data is ready.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_StartRanging();

    /**
     * @brief This function stops the ranging in progress.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_StopRanging();

    /**
     * @brief This function check if a new data is available by polling a dedicated
     * register.
     * @param (uint8_t) *p_is_data_ready : Pointer containing a flag to know if a
     * data is ready : 0 = no data ready, 1 = data ready.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_CheckForDataReady(uint8_t *p_is_data_ready);

    /**
     * @brief This function sets new range timing. Timing are composed of
     * TimingBudget and InterMeasurement. TimingBudget represents the timing during
     * VCSEL enabled, and InterMeasurement the time between two measurements.
     * The sensor can have different ranging mode depending of the configuration,
     * please refer to the user manual for more information.
     * @param (uint32_t) timing_budget_ms :  New timing budget in ms. Value can be
     * between 10ms and 200ms. Default is 50ms.
     * @param (uint32_t) inter_measurement_ms :  New inter-measurement in ms. If the
     * value is equal to 0, the ranging period is defined by the timing budget.
     * Otherwise, inter-measurement must be > timing budget. When all the timing
     * budget is consumed, the device goes in low power mode until inter-measurement
     * is done.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_SetRangeTiming(
      uint32_t timing_budget_ms,
      uint32_t inter_measurement_ms);

    /**
     * @brief This function gets the current range timing. Timing are composed of
     * TimingBudget and InterMeasurement. TimingBudget represents the timing during
     * VCSEL enabled, and InterMeasurement the time between two measurements.
     * The sensor can have different ranging mode depending of the configuration,
     * please refer to the user manual for more information.
     * @param (uint32_t) *p_timing_budget_ms :  Pointer containing the current
     * timing budget in ms.
     * @param (uint32_t) *p_inter_measurement_ms :  Pointer containing the current
     * inter-measurement in ms.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_GetRangeTiming(
      uint32_t *p_timing_budget_ms,
      uint32_t *p_inter_measurement_ms);

    /**
     * @brief This function gets the results reported by the sensor.
     * @param (VL53L4ED_ResultsData_t) *pResult :  Pointer of structure, filled with the
     * ranging results.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_GetResult(VL53L4ED_ResultsData_t *pResult);

    /**
     * @brief This function sets a new offset correction in mm. Offset corresponds
     * to the difference in millimeters between real distance and measured distance.
     * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The minimum
     *  value is -1024mm and maximum is 1023mm.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_SetOffset(int16_t OffsetValueInMm);

    /**
     * @brief This function gets the current offset correction in mm. Offset
     * corresponds to the difference in millimeters between real distance and
     * measured distance.
     * @param (int16_t) OffsetValueInMm :  Offset value in millimeters. The minimum
     *  value is -1024mm and maximum is 1023mm.
     * @return (uint8_t) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_GetOffset(int16_t *Offset);

    /**
     * @brief This function sets a new Xtalk value in kcps. Xtalk represents the
     * correction to apply to the sensor when a protective coverglass is placed
     * at the top of the sensor.
     * @param (uint16_t) XtalkValueKcps : New xtalk value in kcps. The default
     * value is 0 kcps (no coverglass). Minimum is 0 kcps , and maximum is 128
     * kcps.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_SetXtalk(uint16_t XtalkValueKcps);

    /**
     * @brief This function gets the current Xtalk value in kcps. Xtalk represents
     * the correction to apply to the sensor when a protective coverglass is placed
     * at the top of the sensor.
     * @param (uint16_t) p_xtalk_kcps : Pointer of current xtalk value in kcps.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_GetXtalk(uint16_t *p_xtalk_kcps);

    /**
     * @brief This function sets new detection thresholds. The detection
     * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only
     * when a condition on distance is reach. Example:
     * VL53L4ED_SetDistanceThreshold(dev,100,300,0): Below 100 mm
     * VL53L4ED_SetDistanceThreshold(dev,100,300,1): Above 300 mm
     * VL53L4ED_SetDistanceThreshold(dev,100,300,2): Below 100mm or above 300mm
     * VL53L4ED_SetDistanceThreshold(dev,100,300,3): Above 100mm or below 300mm
     * @param (uint16_t) distance_low_mm : Low distance threshold in millimeters.
     * @param (uint16_t) distance_high_mm : High distance threshold in millimeters.
     * @param (uint8_t) window : Interrupt windows (0=below low threshold;
     * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_SetDetectionThresholds(
      uint16_t distance_low_mm,
      uint16_t distance_high_mm,
      uint8_t window);

    /**
     * @brief This function gets the current detection thresholds. The detection
     * thresholds can be programmed to generate an interrupt on pin 7 (GPIO1), only
     * when a condition on distance is reach.
     * @param (uint16_t) *p_distance_low_mm : Pointer of low distance threshold in
     * millimeters.
     * @param (uint16_t) *p_distance_high_mm : Pointer of high distance threshold in
     * millimeters.
     * @param (uint8_t) *p_window : Interrupt windows (0=below low threshold;
     * 1=above high threshold; 2=out of low/high windows; 3=in low/high windows)
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_GetDetectionThresholds(
      uint16_t *p_distance_low_mm,
      uint16_t *p_distance_high_mm,
      uint8_t *p_window);

    /**
     * @brief This function sets a new signal threshold in kcps. If a
     * target has a lower signal as the programmed value, the result status in
     * structure 'VL53L4ED_ResultsData_t' will be equal to 2.
     * @param (uint16_t) signal_kcps : New signal threshold in kcps. The default
     * value is 1024 kcps. Minimum is 0 kcps (no threshold), and maximum is 16384
     * kcps.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_SetSignalThreshold(uint16_t signal_kcps);

    /**
     * @brief This function returns the current signal threshold in kcps. If a
     * target has a lower signal as the programmed value, the result status in
     * structure 'VL53L4ED_ResultsData_t' will be equal to 2.
     * @param (uint16_t) *p_signal_kcps : Pointer of signal threshold in kcps.
     * @return (VL53L4ED_ERROR) status : 0 if OK.
     */

    VL53L4ED_Error VL53L4ED_GetSignalThreshold(uint16_t *p_signal_kcps);

    /**
     * @brief This function programs a new sigma threshold. The sigma corresponds to
     * the standard deviation of the returned pulse. If the computed sigma is above
     * the programmed value, the result status in structure 'VL53L4ED_ResultsData_t'
     * will be equal to 1.
     * @param (uint16_t) sigma_mm : New sigma threshold in mm. The default value is
     * 15mm. Minimum is 0mm (not threshold), and maximum is 16383mm.
     * @return (VL53L4ED_ERROR) status : 0 if programming is or 255 if value is too
     * high.
     */

    VL53L4ED_Error VL53L4ED_SetSigmaThreshold(uint16_t  sigma_mm);

    /**
     * @brief This function gets the current sigma threshold. The sigma corresponds
     * to the standard deviation of the returned pulse. If the computed sigma is
     * above the programmed value, the result status in structure
     * 'VL53L4ED_ResultsData_t' will be equal to 1.
     * @param (uint16_t) *p_sigma_mm : Current sigma threshold in mm.
     * @return (VL53L4ED_ERROR) status : 0 if programming is OK.
     */

    VL53L4ED_Error VL53L4ED_GetSigmaThreshold(uint16_t  *p_sigma_mm);

    /**
     * @brief This function can be called when the temperature might have changed by
     * more than 8 degrees Celsius. The function can only be used if the sensor is
     * not ranging, otherwise, the ranging needs to be stopped using function
     * 'VL53L4ED_StopRanging()'. After calling this function, the ranging can
     * restart normally.
     * @return (VL53L4ED_ERROR) status : 0 if update is OK.
     */

    VL53L4ED_Error VL53L4ED_StartTemperatureUpdate();

    /* Calibration APIs */

    /**
     * @brief This function can be used to perform an offset calibration. Offset
     * corresponds to the difference in millimeters between real distance and
     * measured distance. ST recommend to perform offset at 100m, on a grey17%
     * reflective target, but any other distance and reflectance can be used.
     * The function returns the offset value found and programs the offset
     * compensation into the device.
     * @param (int16_t) TargetDistInMm : Real distance between the sensor and the
     * target in millimeters. ST recommend 100mm. Min distance is 10mm and max is
     * 1000mm.
     * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A higher
     * number of samples increases the accuracy, but it also takes more time. ST
     * recommend to use at least 10 samples.
     * @return (VL53L4ED_ERROR) status : 0 if OK, or 255 if something occurred (e.g
     * invalid nb of samples).
     */

    VL53L4ED_Error VL53L4ED_CalibrateOffset(
      int16_t TargetDistInMm,
      int16_t *p_measured_offset_mm,
      int16_t nb_samples);

    /**
     * @brief This function can be used to perform a Xtalk calibration. Xtalk
     * represents the correction to apply to the sensor when a protective coverglass
     * is placed at the top of the sensor. The distance for calibration depends of
     * the coverglass, it needs to be characterized. Please refer to the User Manual
     * for more information.
     * The function returns the Xtalk value found and programs the Xtalk
     * compensation into the device.
     * @param uint16_t) TargetDistInMm : Real distance between the sensor and the
     * target in millimeters. This distance needs to be characterized, as described
     * into the User Manual.
     * @param (int16_t) nb_samples : Number of samples (between 5 and 255). A higher
     * number of samples increases the accuracy, but it also takes more time. ST
     * recommend to use at least 10 samples.
     * @return (VL53L4ED_ERROR) status : 0 if OK, or 255 if something occurred (e.g
     * invalid nb of samples).
     */

    VL53L4ED_Error VL53L4ED_CalibrateXtalk(
      int16_t TargetDistInMm,
      uint16_t *p_measured_xtalk_kcps,
      int16_t nb_samples);

    uint8_t VL53L4ED_WrByte(uint16_t index, uint8_t data);
    uint8_t VL53L4ED_WrWord(uint16_t index, uint16_t data);
    uint8_t VL53L4ED_WrDWord(uint16_t index, uint32_t data);
    uint8_t VL53L4ED_RdByte(uint16_t index, uint8_t *data);
    uint8_t VL53L4ED_RdWord(uint16_t index, uint16_t *data);
    uint8_t VL53L4ED_RdDWord(uint16_t index, uint32_t *data);
    uint8_t WaitMs(uint32_t TimeMs);

  protected:
    uint8_t   address;
    TwoWire *dev_i2c;
    int xshut;

    uint8_t _I2CRead(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
    {
      int status = 0;
      uint8_t buffer[2];

      // Loop until the port is transmitted correctly
      do {
        dev_i2c->beginTransmission((uint8_t)((address >> 1) & 0x7F));

        // Target register address for transfer
        buffer[0] = (uint8_t)(RegisterAddress >> 8);
        buffer[1] = (uint8_t)(RegisterAddress & 0xFF);
        dev_i2c->write(buffer, 2);

        status = dev_i2c->endTransmission(false);

        // Fix for some STM32 boards
        // Reinitialize the i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
        if (status) {
          dev_i2c->end();
          dev_i2c->begin();
        }
#endif
        // End of fix

      } while (status != 0);

      uint32_t i = 0;
      if (size > DEFAULT_I2C_BUFFER_LEN) {
        while (i < size) {
          // If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
          // else the remaining number of bytes
          uint8_t current_read_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);
          dev_i2c->requestFrom(((uint8_t)((address >> 1) & 0x7F)),
                               current_read_size);
          while (dev_i2c->available()) {
            p_values[i] = dev_i2c->read();
            i++;
          }
        }
      } else {
        dev_i2c->requestFrom(((uint8_t)((address >> 1) & 0x7F)), size);
        while (dev_i2c->available()) {
          p_values[i] = dev_i2c->read();
          i++;
        }
      }

      return i != size;
    }

    uint8_t _I2CWrite(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
    {
      uint32_t i = 0;
      uint8_t buffer[2];

      while (i < size) {
        // If still more than DEFAULT_I2C_BUFFER_LEN bytes to go, DEFAULT_I2C_BUFFER_LEN,
        // else the remaining number of bytes
        size_t current_write_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);

        dev_i2c->beginTransmission((uint8_t)((address >> 1) & 0x7F));

        // Target register address for transfer
        buffer[0] = (uint8_t)((RegisterAddress + i) >> 8);
        buffer[1] = (uint8_t)((RegisterAddress + i) & 0xFF);
        dev_i2c->write(buffer, 2);
        if (dev_i2c->write(p_values + i, current_write_size) == 0) {
          return 1;
        } else {
          i += current_write_size;
          if (size - i) {
            // Flush buffer but do not send stop bit so we can keep going
            dev_i2c->endTransmission(false);
          }
        }
      }

      return dev_i2c->endTransmission(true);
    }
};

#endif /* __VL53L4ED_CLASS_H */
