/*------------------------------------------------------------------------------

  LIDARLite_v3HP Arduino Library
  LIDARLite_v3HP_v3HP.h

  This library provides quick access to all the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2018 Garmin Ltd. or its subsidiaries.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

------------------------------------------------------------------------------*/
#ifndef LIDARLite_v3HP_h
#define LIDARLite_v3HP_h

#define LIDARLITE_ADDR_DEFAULT 0x62
#define LIDARLITE_BUFFER_DEPTH 8 // device only has 3 consecutive registers maximum so this is plenty

#include "mbed.h"

class LIDARLite_v3HP {

public:
    LIDARLite_v3HP(I2C *i2c);

    LIDARLite_v3HP(I2C *i2c, uint8_t &addr);

    /**
      Configure

      Selects one of several preset configurations.

      Parameters
      ------------------------------------------------------------------------------
      configuration:  Default 0.
        0: Default mode, balanced performance.
        1: Short range, high speed. Uses 0x1d maximum acquisition count.
        2: Default range, higher speed short range. Turns on quick termination
            detection for faster measurements at short range (with decreased
            accuracy)
        3: Maximum range. Uses 0xff maximum acquisition count.
        4: High sensitivity detection. Overrides default valid measurement detection
            algorithm, and uses a threshold value for high sensitivity and noise.
        5: Low sensitivity detection. Overrides default valid measurement detection
            algorithm, and uses a threshold value for low sensitivity and noise.
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    */
    void configure(const uint8_t &configuration = 0, const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Set I2C Address

      Set Alternate I2C Device Address. See Operation Manual for additional info.

      Parameters
      ------------------------------------------------------------------------------
      newAddress: desired secondary I2C device address
      disableDefault: a non-zero value here means the default 0x62 I2C device
        address will be disabled.
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    */
    void setI2Caddr(const uint8_t &newAddress, uint8_t &disableDefault, const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Take Range

      Initiate a distance measurement by writing to register 0x00.

      Parameters
      ------------------------------------------------------------------------------
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    */
    void takeRange(const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Read Distance

      Read and return result of distance measurement.

      Process
      ------------------------------------------------------------------------------
      1.  Read two bytes from register 0x8f and save
      2.  Shift the first value from 0x8f << 8 and add to second value from 0x8f.
          The result is the measured distance in centimeters.

      Parameters
      ------------------------------------------------------------------------------
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    */
    uint16_t readDistance(const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Wait for Busy Flag

      Blocking function to wait until the Lidar Lite's internal busy flag goes low

      Parameters
      ------------------------------------------------------------------------------
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    */
    void waitForBusy(const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Get Busy Flag

      Read BUSY flag from device registers. Function will return 0x00 if not busy.

      Parameters
      ------------------------------------------------------------------------------
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
     */
    uint8_t getBusyFlag(const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Reset Reference Filter

      In some scenarios, power-on transients in the LIDAR-Lite v3HP can result in
      initial measurements that may be a few centimeters short of actual distance.
      This symptom will eventually rectify itself after a few hundred measurements.
      This symptom can also be rectified more quickly by resetting the unit's internal
      reference filter. The process here illustrates how to disable the internal
      reference filter, trigger a measurement (forcing re-init of the reference
      filter), and then re-enable the filter.

      Process
      ------------------------------------------------------------------------------
      1.  Disable the LIDAR-Lite reference filter
      2.  Set reference integration count to max
      3.  Trigger a measurement
      4.  Restore reference integration count
      5.  Re-enable reference filter

      Parameters
      ------------------------------------------------------------------------------
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
     */
    void resetReferenceFilter(const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Write

      Perform I2C write to device. The I2C peripheral in the LidarLite v3 HP
      will receive multiple bytes in one I2C transmission. The first byte is
      always the register address. The the bytes that follow will be written
      into the specified register address first and then the internal address
      in the Lidar Lite will be auto-incremented for all following bytes.

      Parameters
      ------------------------------------------------------------------------------
      regAddr:   register address to write to
      dataBytes: pointer to array of bytes to write
      numBytes:  number of bytes in 'dataBytes' array to write
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
    */
    void write(const uint8_t &regAddr, uint8_t *dataBytes, const uint16_t &numBytes, const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Read

      Perform I2C read from device.  The I2C peripheral in the LidarLite v3 HP
      will send multiple bytes in one I2C transmission. The register address must
      be set up by a previous I2C write. The bytes that follow will be read
      from the specified register address first and then the internal address
      pointer in the Lidar Lite will be auto-incremented for following bytes.

      Will detect an unresponsive device and report the error over serial.

      Parameters
      ------------------------------------------------------------------------------
      regAddr:   register address to write to
      dataBytes: pointer to array of bytes to write
      numBytes:  number of bytes in 'dataBytes' array to write
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    */
    void read(const uint8_t &regAddr, uint8_t *dataBytes, const uint16_t &numBytes, const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

    /**
      Correlation Record To Serial

      The correlation record used to calculate distance can be read from the device.
      It has a bipolar wave shape, transitioning from a positive going portion to a
      roughly symmetrical negative going pulse. The point where the signal crosses
      zero represents the effective delay for the reference and return signals.

      Process
      ------------------------------------------------------------------------------
      1.  Take a distance reading (there is no correlation record without at least
          one distance reading being taken)
      2.  Set test mode select by writing 0x07 to register 0x40
      3.  For as many readings as you want to take (max is 1024)
          1.  Read two bytes from 0x52
          2.  The Low byte is the value from the record
          3.  The high byte is the sign from the record

      Parameters
      ------------------------------------------------------------------------------
      separator: the separator between serial data words
      numberOfReadings: Default: 1024. Maximum of 1024
      lidarliteAddress: Default 0x62. Fill in new address here if changed. See
        operating manual for instructions.
    */
    void correlationRecordToSerial(const uint16_t &numberOfReadings = 1024, const uint8_t &lidarliteAddress = LIDARLITE_ADDR_DEFAULT);

private:
    uint8_t _addr;
    I2C     *_i2c;
    uint8_t *_buffer;
};

#endif
