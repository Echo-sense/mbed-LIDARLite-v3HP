/*------------------------------------------------------------------------------

  LIDARLite_v3HP mbed Library
  LIDARLite_v3HP.h

  This library provides quick access to all the basic functions of LIDAR-Lite
  via the mbed interface. Additionally, it can provide a user of any
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

#define LIDARLITE_ACQ_COMMAND 0x00
#define LIDARLITE_STATUS 0x01
#define LIDARLITE_SIG_COUNT_VAL 0x02
#define LIDARLITE_ACQ_CONFIG_REG 0x04
#define LIDARLITE_LEGACY_RESET_EN 0x06
#define LIDARLITE_SIGNAL_STRENGTH 0x0E
#define LIDARLITE_FULL_DELAY_HIGH 0x0F
#define LIDARLITE_FULL_DELAY_LOW 0x10
#define LIDARLITE_REF_COUNT_VAL 0x12
#define LIDARLITE_UNIT_ID_HIGH 0x16
#define LIDARLITE_UNIT_ID_LOW 0x17
#define LIDARLITE_I2C_ID_HIGH 0x18
#define LIDARLITE_I2C_ID_LOW 0x19
#define LIDARLITE_I2C_SEC_ADDR 0x1A
#define LIDARLITE_THRESHOLD_BYPASS 0x1C
#define LIDARLITE_I2C_CONFIG 0x1E
#define LIDARLITE_PEAK_STACK_HIGH 0x26
#define LIDARLITE_PEAK_STACK_LOW 0x27
#define LIDARLITE_COMMAND 0x40
#define LIDARLITE_HEALTH  0x48
#define LIDARLITE_CORR_DATA 0x52
#define LIDARLITE_CORR_DATA_SIGN 0x53
#define LIDARLITE_POWER_CONTROL 0x65


#include "mbed.h"

/**
 * @brief Class for interfacing with the LIDARLite v3HP sensor
 */
class LIDARLite_v3HP {

public:

    /**
     * @brief Constructs a new LIDARLite_v3HP object
     *
     * @param i2c I2C bus to communicate over
     */
    LIDARLite_v3HP(I2C *i2c);

    /**
     * @brief Constructs a new LIDARLite_v3HP object with a custom address
     *
     * The address is automatically written to the device
     *
     * @param i2c  I2C bus to communicate over
     * @param addr 7-bit I2C address to use
     */
    LIDARLite_v3HP(I2C *i2c, uint8_t &addr);

    /**
     * @brief Selects one of several preset configurations.
     *
     * @param configuration configuration value. Default 0.\n
     *  0: Default mode, balanced performance.\n
     *  1: Short range, high speed. Uses 0x1d maximum acquisition count.\n
     *  2: Default range, higher speed short range. Turns on quick termination
     *      detection for faster measurements at short range (with decreased
     *      accuracy).\n
     *  3: Maximum range. Uses 0xff maximum acquisition count.\n
     *  4: High sensitivity detection. Overrides default valid measurement detection
     *      algorithm, and uses a threshold value for high sensitivity and noise.\n
     *  5: Low sensitivity detection. Overrides default valid measurement detection
     *      algorithm, and uses a threshold value for low sensitivity and noise.\n
     */
    void configure(const uint8_t &configuration = 0, const uint8_t &mode = 0);

    /**
     * @brief Set I2C Address
     *
     * Set Alternate I2C Device Address. See Operation Manual for additional info.
     *
     * @param newAddress desired secondary I2C device address
     * @param disableDefault should the default 0x62 I2C device
     * address be disabled.
     */
    void setI2Caddr(const uint8_t &newAddress, bool disableDefault);

    /**
     * @brief Get currently configured I2C Address
     * @return current I2C address
     */
    uint8_t getI2Caddr();

    /**
     * @brief Initiate a distance measurement
     *
     * Initiates a distance measurement by writing a 1 to register 0x00
     */
    void takeRange();

    /**
     * @brief Read and return result of distance measurement.
     *
     * Process:
     * 1. Read two bytes from register 0x8f and save
     * 2. Shift the first value from 0x8f << 8 and add to second value from 0x8f.
     *    The result is the measured distance in centimeters.
     *
     * @return distance measurement in centimeters
     */
    uint16_t readDistance();

    /**
     * @brief Wait for busy flag to clear
     *
     * Blocking function to wait until the Lidar Lite's internal busy flag goes low
     */
    void waitForBusy();

    /**
     * @brief Get value of the busy flag
     *
     * Read BUSY flag from device registers. Function will return 0x00 if not busy.
     *
     * @return 0 if not busy, 1 if busy
     */
    uint8_t getBusyFlag();

    /**
     * @brief Reset Reference Filter
     *
     * In some scenarios, power-on transients in the LIDAR-Lite v3HP can result in
     * initial measurements that may be a few centimeters short of actual distance.
     * This symptom will eventually rectify itself after a few hundred measurements.
     * This symptom can also be rectified more quickly by resetting the unit's internal
     * reference filter. The process here illustrates how to disable the internal
     * reference filter, trigger a measurement (forcing re-init of the reference
     * filter), and then re-enable the filter.
     *
     * Process:
     * 1. Disable the LIDAR-Lite reference filter
     * 2. Set reference integration count to max
     * 3. Trigger a measurement
     * 4. Restore reference integration count
     * 5. Re-enable reference filter
     */
    void resetReferenceFilter();

    /**
     * @brief print correlation record to serial
     *
     * The correlation record used to calculate distance can be read from the device.
     * It has a bipolar wave shape, transitioning from a positive going portion to a
     * roughly symmetrical negative going pulse. The point where the signal crosses
     * zero represents the effective delay for the reference and return signals.
     *
     * Process:
     * 1. Take a distance reading (there is no correlation record without at least
     *    one distance reading being taken)
     * 2. Set test mode select by writing 0x07 to register 0x40
     * 3. For as many readings as you want to take (max is 1024)
     *    1.  Read two bytes from 0x52
     *    2.  The Low byte is the value from the record
     *    3.  The high byte is the sign from the record
     *
     * @param numberOfReadings Default: 1024. Maximum of 1024
    */
    void correlationRecordToSerial(const uint16_t &numberOfReadings = 1024);

    /**
     * @brief Write device register over I2C
     *
     * Perform I2C write to device. The I2C peripheral in the LidarLite v3 HP
     * will receive multiple bytes in one I2C transmission. The first byte is
     * always the register address. The the bytes that follow will be written
     * into the specified register address first and then the internal address
     * in the Lidar Lite will be auto-incremented for all following bytes.
     *
     * @param regAddr   register address to write to
     * @param dataBytes pointer to array of bytes to write
     * @param numBytes  number of bytes in 'dataBytes' array to write
    */
    void write(const uint8_t &regAddr, uint8_t *dataBytes, const uint16_t &numBytes);

    /**
     * @brief Read device register over I2C
     *
     * Perform I2C read from device.  The I2C peripheral in the LidarLite v3 HP
     * will send multiple bytes in one I2C transmission. The register address must
     * be set up by a previous I2C write. The bytes that follow will be read
     * from the specified register address first and then the internal address
     * pointer in the Lidar Lite will be auto-incremented for following bytes.
     *
     * Will detect an unresponsive device and report the error over serial.
     *
     * @param regAddr   register address to read from
     * @param dataBytes pointer to array of bytes to read into
     * @param numBytes  number of bytes to read
    */
    void read(const uint8_t &regAddr, uint8_t *dataBytes, const uint16_t &numBytes);

private:
    uint8_t _addr7;
    uint8_t _addr8;
    I2C     *_i2c;
    uint8_t *_buffer;
};

#endif
