/*------------------------------------------------------------------------------

  LIDARLite_v3HP mbed Library
  LIDARLite_v3HP.cpp

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

#include "LIDARLite_v3HP.h"
#include "mbed.h"

LIDARLite_v3HP::LIDARLite_v3HP(I2C *i2c) {
    _i2c    = i2c;
    _addr7  = LIDARLITE_ADDR_DEFAULT;
    _addr8  = LIDARLITE_ADDR_DEFAULT << 1;
    _buffer = (uint8_t *) malloc(LIDARLITE_BUFFER_DEPTH);
} /* LIDARLite_v3HP::LIDARLite_v3HP */

LIDARLite_v3HP::LIDARLite_v3HP(I2C *i2c, uint8_t &addr) {
    _i2c    = i2c;
    _addr7  = LIDARLITE_ADDR_DEFAULT;
    _addr8  = LIDARLITE_ADDR_DEFAULT << 1;
    _buffer = (uint8_t *) malloc(LIDARLITE_BUFFER_DEPTH);

    this->setI2Caddr(addr, true);
} /* LIDARLite_v3HP::LIDARLite_v3HP */

void LIDARLite_v3HP::configure(const uint8_t &configuration) {
    uint8_t sigCountMax;
    uint8_t acqConfigReg;
    uint8_t refCountMax;
    uint8_t thresholdBypass;

    switch (configuration) {
    default:
    case 0: // Default mode, balanced performance
        sigCountMax     = 0x80; // Default
        acqConfigReg    = 0x08; // Default
        refCountMax     = 0x05; // Default
        thresholdBypass = 0x00; // Default
        break;

    case 1: // Short range, high speed
        sigCountMax     = 0x1d;
        acqConfigReg    = 0x08; // Default
        refCountMax     = 0x03;
        thresholdBypass = 0x00; // Default
        break;

    case 2: // Default range, higher speed short range
        sigCountMax     = 0x80; // Default
        acqConfigReg    = 0x00;
        refCountMax     = 0x03;
        thresholdBypass = 0x00; // Default
        break;

    case 3: // Maximum range
        sigCountMax     = 0xff;
        acqConfigReg    = 0x08; // Default
        refCountMax     = 0x05; // Default
        thresholdBypass = 0x00; // Default
        break;

    case 4: // High sensitivity detection, high erroneous measurements
        sigCountMax     = 0x80; // Default
        acqConfigReg    = 0x08; // Default
        refCountMax     = 0x05; // Default
        thresholdBypass = 0x80;
        break;

    case 5: // Low sensitivity detection, low erroneous measurements
        sigCountMax     = 0x80; // Default
        acqConfigReg    = 0x08; // Default
        refCountMax     = 0x05; // Default
        thresholdBypass = 0xb0;
        break;

    case 6: // Short range, high speed, higher error
        sigCountMax     = 0x04;
        acqConfigReg    = 0x01; // turn off short_sig, mode pin = status output mode
        refCountMax     = 0x03;
        thresholdBypass = 0x00;
        break;
    }

    write(0x02, &sigCountMax, 1);
    write(0x04, &acqConfigReg, 1);
    write(0x12, &refCountMax, 1);
    write(0x1c, &thresholdBypass, 1);
} /* LIDARLite_v3HP::configure */

void LIDARLite_v3HP::setI2Caddr(const uint8_t &newAddress, bool disableDefault) {
    uint8_t dataBytes[2];

    // Read UNIT_ID serial number bytes and write them into I2C_ID byte locations
    read(0x16, dataBytes, 2);
    write(0x18, dataBytes, 2);

    // Write the new I2C device address to registers
    // left shift by one to work around data alignment issue in v3HP
    dataBytes[0] = newAddress << 1;
    write(0x1a, dataBytes, 1);

    // Enable the new I2C device address using the default I2C device address
    read(0x1e, dataBytes, 1);
    dataBytes[0] = dataBytes[0] | (1 << 4); // set bit to enable the new address
    write(0x1e, dataBytes, 1);

    // Update local address field
    _addr7 = newAddress;
    _addr8 = newAddress << 1;

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefault) {
        read(0x1e, dataBytes, 1);
        dataBytes[0] = dataBytes[0] | (1 << 3); // set bit to disable default address
        write(0x1e, dataBytes, 1);
    }
} /* LIDARLite_v3HP::setI2Caddr */

uint8_t LIDARLite_v3HP::getI2Caddr() {
    return _addr7;
}

void LIDARLite_v3HP::takeRange() {
    uint8_t dataByte = 0x04;

    write(0x00, &dataByte, 1);
} /* LIDARLite_v3HP::takeRange */

void LIDARLite_v3HP::waitForBusy() {
    uint16_t busyCounter = 0; // busyCounter counts number of times busy flag is checked, for timeout
    uint8_t  busyFlag    = 1; // busyFlag monitors when the device is done with a measurement

    while (busyFlag)      // Loop until device is not busy
    {
        // Handle timeout condition, exit while loop and goto bailout
        if (busyCounter > 9999) {
            break;
        }

        busyFlag = getBusyFlag();

        // Increment busyCounter for timeout
        busyCounter++;

        wait_us(100);
    }

    // bailout reports error over serial
    if (busyCounter > 9999) {
        printf("waitForBusy timed out");
    }
} /* LIDARLite_v3HP::waitForBusy */

uint8_t LIDARLite_v3HP::getBusyFlag() {
    uint8_t busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    read(0x01, &busyFlag, 1);

    // STATUS bit 0 is busyFlag
    busyFlag &= 0x01;

    return busyFlag;
} /* LIDARLite_v3HP::getBusyFlag */


uint16_t LIDARLite_v3HP::readDistance() {
    uint16_t distance;
    uint8_t  dataBytes[2];

    // Read two bytes from register 0x0f and 0x10 (autoincrement)
    read(0x0f, dataBytes, 2);

    // Shift high byte and add to low byte
    distance = (dataBytes[0] << 8) | dataBytes[1];

    return (distance);
} /* LIDARLite_v3HP::readDistance */

void LIDARLite_v3HP::resetReferenceFilter() {
    uint8_t dataBytes[2];
    uint8_t acqConfigReg;
    uint8_t refCountMax;

    // Set bit 4 of the acquisition configuration register (disable reference filter)
    read(0x04, dataBytes, 1);  // Read address 0x04 (acquisition config register)
    acqConfigReg = dataBytes[0];                 // store for later restoration
    dataBytes[0] = dataBytes[0] | 0x10;          // turn on disable of ref filter
    write(0x04, dataBytes, 1); // write it back

    // Set reference integration count to max
    read(0x12, dataBytes, 1);  // Read address 0x12 (ref integration count)
    refCountMax = dataBytes[0];                  // store for later restoration
    dataBytes[0] = 0xff;                         // we want to reference to overflow quickly
    write(0x12, dataBytes, 1); // write ref integration count

    // Trigger a measurement
    waitForBusy();
    takeRange();
    waitForBusy();
    // ... no need to read the distance, it is immaterial

    // Restore previous reference integration count
    dataBytes[0] = refCountMax;
    write(0x12, dataBytes, 1);

    // Restore previous acquisition configuration register (re-enabling reference filter)
    dataBytes[0] = acqConfigReg;
    write(0x04, dataBytes, 1);
} /* LIDARLite_v3HP::resetReferenceFilter */

void LIDARLite_v3HP::correlationRecordToSerial(
    const uint16_t &numberOfReadings) {
    uint16_t i                = 0;
    uint8_t  dataBytes[2];          // Array to store read / write data
    int16_t  correlationValue = 0;  // Var to store value of correlation record

    // Test mode enable
    dataBytes[0] = 0x07;
    write(0x40, dataBytes, 1);

    for (i = 0; i < numberOfReadings; i++) {
        read(0x52, dataBytes, 2);

        //  Low byte is the value of the correlation record
        correlationValue = (uint16_t) dataBytes[0];

        // if upper byte lsb is one, the value is negative
        // so here we test to artifically sign extend the data
        if ((int) dataBytes[1] == 1) {
            correlationValue |= 0xff00;
        }
        printf("%d\n\r", correlationValue);
    }

    // test mode disable
    dataBytes[0] = 0;
    write(0x40, dataBytes, 1);
} /* LIDARLite_v3HP::correlationRecordToSerial */

void LIDARLite_v3HP::write(const uint8_t &regAddr, uint8_t *dataBytes,
                           const uint16_t &numBytes) {
    int nackCatcher;

    *_buffer = regAddr;
    memcpy(_buffer + 1, dataBytes, numBytes);

    nackCatcher = _i2c->write(_addr8, (char *) _buffer, numBytes + 1, false);

    // A nack means the device is not responding. Report the error over serial.
    if (nackCatcher != 0) {
        printf("> nack %d\n\r", nackCatcher);
    }

    wait_us(100); // 100 us delay for robustness with successive reads and writes
} /* LIDARLite_v3HP::write */

void LIDARLite_v3HP::read(const uint8_t &regAddr, uint8_t *dataBytes,
                          const uint16_t &numBytes) {
    int nackCatcher = 0;

    char regAddrC = char(regAddr);

    // A nack means the device is not responding, report the error over serial
    nackCatcher = _i2c->write(_addr8, &regAddrC, 1, false); // false means perform repeated start

    if (nackCatcher != 0) {
        printf("> nack %d\n\r", nackCatcher);
    }

    // Perform read, save in dataBytes array
    _i2c->read(_addr8, (char *) dataBytes, numBytes);
} /* LIDARLite_v3HP::read */
