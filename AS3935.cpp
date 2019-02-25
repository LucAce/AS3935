//*****************************************************************************
// Copyright (c) 2019 LucAce
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//*****************************************************************************
//
// Arduino I2C Library for AMS AS3935 Franklin Lightning Sensor
//
// Library Dependencies:
// - Wire
//
// Notes:
// - Library based off of ams Datasheet [v1-04] 2016-Jan-13.
// - Source code uses Natural Docs for document generation.
//
//*****************************************************************************

#include "AS3935.h"

// Group: Public Functions

//*****************************************************************************
// Function: begin
// Initialize the AS3935 Sensor.
//
// Parameters:
// addr - I2C address of the sensor
//
// Returns:
// bool - true if successful; false otherwise
//*****************************************************************************
bool AS3935::begin(uint8_t addr) {
    // Set the I2C Address used by this library
    _i2c_addr = addr;

    // Initialize I2C interface
    this->_i2c_init();

    // Issue reset to sensor
    this->commandReset();

    // Issue power up
    this->setPowerUp();

    // Read Address 0x00 to ensure powered up
    if (this->_readRegisterField(AS3935_REG0x00_PWD) == 0)
        return true;
    else
        return false;
}


//*****************************************************************************
// Function: reset
// Re-initialize the AS3935 Sensor using the same I2C address already
// configured.
//
// Returns:
// bool - true if successful; false otherwise
//*****************************************************************************
bool AS3935::reset() {
    return this->begin(_i2c_addr);
}


//*****************************************************************************
// Function: commandReset()
// Issue Direct Command to reset all registers.
//*****************************************************************************
void AS3935::commandReset() {
    this->_writeCommandField(
        AS3935_REG0x3C_PRESET_DEFAULT,
        AS3935_DIRECT_COMMAND
    );
    delay(2);
}


//*****************************************************************************
// Function: commandCalibrateRcOscillators()
// Issue Direct Command to calibrate Rc Oscillators.
//*****************************************************************************
void AS3935::commandCalibrateRcOscillators() {
    this->_writeCommandField(
        AS3935_REG0x3D_CALIB_RCO,
        AS3935_DIRECT_COMMAND
    );
    delay(2);
}


//*****************************************************************************
// Function: tuneAntenna()
// Find the best tuning value for the antenna.
//
// Notes:
// - IRQ pin required for tuning
// - See Antenna Tunning section of datasheet
//
// Parameters:
// irq_pin - IRQ Input Pin
//
// Returns:
// bool - true if successful; false otherwise
//*****************************************************************************
bool AS3935::tuneAntenna(uint8_t irq_pin) {
    unsigned long timestamp;

    int      target_count = 3125;
    int      target_diff  = 0;
    int      best_diff    = 32767;
    int      count        = 0;
    uint8_t  best_tune    = 0;

    uint8_t  current_irq;
    uint8_t  previous_irq;

    // Set the LCO divider ratio to 16
    this->setLcoFDiv(0);

    // Set Display LCO on IRQ Pin
    this->setDisplayLco();

    // Walk through each capacitor value to find the value which
    // results in the smallest difference from ideal.
    //
    // For each capacitor value count the number of edges seen on
    // the IRQ pin.  This count after 100ms should ideally be equal
    // to 3125 which is the target frequency divided by the LCO divider
    // ratio.  For example, 500khz/16 = 31250 hz.  A reasonable test
    // period is 100ms.  Therefore the target count is 3125.
    for (uint8_t i = 0; i <= 0x0F; i++) {
        this->setTuningCapacitor(i<<3);
        delay(2);

        count        = 0;
        timestamp    = millis();
        previous_irq = digitalRead(irq_pin);

        while ((millis() - timestamp) < 100) {
            current_irq = digitalRead(irq_pin);

            // Count the rising edges of the IRQ pin
            if (current_irq > previous_irq) {
                count++;
            }
            previous_irq = current_irq;
        }

        // Get the count difference from target
        target_diff = target_count - count;
        if (target_diff < 0)
            target_diff = -target_diff;

        #if AS3935_TUNING_VERBOSE == 1
        Serial.print(F("Testing pf: "));
        Serial.print(i<<3, DEC);
        Serial.print(F("; Timing Difference: "));
        Serial.println(target_diff);
        #endif

        if (best_diff > target_diff) {
            best_diff = target_diff;
            best_tune = (i<<3);
        }
    }

    // Set the best tuning capacitor value
    this->setTuningCapacitor(best_tune);
    delay(2);

    // Display LCO
    this->clearDisplayLco();

    // Power Up
    this->setPowerUp();

    // Error if over 109, outside allowed tuning range of +/-3.5%
    if (best_diff > 109)
        return false;
    else
        return true;
}


//*****************************************************************************
// Function: getAfeGainBoost()
// Get the current AFE Gain Boost value.
//
// Returns:
// uint8_t - AFE Gain Boost
//*****************************************************************************
uint8_t AS3935::getAfeGainBoost() {
    return this->_readRegisterField(AS3935_REG0x00_AFE_GB);
}


//*****************************************************************************
// Function: setAfeGainBoostIndoor()
// Set the AFE Gain Boost for Indoor.
//*****************************************************************************
void AS3935::setAfeGainBoostIndoor() {
    this->_writeRegisterField(AS3935_REG0x00_AFE_GB, AS3935_AFE_INDOOR);
}


//*****************************************************************************
// Function: setAfeGainBoostOutdoor()
// Set the AFE Gain Boost for Outdoor.
//*****************************************************************************
void AS3935::setAfeGainBoostOutdoor() {
    this->_writeRegisterField(AS3935_REG0x00_AFE_GB, AS3935_AFE_OUTDOOR);
}


//*****************************************************************************
// Function: isPowerUp()
// Get the current Power value.
//
// Notes:
// - PWD Field: 1: Power Down; 0: Active
//
// Returns:
// bool - Is Power Up (true: Power Up; false: Power Down)
//*****************************************************************************
bool AS3935::isPowerUp() {
    uint8_t reg_value;

    reg_value = this->_readRegisterField(AS3935_REG0x00_PWD);
    if (reg_value == 0)
        return true;
    else
        return false;
}


//*****************************************************************************
// Function: setPowerUp()
// Command sensor to recalibrate oscillators and set sensor to Active.
//
// Notes:
// - Recalibrate procedure defined in Clock Generation section of datasheet
//*****************************************************************************
void AS3935::setPowerUp() {
    this->_writeRegisterField(AS3935_REG0x00_PWD, 0);
    this->commandCalibrateRcOscillators();
    this->setDisplaySrco();
    delay(2);
    this->clearDisplaySrco();
}


//*****************************************************************************
// Function: setPowerDown()
// Set sensor to Power Down.
//*****************************************************************************
void AS3935::setPowerDown() {
    this->_writeRegisterField(AS3935_REG0x00_PWD, 1);
}


//*****************************************************************************
// Function: getNoiseFloorLevel()
// Get the current Noise Floor Level value.
//
// Returns:
// uint8_t - Noise Floor Level
//*****************************************************************************
uint8_t AS3935::getNoiseFloorLevel() {
    return this->_readRegisterField(AS3935_REG0x01_NF_LEV);
}


//*****************************************************************************
// Function: setNoiseFloorLevel()
// Set Noise Floor Level value.
//
// Parameters:
// data - Noise Floor Level
//*****************************************************************************
void AS3935::setNoiseFloorLevel(uint8_t data) {
    this->_writeRegisterField(AS3935_REG0x01_NF_LEV, data);
}


//*****************************************************************************
// Function: getWatchdogThreshold
// Get the current Watchdog Threshold value.
//
// Returns:
// uint8_t - Watchdog Threshold
//*****************************************************************************
uint8_t AS3935::getWatchdogThreshold() {
    return this->_readRegisterField(AS3935_REG0x01_WDTH);
}


//*****************************************************************************
// Function: setWatchdogThreshold()
// Set Watchdog Threshold value.
//
// Parameters:
// data - Watchdog Threshold
//*****************************************************************************
void AS3935::setWatchdogThreshold(uint8_t data) {
    this->_writeRegisterField(AS3935_REG0x01_WDTH, data);
}


//*****************************************************************************
// Function: commandClearStatistics
// Issue a clear statistics command.
//
// Notes:
// - Clear Statistics command issue issued with the CL_STAT fiels is written
//   three times with a pattern of 1, 0, 1
//*****************************************************************************
void AS3935::commandClearStatistics() {
    uint8_t reg_value;

    reg_value = this->_readRegister(AS3935_REG0x02);
    this->_writeRegister(AS3935_REG0x02, (reg_value | 0x01));
    this->_writeRegister(AS3935_REG0x02, (reg_value & 0xFE));
    this->_writeRegister(AS3935_REG0x02, (reg_value | 0x01));
}


//*****************************************************************************
// Function: getMinNumberOfLightning
// Get the Minimum Number of Lighting value.
//
// Returns:
// uint8_t - Minimum Number of Lighting
//*****************************************************************************
uint8_t AS3935::getMinNumberOfLightning() {
    return this->_readRegisterField(AS3935_REG0x02_MIN_NUM_LIGH);
}


//*****************************************************************************
// Function: setMinNumberOfLightning()
// Set Minimum Number of Lighting value.
//
// Notes:
// - Value Translation:
//      0b00: 1
//      0b01: 5
//      0b10: 9
//      0b11: 16
//
// Parameters:
// data - Minimum Number of Lighting
//*****************************************************************************
void AS3935::setMinNumberOfLightning(uint8_t data) {
    this->_writeRegisterField(AS3935_REG0x02_MIN_NUM_LIGH, data);
}


//*****************************************************************************
// Function: getSpikeRejection
// Get the Spike Rejection value.
//
// Returns:
// uint8_t - Spike Rejection
//*****************************************************************************
uint8_t AS3935::getSpikeRejection() {
    return this->_readRegisterField(AS3935_REG0x02_SREJ);
}


//*****************************************************************************
// Function: setSpikeRejection()
// Set Spike Rejection value.
//
// Parameters:
// data - Spike Rejection
//*****************************************************************************
void AS3935::setSpikeRejection(uint8_t data) {
    this->_writeRegisterField(AS3935_REG0x02_SREJ, data);
}


//*****************************************************************************
// Function: getLcoFDiv
// Get the Frequency division ratio for antenna tuning value.
//
// Returns:
// uint8_t - Frequency division ration
//*****************************************************************************
uint8_t AS3935::getLcoFDiv() {
    return this->_readRegisterField(AS3935_REG0x03_LCO_FDIV);
}


//*****************************************************************************
// Function: setLcoFDiv()
// Set Frequency division ratio for antenna tuning value.
//
// Notes:
// - Division Ratio:
//      0b00: 16
//      0b01: 32
//      0b10: 64
//      0b11: 128
//
// Parameters:
// data - Frequency division ration
//*****************************************************************************
void AS3935::setLcoFDiv(uint8_t data) {
    this->_writeRegisterField(AS3935_REG0x03_LCO_FDIV, data);
}


//*****************************************************************************
// Function: getMaskDisturber
// Get the Mask Disturber value.
//
// Returns:
// uint8_t - Mask Disturber
//*****************************************************************************
uint8_t AS3935::getMaskDisturber() {
    return this->_readRegisterField(AS3935_REG0x03_MASK_DIST);
}


//*****************************************************************************
// Function: setMaskDisturber
// Set the Mask Disturber value.
//*****************************************************************************
void AS3935::setMaskDisturber() {
    this->_writeRegisterField(AS3935_REG0x03_MASK_DIST, 1);
}


//*****************************************************************************
// Function: clearMaskDisturber
// Clear the Mask Disturber value.
//*****************************************************************************
void AS3935::clearMaskDisturber() {
    this->_writeRegisterField(AS3935_REG0x03_MASK_DIST, 0);
}


//*****************************************************************************
// Function: getInterrupt
// Get the Interrupt value.
//
// Notes:
// - After the IRQ signal is asserted, the master should wait 2ms before
//   reading the interrupt register.
// - The interrupt register will be cleared upon being read.
//
// Returns:
// uint8_t - Interrupt
//*****************************************************************************
uint8_t AS3935::getInterrupt() {
    delay(2);
    return this->_readRegisterField(AS3935_REG0x03_INT);
}


//*****************************************************************************
// Function: getEnergyOfSingleLightning
// Get the Energy of the Single Lightning 32bit value.
//
// Returns:
// uint32_t - Energy of the Single Lightning
//*****************************************************************************
uint32_t AS3935::getEnergyOfSingleLightning() {
    uint8_t  lsb;
    uint8_t  msb;
    uint8_t  mmsb;

    lsb  = this->_readRegisterField(AS3935_REG0x04_S_LIG_L);
    msb  = this->_readRegisterField(AS3935_REG0x05_S_LIG_M);
    mmsb = this->_readRegisterField(AS3935_REG0x06_S_LIG_MM);

    return (((uint32_t)mmsb << 16) | ((uint32_t)msb << 8) | ((uint32_t)lsb));
}


//*****************************************************************************
// Function: getDistance
// Get the Distance value.
//
// Returns:
// uint8_t - Distance
//*****************************************************************************
uint8_t AS3935::getDistance() {
    return this->_readRegisterField(AS3935_REG0x07_DISTANCE);
}


//*****************************************************************************
// Function: getDisplayLco
// Get the Display LCO on IRQ Pin value.
//
// Returns:
// uint8_t - Display LCO on IRQ Pin
//*****************************************************************************
uint8_t AS3935::getDisplayLco() {
    return this->_readRegisterField(AS3935_REG0x08_DISP_LCO);
}


//*****************************************************************************
// Function: setDisplayLco()
// Set to 1 the Display LCO on IRQ Pin value.
//*****************************************************************************
void AS3935::setDisplayLco() {
    this->_writeRegisterField(AS3935_REG0x08_DISP_LCO, 1);
}


//*****************************************************************************
// Function: clearDisplayLco()
// Clear to 0 the Display LCO on IRQ Pin value.
//*****************************************************************************
void AS3935::clearDisplayLco() {
    this->_writeRegisterField(AS3935_REG0x08_DISP_LCO, 0);
}


//*****************************************************************************
// Function: getDisplaySrco
// Get the Display SRCO on IRQ Pin value.
//
// Returns:
// uint8_t - Display SRCO on IRQ Pin
//*****************************************************************************
uint8_t AS3935::getDisplaySrco() {
    return this->_readRegisterField(AS3935_REG0x08_DISP_SRCO);
}


//*****************************************************************************
// Function: setDisplaySrco()
// Set to 1 the Display SRCO on IRQ Pin value.
//*****************************************************************************
void AS3935::setDisplaySrco() {
    this->_writeRegisterField(AS3935_REG0x08_DISP_SRCO, 1);
}


//*****************************************************************************
// Function: clearDisplaySrco()
// Clear to 0 the Display SRCO on IRQ Pin value.
//*****************************************************************************
void AS3935::clearDisplaySrco() {
    this->_writeRegisterField(AS3935_REG0x08_DISP_SRCO, 0);
}


//*****************************************************************************
// Function: getDisplayTrco
// Get the Display TRCO on IRQ Pin value.
//
// Returns:
// uint8_t - Display TRCO on IRQ Pin
//*****************************************************************************
uint8_t AS3935::getDisplayTrco() {
    return this->_readRegisterField(AS3935_REG0x08_DISP_TRCO);
}


//*****************************************************************************
// Function: setDisplayTrco()
// Set to 1 the Display TRCO on IRQ Pin value.
//*****************************************************************************
void AS3935::setDisplayTrco() {
    this->_writeRegisterField(AS3935_REG0x08_DISP_TRCO, 1);
}


//*****************************************************************************
// Function: clearDisplayTrco()
// Clear to 0 the Display TRCO on IRQ Pin value.
//*****************************************************************************
void AS3935::clearDisplayTrco() {
    this->_writeRegisterField(AS3935_REG0x08_DISP_TRCO, 0);
}


//*****************************************************************************
// Function: getTuningCapacitor
// Get the Internal Tunning Capacitor value.
//
// Notes:
// - Returns capacitory value [reg << 3]
//
// Returns:
// uint8_t - Internal Tunning Capacitor
//*****************************************************************************
uint8_t AS3935::getTuningCapacitor() {
    uint8_t cap_value;

    cap_value = this->_readRegisterField(AS3935_REG0x08_TUN_CAP);
    return (cap_value << 3);
}


//*****************************************************************************
// Function: setTuningCapacitor()
// Set the Internal Tunning Capacitor value.
//
// Notes:
// - Valid range is from 0pF to 120pF in steps of 8pF
// - Function expects capacitor value in pF and not raw register value
//
// Parameters:
// data - Frequency division ration
//*****************************************************************************
void AS3935::setTuningCapacitor(uint8_t data) {
    // Limit value to 120
    if (data > 120) {
        data = 120;
    }
    this->_writeRegisterField(AS3935_REG0x08_TUN_CAP, (data>>3));
}


//*****************************************************************************
// Function: isTrcoCalibrationDone
// Get the TRCO Calibration Done value.
//
// Returns:
// bool - TRCO Calibration Done
//*****************************************************************************
bool AS3935::isTrcoCalibrationDone() {
    uint8_t reg_value;
    reg_value = this->_readRegisterField(AS3935_REG0x3A_TRCO_CALIB_DONE);
    if (reg_value == 1)
        return true;
    else
        return false;
}


//*****************************************************************************
// Function: isTrcoCalibrationUnsuccessful
// Get the Calibration of TRCO Unsuccessful value.
//
// Returns:
// bool - Calibration of TRCO Unsuccessful
//*****************************************************************************
bool AS3935::isTrcoCalibrationUnsuccessful() {
    uint8_t reg_value;
    reg_value = this->_readRegisterField(AS3935_REG0x3A_TRCO_CALIB_NOK);
    if (reg_value == 1)
        return true;
    else
        return false;
}


//*****************************************************************************
// Function: isSrcoCalibrationDone
// Get the SRCO Calibration Done value.
//
// Returns:
// bool - SRCO Calibration Done
//*****************************************************************************
bool AS3935::isSrcoCalibrationDone() {
    uint8_t reg_value;
    reg_value = this->_readRegisterField(AS3935_REG0x3B_SRCO_CALIB_DONE);
    if (reg_value == 1)
        return true;
    else
        return false;
}


//*****************************************************************************
// Function: isSrcoCalibrationUnsuccessful
// Get the Calibration of SRCO Unsuccessful value.
//
// Returns:
// bool - Calibration of SRCO Unsuccessful
//*****************************************************************************
bool AS3935::isSrcoCalibrationUnsuccessful() {
    uint8_t reg_value;
    reg_value = this->_readRegisterField(AS3935_REG0x3B_SRCO_CALIB_NOK);
    if (reg_value == 1)
        return true;
    else
        return false;
}


// Group: Private Functions

//*****************************************************************************
// Function: _i2c_init
// Initialize the I2C interface.
//*****************************************************************************
void AS3935::_i2c_init() {
    Wire.begin();
}


//*****************************************************************************
// Function: _readRegisterField
// Read one register field.
//
// Parameters:
// reg   - Register to read
// field - Register field to read
//
// Returns:
// uint8_t - Value read
//*****************************************************************************
uint8_t AS3935::_readRegisterField(uint8_t reg, uint8_t field) {
    uint8_t reg_value;
    uint8_t field_shift;

    // Read existing register value
    reg_value =  this->_readRegister(reg);
    reg_value &= field;
    if (field) {
        for (field_shift = 1; ~field & 1; field_shift++)
            field >>= 1;

        reg_value >>= (field_shift-1);
    }

    return reg_value;
}


//*****************************************************************************
// Function: _writeRegisterField
// Write one register field.
//
// Parameters:
// reg   - Register to write
// field - Register field to write
// value - Value to write
//*****************************************************************************
void AS3935::_writeRegisterField(uint8_t reg, uint8_t field, uint8_t value) {
    uint8_t reg_value;
    uint8_t field_shift;

    // Read existing register value
    reg_value = this->_readRegister(reg);

    // Modify only field bits
    reg_value &= ~(field);
    if (field) {
        for (field_shift = 1; ~field & 1; field_shift++)
            field >>= 1;

        reg_value |= (value << (field_shift-1));
    }
    else {
        reg_value |= value;
    }

    // Write new register value
    this->_writeRegister(reg, reg_value);
}


//*****************************************************************************
// Function: _writeCommandField
// Write one command field.
//
// Parameters:
// reg   - Register to write
// field - Register field to write
// value - Value to write
//*****************************************************************************
void AS3935::_writeCommandField(uint8_t reg, uint8_t field, uint8_t value) {
    this->_writeRegister(reg, value);
}


//*****************************************************************************
// Function: _readRegister
// Read one byte of data from the I2C interface.
//
// Parameters:
// reg - Register to read
//
// Returns:
// uint8_t - Value read
//*****************************************************************************
uint8_t AS3935::_readRegister(uint8_t reg) {
    uint8_t ret;
    this->_read(reg, &ret, 1);

    #if AS3935_REGISTER_VERBOSE == 1
    Serial.print(F("AS3935::_readRegister(0x"));
    Serial.print(reg, HEX);
    Serial.print(F("): "));
    Serial.println(ret, HEX);
    #endif

    return ret;
}


//*****************************************************************************
// Function: _writeRegister
// Write one byte of data over the I2C interface.
//
// Parameters:
// reg   - Register to write
// value - Value to write
//*****************************************************************************
void AS3935::_writeRegister(uint8_t reg, uint8_t value) {
    #if AS3935_REGISTER_VERBOSE == 1
    Serial.print(("AS3935::_writeRegister(0x"));
    Serial.print(reg, HEX);
    Serial.print(", 0x");
    Serial.print(value, HEX);
    Serial.println(")");
    #endif

    this->_write(reg, &value, 1);
}


//*****************************************************************************
// Function: _read
// Read multi-byte data from the I2C interface.
//
// Notes:
// - The AS3935 requires a restart message after transmission
//   "Wire.endTransmission(false);"
//
// Parameters:
// reg  - Register/Mailbox to read
// *buf - Pointer to data read on interface
// num  - Number of bytes to read
//*****************************************************************************
void AS3935::_read(uint8_t reg, uint8_t *buf, uint8_t num) {
    uint8_t value;
    uint8_t pos = 0;

    // On arduino we need to read in 32 byte chunks
    while (pos < num){
        uint8_t read_now = min(32, num - pos);
        Wire.beginTransmission((uint8_t)_i2c_addr);
        Wire.write((uint8_t)reg + pos);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)_i2c_addr, read_now);

        for (int i=0; i<read_now; i++){
            buf[pos] = Wire.read();
            pos++;
        }
    }
}


//*****************************************************************************
// Function: _write
// Write multi-byte data to the I2C interface.
//
// Parameters:
// reg  - Register/Mailbox to write
// *buf - Pointer to data to write on interface
// num  - Number of bytes to write
//*****************************************************************************
void AS3935::_write(uint8_t reg, uint8_t *buf, uint8_t num) {
    Wire.beginTransmission((uint8_t)_i2c_addr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t *)buf, num);
    Wire.endTransmission();
}
