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

#ifndef __AS3935_H__
#define __AS3935_H__

#include "Arduino.h"
#include <Wire.h>

// Default AS3935 I2C Address
#define AS3935_I2C_ADDR     0x03

// Enable AS3935 Verbose Messaging
#define AS3935_TUNING_VERBOSE   0
#define AS3935_REGISTER_VERBOSE 0


//
// Register Map
//
#define AS3935_REG0x00      0x00
#define AS3935_REG0x01      0x01
#define AS3935_REG0x02      0x02
#define AS3935_REG0x03      0x03
#define AS3935_REG0x04      0x04
#define AS3935_REG0x05      0x05
#define AS3935_REG0x06      0x06
#define AS3935_REG0x07      0x07
#define AS3935_REG0x08      0x08
#define AS3935_REG0x3A      0x3A
#define AS3935_REG0x3B      0x3B
#define AS3935_REG0x3C      0x3C
#define AS3935_REG0x3D      0x3D


//
// Registers and Fields
//

// Register 0x00
// R/W - AFE Gain Boost
#define AS3935_REG0x00_AFE_GB           0x00,0x3E
// R/W - 1: Power Down; 0: Active
#define AS3935_REG0x00_PWD              0x00,0x01

// Register 0x01
// R/W - Noise Floor Level
#define AS3935_REG0x01_NF_LEV           0x01,0x70
// R/W - Watchdog threshold
#define AS3935_REG0x01_WDTH             0x01,0x0F

// Register 0x02
// R/W - Clear statistics
#define AS3935_REG0x02_CL_STAT          0x02,0x40
// R/W - Minimum number of lightning
#define AS3935_REG0x02_MIN_NUM_LIGH     0x02,0x30
// R/W - Spike Rejection
#define AS3935_REG0x02_SREJ             0x02,0x0F

// Register 0x03
// R/W - Frequency division ration for antenna tuning
#define AS3935_REG0x03_LCO_FDIV         0x03,0xC0
// R/W - Mask Disturber
#define AS3935_REG0x03_MASK_DIST        0x03,0x20
 // RO  - Interrupt
#define AS3935_REG0x03_INT              0x03,0x0F

// Register 0x04
// RO  - Energy of Single Lightning LSBYTE
#define AS3935_REG0x04_S_LIG_L          0x04,0xFF

// Register 0x05
// RO  - Energy of Single Lightning MSBYTE
#define AS3935_REG0x05_S_LIG_M          0x05,0xFF

// Register 0x06
// RO  - Energy of the Single Lightning MMSBYTE
#define AS3935_REG0x06_S_LIG_MM         0x06,0x1F

// Register 0x07
// RO  - Distance estimation
#define AS3935_REG0x07_DISTANCE         0x07,0x3F

// Register 0x08
// R/W - Display LCO on IRQ pin
#define AS3935_REG0x08_DISP_LCO         0x08,0x80
// R/W - Display SRCO on IRQ pin
#define AS3935_REG0x08_DISP_SRCO        0x08,0x40
// R/W - Display TRCO on IRQ pin
#define AS3935_REG0x08_DISP_TRCO        0x08,0x20
// R/W - Internal Tuning Capacitors
#define AS3935_REG0x08_TUN_CAP          0x08,0x0F

// Register 0x3A
// RO  - Calibration of TRCO done (1: Successful)
#define AS3935_REG0x3A_TRCO_CALIB_DONE  0x3A,0x80
// RO  - Calibration of TRCO unsuccessful (1: Not successful)
#define AS3935_REG0x3A_TRCO_CALIB_NOK   0x3A,0x40

// Register 0x3B
// RO  - Calibration of SRCO done (1: Successful)
#define AS3935_REG0x3B_SRCO_CALIB_DONE  0x3B,0x80
// RO  - Calibration of SRCO unsuccessful (1: Not successful)
#define AS3935_REG0x3B_SRCO_CALIB_NOK   0x3B,0x40

// Register 0x3C
// Reset Direct Command Address
#define AS3935_REG0x3C_PRESET_DEFAULT   0x3C,0xFF

// Register 0x3D
// Internal RC Calibration Direct Command Address
#define AS3935_REG0x3D_CALIB_RCO        0x3D,0xFF


//
// Register Values
//

// Register 0x00, Analog Front-end (AFE)
#define AS3935_AFE_INDOOR       0x12
#define AS3935_AFE_OUTDOOR      0x0E

// Direct Command Value
#define AS3935_DIRECT_COMMAND   0x96


// AS3935 Class Definition
class AS3935 {

    public:
        AS3935(void)  {};
        ~AS3935(void) {};

        // Initialization
        bool     begin(uint8_t addr = AS3935_I2C_ADDR);
        bool     reset();

        // Direct Commands
        void     commandReset();
        void     commandCalibrateRcOscillators();
        void     commandClearStatistics();

        // Calibration
        bool     tuneAntenna(uint8_t irq_pin);

        // Register Operations

        // Register 0x00
        uint8_t  getAfeGainBoost();
        void     setAfeGainBoostIndoor();
        void     setAfeGainBoostOutdoor();

        bool     isPowerUp();
        void     setPowerUp();
        void     setPowerDown();

        // Register 0x01
        uint8_t  getNoiseFloorLevel();
        void     setNoiseFloorLevel(uint8_t data);

        uint8_t  getWatchdogThreshold();
        void     setWatchdogThreshold(uint8_t data);

        // Register 0x02
        uint8_t  getMinNumberOfLightning();
        void     setMinNumberOfLightning(uint8_t data);

        uint8_t  getSpikeRejection();
        void     setSpikeRejection(uint8_t data);

        // Register 0x03
        uint8_t  getLcoFDiv();
        void     setLcoFDiv(uint8_t data);

        uint8_t  getMaskDisturber();
        void     setMaskDisturber();
        void     clearMaskDisturber();

        uint8_t  getInterrupt();

        // Register 0x04, 0x05, 0x06
        uint32_t getEnergyOfSingleLightning();

        // Register 0x07
        uint8_t  getDistance();

        // Register 0x08
        uint8_t  getDisplayLco();
        void     setDisplayLco();
        void     clearDisplayLco();

        uint8_t  getDisplaySrco();
        void     setDisplaySrco();
        void     clearDisplaySrco();

        uint8_t  getDisplayTrco();
        void     setDisplayTrco();
        void     clearDisplayTrco();

        uint8_t  getTuningCapacitor();
        void     setTuningCapacitor(uint8_t data);

        // Register 0x3A
        bool     isTrcoCalibrationDone();
        bool     isTrcoCalibrationUnsuccessful();

        // Register 0x3B
        bool     isSrcoCalibrationDone();
        bool     isSrcoCalibrationUnsuccessful();

    private:
        // Sensor attributes
        uint8_t  _i2c_addr;

        // I2C operators
        void     _i2c_init();

        // Register operations
        uint8_t  _readRegisterField(uint8_t reg, uint8_t field);
        void     _writeRegisterField(uint8_t reg, uint8_t field, uint8_t value);
        void     _writeCommandField(uint8_t reg, uint8_t field, uint8_t value);
        uint8_t  _readRegister(uint8_t reg);
        void     _writeRegister(uint8_t reg, uint8_t value);
        void     _read(uint8_t reg, uint8_t *buf, uint8_t num);
        void     _write(uint8_t reg, uint8_t *buf, uint8_t num);
};

#endif
