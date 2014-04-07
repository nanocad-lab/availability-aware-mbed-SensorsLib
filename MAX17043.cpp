/* MAX17043.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

 
#include "mbed.h"
#include "I2CSensor.h"
#include "PeriodicSensor.h"
#include "MAX17043.h"


MAX17043::MAX17043(PinName sda, PinName scl, int i2c_addr) :
                                I2CSensor(sda, scl, i2c_addr),
                                PeriodicSensor(0.5), //Default max sampling rate of 2 Hz, because the device internally samples every 500ms after initial POS report
                                __soc(0),
                                __vcell(0)
                                {
}

MAX17043::~MAX17043() { }

void MAX17043::selfInit() {
    __i2c.frequency(400000);
    reset();
}

void MAX17043::reset() {
    __disable_irq();
    uint16_t data = RST_CODE;
    setRegister16b(COMMAND_MSB, data);
    __enable_irq();
    wait(0.130); //wait 130ms until first readings are valid (125ms est)
}

uint16_t MAX17043::getVersion() {
    uint8_t data = getRegister(VERSION_MSB);
    return (data << 8) | (getRegister(VERSION_LSB));
}

uint16_t MAX17043::getVCell(bool sampleNow) {
    __disable_irq();
    
    if (sampleNow) {
        uint16_t data = getRegister16b(VCELL_MSB);
        __vcell = data >> 4; //right shift by 4 to throw out the don't care bits
    }
    __dataReady = false;
    
    __enable_irq();
    
    return __vcell;
}
 
float MAX17043::getFloatVCell(bool sampleNow) {
    return getVCell(sampleNow) * DIV_VCELL;       
}
    
uint16_t MAX17043::getSOC(bool sampleNow) {
    __disable_irq();
    
    if (sampleNow) {
        uint8_t data = getRegister(SOC_MSB);
        __soc =  (data << 8) | (getRegister(SOC_LSB));
    }
    __dataReady = false;
    
    __enable_irq();
    
    return __soc;
}

float MAX17043::getFloatSOC(bool sampleNow) {
    return getSOC(sampleNow) * DIV_SOC;   
}

void MAX17043::__sample_data_ISR() {
    getSOC(true);
    getVCell(true);
    __dataReady = true;
}