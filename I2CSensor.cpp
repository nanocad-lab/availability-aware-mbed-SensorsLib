/* I2CSensor.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#include "mbed.h"
#include "I2CSensor.h"
 
using namespace std;

///////////////////// Public methods /////////////////////////////

I2CSensor::I2CSensor(PinName sda, PinName scl, int i2c_addr) :
                            __sda_pin(sda),
                            __scl_pin(scl),
                            __i2c_addr(i2c_addr),
                            __who_am_i(0),
                            __i2c(sda, scl)
                            { }

I2CSensor::~I2CSensor() { }
        
PinName I2CSensor::getSDAPin() { return __sda_pin; }

PinName I2CSensor::getSCLPin() { return __scl_pin; }

uint8_t I2CSensor::getDeviceI2CAddress() { return __i2c_addr; }

uint8_t I2CSensor::getRegister(const uint8_t reg_addr) {
    uint8_t data;
    __readReg(reg_addr, &data, 1);
    return data;
}

uint16_t I2CSensor::getRegister16b(const uint8_t reg_addr) {
    uint8_t payload[2];
    __readReg(reg_addr, payload, 2);
    uint16_t data = (payload[0] << 8) | (payload[1]);
    return data;
}

void I2CSensor::setRegister(const uint8_t reg_addr, const uint8_t data) {
    uint8_t payload[2] = {reg_addr, data};
    __writeReg(payload, 2);
}

void I2CSensor::setRegister16b(const uint8_t reg_addr, const uint16_t data) {
    uint8_t dataMSB = (data >> 8) & 0x00FF;
    uint8_t dataLSB = data & 0x00FF;
    uint8_t payload[3] = {reg_addr, dataMSB, dataLSB};
    __writeReg(payload, 3);
}

///////////////////// Protected methods /////////////////////////////

int I2CSensor::__readReg(const uint8_t reg_addr, uint8_t *data, int len) { 
    int retval = 0;
    
    __disable_irq();
    retval = __i2c.write(__i2c_addr, (char *) &reg_addr, 1, true);
    if (retval != 0) {
        __enable_irq();
        return retval;
    }
    retval = __i2c.read(__i2c_addr, (char *) data, len);
    __enable_irq();
    
    return retval;
}

int I2CSensor::__writeReg(const uint8_t *data, int total_len) {
    int retval = 0;
    
    __disable_irq();
    retval = __i2c.write(__i2c_addr, (char *) data, total_len);
    __enable_irq();
   
    return retval;
}