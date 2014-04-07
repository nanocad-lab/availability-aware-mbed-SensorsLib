/* I2CSensor.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "mbed.h"
 
/**
 * Base class from which digital sensors using I2C should be derived to simplify low-level communications with the device.
 */
class I2CSensor {
    public:
        /**
         * @param sda I2C SDA pin ID
         * @param scl I2C SCL pin ID
         * @param i2c_addr I2C 8-bit address (LSB is actually don't care)
         */
        I2CSensor(PinName sda, PinName scl, int i2c_addr);
        
        /**
         */
        ~I2CSensor();
        
        /**
         * @returns I2C SDA pin ID
         */
        PinName getSDAPin();
        
        /**
         * @returns I2C SCL pin ID
         */
        PinName getSCLPin();
        
        /**
         * @returns Device I2C address (LSB always 0 in this case, it is don't care)
         */
        uint8_t getDeviceI2CAddress();
        
        /**
         * Read an 8-bit register.
         * @param reg_addr the register in the device
         * @returns The raw value from the register specified by reg_addr.
         */
        uint8_t getRegister(const uint8_t reg_addr);
        
        /**
         * Read a 16-bit register.
         * @param reg_addr the register in the device
         * @returns The raw value from the register specified by reg_addr.
         */
        uint16_t getRegister16b(const uint8_t reg_addr);
        
        /**
         * Set an 8-bit register.
         * @param reg_addr the register in the device
         * @param data the byte to write to the register
         */
        void setRegister(const uint8_t reg_addr, const uint8_t data);
        
        /**
         * Set a 16-bit register.
         * @param reg_addr the register in the device
         * @param data the byte to write to the register
         */
        void setRegister16b(const uint8_t reg_addr, const uint16_t data);
        
    protected:
        /**
         * @param reg_addr 8-bit register address inside the device
         * @param data 8-bit data that will be read from the register. This pointer MUST be valid. This array MUST be at least as long as len.
         * @param len total number of bytes to read. len must be >= 1.
         * @returns 0 on success, otherwise error code from I2C
         */ 
        int __readReg(const uint8_t reg_addr, uint8_t *data, int len);
        
        /**
         * @param data 8-bit data that will be written to the register. This pointer MUST be valid. This array MUST be at least as long as total_len.
         * data[0] should be set to the 8-bit register address. The data payload should start at index 1.
         * @param total_len total length of the data array, which is the length of the payload + 1 for the register address. total_len must be >= 2.
         * @returns 0 on success, otherwise error code from I2C
         */ 
        int __writeReg(const uint8_t *data, int total_len);
        
        PinName __sda_pin;
        PinName __scl_pin;  
        int __i2c_addr;
        uint8_t __who_am_i;
        I2C __i2c; //We wrap the mbed i2c SDK functionality
};

#endif