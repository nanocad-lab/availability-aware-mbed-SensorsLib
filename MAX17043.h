/* MAX17043.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#ifndef MAX17043_H
#define MAX17043_H

#include "mbed.h"
#include "I2CSensor.h"
#include "PeriodicSensor.h"
 
/**
* This class allows for easy control over a MAX17043 LiPo fuel gauge IC.
*/
class MAX17043 : public I2CSensor, public PeriodicSensor {
    public:
        /**
         * @param sda the pin identifier for SDA I2C signal
         * @param scl the pin identifier for SCL I2C signal
         * @param i2c_addr the 8-bit I2C address for this device. Note that LSB is a don't care.
         */
        MAX17043(PinName sda, PinName scl, int i2c_addr);
        
        /**
         *
         */
        ~MAX17043();
        
        /**
         * Initializes the device to some preferred state.
         */
        void selfInit();
        
        /**
         * Performs a software reset of the device.
         */
        void reset();
        
        /**
         * @returns the IC version code
         */
        uint16_t getVersion();
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the battery voltage raw ADC value
         */
        uint16_t getVCell(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the battery voltage as floating point
         */
        float getFloatVCell(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the battery state of charge as computed by the ModelGauge algorithm. High byte: units of %. Low byte: units of 1/256%.
         */
        uint16_t getSOC(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the battery state of charge in %, as a floating point #
         */
        float getFloatSOC(bool sampleNow); 
     
    private:
        /**
         * Interrupt service routine for fetching SOC and VCell data from the device.
         */
        virtual void __sample_data_ISR();
        
        uint16_t __soc;
        uint16_t __vcell;
    
        ///////////////// CONSTANTS /////////////////////
        
        //Device register addresses
        static const uint8_t VCELL_MSB =            0x02; //Read only
        static const uint8_t VCELL_LSB =            0x03; //Read only
        static const uint8_t SOC_MSB =              0x04; //Read only
        static const uint8_t SOC_LSB =              0x05; //Read only
        static const uint8_t MODE_MSB =             0x06; //Write only
        static const uint8_t MODE_LSB =             0x07; //Write only
        static const uint8_t VERSION_MSB =          0x08; //Read only
        static const uint8_t VERSION_LSB =          0x09; //Read only
        static const uint8_t CONFIG_MSB =           0x0C; //Read/write
        static const uint8_t CONFIG_LSB =           0x0D; //Read/write
        static const uint8_t COMMAND_MSB =          0xFE; //Write only
        static const uint8_t COMMAND_LSB =          0xFF; //Write only
        
        static const uint16_t RST_CODE =            0x5400; //reset code for COMMAND 16-bit register
    
        //Levels
        static const float DIV_VCELL =              1.25e-3; //1.25 mV/level
        static const float DIV_SOC =                0.00390625; //1/256% / level
};

 
 #endif