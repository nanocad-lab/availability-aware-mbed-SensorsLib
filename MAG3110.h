/* MAG3110.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#ifndef MAG3110_H
#define MAG3110_H

#include "mbed.h"
#include "I2CSensor.h"
#include "PeriodicSensor.h"


/**
 * This class allows for easy control of an MAG3110 magnetometer IC.
 */
class MAG3110 : public I2CSensor, public PeriodicSensor {
    public:
        /**
         * Enumeration of allowed ADC sampling data rates.
         * The device does sample averages such that
         * the output data rate ODR = ADC_RATE / OVERSMPL_RATIO in all cases except for when ADC_RATE is 80 Hz (min). In this case,
         * there are multiple ODR for the same ADC_RATE, OVERSMPL_RATIO combination.
         * Note that there are non-unique combinations for the same ODR.
         */
        typedef enum {
            AHZ1280, //1280 Hz
            AHZ640,  //640 Hz
            AHZ320,  //320 Hz
            AHZ160,  //160 Hz
            AHZ80    //80 Hz
        } adc_smpl_rate_t;
        
        /**
         * Enumeration of allowed oversampling ratios.
         * The device does sample averages such that
         * the output data rate ODR = ADC_RATE / OVERSMPL_RATIO in all cases except for when ADC_RATE is 80 Hz (min). In this case,
         * there are multiple ODR for the same ADC_RATE, OVERSMPL_RATIO combination.
         * Note that there are non-unique combinations for the same ODR.
         */
        typedef enum {
            O16, //16:1
            O32, //32:1
            O64, //64:1
            O128 //128:1
        } oversmpl_ratio_t;
        
        /**
         * Enumeration of possible output data rates.
         * The device does sample averages such that
         * the output data rate ODR = ADC_RATE / OVERSMPL_RATIO in all cases except for when ADC_RATE is 80 Hz (min). In this case,
         * there are multiple ODR for the same ADC_RATE, OVERSMPL_RATIO combination.
         * Note that there are non-unique combinations for the same ODR.
         */
        typedef enum {
            HZ80,   //80 Hz
            HZ40,   //40 Hz
            HZ20,   //20 Hz
            HZ10,   //10 Hz
            HZ5,    //5 Hz
            HZ2_5,  //2.5 Hz
            HZ1_25, //1.25 Hz
            HZ0_63, //0.63 Hz
            HZ0_31, //0.31 Hz
            HZ0_16, //0.16 Hz
            HZ0_08  //0.08 Hz
        } smpl_rate_t;
    
        /**
         * @param sda the pin identifier for SDA I2C signal
         * @param scl the pin identifier for SCL I2C signal
         * @param i2c_addr the 8-bit I2C address for this device. Note that LSB is a don't care.
         */
        MAG3110(PinName sda, PinName scl, int i2c_addr);
        
        /**
         *
         */
        ~MAG3110();
        
        /**
         * Self-initialization to some nice preset. You must ensure the device is first deactivated using setActive().
         */
        void selfInit();
        
        //I2C-specific methods
        
        /**
         * Implements the pure virtual method of the parent I2CSensor class.
         * @returns the 8-bit device identifier.
         */
        uint8_t whoAmI();
        
        //Device-specific methods
        
        /**
         * @returns true if the device is active
         */
        bool isActive();
        
        /**
         * @param activate if true, enables the device, else disables it
         */
        void setActive(bool activate);
        
        /**
         * @returns the 8-bit system mode status
         */
        uint8_t getSystemMode();
        
        /**
         * @param rate optional pointer, if provided, will be set to the output sampling rate
         * @param ratio optional pointer, if provided, will be set to the oversampling ratio
         * @param adc_rate optional pointer, if provided, will be set to the ADC rate
         */
        void getOutputSamplingParameters(smpl_rate_t *rate, oversmpl_ratio_t *ratio, adc_smpl_rate_t *adc_rate);
        
        /**
         * @param rate the enumerated value corresponding to the output data sample rate to use
         * @param ratio the number of ADC samples per output sample (averaged)
         * @param adc_rate optional pointer, set to the resulting adc_rate used for the combination of rate and ratio
         * @returns true if the operation succeeded and the first two parameters were correct
         */
        bool setOutputSamplingParameters(smpl_rate_t rate, oversmpl_ratio_t ratio, adc_smpl_rate_t *adc_rate);
        
        /**
         * @returns the value in the data register
         */
        uint8_t getDataRegisterStatus();
        
        
        //Device-specific data sampling methods
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns a 16-bit value representing the latest data sample for the X dimension, centered at 0.
         */
        int16_t getX(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns a 16-bit value representing the latest data sample for the Y dimension, centered at 0.
         */
        int16_t getY(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns a 16-bit value representing the latest data sample for the Z dimension, centered at 0.
         */
        int16_t getZ(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * Returns the latest X data reading as a float in uT
         */
        float getFloatX(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * Returns the latest Y data reading as a float in uT
         */
        float getFloatY(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * Returns the latest Z data reading as a float in uT
         */
        float getFloatZ(bool sampleNow);

        /**
         * Get the die temperature. Note that the actual sensor range is only -40C to 125C, so not all outputs are valid.
         * @returns 8-bit die temperature data
         */
        int8_t getDieTemp();   
        
        /**
         * @returns the die temperature in deg Celsius. Note that the actual sensor range is only -40C to 125C, so not all possible outputs may be valid.
         */
        float getFloatDieTemp();
     
    private:
        /**
         * Interrupt service routine for fetching magnetometer data from the device.
         */
        virtual void __sample_data_ISR();
    
        ///////////////// CONSTANTS /////////////////////
        
        //Device register addresses
        static const uint8_t DR_STATUS =        0x00;
        static const uint8_t OUT_X_MSB =        0x01;
        static const uint8_t OUT_X_LSB =        0x02;
        static const uint8_t OUT_Y_MSB =        0x03;
        static const uint8_t OUT_Y_LSB =        0x04;
        static const uint8_t OUT_Z_MSB =        0x05;
        static const uint8_t OUT_Z_LSB =        0x06;
        static const uint8_t WHO_AM_I =         0x07;
        static const uint8_t SYSMOD =           0x08;
        static const uint8_t OFF_X_MSB =        0x09;
        static const uint8_t OFF_X_LSB =        0x0A;
        static const uint8_t OFF_Y_MSB =        0x0B;
        static const uint8_t OFF_Y_LSB =        0x0C;
        static const uint8_t OFF_Z_MSB =        0x0D;
        static const uint8_t OFF_Z_LSB =        0x0E;
        static const uint8_t DIE_TEMP =         0x0F;
        static const uint8_t CTRL_REG1 =        0x10;
        static const uint8_t CTRL_REG2 =        0x11; 
        
        //Register masks
        static const uint8_t DR_STATUS_ZYXOW_MASK =                 0x80; //b1000 0000
        static const uint8_t DR_STATUS_ZOW_MASK =                   0x40; //b0100 0000
        static const uint8_t DR_STATUS_YOW_MASK =                   0x20; //b0010 0000
        static const uint8_t DR_STATUS_XOW_MASK =                   0x10; //b0001 0000
        static const uint8_t DR_STATUS_ZYXDR_MASK =                 0x08; //b0000 1000
        static const uint8_t DR_STATUS_ZDR_MASK =                   0x04; //b0000 0100
        static const uint8_t DR_STATUS_YDR_MASK =                   0x02; //b0000 0010
        static const uint8_t DR_STATUS_XDR_MASK =                   0x01; //b0000 0001
        static const uint8_t SYSMOD_MASK =                          0x03; //b0000 0011
        static const uint8_t OFF_X_LSB_MASK =                       0xFE; //b1111 1110
        static const uint8_t OFF_Y_LSB_MASK =                       0xFE; //b1111 1110
        static const uint8_t OFF_Z_LSB_MASK =                       0xFE; //b1111 1110
        static const uint8_t CTRL_REG1_DR_MASK =                    0xB0; //b1110 0000
        static const uint8_t CTRL_REG1_OS_MASK =                    0x18; //b0001 1000
        static const uint8_t CTRL_REG1_FR_MASK =                    0x04; //b0000 0100
        static const uint8_t CTRL_REG1_TM_MASK =                    0x02; //b0000 0010
        static const uint8_t CTRL_REG1_AC_MASK =                    0x01; //b0000 0001
        static const uint8_t CTRL_REG2_AUTO_MRST_EN_MASK =          0x80; //b1000 0000
        static const uint8_t CTRL_REG2_RAW_MASK =                   0x20; //b0010 0000
        static const uint8_t CTRL_REG2_MAG_RST_MASK =               0x10; //b0001 0000
        
        //Mapping of data values
        static const float TEMP_DIV = 1; //deg Celsius/level. Note that 8-bit range is -128C to 127C, but the sensor can only do -40C to 125C.
        static const float DATA_CONVERSION = 0.10; //uT/level
        
        //////////////// VARIABLES /////////////////////
        //MAG3110 state ("cached" from the values actually on the device)
        volatile int16_t __x;
        volatile int16_t __y;
        volatile int16_t __z;
        
        bool __active;
        adc_smpl_rate_t __adc_rate;
        oversmpl_ratio_t __ratio;
        smpl_rate_t __rate;
};

#endif