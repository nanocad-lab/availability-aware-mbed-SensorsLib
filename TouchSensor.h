/* TouchSensor.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#ifndef TOUCHSENSOR_H
#define TOUCHSENSOR_H

#include "mbed.h"
#include "TSISensor.h"
#include "PeriodicSensor.h"

class TouchSensor : public PeriodicSensor {
    public:
        /**
         * Initialize the touch sensor
         */
        TouchSensor();
        
        /**
         * Destroy the touch sensor
         */
        ~TouchSensor();
        
        /**
         * Read Touch Sensor percentage value.
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * @returns percentage value between [0 ... 1]
         */
        float getPercentage(bool sampleNow);
        
        /**
         * Read Touch Sensor distance
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * @returns distance in mm. The value is between [0 ... 40]
         */
        uint8_t getDistance(bool sampleNow);
        
    private:
        /**
         * Interrupt service routine for sampling the touch sensor.
         */
        virtual void __sample_data_ISR();
    
        TSISensor __sensor;
        
        volatile uint8_t __distance;
        volatile float __percentage;
};

#endif