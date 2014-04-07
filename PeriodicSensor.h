/* PeriodicSensor.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#ifndef PERIODICSENSOR_H
#define PERIODICSENSOR_H

#include "mbed.h"

/**
 * Abstract class from which most sensor classes can be derived.
 * This class implements interrupt-driven periodic sampling
 */
class PeriodicSensor {
    public:
        /**
         * Constructs a PeriodicSensor object. Default minimum sampling period is set to 0.05 sec.
         */
        PeriodicSensor();
        
        /**
         * Constructs a PeriodicSensor object with a custom minimum sampling period (must be positive).
         */
        PeriodicSensor(float min_sampling_period);
         
        /**
         * Destroys the object
         */
        ~PeriodicSensor();
        
        /**
         * @returns true if there is new data ready to be retrieved.
         */
        bool isDataReady();
        
        /**
         * Control the background sampling of the device via interrupts. If disabled, the device can still be sampled using the getter methods directly.
         * @param enable if true, enables background sampling with the given sample period.
         * @param sample_period sampling period in seconds. Must be at least 0.005 sec unless enable is false, in which case it is don't care.
         */
        void enableBackgroundSampling(bool enable, float sample_period);
        
        /**
         * @returns true if background sampling is enabled.
         */
        bool isBackgroundSamplingEnabled();
        
        /**
         * @returns the sample period in seconds, if background sampling is enabled. Else, non-positive return.
         */
        float getSamplePeriod();
        
        /**
         * @returns the minimum sample period in seconds
         */
        float getMinSamplePeriod();
        
    protected:
        volatile bool __dataReady;
        
    private:
        /**
         * This is the interrupt service routine that is called periodically when enabled.
         */
        virtual void __sample_data_ISR() = 0;
        
        Ticker __interrupt;
        float __sample_period;
        bool __background_sampling;
        float __min_sample_period;
};

#endif