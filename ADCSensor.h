/* ADCSensor.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#ifndef ADCSensor_H
#define ADCSensor_H

#include "mbed.h"
#include "PeriodicSensor.h"

class ADCSensor : public PeriodicSensor {
    public:
        /**
         * Constructs an ADCSensor.
         * @param adc_pin the ADC pin to use
         */
        ADCSensor(PinName adc_pin);
        
        ~ADCSensor();
        
        /**
         * @param sampleNow if true, samples the ADC returns it. if false, gets the last sampled value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns a raw ADC reading
         */
        uint16_t getSample(bool sampleNow);
        
        /**
         * @param sampleNow if true, samples the ADC returns it. if false, gets the last sampled value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the ADC reading in Volts
         */
        float getSampleFloat(bool sampleNow);
        
    private:
        /**
         * Interrupt service routine for sampling the ADC.
         */
        virtual void __sample_data_ISR();
        
        AnalogIn __adc;
        volatile uint16_t __data;
        
        const static float ADC_DIV = 0.00005035; //Volts/level
};

#endif