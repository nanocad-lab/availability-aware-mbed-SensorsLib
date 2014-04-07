/* ADCSensor.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#include "mbed.h"
#include "ADCSensor.h"
#include "PeriodicSensor.h"

ADCSensor::ADCSensor(PinName adc_pin) :
                    PeriodicSensor(0.005), //default max sampling rate of 200Hz
                    __adc(adc_pin),
                    __data(0)
                    {
}

ADCSensor::~ADCSensor() { }

uint16_t ADCSensor::getSample(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        __data = __adc.read_u16();
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __data;   
}

float ADCSensor::getSampleFloat(bool sampleNow) {
    return getSample(sampleNow) * ADC_DIV;   
}

void ADCSensor::__sample_data_ISR() {
    getSample(true);
    __dataReady = true;
}