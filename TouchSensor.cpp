/* TouchSensor.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#include "mbed.h"
#include "TouchSensor.h"
#include "PeriodicSensor.h"

TouchSensor::TouchSensor() :
                PeriodicSensor(0.005), //default min sampling rate of 200 Hz
                __sensor(),
                __distance(0),
                __percentage(0)
                {
}

TouchSensor::~TouchSensor() { }

float TouchSensor::getPercentage(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        __percentage = __sensor.readPercentage();
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __percentage;
}

uint8_t TouchSensor::getDistance(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        __distance = __sensor.readDistance();
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __distance;   
}

void TouchSensor::__sample_data_ISR() {
    getDistance(true);
    getPercentage(true);
    __dataReady = true;
}