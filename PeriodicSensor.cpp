/* PeriodicSensor.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#include "mbed.h"
#include "PeriodicSensor.h"

PeriodicSensor::PeriodicSensor() :
                __dataReady(false),
                __interrupt(),
                __sample_period(0),
                __background_sampling(false),
                __min_sample_period(0.05) {
    __interrupt.detach();
}

PeriodicSensor::PeriodicSensor(float min_sample_period) :
                __dataReady(false),
                __interrupt(),
                __sample_period(0),
                __background_sampling(false),
                __min_sample_period(0.05) {
    if (min_sample_period > 0)
        __min_sample_period = min_sample_period;
    __interrupt.detach();
}

PeriodicSensor::~PeriodicSensor() { }

bool PeriodicSensor::isDataReady() {
    return __dataReady;
}

void PeriodicSensor::enableBackgroundSampling(bool enable, float sample_period) {
    if (enable && sample_period >= __min_sample_period-0.0001) {
       __sample_period = sample_period;
       __background_sampling = true;
       __interrupt.attach(this, &PeriodicSensor::__sample_data_ISR, sample_period);
    }
    else if (!enable) {
        __sample_period = 0;
        __background_sampling = false;
        __interrupt.detach();   
    }
}

bool PeriodicSensor::isBackgroundSamplingEnabled() {
    return __background_sampling;   
}

float PeriodicSensor::getSamplePeriod() {
    return __sample_period;
}

float PeriodicSensor::getMinSamplePeriod() {
    return __min_sample_period;
}