/* SleepableSensor.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#include "SleepableSensor.h"

SleepableSensor::SleepableSensor() :
                        __sleeping(false)
                        {
}

SleepableSensor::~SleepableSensor() { }

bool SleepableSensor::isAsleep() {
    return __sleeping;   
}