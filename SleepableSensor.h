/* SleepableSensor.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#ifndef SLEEPABLE_SENSOR_H
#define SLEEPABLE_SENSOR_H

class SleepableSensor {
    public:
        SleepableSensor();
        ~SleepableSensor();
        
        virtual void sleep() = 0;
        virtual void wake() = 0;
        
        bool isAsleep();
        
    private:
        bool __sleeping;  
};

#endif