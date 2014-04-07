/* INA219.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#include "mbed.h"
#include "I2CSensor.h"
#include "INA219.h"
 
INA219::INA219(PinName sda, PinName scl, int i2c_addr) :
                                    I2CSensor(sda, scl, i2c_addr),
                                    PeriodicSensor(0.1), //default max sampling rate of 10Hz
                                    __bus_voltage_range_32V(true),
                                    __shunt_amp_gain(8),
                                    __shunt_resolution(12),
                                    __shunt_num_samples_in_average(0),
                                    __bus_resolution(12),
                                    __bus_num_samples_in_average(0),
                                    __active(true),
                                    __measure_shunt_voltage(true),
                                    __measure_bus_voltage(true),
                                    __continuous_measurements(true),
                                    __shunt_voltage(0),
                                    __bus_voltage(0),
                                    __power(0),
                                    __shunt_current(0),
                                    __calibration(0),
                                    __current_div(1),
                                    __power_div(1)
                    {
     
}

INA219::~INA219() { }

void INA219::selfInit() {
    __i2c.frequency(400000);
    reset();
    __calibrate();
}

void INA219::reset() {
    uint16_t data = CONFIG_RST_MASK;
    setRegister16b(CONFIG, data);
    wait(0.1);
    
    //reset private data to match device default state
    __bus_voltage_range_32V = true;
    __shunt_amp_gain = 8;
    __shunt_resolution = 12;
    __shunt_num_samples_in_average = 0;
    __bus_resolution = 12;
    __bus_num_samples_in_average = 0;
    __active = true;
    __measure_shunt_voltage = true;
    __measure_bus_voltage = true;
    __continuous_measurements = true;
    __shunt_voltage = 0;
    __bus_voltage = 0;
    __power = 0;
    __shunt_current = 0;
    __calibration = 0;
    __current_div = 1;
    __power_div = 1;
}

void INA219::setBusVoltageRange32V(bool enable) {
    uint16_t data = getRegister16b(CONFIG);
    if (enable) {
        data |= CONFIG_BRNG_MASK; //set bit
        __bus_voltage_range_32V = true;
    }
    else {
        data &= ~CONFIG_BRNG_MASK; //clear bit
        __bus_voltage_range_32V = false;
    }
    setRegister16b(CONFIG, data);
}

bool INA219::isBusVoltageRange32V() {
    return __bus_voltage_range_32V;
}

void INA219::setShuntAmpGain(unsigned int gain) {
    uint16_t tmp = 0;
    switch (gain) {
        case 1:
            __shunt_amp_gain = 1;
            break; //PG = b00
        case 2:
            tmp |= (CONFIG_PG_MASK & (1 << 11)); //PG = b01
            __shunt_amp_gain = 2;
            break;
        case 4:
            tmp |= (CONFIG_PG_MASK & (2 << 11)); //PG = b10
            __shunt_amp_gain = 4;
            break;
        case 8:
            tmp |= (CONFIG_PG_MASK & (3 << 11)); //PG = b11
            __shunt_amp_gain = 8;
            break;
        default:
            return; //bad input, do nothing
    }
    uint16_t data = getRegister16b(CONFIG);
    data = (data & (~CONFIG_PG_MASK)) | tmp; //Set the gain bits
    setRegister16b(CONFIG, data);
}

unsigned int INA219::getShuntAmpGain() {
    return __shunt_amp_gain;
}

void INA219::setADCResolutionAndAveraging(bool shunt, bool resolution, unsigned int value) {
    uint16_t code = 0;
    uint16_t resolution_tmp;
    uint16_t num_samples_in_average_tmp;
    
    if (resolution) {
        switch (value) {
            case 9:
                resolution_tmp = 9;
                code = 0x00; //b0000 0X00
                break; 
            case 10:
                resolution_tmp = 10;
                code = 0x01; //b0000 0X01
                break;
            case 11:
                resolution_tmp = 11;
                code = 0x02; //b0000 0X10
                break;
            case 12:
                resolution_tmp = 12;
                code = 0x03; //b0000 0X11, default
                break;
            default:
                return; //bad input, do nothing
        } 
        num_samples_in_average_tmp = 0; 
    } else { //sample averaging
        switch (value) {
            case 2:
                num_samples_in_average_tmp = 2;
                code = 0x09; //b0000 1001
                break; 
            case 4:
                num_samples_in_average_tmp = 4;
                code = 0x0A; //b0000 1010
                break; 
            case 8:
                num_samples_in_average_tmp = 8;
                code = 0x0B; //b0000 1011
                break;
            case 16:
                num_samples_in_average_tmp = 16;
                code = 0x0C; //b0000 1100
                break;
            case 32:
                num_samples_in_average_tmp = 32;
                code = 0x0D; //b0000 1101
                break;
            case 64:
                num_samples_in_average_tmp = 64;
                code = 0x0E; //b0000 1110
                break;
            case 128:
                num_samples_in_average_tmp = 128;
                code = 0x0F; //b0000 1111
                break;
            default:
                return; //bad input, do nothing
        } 
        resolution_tmp = 0; 
    }
    
    //Now set the actual bits based on whether it is shunt or bus setting
    uint16_t data = getRegister16b(CONFIG);
    if (shunt) { //shunt ADC
        data &= ~CONFIG_SADC_MASK; //clear SADC bits
        data |= (code << 3); //set SADC bits
        __shunt_resolution = resolution_tmp;
        __shunt_num_samples_in_average = num_samples_in_average_tmp;
    } else { //bus ADC
        data &= ~CONFIG_BADC_MASK; //clear BADC bits
        data |= (code << 7); //set BADC bits
        __bus_resolution = resolution_tmp;
        __bus_num_samples_in_average = num_samples_in_average_tmp;
    }
    
    setRegister16b(CONFIG, data); //update the register
}

unsigned int INA219::getADCResolutionAndAveraging(bool shunt, bool &resolution) {
    if (shunt) { //get shunt ADC setting
        if (__shunt_resolution == 0) { //sample averaging
            resolution = false;
            return __shunt_num_samples_in_average;
        } else if (__shunt_num_samples_in_average == 0) { //resolution
            resolution = true;
            return __shunt_resolution;
        } else { //this should never happen
            return 0; //to keep compiler happy, this should never get reached
        }
    }
    else {//get bus ADC setting
        if (__bus_resolution == 0) { //sample averaging
            resolution = false;
            return __bus_num_samples_in_average;
        } else if (__bus_num_samples_in_average == 0) { //resolution
            resolution = true;
            return __bus_resolution;
        } else { //this should never happen
            return 0; //to keep compiler happy, this should never get reached
        }
    }
}

void INA219::setMode(bool shuntVoltage, bool busVoltage, bool continuous) {
    uint16_t code = 0;
    
    //Handle all 8 cases
    if (continuous) {
        code += 4;
        __continuous_measurements = true;
    } else
        __continuous_measurements = false;
    if (busVoltage) {
        code += 2;
        __measure_bus_voltage = true;
    } else
        __measure_bus_voltage = false;
    if (shuntVoltage) {
        code += 1;
        __measure_shunt_voltage = true;
    } else
        __measure_shunt_voltage = false;
        
    uint16_t data = getRegister16b(CONFIG);
    data &= ~CONFIG_MODE_MASK; //Clear mode bits
    data |= (CONFIG_MODE_MASK & code); //Set mode bits
    setRegister16b(CONFIG, data);
}

void INA219::getMode(bool &shuntVoltage, bool &busVoltage, bool &continuous) {
    shuntVoltage = __measure_shunt_voltage;
    busVoltage = __measure_bus_voltage;
    continuous = __continuous_measurements;
}


int16_t INA219::getShuntVoltage(bool sampleNow) {
     __disable_irq();
    if (sampleNow)
        __shunt_voltage = (int16_t) getRegister16b(SHUNT_VOLTAGE);
    __dataReady = false;
    __enable_irq();
    
    return __shunt_voltage;
}

float INA219::getShuntVoltageFloat(bool sampleNow) {
    return getShuntVoltage(sampleNow) * SHUNT_VOLTAGE_DIV;
}

int16_t INA219::getBusVoltage(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        uint16_t tmp = getRegister16b(BUS_VOLTAGE);
        tmp &= BUS_VOLTAGE_BD_MASK; //Clear the irrelevant bits
        int16_t data = (((int16_t) tmp) >> 3); //extract bits and keep sign from arithmetic right shift
        __bus_voltage = data;
    }
    __dataReady = false;
    __enable_irq();
    
    return __bus_voltage;
}

float INA219::getBusVoltageFloat(bool sampleNow) {
    return getBusVoltage(sampleNow) * BUS_VOLTAGE_DIV;
}

int16_t INA219::getPower(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        uint16_t unsignedPower = getRegister16b(POWER); //The INA219 does not internally track sign of power, only current and voltage, so we need to do it
        __shunt_current = (int16_t) getRegister16b(CURRENT);
        if (__shunt_current < 0) //check sign
            __power = -((int16_t) unsignedPower);
        else
            __power = (int16_t) unsignedPower;
    }
    __dataReady = false;
    __enable_irq();
    
    return __power;
}

float INA219::getPowerFloat(bool sampleNow) {
    return getPower(sampleNow) * __power_div;
}

int16_t INA219::getCurrent(bool sampleNow) {
    __disable_irq();
    if (sampleNow)
        __shunt_current = (int16_t) getRegister16b(CURRENT);   
    __dataReady = false;
    __enable_irq();
    
    return __shunt_current;
}

float INA219::getCurrentFloat(bool sampleNow) {
    return getCurrent(sampleNow) * __current_div;
}

uint16_t INA219::getCalibration() {
    __disable_irq();
    __calibration = getRegister16b(CURRENT);   
    __enable_irq();
    
    return __calibration;
}

void INA219::__calibrate() {
    /*float vbus_max = 6; //5V supply, so guardband to 6V just in case
    float rshunt = 0.1; //0.1 Ohm CSR
    float curr_max = 0.6; //600 mA worst case
    float vshunt_max = rshunt * curr_max;
    float curr_max_expected = 0.5; //500 mA expected worst case
    float min_lsb = 0.00001525925474; //curr_max_expected / 32767
    float max_lsb = 0.00012207031250; //curr_max_expected / 4096*/
    __current_div = 0.000016; //round up min_lsb to a nice number, in this case 16 uA
    __power_div = __current_div * 20;
    uint16_t calibration = 25600 & 0xFFFE; //trunc(0.04096 / (sel_lsb * Rshunt))
    setRegister16b(CALIBRATION, calibration);
    __calibration = calibration;
}

void INA219::__sample_data_ISR() {
    getBusVoltage(true);
    getShuntVoltage(true);
    getCurrent(true);
    getPower(true);
    __dataReady = true;
}