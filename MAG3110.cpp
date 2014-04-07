/* MAG3110.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#include "mbed.h"
#include "I2CSensor.h"
#include "PeriodicSensor.h"
#include "MAG3110.h"

using namespace std;

MAG3110::MAG3110(PinName sda, PinName scl, int i2c_addr) : 
                                    I2CSensor(sda, scl, i2c_addr),
                                    PeriodicSensor(0.05), //default max sampling rate of 20Hz
                                    __x(0),
                                    __y(0),
                                    __z(0),
                                    __active(false),
                                    __adc_rate(adc_smpl_rate_t(0)),
                                    __ratio(oversmpl_ratio_t(0)),
                                    __rate(smpl_rate_t(0))
                                    {
}

MAG3110::~MAG3110() {}

void MAG3110::selfInit() {
    __i2c.frequency(400000);
    setOutputSamplingParameters(HZ80, O16, NULL);
    
    //Enable auto magnetic sensor reset before each sample, as recommended in the datasheet
    uint8_t data = getRegister(CTRL_REG2);
    data |= CTRL_REG2_AUTO_MRST_EN_MASK; //Set the AUTO_MRST_EN bit
    setRegister(CTRL_REG2, data);
}

uint8_t MAG3110::whoAmI() {
    return getRegister(WHO_AM_I);
}
        
uint8_t MAG3110::getDataRegisterStatus() {
    return getRegister(DR_STATUS);      
}

uint8_t MAG3110::getSystemMode() {
    return getRegister(SYSMOD);       
}

bool MAG3110::isActive() {
    return (bool) (getRegister(CTRL_REG1) & 0x01);
}

void MAG3110::setActive(bool activate) {
    uint8_t data;
    data = getRegister(CTRL_REG1);
    if (activate)
        data |= CTRL_REG1_AC_MASK; //Set bit
    else
        data &= ~CTRL_REG1_AC_MASK; //Clear bit
    setRegister(CTRL_REG1, data);
    __active = activate;
}


void MAG3110::getOutputSamplingParameters(smpl_rate_t *rate, oversmpl_ratio_t *ratio, adc_smpl_rate_t *adc_rate) {
    if (rate != NULL)
        *rate = __rate;
    if (ratio != NULL)
        *ratio = __ratio;
    if (ratio != NULL)
        *adc_rate = __adc_rate;
}


bool MAG3110::setOutputSamplingParameters(smpl_rate_t rate, oversmpl_ratio_t ratio, adc_smpl_rate_t *adc_rate) {
    uint8_t dr;
    uint8_t os;
    adc_smpl_rate_t tmp_adc_rate;
    
    switch (rate) {
        default:
        case HZ80:
            switch (ratio) {
                case O16:
                    tmp_adc_rate = AHZ1280;
                    dr = 0;
                    os = 0;
                    break;
                //Other rate-ratio combinations are illegal
                default:
                    return false;
            }
            break;
        
        case HZ40:
            switch (ratio) {
                case O16:
                    tmp_adc_rate = AHZ640;
                    dr = 1;
                    os = 0;
                    break;
                case O32:
                    tmp_adc_rate = AHZ1280;
                    dr = 0;
                    os = 1;
                    break;
                //Other rate-ratio combinations are illegal
                default:
                    return false; 
            }  
            break;
        
        case HZ20:
            switch (ratio) {
                case O16:
                    tmp_adc_rate = AHZ320;
                    dr = 2;
                    os = 0;
                    break;
                case O32: 
                    tmp_adc_rate = AHZ640;
                    dr = 1;
                    os = 1;
                    break;
                case O64:
                    tmp_adc_rate = AHZ1280;
                    dr = 0;
                    os = 2;
                    break;
                //Other rate-ratio combinations are illegal
                default:
                    return false;
            }
            break;
            
        case HZ10:
            switch (ratio) {
                case O16:
                    tmp_adc_rate = AHZ160;
                    dr = 3;
                    os = 0;
                    break;
                case O32:
                    tmp_adc_rate = AHZ320;
                    dr = 2;
                    os = 1;
                    break;
                case O64:
                    tmp_adc_rate = AHZ640;
                    dr = 1;
                    os = 2;
                    break;
                case O128:
                    tmp_adc_rate = AHZ1280;
                    dr = 0;
                    os = 3;
                    break;
                //This should be impossible
                default:
                    return false;     
            }
            break;
        
        case HZ5:
            switch (ratio) {
                case O16:
                    tmp_adc_rate = AHZ80;
                    dr = 4;
                    os = 0;
                    break;
                case O32:
                    tmp_adc_rate = AHZ160;
                    dr = 3;
                    os = 1;
                    break;
                case O64:
                    tmp_adc_rate = AHZ320;
                    dr = 2;
                    os = 2;
                    break;
                case O128:
                    tmp_adc_rate = AHZ640;
                    dr = 1;
                    os = 3;
                    break;
                //This should be impossible
                default:
                    return false;  
            }
            break;
            
        case HZ2_5:
            switch (ratio) {
                case O16:
                    tmp_adc_rate = AHZ80;
                    dr = 5;
                    os = 0;
                    break;
                case O32:
                    tmp_adc_rate = AHZ80;
                    dr = 4;
                    os = 1;
                    break;
                case O64:
                    tmp_adc_rate = AHZ160;
                    dr = 3;
                    os = 2;
                    break;
                case O128:
                    tmp_adc_rate = AHZ320;
                    dr = 2;
                    os = 3;
                    break;
                //This should be impossible
                default:
                    return false;
            }
            break;
            
            case HZ1_25:
                switch (ratio) {
                    case O16:
                        tmp_adc_rate = AHZ80;
                        dr = 6;
                        os = 0;
                        break;
                    case O32:
                        tmp_adc_rate = AHZ80;
                        dr = 5;
                        os = 1;
                        break;
                    case O64:
                        tmp_adc_rate = AHZ80;
                        dr = 4;
                        os = 2;
                        break;
                    case O128:
                        tmp_adc_rate = AHZ160;
                        dr = 3;
                        os = 3;
                        break;
                    //This should be impossible
                    default:
                        return false;   
                }
                break;
            
            case HZ0_63:
                switch (ratio) {
                    case O16:
                        tmp_adc_rate = AHZ80;
                        dr = 7;
                        os = 0;
                        break;
                    case O32:
                        tmp_adc_rate = AHZ80;
                        dr = 6;
                        os = 1;
                        break;
                    case O64:
                        tmp_adc_rate = AHZ80;
                        dr = 5;
                        os = 2;
                        break;
                    case O128:
                        tmp_adc_rate = AHZ80;
                        dr = 4;
                        os = 3;
                        break;
                    //This should be impossible
                    default:
                        return false;   
                }
                break;
                
            case HZ0_31:
                switch (ratio) {
                    case O32:
                        tmp_adc_rate = AHZ80;
                        dr = 7;
                        os = 1;
                        break;
                    case O64:
                        tmp_adc_rate = AHZ80;
                        dr = 6;
                        os = 2;
                        break;
                    case O128:
                        tmp_adc_rate = AHZ80;
                        dr = 5;
                        os = 3;
                        break;
                    default:
                        return false;   
                }
                break;
                
            case HZ0_16:
                switch (ratio) {
                    case O64:
                        tmp_adc_rate = AHZ80;
                        dr = 7;
                        os = 2;
                        break;
                    case O128:
                        tmp_adc_rate = AHZ80;
                        dr = 6;
                        os = 3;
                        break;
                    default:
                        return false;   
                }
                break;
                
            case HZ0_08:
                switch (ratio) {
                    case O128:
                        tmp_adc_rate = AHZ80;
                        dr = 7;
                        os = 3;
                        break;
                    default:
                        return false;   
                }
                break;
    }
    
    //Deactivate to update register
    bool wasActive = __active;
    setActive(false);
    
    //Update value for the caller
    if (adc_rate != NULL)
        *adc_rate = tmp_adc_rate;
        
    //Update CTRL_REG1 DR and OS fields
    uint8_t data = getRegister(CTRL_REG1);
    uint8_t dr_os = (dr << 5) | (os << 3); //Set DR in 3 MSB, OS in next 2 bits. 3 LSB are 0
    data = (data & ~(CTRL_REG1_DR_MASK | CTRL_REG1_OS_MASK)) | ((CTRL_REG1_DR_MASK | CTRL_REG1_OS_MASK) & dr_os); //Update 5 MSB
    setRegister(CTRL_REG1, data);
    
    //Update cached values
    __adc_rate = tmp_adc_rate;
    __ratio = ratio;
    __rate = rate;
    
    if (wasActive)
        setActive(true);
    
    return true;
}



int16_t MAG3110::getX(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        uint8_t data_msb, data_lsb;
        data_msb = getRegister(OUT_X_MSB);
        data_lsb = getRegister(OUT_X_LSB);
        __x = data_msb << 8;
        __x |= data_lsb;
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __x;
}

int16_t MAG3110::getY(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        uint8_t data_msb, data_lsb;
        data_msb = getRegister(OUT_Y_MSB);
        data_lsb = getRegister(OUT_Y_LSB);
        __y = data_msb << 8;
        __y |= data_lsb;
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __y;
}

int16_t MAG3110::getZ(bool sampleNow) {
    __disable_irq();    
    if (sampleNow) {
        uint8_t data_msb, data_lsb;
        data_msb = getRegister(OUT_Z_MSB);
        data_lsb = getRegister(OUT_Z_LSB);
        __z = data_msb << 8;
        __z |= data_lsb;
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __z;
}  

float MAG3110::getFloatX(bool sampleNow) {
    return getX(sampleNow) * DATA_CONVERSION;   
}

float MAG3110::getFloatY(bool sampleNow) {
    return getY(sampleNow) * DATA_CONVERSION;   
}

float MAG3110::getFloatZ(bool sampleNow) {
    return getZ(sampleNow) * DATA_CONVERSION;   
}

int8_t MAG3110::getDieTemp() {
    return (int8_t) getRegister(DIE_TEMP);
}

float MAG3110::getFloatDieTemp() {
    return getDieTemp() * TEMP_DIV;   
}

void MAG3110::__sample_data_ISR() {
    getX(true);
    getY(true);
    getZ(true);
    __dataReady = true;
}