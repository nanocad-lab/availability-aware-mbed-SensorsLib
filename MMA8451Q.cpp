/* MMA8451Q.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
#include "mbed.h"
#include "I2CSensor.h"
#include "PeriodicSensor.h"
#include "SleepableSensor.h"
#include "MMA8451Q.h"

using namespace std;

//////////////////// PUBLIC METHODS ////////////////////

MMA8451Q::MMA8451Q(PinName sda, PinName scl, int i2c_addr) : 
                                        I2CSensor(sda, scl, i2c_addr), //parent constructor
                                        PeriodicSensor(0.05), //default max sampling rate of 20Hz
                                        SleepableSensor(),
                                        __x(0),
                                        __y(0),
                                        __z(0),
                                        __active(false),
                                        __14b_data_enabled(true),
                                        __output_data_rate(MMA8451Q::smpl_rate_t(0)),
                                        __scale(MMA8451Q::scale_t(0)),
                                        __div(G2_DIV)
                                        { }

MMA8451Q::~MMA8451Q() {}

void MMA8451Q::selfInit() {
    __i2c.frequency(400000);
    reset();
    setOutputDataRate(HZ800);
    setScale(G2);
    set14bData(true);
    //enableDataReadyInterrupt(true, true); //INT1
}

void MMA8451Q::reset() {
    uint8_t data = CTRL_REG2_RST_MASK;
    setRegister(CTRL_REG2, data);
    wait(0.1); 
}

uint8_t MMA8451Q::whoAmI() {
    return getRegister(WHO_AM_I);
}

bool MMA8451Q::isActive() {
    return __active;
}

void MMA8451Q::setActive(bool activate) {
    uint8_t data;
    data = getRegister(CTRL_REG1);
    if (activate)
        data |= CTRL_REG1_ACTIVE_MASK; //Set bit
    else
        data &= ~CTRL_REG1_ACTIVE_MASK; //Clear bit
    setRegister(CTRL_REG1, data);
    __active = activate;
}

uint8_t MMA8451Q::getSystemMode() {
    return getRegister(SYSMOD);  
}

bool MMA8451Q::is14bDataEnabled() {
    return __14b_data_enabled;
}

void MMA8451Q::set14bData(bool enable) {
    bool wasActive = isActive();
    if (wasActive)
        setActive(false); //deactivate before updating control bits
        
    uint8_t data;
    data = getRegister(CTRL_REG1);
    data = (data & ~CTRL_REG1_F_READ_MASK) | (CTRL_REG1_F_READ_MASK & enable); //Set 2nd LSB to enable
    setRegister(CTRL_REG1, data);
    __14b_data_enabled = enable;
    
    if (wasActive)
        setActive(true); //restore activity
}

MMA8451Q::smpl_rate_t MMA8451Q::getOutputDataRate() {
    return __output_data_rate;   
}

void MMA8451Q::setOutputDataRate(MMA8451Q::smpl_rate_t rate) {
    bool wasActive = __active;
    if (wasActive)
        setActive(false); //must disable to update register
        
    uint8_t data, dr;
    data = getRegister(CTRL_REG1);
    switch (rate) {
        case HZ800:
            dr = 0;
            break;
        case HZ400:
            dr = 1;
            break;
        case HZ200:
            dr = 2;
            break;
        case HZ100:
            dr = 3;
            break;
        case HZ50:
            dr = 4;
            break;
        case HZ12_5:
            dr = 5;
            break;
        case HZ6_25:
            dr = 6;
            break;
        case HZ1_56:
            dr = 7;
            break;
        default:
            dr = 0; //800HZ
            break;    
    }   
    data = (data & ~CTRL_REG1_DR_MASK) | (dr << 3);
    setRegister(CTRL_REG1, data);
    
    if (wasActive)
        setActive(true); //Restore active state
        
    __output_data_rate = rate;
}

MMA8451Q::scale_t MMA8451Q::getScale() {
    return __scale;
}

void MMA8451Q::setScale(MMA8451Q::scale_t scale) {
    bool wasActive = __active;
    if (wasActive)
        setActive(false); //deactivate before updating control bits
        
    uint8_t data = getRegister(XYZ_DATA_CFG);
    data = (data & ~XYZ_DATA_CFG_FS_MASK) | scale;
    setRegister(XYZ_DATA_CFG, data);
    
    if (wasActive)
        setActive(true); //restore activity
        
    __scale = scale;
    switch (scale) {
        default:
        case G2:
            __div = G2_DIV;
            break;
        case G4:
            __div = G4_DIV;
            break;
        case G8:
            __div = G8_DIV;
            break;
    }
}

void MMA8451Q::enableDataReadyInterrupt(bool enable, bool pinSelect) {
    //Deactivate sensor
    bool wasActive = __active;
    if (wasActive)
        setActive(false);

    //Configure interrupt
    uint8_t tmp;
    if (pinSelect)
        tmp = 0xFF;
    else
        tmp = 0x00;
    uint8_t data = tmp & CTRL_REG5_INT_CFG_DRDY_MASK; //Clear all other interrupt configurations, because I said so
    setRegister(CTRL_REG5, data);
    
    //Enable interrupt
    if (enable)
        tmp = 0xFF;
    else
        tmp = 0x00;
    data = tmp & CTRL_REG4_INT_EN_DRDY_MASK; //Clear all other interrupt configurations, because I said so
    setRegister(CTRL_REG4, data);
    
    //Clear IPOL bit (2nd LSB), such that active interrupt is LOW, and set PP_OD bit (LSB) such that it is in open drain mode
    data = getRegister(CTRL_REG3);
    data = (data & ~CTRL_REG3_IPOL_MASK); // | CTRL_REG3_PP_OD_MASK;
    setRegister(CTRL_REG3, data);
    
    //Re-activate sensor
    if (wasActive)
        setActive(true);    
}
        
int16_t MMA8451Q::getX(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        uint8_t data_msb, data_lsb;
        
        //Do bitwise ops on unsigned 8-bit parts
        uint16_t x_tmp = 0;
        data_msb = getRegister(OUT_X_MSB);
        if (__14b_data_enabled) {
            data_lsb = getRegister(OUT_X_LSB);
            data_lsb &= 0xFC; //ensure 2 LSB are cleared!
        } else
            data_lsb = 0;
        x_tmp = data_msb << 8;
        x_tmp |= data_lsb;
        
        //Now, treat the number as signed, then arithmetic right shift by 2
        __x = x_tmp;
        __x = __x >> 2;
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __x;
}

int16_t MMA8451Q::getY(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        uint8_t data_msb, data_lsb;
    
        //Do bitwise ops on unsigned 8-bit parts
        uint16_t y_tmp = 0;
        data_msb = getRegister(OUT_Y_MSB);
        if (__14b_data_enabled) {
            data_lsb = getRegister(OUT_Y_LSB);
            data_lsb &= 0xFC; //ensure 2 LSB are cleared!
        } else
            data_lsb = 0;
        y_tmp = data_msb << 8;
        y_tmp |= data_lsb;
        
        //Now, treat the number as signed, then arithmetic right shift by 2
        __y = y_tmp;
        __y = __y >> 2;
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __y;
}

int16_t MMA8451Q::getZ(bool sampleNow) {
    __disable_irq();
    if (sampleNow) {
        uint8_t data_msb, data_lsb;
        
        //Do bitwise ops on unsigned 8-bit parts
        uint16_t z_tmp = 0;
        data_msb = getRegister(OUT_Z_MSB);
        if (__14b_data_enabled) {
            data_lsb = getRegister(OUT_Z_LSB);
            data_lsb &= 0xFC; //ensure 2 LSB are cleared!
        } else
            data_lsb = 0;
        z_tmp = data_msb << 8;
        z_tmp |= data_lsb;
        
        //Now, treat the number as signed, then arithmetic right shift by 2
        __z = z_tmp;
        __z = __z >> 2;
    }
    
    __dataReady = false;
    __enable_irq();
    
    return __z;
}

float MMA8451Q::getFloatX(bool sampleNow) {
    return getX(sampleNow) * __div;   
}

float MMA8451Q::getFloatY(bool sampleNow) {
    return getY(sampleNow) * __div;   
}

float MMA8451Q::getFloatZ(bool sampleNow) {
    return getZ(sampleNow) * __div;   
}

void MMA8451Q::sleep() {
    
}

void MMA8451Q::wake() {
    
}

void MMA8451Q::__sample_data_ISR() {
    getX(true);
    getY(true);
    getZ(true);
    __dataReady = true;
}