/* MMA8451Q.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#ifndef MMA8451Q_H
#define MMA8451Q_H

#include "mbed.h"
#include "I2CSensor.h"
#include "PeriodicSensor.h"
#include "SleepableSensor.h"
        
/**
 * This class allows for easy control of an MMA8451Q accelerometer IC.
 */
class MMA8451Q : public I2CSensor, public PeriodicSensor, public SleepableSensor {
    public:
        /**
         * Enumeration of allowed output data sampling rates.
         */
        typedef enum {
            HZ800,
            HZ400,
            HZ200,
            HZ100,
            HZ50,
            HZ12_5,
            HZ6_25,
            HZ1_56
        } smpl_rate_t;
        
        /**
         * Enumeration of allowed dynamic ranges (full scale) of the accelerometer data.
         * G2: +/- 2G (0.25 mg/level at 14b, 15.6 mg/level at 8b)
         * G4: +/- 4G (0.5 mg/level at 14b, 31.25 mg/level at 8b)
         * G8: +/- 8G (1.0 mg/level at 14b, 62.5 mg/level at 8b)
         */
        typedef enum {
            G2,
            G4,
            G8
        } scale_t;
        
        /**
         * @param sda the pin identifier for SDA I2C signal
         * @param scl the pin identifier for SCL I2C signal
         * @param i2c_addr the 8-bit I2C address for this device. Note that LSB is a don't care.
         */
        MMA8451Q(PinName sda, PinName scl, int i2c_addr);
        
        /**
         */
        ~MMA8451Q();
        
        /**
         * Self-initialization to some nice preset. You must ensure the device is first deactivated using setActive().
         */
        void selfInit();
        
        /**
         * Does a soft reset of the device.
         */
        void reset();
        
        //I2C specific methods
        
        /**
         * Implements the pure virtual method of the parent I2CSensor class.
         * @returns the 8-bit device identifier.
         */
        uint8_t whoAmI();
        
        //Device-specific methods
        
        /**
         * @returns true if the device is active
         */
        bool isActive();
        
        /**
         * @param activate if true, enables the device, else disables it
         */
        void setActive(bool activate);
        
        /**
         * @returns the 8-bit system mode status
         */
        uint8_t getSystemMode();
        
        /**
         * @returns true if the device is generating 14b resolution data, otherwise 8b
         */
        bool is14bDataEnabled();
        
        /**
         * @param enable if true, sets 14b data, otherwise 8b data
         */
        void set14bData(bool enable);
        
        /**
         * @returns an enumerated value corresponding to the current output data sample rate
         */
        smpl_rate_t getOutputDataRate();
        
        /**
         * @param the enumerated value corresponding to the output data sample rate to use
         */
        void setOutputDataRate(smpl_rate_t rate);
        
        /**
         * @returns an enumerated value corresponding to the full scale (range) of the output data
         */
        scale_t getScale();
        
        /**
         * @param scale sets the sample scale via the enumerated value
         */
        void setScale(scale_t scale);
        
        /**
         * Enables an interrupt output signal from the device whenever data (XYZ) is generated.
         * @param enable Enables/disables interrupt
         * @param pinSelect if false, INT2. if true, INT1
         */
        void enableDataReadyInterrupt(bool enable, bool pinSelect);
        
        
        //Device-specific data retrieval methods
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns a 14-bit value representing the latest data sample for the X dimension, centered at 0.
         */
        int16_t getX(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns a 14-bit value representing the latest data sample for the Y dimension, centered at 0.
         */
        int16_t getY(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns a 14-bit value representing the latest data sample for the Z dimension, centered at 0.
         */
        int16_t getZ(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * Returns the latest X data reading as a float in Gs
         */
        float getFloatX(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * Returns the latest Y data reading as a float in Gs
         */
        float getFloatY(bool sampleNow);
        
        /**
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * Returns the latest Z data reading as a float in Gs
         */
        float getFloatZ(bool sampleNow);
        
        virtual void sleep();
        
        virtual void wake();
        
    private:
        /**
         * Interrupt service routine for fetching accelerometer data from the device.
         */
        virtual void __sample_data_ISR();
    
        ///////////////// CONSTANTS /////////////////////
        
        //Device register addresses
        static const uint8_t STATUS =           0x00;
        static const uint8_t F_STATUS =         0x00;        
        static const uint8_t OUT_X_MSB =        0x01;
        static const uint8_t OUT_X_LSB =        0x02;
        static const uint8_t OUT_Y_MSB =        0x03;
        static const uint8_t OUT_Y_LSB =        0x04;
        static const uint8_t OUT_Z_MSB =        0x05;
        static const uint8_t OUT_Z_LSB =        0x06;
        static const uint8_t F_SETUP =          0x09;
        static const uint8_t TRIG_CFG =         0x0A;
        static const uint8_t SYSMOD =           0x0B;
        static const uint8_t INT_SOURCE =       0x0C;
        static const uint8_t WHO_AM_I =         0x0D;
        static const uint8_t XYZ_DATA_CFG =     0x0E;
        static const uint8_t HP_FILTER_CUTOFF = 0x0F;
        static const uint8_t PL_STATUS =        0x10;
        static const uint8_t PL_CFG =           0x11;
        static const uint8_t PL_COUNT =         0x12;
        static const uint8_t PL_BF_ZCOMP =      0x13;
        static const uint8_t P_L_THS_REG =      0x14;
        static const uint8_t FF_MT_CFG =        0x15;
        static const uint8_t FF_MT_SRC =        0x16;
        static const uint8_t FF_MT_THS =        0x17;
        static const uint8_t FF_MT_COUNT =      0x18;
        static const uint8_t TRANSIENT_CFG =    0x1D;
        static const uint8_t TRANSIENT_SRC =    0x1E;
        static const uint8_t TRANSIENT_THS =    0x1F;
        static const uint8_t PULSE_CFG =        0x21;
        static const uint8_t PULSE_SRC =        0x22;
        static const uint8_t PULSE_THSX =       0x23;
        static const uint8_t PULSE_THSY =       0x24;
        static const uint8_t PULSE_THSZ =       0x25;
        static const uint8_t PULSE_TMLT =       0x26;
        static const uint8_t PULSE_LTCY =       0x27;
        static const uint8_t PULSE_WIND =       0x28;
        static const uint8_t ASLP_COUNT =       0x29;
        static const uint8_t CTRL_REG1 =        0x2A;
        static const uint8_t CTRL_REG2 =        0x2B;
        static const uint8_t CTRL_REG3 =        0x2C;
        static const uint8_t CTRL_REG4 =        0x2D;
        static const uint8_t CTRL_REG5 =        0x2E;
        static const uint8_t OFF_X =            0x2F;
        static const uint8_t OFF_Y =            0x30;
        static const uint8_t OFF_Z =            0x31;
                
        //Register masks
        static const uint8_t XYZ_DATA_CFG_HPF_OUT_MASK =            0x10; //b0001 0000
        static const uint8_t XYZ_DATA_CFG_FS_MASK =                 0x03; //b0000 0011
        static const uint8_t CTRL_REG1_ASLP_RATE_MASK =             0xC0; //b1100 0000
        static const uint8_t CTRL_REG1_DR_MASK =                    0x38; //b0011 1000
        static const uint8_t CTRL_REG1_LNOISE_MASK =                0x04; //b0000 0100
        static const uint8_t CTRL_REG1_F_READ_MASK =                0x02; //b0000 0010
        static const uint8_t CTRL_REG1_ACTIVE_MASK =                0x01; //b0000 0001     
        static const uint8_t CTRL_REG2_ST_MASK =                    0x80; //b1000 0000
        static const uint8_t CTRL_REG2_RST_MASK =                   0x40; //b0100 0000
        static const uint8_t CTRL_REG2_SMODS_MASK =                 0x18; //b0001 1000
        static const uint8_t CTRL_REG2_SLPE_MASK =                  0x04; //b0000 0100
        static const uint8_t CTRL_REG2_MODS_MASK =                  0x03; //b0000 0011     
        static const uint8_t CTRL_REG3_FIFO_GATE_MASK =             0x80; //b1000 0000
        static const uint8_t CTRL_REG3_WAKE_TRANS_MASK =            0x40; //b0100 0000
        static const uint8_t CTRL_REG3_WAKE_LNDPRT_MASK =           0x20; //b0010 0000
        static const uint8_t CTRL_REG3_WAKE_PULSE_MASK =            0x10; //b0001 0000
        static const uint8_t CTRL_REG3_WAKE_FF_MT_MASK =            0x08; //b0000 1000
        static const uint8_t CTRL_REG3_IPOL_MASK =                  0x02; //b0000 0010
        static const uint8_t CTRL_REG3_PP_OD_MASK =                 0x01; //b0000 0001    
        static const uint8_t CTRL_REG4_INT_EN_ASLP_MASK =           0x80; //b1000 0000
        static const uint8_t CTRL_REG4_INT_EN_FIFO_MASK =           0x40; //b0100 0000
        static const uint8_t CTRL_REG4_INT_EN_TRANS_MASK =          0x20; //b0010 0000
        static const uint8_t CTRL_REG4_INT_EN_LNDPR_MASK =          0x10; //b0001 0000
        static const uint8_t CTRL_REG4_INT_EN_PULSE_MASK =          0x08; //b0000 1000
        static const uint8_t CTRL_REG4_INT_EN_FF_MT_MASK =          0x04; //b0000 0100
        static const uint8_t CTRL_REG4_INT_EN_DRDY_MASK =           0x01; //b0000 0001  
        static const uint8_t CTRL_REG5_INT_CFG_ASLP_MASK =          0x80; //b1000 0000
        static const uint8_t CTRL_REG5_INT_CFG_FIFO_MASK =          0x40; //b0100 0000
        static const uint8_t CTRL_REG5_INT_CFG_TRANS_MASK =         0x20; //b0010 0000
        static const uint8_t CTRL_REG5_INT_CFG_LNDPRT_MASK =        0x10; //b0001 0000
        static const uint8_t CTRL_REG5_INT_CFG_PULSE_MASK =         0x08; //b0000 1000
        static const uint8_t CTRL_REG5_INT_CFG_FF_MT_MASK =         0x04; //b0000 0100
        static const uint8_t CTRL_REG5_INT_CFG_DRDY_MASK =          0x01; //b0000 0001
        
        //Mapping of sensor values
        static const float G2_DIV = 0.00024414; // in Gs/level, 1/4096
        static const float G4_DIV = 0.00048828; // in Gs/level, 1/2048
        static const float G8_DIV = 0.00097656; // in Gs/level, 1/1024
        
    
        ////////////// VARIABLES /////////////////
        
        //MMA8451Q state ("cached" from the values actually on the device)
        volatile int16_t __x;
        volatile int16_t __y;
        volatile int16_t __z;
        
        bool __active;
        bool __14b_data_enabled;
        smpl_rate_t __output_data_rate;
        scale_t __scale;
        float __div; //current sensor mapping
};

#endif