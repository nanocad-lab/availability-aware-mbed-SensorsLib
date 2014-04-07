/* INA219.h
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */
 
 #ifndef INA219_H
 #define INA219_H
 
 #include "mbed.h"
 #include "I2CSensor.h"
 #include "PeriodicSensor.h"
 
 /**
 * This class allows for easy control over a INA219 current/power sensing IC.
 */
class INA219 : public I2CSensor, public PeriodicSensor {
    public:
        /**
         * @param sda the pin identifier for SDA I2C signal
         * @param scl the pin identifier for SCL I2C signal
         * @param i2c_addr the 8-bit I2C address for this device. Note that LSB is a don't care.
         */
        INA219(PinName sda, PinName scl, int i2c_addr);
        
        /**
         * Destroys this object
         */
        ~INA219();
        
        /**
         * Initializes the device to some nice state.
         */
        void selfInit();
        
        /**
         * Performs a software reset of the device.
         */
        void reset();
        
        /**
         * Sets the bus voltage range to either 16V or 32V.
         * @param enable if true, sets the bus voltage range to 32V. Else, sets it to 16V.
         */
        void setBusVoltageRange32V(bool enable);
        
        /**
         * @returns true if the bus voltage range is set to 32V. if false, bus voltage range is 16V.
         */
        bool isBusVoltageRange32V();
        
        /**
         * Sets the PGA amplifier gain for shunt voltage measurement.
         * @param gain the gain level to set. Allowed values: 1, 2, 4, 8. Default is 8. Any other values will have no effect.
         */
        void setShuntAmpGain(unsigned int gain);
        
        /**
         * Gets the PGA amplifier gain for shunt voltage measurement.
         * @returns the gain level
         */
        unsigned int getShuntAmpGain();
        
        /**
         * Sets the resolution and sample averaging of the bus or shunt ADC.
         * If resolution is set, sample averaging is disabled.
         * If sample averaging is set, resolution setting is disabled.
         * @param shunt if true, sets for shunt ADC. if false, sets for bus ADC.
         * @param resolution if true, sets ADC resolution using the third parameter. if false, sets the ADC sample averaging
         * using the third parameter.
         * @param value if resolution is true, then this is the resolution in bits to use. Allowed values:
         * 9, 10, 11, or 12 bits.
         * If resolution is false, then this is the number of samples to average in each data sample. Allowed values:
         * 2, 4, 8, 16, 32, 64, or 128 samples.
         * Any other values will cause this method to have no effect. Default is 12-bit resolution mode.
         */
        void setADCResolutionAndAveraging(bool shunt, bool resolution, unsigned int value);
        
        /**
         * Gets the resolution or number of samples in an average of the bus or shunt ADC.
         * @param shunt if true, gets for shunt ADC. if false, gets for bus ADC.
         * @param resolution sets this to true if the return value represents resolution in bits. else, sets it to false if return
         * value represents sample averaging in # samples.
         */
        unsigned int getADCResolutionAndAveraging(bool shunt, bool &resolution);
        
        /**
         * Sets the operating mode of the device. If all parameters are false, the device powers down.
         * If shuntVoltage and busVoltage are false but continuous is true, the device remains on but
         * the ADC is powered down.
         * @param shuntVoltage if true, samples the shunt voltage.
         * @param busVoltage if true, samples the bus voltage.
         * @param continuous if true, samples continuously, otherwise only on triggered reads.
         */
        void setMode(bool shuntVoltage, bool busVoltage, bool continuous);
        
        /**
         * Gets the operating mode of the device. If all parameters are false, the device is powered down.
         * If shuntVoltage and busVoltage are false but continuous is true, the device is on but
         * the ADC is powered down.
         * @param shuntVoltage sets true if shunt voltage is being measured.
         * @param busVoltage sets true if bus voltage is being measured.
         * @param continuous sets true if continuous readings are being done.
         */
        void getMode(bool &shuntVoltage, bool &busVoltage, bool &continuous);
        
        /**
         * Gets the shunt voltage across the current sensing resistor.
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns 16-bit integer representing the shunt voltage.
         */
        int16_t getShuntVoltage(bool sampleNow);
        
        /**
         * Gets the shunt voltage across the current sensing resistor.
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the shunt voltage in Volts
         */
        float getShuntVoltageFloat(bool sampleNow);
        
        
        /**
         * Gets the bus voltage at the negative terminal of the current sensing resistor (V-).
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns 16-bit integer representing the bus voltage.
         */
        int16_t getBusVoltage(bool sampleNow);
        
        /**
         * Gets the bus voltage across the current sensing resistor.
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the bus voltage in Volts
         */
        float getBusVoltageFloat(bool sampleNow);
        
        /**
         * Gets the power consumed by the load. Note that calling this method with sampleNow == true
         * will also implicitly sample current as well, which can be retrieved with a call to
         * getCurrent(false) after calling getPower(true).
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns 16-bit integer representing power consumption of the load.
         */
        int16_t getPower(bool sampleNow);
        
        /**
         * Gets the power consumed by the load. Note that calling this method with sampleNow == true
         * will also implicitly sample current as well, which can be retrieved with a call to
         * getCurrentFloat(false) after calling getPowerFloat(true).
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the load power in Watts
         */
        float getPowerFloat(bool sampleNow);
        
        /**
         * Gets the current sunk by the load.
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns 16-bit integer representing current sunk by the load.
         */
        int16_t getCurrent(bool sampleNow);
        
        /**
         * Gets the current delivered to the load
         * @param sampleNow if true, queries the device for the sample and returns it. if false, gets the last queried value.
         * The latter is preferred if this object is set up to sample using interrupts.
         * @returns the load current in Amperes
         */
        float getCurrentFloat(bool sampleNow);
        
        /**
         * Gets the calibration configuration setting for current range.
         * @returns 16-bit unsigned integer representing calibration register value
         */
        uint16_t getCalibration();
     
    private:
        /**
         * Interrupt service routine for fetching power data from the device.
         */
        virtual void __sample_data_ISR();
        
        /**
         * Sets the calibration register of the device so that current and power measurements can be made. NOTE: This is a custom method for my project.
         */
        void __calibrate();
        
        /////////////////////// VARIABLES //////////////////////////
        bool __bus_voltage_range_32V;
        unsigned int __shunt_amp_gain;
        unsigned int __shunt_resolution;
        unsigned int __shunt_num_samples_in_average;
        unsigned int __bus_resolution;
        unsigned int __bus_num_samples_in_average;
        bool __active;
        bool __measure_shunt_voltage;
        bool __measure_bus_voltage;
        bool __continuous_measurements;
        
        volatile int16_t __shunt_voltage;
        volatile int16_t __bus_voltage;
        volatile int16_t __power;
        volatile int16_t __shunt_current;
        volatile uint16_t __calibration;
        
        float __current_div; //Amps/level
        float __power_div; //Watts/level

        
        
        /////////////////////// CONSTANTS //////////////////////////
        
        //Device register addresses
        const static uint8_t CONFIG =                     0x00;
        const static uint8_t SHUNT_VOLTAGE =              0x01;
        const static uint8_t BUS_VOLTAGE =                0x02;
        const static uint8_t POWER =                      0x03;
        const static uint8_t CURRENT =                    0x04;
        const static uint8_t CALIBRATION =                0x05;
        
        //Register masks
        const static uint16_t CONFIG_RST_MASK =           0x8000; //b1000 0000 0000 0000
        const static uint16_t CONFIG_BRNG_MASK =          0x2000; //b0010 0000 0000 0000
        const static uint16_t CONFIG_PG_MASK =            0x1800; //b0001 1000 0000 0000
        const static uint16_t CONFIG_BADC_MASK =          0x0780; //b0000 0111 1000 0000
        const static uint16_t CONFIG_SADC_MASK =          0x0078; //b0000 0000 0111 1000
        const static uint16_t CONFIG_MODE_MASK =          0x0007; //b0000 0000 0000 0111
        
        //No masks needed for SHUNT_VOLTAGE register
        
        const static uint16_t BUS_VOLTAGE_BD_MASK =       0xFFF8; //b1111 1111 1111 1000
        const static uint16_t BUS_VOLTAGE_CNVR_MASK =     0x0002; //b0000 0000 0000 0010
        const static uint16_t BUS_VOLTAGE_OVF_MASK =      0x0001; //b0000 0000 0000 0001
        
        //No masks needed for POWER register
        
        const static uint16_t CURRENT_CSIGN_MASK =        0x8000; //b1000 0000 0000 0000
        const static uint16_t CURRENT_CD_MASK =           0x7FFF; //b0111 1111 1111 1111
        
        const static uint16_t CALIBRATION_FS_MASK =       0xFFFE; //b1111 1111 1111 1110
        
        const static float SHUNT_VOLTAGE_DIV =            0.00001; //10uV / level
        const static float BUS_VOLTAGE_DIV =              0.004; //4mV / level
};
 
#endif