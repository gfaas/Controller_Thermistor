#define TRC_VERSION "2025.06.19"

#define HEAT 5                    //GPIO pin for heating      
#define MEASURE 4           //GPIO pin for measurement  
#define THERMISTOR A2                         // ADC_measure
#define REFERENCE A1                          // ADC_current

#define TRC_SIMULATED_ADDR 0x18 //I2C address simulated by thermistor regulator
//registers for calculations/sampling
#define TRC_REG_ON_OFF 0x00         //Sampling on/off
#define TRC_REG_TEMP 0x01       //register to R temperature
#define TRC_REG_T_SET 0x02      //register to RW temperature goal during pulse
#define TRC_REG_HEATING 0x03    //register to XXXX on heating pulse
#define TRC_REG_T1 0x04         //register to RW maximum time to reach goal temperature
#define TRC_REG_T2 0x05         //register to RW time to clamp goal temperature
#define TRC_REG_Kp 0x06         //register to RW P factor
#define TRC_REG_Ki 0x07         //register to RW I factor
#define TRC_REG_Kd 0x08         //register to RW D factor
#define TRC_REG_LIMIT 0x09      //register to RW max allowablke temperature (temperature shutoff)
// #DO NOT USE 0x10, reserved for stim!! (to avoid any confusion)
#define TRC_REG_REPEAT 0x0A      //register to RW sample loops per measurement
#define TRC_REG_DUR_HEAT 0x0B    //register to RW lenght of injection loop in us 
#define TRC_REG_FIRMWARE 0x0C
//registers for hardware settings
#define TRC_HARDWARE_Rt 0x0D    //RW thermistor resistance in ohms
#define TRC_HARDWARE_Rf 0x0E    //RW feedback resistor in ohms
#define TRC_HARDWARE_Rc 0x0F    //RW restsance corrected for cable in ohms total

#define TRC_REG_H_DUITY 0x20 //register to R current heating duty cycle (0-9999) for PWM

#define TRC_REPEATS 500
#define TRC_RAISE_TIME 2000
#define TRC_H_DURATION 500
#define TRC_T_LIMIT 120
#define TRC_PID_P 200
#define TRC_PID_I 30
#define TRC_PID_D 200
#define TRC_R_THERMISTOR 100
#define TRC_R_FEEDBACK 20000
#define TRC_R_CABLE 2