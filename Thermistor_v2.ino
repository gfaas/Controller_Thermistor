#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"


#define HEAT 5                          
#define MEASURE 4
#define THERMISTOR A2                         // A_measure
#define REFERENCE A1                          // A_current

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

#define TRC_REDBLINK 0x20
#define TRC_GREENBLINK 0x21
#define TRC_BLUEBLINK 0x22

#define TRC_REPEATS 55
#define TRC_RAISE_TIME 99
#define TRC_H_DURATION 3000
#define TRC_T_LIMIT 66
#define TRC_PID_P 2500
#define TRC_PID_I 15
#define TRC_PID_D 1500
#define TRC_R_THERMISTOR 100
#define TRC_R_FEEDBACK 20000
#define TRC_R_CABLE 2

#define TRC_VERSION "2025.04.23"

// #DO NOT USE 0xFF, reserved for RECORDING on/off on RBP (to avoid any confusion)

char textout[256]; //string for serial output
uint8_t i2c_input[8]; //buffer for I2C input
int counter=0;

#define NUMPIXELS        1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

uint8_t PWM_slice_num; // PWM slice number for PWM current injection

//constants for hardware
u_int32_t firmware_version= 20250423;
float R_ratio_refrence, alpha, R_ratio, temperature;
float R1= TRC_R_FEEDBACK;
float R2= TRC_R_THERMISTOR;
float R_wire= TRC_R_CABLE;
//variable for measurement
bool active = 0;  //activate device 0=off
float V_t=0;
float V_r=0;
float current_temp=30;     //Assumed starting temp
uint8_t* p_T= (uint8_t*)&current_temp; //pointer to first byte of measured temperature (4 byte float)
int32_t sample_repeat = TRC_REPEATS;

//variables for heating/temperature regulation
float T_set=0;          //Desired temperature
float T_limit= TRC_T_LIMIT;
bool flag_h=0;          //heating flag, TRUE when heating is active/temperature iis actively regulated
bool flag_c1=0;         //Flag for first timer that checks time how long the attempt to reach T is running, TRUE if timer has started (has to be true for heating)
u_int32_t t1= TRC_RAISE_TIME;       //Time limit to reach T
u_int32_t timer_c1;     //Timer for clock descibed above (clock 1)
bool flag_c2=0;         //Flag for second timer that checks time how long ago T is reache , TRUE if timer has started (T has been reached)
int32_t t2=0;      //Time limit to clamp T
int32_t timer_c2;     //Timer for clock descibed above (clock 2)

// PID constants
float Kp = TRC_PID_P;     
float Ki = TRC_PID_I;
float Kd = TRC_PID_D;
// PID variables
float error;            //error in temperature
float previous_error = 0;
float integral = 0;
float derivative = 0;
uint32_t  pid_output = 0;
int32_t heat_duration = 3000; //duration of heating in us. should be well over 80uS. Total cycle time is heat_duration + 200 us

uint8_t incoming[4];


void setup() 
{
  Serial.begin(9600);
  //setpinmodes
  pinMode(HEAT, OUTPUT);                    
  digitalWrite(HEAT, LOW);                 
  pinMode(MEASURE, OUTPUT);
  digitalWrite(MEASURE,LOW);   
  gpio_set_function(HEAT, GPIO_FUNC_PWM); //set HEAT to PWM
  PWM_slice_num = pwm_gpio_to_slice_num(HEAT); //find out which PWM slice is connected to GPIO 0 (it's slice 0)
  pwm_set_wrap(PWM_slice_num, 10000); //set period of 1000 cycles (0 to 999 inclusive)  at 125 MHz this is a 80 uS
  analogReadResolution(12);

  #if defined(NEOPIXEL_POWER)
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, HIGH);
  #endif

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(20); // not so bright

  delay(1000);  //join i2c after control has started
  //start I2C
  Wire.begin(TRC_SIMULATED_ADDR);                             // join i2c bus with address #0x18, subject to change
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent); 
}


void loop() 
{
  if(active>0)
  {
    current_temp=MeasureRT(sample_repeat);
  }
  
  //sprintf(textout, "temp: %.2fC  rep: %d  clamp: %2.2fC  t1: %4d  t2: %4d  H_dur: %4d  T_max: %3.0f  P: %4.0f I: %4.0f D: %4.0f ver: %d Rt: %.0f Rf: %.0f Rc: %.2f", current_temp, sample_repeat,T_set, t1, t2, heat_duration, T_limit, Kp, Ki, Kd, firmware_version, R2, R1, R_wire);
  sprintf(textout, "UINT32: rep: %d t1: %4d H_dur: %4d      FLOATS: T_max: %3.0f  P: %4.0f I: %4.0f D: %4.0f Rt: %.0f Rf: %.0f Rc: %.2f", sample_repeat, t1, heat_duration, T_limit, Kp, Ki, Kd, R2, R1, R_wire);
  Serial.println(textout);

  if (flag_h) //if heat clamping is active
  {          
    if (!flag_c1) //START FIRST TIMER FOR MAXIMUM TIME ALLOWED TO REACH DESIRED TEMP
    {   
      flag_c1=1;
      timer_c1=millis();
      integral = 0;
    }
    error = T_set - current_temp; // Calculate the error
    integral += error; // Calculate the integral
    derivative = error - previous_error; // Calculate the derivative
    pid_output = Kp * error + Ki * integral + Kd * derivative; // PID output
    previous_error = error; 
    if(active>0)
    {
      HeatRT(heat_duration, pid_output); // Heat for 3000 us with duty cycle pid_output
    }
  }

  if ((millis()-timer_c1)>t1 && flag_c1 && !flag_c2) //if T1 has lapsed without reaching clamp temperature 
  {  
    AllStop();
  }

  if ((current_temp >= T_set) && flag_h && !flag_c2)  //START SECOND TIMER FOR CLAMP
  {  
    flag_c1=0;
    flag_c2=1;
    timer_c2=millis();
  }

  if (((millis()-timer_c2)>t2 && flag_c2) || current_temp>T_limit) //clamp time (T2) has lapsed or abolute temp limit has been reached -> SHUTOFF
    {   
     AllStop();
    }
} 

void AllStop() 
{
  flag_c1=0;
  flag_c2=0;
  flag_h=0;
  t2=0;
  T_set=0;
  integral = 0;
  derivative = 0;
  pid_output=0;
  HeatRT(1000, 0); //flush PWM
}

float MeasureRT(int repeats) 
{ 
  V_t=0;
  V_r=0;
  temperature = 0;
  digitalWrite(MEASURE, HIGH);
  delayMicroseconds (500);
  for (int i=0; i<repeats; i++)
  {
    V_t = V_t+(float)analogRead(THERMISTOR);
    V_r = V_r+(float)analogRead(REFERENCE);
  }
  digitalWrite(MEASURE, LOW);
  alpha = V_r/V_t;
  R_ratio = 1/((alpha*(R1+R2) - R2)/R1) - (R_wire)/R2;
  temperature = (28.54*pow(R_ratio, 3) - 158.5*pow(R_ratio, 2) + 474.8*R_ratio - 319.85);
  return temperature;
}

void HeatRT(uint16_t heat_time, uint16_t heat_duty_cycle) //heat_time in us, heat_duty_cycle in 0-10000
{
    pwm_set_chan_level(PWM_slice_num, PWM_CHAN_B, heat_duty_cycle);
    pwm_set_enabled(PWM_slice_num, true);
    delayMicroseconds(heat_time);
    pwm_set_chan_level(PWM_slice_num, PWM_CHAN_B, 0);   //some time to make sure the cycle always ends low
    delayMicroseconds(200);
    pwm_set_enabled(PWM_slice_num, false);
}

void requestEvent() //SENDING DATA TO RBP
{     
  switch (i2c_input[0])
  {
    case TRC_REG_TEMP:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&current_temp+i));
      };
    case TRC_REG_T_SET:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&T_set+i));
      };
      break;
    case TRC_REG_HEATING:
      Wire.write(flag_h);
      break;
    case TRC_REG_T1:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&t1+i));
      }
      break;
    case TRC_REG_T2:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&t2+i));
      }
      break;
    case TRC_REG_Kp:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&Kp+i));
      }
      break;

    case TRC_REG_Ki:
      for (int i=0;i<4;i++){
        Wire.write(*((uint8_t*)&Ki+i));
      }
      break;

    case TRC_REG_Kd:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&Kd+i));
      }
      break;
      
    case TRC_REG_LIMIT:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&T_limit+i));
      }
      break;

    case TRC_REG_REPEAT: 
      for (int i=0;i<4;i++)
      {   
        Wire.write(*((uint8_t*)&sample_repeat+i));
      }
      break;

    case TRC_REG_DUR_HEAT:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&heat_duration+i));
      }
      break;

    case TRC_REG_FIRMWARE:
      for (int i=0;i<4;i++)
      {
        Wire.write(*((uint8_t*)&firmware_version+i));
      }
      break;
    default:
      break;
  }     
}

void receiveEvent(int howMany)
{
  int i=0;
  while (i<8) 
  {
    i2c_input[i] = 0; i++;
  }
  i=0;
  while (Wire.available()) 
  {
    i2c_input[i] = Wire.read();
    i++;
  }
  if (howMany>1)  //RECEIVING DATA FROM RBP
  {
    flag_h=0;
    flag_c1=0;
    flag_c2=0;
    // for (int i=0;i<4;i++)
    //     {
    //       incoming[i]=i2c_input[i+1];
    //     }
    switch (i2c_input[0])
    {

      case TRC_REG_ON_OFF: //SAMPLING ON/OFF
        active = i2c_input[1];
        break;
      case TRC_REG_T_SET: //SET GOAL TEMPERATURE
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&T_set+i))=i2c_input[i+1];
        }
        break;
      case TRC_REG_HEATING: //START HEATING
            flag_h=i2c_input[1];
        break;
      case TRC_REG_T1: //SET T1 (MAX DELAY TIME TO REACH TEMP)
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&t1+i))=i2c_input[i+1];
          incoming[i]=i2c_input[i+1];
        }
        break;
      case TRC_REG_T2: //SET T2 (TIME TO CLAMP TEMP)
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&t2+i))=i2c_input[i+1];
        }
        // counter++;
        // t2=t2/1000;
        break;
      case TRC_REG_Kp: //SET td_amp (Amplitude of sigmoid function, FEEDBACK)
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&Kp+i))=i2c_input[i+1];
        }
        break;
      case TRC_REG_Ki: //SET td_half (half max value of sigmoid function, FEEDBACK)
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&Ki+i))=i2c_input[i+1];
        }
        break;
      case TRC_REG_Kd: //SET td_n (power slope of sigmoid function, FEEDBACK)
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&Kd+i))=i2c_input[i+1];
        }
        break;
      case TRC_REG_LIMIT: //SET T_LIMIT
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&T_limit+i))=i2c_input[i+1];
        }
        break;

      case TRC_REG_REPEAT: //SET SAMPLE REPEATS
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&sample_repeat+i))=i2c_input[i+1];
        }
        break;

      case TRC_REG_DUR_HEAT: //SET SAMPLE REPEATS
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&heat_duration+i))=i2c_input[i+1];
        }
        break;



      case TRC_HARDWARE_Rt: //SET R tghermistor
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&R2+i))=i2c_input[i+1];
        }
        break;
      case TRC_HARDWARE_Rf: //SET R feedback
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&R1+i))=i2c_input[i+1];
        }
        break;
      case TRC_HARDWARE_Rc: //SET R corrected
        for (int i=0;i<4;i++)
        {
          (*((uint8_t*)&R_wire+i))=i2c_input[i+1];
        }
        break;
      default:
      break;
    }
  }
}