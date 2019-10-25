//**************************************************
//
// A u t o p i l o t   M a x s i m p l e x
//
// Autopilot Maxsimplex
// incl. 270° Raymarine rudder sensor 4k5 Ohm
// incl. activating clutch
// sets autom. course_fix after start up
// Compass without tilt compensation, got from
// https://circuitdigest.com/microcontr...gital-compass-with-arduino-and-hmc5883l-magnetometer
// inserted tilt compensation per function
// compass output calmed by aritmetic Filter
//
//**************************************************

// Library for arduino pin
#include<Wire.h>

// Libraries for GY-88, files added to directory
#include "HMC5883L.h" 
#include "MPU6050.h"

// Library for I2C-Bus, file added to directory
#include "I2Cdev.h"


//**************************************************
// global var
//**************************************************
float TARGET_HEADING = 0;
int SENSOR_SMOOTHNESS = 10; // Settings, how much values for average


//**************************************************
// S e n o r s
//**************************************************
HMC5883L COMPASS; // compass
MPU6050 ACCELGYRO; // compass

const int MAG = 0x1E;   // I2C address of magnetic compass
const int MPU = 0x68;   // I2C address of the MPU-6050


//**************************************************
// A k t u a t o r s
//**************************************************
// 1/2 part of BTN7971 clutch part
// The outputs IN1 and IN2 connected to arduino/DO5 and /DO10
// current sense clutch /A6
//**************************************************
int CLUTCH_EN = 7;      // ENable clutch connected to DO7
int CLUTCH_IN1 = 5;     // IN1 connected to DO5
int CLUTCH_IN2 = 10;    // IN2 connected to DO10
int CLUTCH_CS = A6;     // current sense for clutch connected to AnIn6
int CLUTCH_PWM = 0;     // init/start value of clutch pwm, range, 0-255


//**************************************************
// 1/2 part of BTN7971 Motor part
// The outputs IN1 and IN2 connected to arduino/DO3 and DO6
// current sense motor /A3
//**************************************************
int MOTOR_EN = 8;       // ENable Motor connected to DO8
int MOTOR_IN1 = 6;      // IN1 motor connected to DO6
int MOTOR_IN2 = 3;      // IN2 motor den connected to DO3
int MOTOR_CS = A3;      // current sense for motor Motor connected to AnIn3
int MOTOR_CURRENT = 0;  // analog output of motor current

//**************************************************
// yacht parameter - fixed numbers, readonly -> const
// Raymarine Rudder angle sensor 270° / 4k5 Ohm action range
// warning, beware of mechanical crash
// max. +/- 45° usable range (1/3 of 4k5)
// additional safety with offline switch at both stop positions
// +Ubat --- 1k5---variable 1k5---1k5--- 0V
// calibration needed for zero angle
// A/D refers to 10bit -> max. count 1024
// count range 0 to 341         absolut illegal
// count range 342 to 682       max. legal (+/- 45°)
// count range 683 to 1024      absolut illegal
//**************************************************
const int RUDDER_STOP_CW =      342;    // cw acting limit 30°
const int RUDDER_ANGLE_ZERO =   512;    // warning: sw- or hw-calibration needed
const int RUDDER_STOP_CCW =     682;    // ccw acting limit -30°
// ruder raw value
int RUDDER_SENSOR = A7; // rudder sensor connected to AnIn7 /10bit "int" hat wieviel bits? 


//**************************************************
//
// Input: 
// Variables are Call-by-Reference
// The Result is returned in the input Variables ( & Operator )
//
// Result:
// Sensor variables Smoothed
// Settings for Smoothness is global Variable: SENSOR_SMOOTHNESS
// 
// m_x, m_y, m_z // magnetic vector x:Laengsachse, y:Querachse, z:Hochachse
// a_x, a_y, a_z // acceleration vector x:roll, y:pitch, z:yaw
// g_x, g_y, g_z // gyro acceleration Drehbeschleunigung
//
//**************************************************
void GetSmoothedSensorVariables(int16_t& mx, int16_t& my, int16_t& mz, 
  int16_t& ax, int16_t& ay, int16_t& az,
  int16_t& gx, int16_t& gy, int16_t& gz)
{
  // init all var with 0
  int summ_a_x = 0, summ_a_y = 0, summ_a_z = 0; 
  int summ_m_x = 0, summ_m_y = 0, summ_m_z = 0; 
  int summ_g_x = 0, summ_g_y = 0, summ_g_z = 0;
  // var for raw values
  int raw_a_x, raw_a_y, raw_a_z, raw_g_x, raw_g_y, raw_g_z;
  for (int i = 0; i < SENSOR_SMOOTHNESS; i++)
  {
    // read raw sensor values
    MagnetometerRaw raw = COMPASS.ReadRawAxis(); 
    ACCELGYRO.getMotion6(&raw_a_x, &raw_a_y, &raw_a_z, &raw_g_x, &raw_g_y, &raw_g_z);
    // sum up to 10 raw compass values
    summ_m_x += raw.XAxis;
    summ_m_y += raw.YAxis;
    summ_m_z += raw.ZAxis;
    summ_a_x += raw_a_x;
    summ_a_y += raw_a_y;
    summ_a_z += raw_a_z;
    summ_g_x += raw_g_x;
    summ_g_y += raw_g_y;
    summ_g_z += raw_g_z;
  }
  // getting arithmetic average
  mx = summ_m_x / SENSOR_SMOOTHNESS; 
  my = summ_m_y / SENSOR_SMOOTHNESS;
  mz = summ_m_z / SENSOR_SMOOTHNESS;
  ax = summ_a_x / SENSOR_SMOOTHNESS; 
  ay = summ_a_y / SENSOR_SMOOTHNESS; 
  az = summ_a_z / SENSOR_SMOOTHNESS;
  gx = summ_g_x / SENSOR_SMOOTHNESS; 
  gy = summ_g_y / SENSOR_SMOOTHNESS; 
  gz = summ_g_z / SENSOR_SMOOTHNESS;
  //after this routine the values are smoothed
}


//**************************************************
//
// Input:
// A Course to calculate the CourseDelta to the TARGET_HEADING
// 
// Returns:
// Difference between the input and the TARGET_HEADING
//
//**************************************************
float GetCourseDelta(float currentCourse)
{
  float courseDelta = currentCourse - TARGET_HEADING;
  if (courseDelta < -180)
    courseDelta += 360;
  if (courseDelta > 180)
    courseDelta -= 360;
  return courseDelta;
}


//**************************************************
// read Rudder angle and test of legal reading
// Raymarine Rudder angle sensor max. +/- 45°
// +Ubat --- 1k5---variable 1k5---1k5--- 0V
// A/D 10bit: max. count 1024
// 0 to 341 fixed range absolut illegal
// 342 to 681 max. variabel range legal +/- 45°
// 682 to 1024 fixed range absolut illegal
//**************************************************
float ReadRudderAngle()
{
  float rudderAngle = analogRead(RUDDER_SENSOR);
  // if: higher then action limit ccw OR lower action limit cw
  // stop Motor
  if (rudderAngle > RUDDER_STOP_CCW || rudderAngle < RUDDER_STOP_CW)        
    digitalWrite(MOTOR_EN, LOW);  // stop Motor
  else
    digitalWrite(MOTOR_EN, HIGH); // enable motor
  return rudderAngle;
}


//**************************************************
//
// Input: 
// True: Calculate CurrentHeading with TiltCompensation
// False: Calculate CurrentHeading without TiltCompensation
//
// Return:
// CurrentHeading
//
// x:roll y:pitch z:yaw
//**************************************************
float GetCurrentHeading(bool tiltCompensation)
{
  // Get the current sensor variables
  int16_t mx = 0, my = 0, mz = 0, ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  GetSmoothedSensorVariables(mx, my, mz, ax, ay, az, gx, gy, gz);

  // Check if Calucate with or without tiltCompensation
  if(!tiltCompensation){
    float course = atan2(my, mx) / 0.0174532925;
    if (course < 0)
        course += 360; // ???
    else
        course = 360 - course;
    return course;
  } else {
    float roll = asin(ax);// berichtigt, war a_y
    float pitch = asin(ay);// berichtigt, war a_x
    if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
      return -1; // error
    // Vorabberechnung, da die Werte gleich mehrfach benötigt werden.
    float sin_roll = sin(roll);
    float cos_pitch = cos(pitch);
    float sin_pitch = sin(pitch);
    // Neigungsausgleich
    float x_h = mx * sin_roll * sin_pitch + my * cos(roll) - mz * sin_roll * cos_pitch; //berichtigt, war y_h  
    float y_h = mx * cos_pitch + mz * sin_pitch; // berichtigt, war x_h
    float course_comp = atan2(y_h, x_h);
    return course_comp * 180 / M_PI; // Umwandlung in Winkelmaß
  }
}


//**************************************************
void activate_clutch() // activate clutch only via setup (Nur im Setup zu aktivieren)
{
  digitalWrite(CLUTCH_EN, HIGH);
  digitalWrite(CLUTCH_IN1, LOW);
  analogWrite(CLUTCH_IN2, 255); //full PWM
}


//**************************************************
//
// Input: 
// a course delta to Set 
//
// Return:
// calc of motor speed for giving course_delta
// set pulse width modulation (pwm) ramp by value *10
// pwm value min=0 to max=255 (uint8_t)
//
//**************************************************
uint8_t GetMotorPwm(float course_delta)
{
  float calculation = abs(course_delta) * 10;
  uint8_t result = 255;
  if (calculation < 255)
    result = (uint8_t)calculation;// "type cast": changing from float to uint8_t
  return result;
}


//**************************************************
//
// Input: 
// course delta which should approach 0 in future
//
//**************************************************
void SetRudder(float courseDelta){
  activate_clutch();
  digitalWrite(MOTOR_EN, HIGH); // doppelt vorhanden, eventuell ständig einschalten
  
  // calculate the motorPwn for the current course delta
  float motorPwm = GetMotorPwm(courseDelta);
    
  if(courseDelta < 0){
    digitalWrite(MOTOR_IN1, LOW);
    analogWrite(MOTOR_IN2, motorPwm);
    Serial.println("SetRudder: rudder_clockwise - MotorPwn: " + String(motorPwm));
  }else if(courseDelta > 0){
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_IN1, motorPwm);
    Serial.println("SetRudder: rudder_counterclockwise - MotorPwn: " + String(motorPwm));
  }else{
    // courseDelta is zero -> no rudder movement needed
    Serial.println("SetRudder: no_rudder_movement");
  }
}


//*************************************************
//
// p r o g r a m   s t a r t
//
//*************************************************
void setup()
{
  Wire.begin();// connect I2C bus ?? or arduino pin-name??
  //initialize sensors
  COMPASS = HMC5883L(); 
  ACCELGYRO.initialize();
  ACCELGYRO.setI2CMasterModeEnabled(false);
  ACCELGYRO.setI2CBypassEnabled(true);
  ACCELGYRO.setSleepEnabled(false);
  // initialize serial monitor
  // operate - carefull! check of right Baudrate
  // maybe 9600 for compass and accelgyro 57600 oder 38400
  // then the baudrate has to change
  Serial.begin(9600); // init serial monitor
  int error = COMPASS.SetMeasurementMode(Measurement_Continuous); // set compass mode continous
  if(error != 0)
     Serial.println(COMPASS.GetErrorText(error)); // if error appears -> print error

  float currentHeading = -1;
  // GetCurrentHeading until we get no Errors
  // currentHeading < 0 is an Error return value
  while(currentHeading < 0){
    // get the current Heading variable
    // true: tilt compensation is on
    // false: tilt compensation is off
    float currentHeading = GetCurrentHeading(true); 
  }
  // Set the global course_fix variable to the initial heading at program start
  TARGET_HEADING = currentHeading; // inits/fills target_heading at start
}


//**************************************************
//
// p r o g r a m   l o o p
// - zeitspanne für den Durchlauf ermitteln
// - Hysterese einbauen
//
//**************************************************
void loop()
{
  // calculate current heading
  // true: tilt compensation is on
  // false: tilt compensation is off
  float currentHeading = GetCurrentHeading(true);

  // check for Errors in current Heading calculation
  if(currentHeading < 0) {
    Serial.print("Error calculating currentHeading");
  } else {
    // No Errors occured
    
    // calculate current Course Delta
    float currentCourseDelta = GetCourseDelta(currentHeading);
    
    // Set the rudder for the current courseDelta
    SetRudder(currentCourseDelta);
    
    // Read the current rudder angle
    float currentRudderAngle = ReadRudderAngle();
    
    Serial.print(" current heading ");
    Serial.print(currentHeading);
    Serial.print(" target_heading ");
    Serial.print(TARGET_HEADING);
    Serial.print(" course_delta ");
    Serial.print(currentCourseDelta);
    Serial.print(" rudder_angle ");   // prints "ascii text"
    Serial.println(currentRudderAngle);     // prints value
    //delay (10);
  }
}
