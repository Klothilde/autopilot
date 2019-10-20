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
// TODO list
// - return() fehlt überall??
// - return value, alles void??
//
//
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
//
// global var
//
//**************************************************
float course_fix = 0;
float course = 0;
int course_delta = 0;


//**************************************************
//
// s e n o r s
//
//**************************************************
HMC5883L compass; // compass
MPU6050 accelgyro; // compass

const int MAG = 0x1E;   // I2C address of magnetic compass
const int MPU = 0x68;   // I2C address of the MPU-6050

int16_t a_x = 0, a_y = 0, a_z = 0; // acceleration vector x:roll, y:pitch, z:yaw
int16_t m_x = 0, m_y = 0, m_z = 0; // magnetic vector x:Laengsachse, y:Querachse, z:Hochachse
int16_t g_x = 0, g_y = 0, g_z = 0; // gyro acceleration Drehbeschleunigung


//**************************************************
//
// compass output calming Filter #1
// arithmeic average output of 10
//
//**************************************************
void compass_output_arith()
{
  int n = 10;
  // init all var with 0
  int summ_a_x = 0, summ_a_y = 0, summ_a_z = 0; 
  int summ_m_x = 0, summ_m_y = 0, summ_m_z = 0; 
  int summ_g_x = 0, summ_g_y = 0, summ_g_z = 0;
  // var for raw values
  int raw_a_x, raw_a_y, raw_a_z, raw_g_x, raw_g_y, raw_g_z;
  for (int i = 0; i < n; i++)
  {
    // read raw sensor values
    MagnetometerRaw raw = compass.ReadRawAxis(); 
    accelgyro.getMotion6(&raw_a_x, &raw_a_y, &raw_a_z, &raw_g_x, &raw_g_y, &raw_g_z);
    // sum up to 10 raw compass values
    summ_m_x + = raw.XAxis;
    summ_m_y + = raw.YAxis;
    summ_m_z + = raw.ZAxis;
    summ_a_x + = raw_a_x;
    summ_a_y + = raw_a_y;
    summ_a_z + = raw_a_z;
    summ_g_x + = raw_g_x;
    summ_g_y + = raw_g_y;
    summ_g_z + = raw_g_z;
  }
  // getting arithmetic average
  m_x = summ_m_x / n;
  m_y = summ_m_y / n;
  m_z = summ_m_z / n;
  a_x = summ_a_x / n;
  a_y = summ_a_y / n;
  a_z = summ_a_z / n;
  g_x = summ_g_x / n;
  g_y = summ_m_y / n;
  g_z = summ_g_z / n;
  //after this routine the values are smoothed
}


//**************************************************
//
// vom Sven empfohlen
// Funktion ist noch nicht im Ablauf eingebunden
// calulate course with tilt compensation
// - translation needed
// x:roll
// y:pitch
// z:yawl
//
//**************************************************
void course_tilt_compensation()
{
  float roll = asin(a_x);// berichtigt, war a_y
  float pitch = asin(a_y);// berichtigt, war a_x
  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
    return;
  // Vorabberechnung, da die Werte gleich mehrfach benötigt werden.
  float sin_roll = sin(roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);
  // Neigungsausgleich
  float x_h = m_x * sin_roll * sin_pitch + m_y * cos(roll) - m_z * sin_roll * cos_pitch;//berichtigt, war y_h  
  float y_h = m_x * cos_pitch + m_z * sin_pitch;// berichtigt, war x_h
  float course_comp = atan2(x_h, y_h); // berichtigt, y_h <-> x_h ausgetauscht
  course = course_comp * 180 / M_PI; // Umwandlung in Winkelmaß
 // course = course_comp; // Neuen course speichern, Zeile ist wohl nicht nötig
}


//**************************************************
//
// delta between course_fix and course
//
//**************************************************
void course_delta()
{
  course_delta = course - course_fix;
  if (course_delta < -180) {
    course_delta = course_delta + 360;
  }
  if (course_delta > 180) {
    course_delta = course_delta - 360;
  }
}


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
int rudder_angle =              512;    // inits rudder angle to 0°
// ruder raw value
int rudder_sensor = A7; // rudder sensor connected to AnIn7 /10bit "int" hat wieviel bits? 


//**************************************************
// read Rudder angle and test of legal reading
// Raymarine Rudder angle sensor max. +/- 45°
// +Ubat --- 1k5---variable 1k5---1k5--- 0V
// A/D 10bit: max. count 1024
// 0 to 341 fixed range absolut illegal
// 342 to 681 max. variabel range legal +/- 45°
// 682 to 1024 fixed range absolut illegal
//**************************************************
void read_rudder_angle()
{
  rudder_angle = analogRead(rudder_sensor);
  if (rudder_angle > RUDDER_STOP_CCW)        // compare to action limit ccw
  {
    digitalWrite(motor_en, LOW);             // stop Motor
  }
  if (rudder_angle < RUDDER_STOP_CW)         // compare to action limit cw
  {
    digitalWrite(motor_en, LOW);             // stop motor
  }
  else
  {
    digitalWrite(motor_en, HIGH)            // enable motor
  }
}


//**************************************************
//
// calculate course
//
//**************************************************
void course()
{
  course = atan2(m_y, m_x) / 0.0174532925;
  if (course < 0)
    course + = 360; // ???
  course = 360 - course;
}


//**************************************************
//**************************************************
//
// A k t u a t o r s
//
//**************************************************
//
// 1/2 part of BTN7971 clutch part
// The outputs IN1 and IN2 connected to arduino/DO5 and /DO10
// current sense clutch /A6
//
//**************************************************
int clutch_en = 7;      // ENable clutch connected to DO7
int clutch_in1 = 5;     // IN1 connected to DO5
int clutch_in2 = 10;    // IN2 connected to DO10
int clutch_cs = A6;     // current sense for clutch connected to AnIn6
int clutch_pwm = 0;     // init/start value of clutch pwm, range, 0-255


//**************************************************
void activate_clutch() // activate clutch only via setup (Nur im Setup zu aktivieren)
{
  digitalWrite(clutch_en, HIGH);
  digitalWrite(clutch_in1, LOW);
  analogWrite(clutch_in2, 255); //full PWM
}


//**************************************************
//
// 1/2 part of BTN7971 Motor part
// The outputs IN1 and IN2 connected to arduino/DO3 and DO6
// current sense motor /A3
//
//**************************************************
int motor_en = 8;       // ENable Motor connected to DO8
int motor_in1 = 6;      // IN1 motor connected to DO6
int motor_in2 = 3;      // IN2 motor den connected to DO3
int motor_cs = A3;      // current sense for motor Motor connected to AnIn3
int motor_current = 0;  // analog output of motor current
int motor_pwm = 0;      // init/start value of motor pwm, range 0-255


//**************************************************
//
// calc of motor speed
// set pulse width modulation (pwm) ramp
// pwm value min=0 to max=255
//
//**************************************************
void motor_pwm()
{
  motor_pwm = abs(course_delta) * 10;// set pwm ramp
  if (motor_pwm > 255) //Verhinderung von sinnlosen Werten ueber 255
  {
    motor_pwm = 255;
  }
}


//**************************************************
void rudder_clockwise() // motor moves Rudder clockwise
{
  activate_clutch();
  digitalWrite(motor_en, HIGH); // doppelt vorhanden?
  digitalWrite(motor_in1, LOW);
  analogWrite(motor_in2, motor_pwm);
  motor_current = analogRead(A3);// reads input of pin A3 (analog pin 3)
  Serial.print("rudder_clockwise");
}


//**************************************************
void rudder_counterclockwise() // motor moves Rudder counterclockwise
{
  activate_clutch();
  digitalWrite(motor_en, HIGH); // doppeltvorhanden?
  digitalWrite(motor_in2, LOW);
  analogWrite(motor_in1, motor_pwm);
  motor_current = analogRead(A3);// reads input of pin A3 (analog pin 3)
  Serial.print("rudder_counterclockwise");
}


//*************************************************
//
// p r o g r a m   s t a r t
//
//*************************************************


/*//*************************************************
// 
// vom Sven nicht empfohlen
//
//*************************************************
void setup()
{
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B); // PWR_MGMT_1 register
Wire.write(0); // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true); Wire.begin(); //Kompass
Wire.beginTransmission(MAG); // Modus continous
Wire.write(byte(0x02)); //Sendung für X MSB register Magnetometer
Wire.write(byte(0x00));
Wire.endTransmission(); // Ende der Sendung
Serial.begin(9600); // Serial Monitor initialisieren
for (iSollkurs=0; iSollkurs<11; iSollkurs ++ ) //Zählschleife zum Lesen des Sollkurses (10*)
{ read_mag();
glaetteMagnetrohwert();
berechneKurs();
Sollkurs = Kurs; //Der im Setup festgestellte Kurs ist der Sollkurs.
}
KupplungEin(); // Kupplung in Eingriff bringen
}*/


//*************************************************
// 
// vom Sven empfohlen
//
//*************************************************
void setup()
{
  Wire.begin();// connect I2C bus ?? or arduino pin-name??
  //initialize sensors
  compass = HMC5883L(); 
  accelgyro.initialize();
  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true);
  accelgyro.setSleepEnabled(false);
  // initialize serial monitor
  // operate - carefull! check of right Baudrate
  // maybe 9600 for compass and accelgyro 57600 oder 38400
  // then the baudrate has to change
  Serial.begin(9600); // init serial monitor
  int error = compass.SetMeasurementMode(Measurement_Continuous); // set compass mode continous
  if(error != 0) 
     Serial.println(compass.GetErrorText(error)); // if error appears -> print error
  compass_output_arith();
  //course_tilt_compensation();// falscher befehl, richtig: course()
  course();
  course_fix = course; // inits/fills course_fix at start
  // activate_clutch(); // ist hier nicht nötig, in die motor-funktion kopiert
}


//**************************************************
//
// p r o g r a m   l o o p
// - zeitbedarf für loop-durchgang ermitteln
//
//**************************************************


/*//*************************************************
// 
// vom Sven nicht empfohlen
//
//*************************************************
void loop()
{
Wire.beginTransmission(MPU);
Wire.write(0x3B); //Accel_XOUT-H
Wire.endTransmission(false);
Wire.requestFrom(MPU,14,true); // requests 6 Register (H+L pro Vektor)
ax=Wire.read()<<8|Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
ay=Wire.read()<<8|Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
az=Wire.read()<<8|Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
gx=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
gy=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
gz=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
read_mag();
glaetteMagnetrohwert();
berechneKurs();
berechneKursdifferenz();
berechneMpwm();
if (Kursdifferenz < 0)//Entscheidung ob Rudermaschine vorwärts oder rückwärts
{
vorwaerts(); //Rudermaschine dreht vorwärts
}
else
{
rueckwaerts(); //Rudermaschine dreht rückwärts
}

Serial.print(" Kurs ");
Serial.print(Kurs);
Serial.print(" Sollkurs ");
Serial.print(Sollkurs);
Serial.print(" Kursdiff. ");
Serial.print(Kursdifferenz);
Serial.print(" Mpwm ");
Serial.print(Mpwm);
Serial.print(" Ruderlage ");
Serial.println(ruderlage);
//delay (10);
}*/


//*************************************************
// 
// vom Sven empfohlen
// zeitspanne für den Durchlauf ermitteln
// Hysterese einbauen
//
//*************************************************
void loop()
{
  compass_output_arith(); //get compass values
  // course_tilt_compensation();// input-Werte aus der Funktion compass_output_arith()?
  course_delta();
  motor_pwm();
  if (course_delta < 0) // decision if rudder turns cw or ccw
    rudder_clockwise(); // Rudder turns cw
  else
    rudder_counterclockwise(); // Rudder turns cw
  Serial.print(" course ");
  Serial.print(course);
  Serial.print(" course_fix ");
  Serial.print(course_fix);
  Serial.print(" course_delta ");
  Serial.print(course_delta);
  Serial.print(" motor_pwm ");
  Serial.print(motor_pwm);
  Serial.print(" rudder_angle ");   // prints "ascii text"
  Serial.println(rudder_angle);     // prints value
  //delay (10);
}









