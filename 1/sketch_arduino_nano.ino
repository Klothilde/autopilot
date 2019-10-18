// Autopilot Maxsimplex
// Mit Ruderlagepotentiometer
// Mit Aktivierung der Rudermaschine
// Mit Aktivierung der Kupplung
// Mit Sollkursfeststellung nach dem Reset.
// Kompass ohne Tiltkompensation angelehnt an eine Formel aus
// https://circuitdigest.com/microcontr...gital-compass-with-arduino-and-hmc5883l-magnetometer
// mit beruhigten Werten
//**************** Funktioniert! *****************************
#include<Wire.h> // Library fuer den I2C-Bus

// Libraries für den GY-88
#include "HMC5883L.h"
#include "MPU6050.h"
#include "I2Cdev.h"


// SCHIFFSPARAMETER - Feste Werte (readonly-Variablen -> const)
//**************************************************
int ruderlage = 0; // Aktuelle Ruderlage Wertebereich: 10-1000 (erreicht nicht den Anschlag)
const int ruderlageMitte = 500; // Noch zu ermittelnder Wert fuer Ruder mittschiffs
const int anschlagLinks = 1000; // Noch zu ermittelnder Wert fuer den Anschlag Links
const int anschlagRechts = 10; // Noch zu ermittelnder Wert fuer den Anschlag Rechts


// Globale Variablen
//**************************************************
float Sollkurs = 0;
float Kurs = 0;
int Kursdifferenz = 0;


// Variablen die von den Sensoren befüllt werden
//**************************************************
int16_t ax = 0, ay = 0, az = 0; // Beschleunigungsvektor: Yaw, Pitch, Roll
int16_t mx = 0, my = 0, mz = 0; // Magnetvektor x: Laengsachse, y: Querachse, z: Hochachse
int16_t gx = 0, gy = 0, gz = 0; // Drehbeschleunigung x: Laengsachse, y: Querachse, z: Hochachse


// Sensoren
//**************************************************
HMC5883L compass;
MPU6050 accelgyro;


// AKTUATOREN
//**************************************************
int ruderlagepin = A7; // Ruderlagepotentiometer ist mit AnIn 7 verbunden
const int MAG = 0x1E; // I2C Adresse des Magnetometers 7bit Adresse
const int MPU = 0x68; // I2C address of the MPU-6050


//**************************************************
//Die Eingaenge IN1 und IN2 des BTN7971 liegen an DO5 bzw DO10
int ENkupplung = 7; // ENable ist mit DO 7 verbunden
int IN1kupplung = 5; // IN1 ist mit DO 5 verbunden
int IN2kupplung = 10; // IN2 ist mit DO 10 verbunden
int CTkupplung = A6; // Strommessung ist mit AnIn 6 verbunden
int Kpwm = 0; // Anfangswert fuer das PWM-Signal, 0-255


//**************************************************
//Die Eingaenge IN1 und IN2 des BTN7971 liegen an DO3 bzw DO6
int ENmotor = 8; // ENable fuer den Motor ist mit DO 8 verbunden
int IN1motor = 6; // IN1 fuer den Motor ist mit DO 6 verbunden
int IN2motor = 3; // IN2 fuer den Motor ist mit DO 3 verbunden
int CTmotor = A3; // Strommessung fuer den Motor ist mit AnIn 3 verbunden
int Mstrom = 0; // Ergebnis der Motorstrommessung
int Mpwm = 0; // Anfangswert fuer das Motor-PWM-Signal, 0-255


//*************************************************
// Programmstart
//*************************************************
void setup() {
  // I2C Bus verbinden
  Wire.begin();
  
  // Sensoren initialisieren
  compass = HMC5883L(); 
  accelgyro.initialize();
  accelgyro.setI2CMasterModeEnabled(false);
  accelgyro.setI2CBypassEnabled(true) ;
  accelgyro.setSleepEnabled(false);
  
  // Serial Monitor initialisieren - Achtung! Baudrate muss übereinstimmen
  // Könnte sein, das der Kompass 9600 benötigt, und der accelgyro 57600 oder 38400
  // Dann müsste man zwischen den Baudraten wechseln
  Serial.begin(9600); 
  
  // Modus des Kompass auf Continous stellen
  int error = compass.SetMeasurementMode(Measurement_Continuous); 
  if(error != 0) // Falls ein Fehler aufgetreten ist -> Fehler anzeigen
    Serial.println(compass.GetErrorText(error));
  
  // Programminitialisierung
  LadeUndGlaetteSensorDaten(); // Lade und Glätte Magnetwerte
  berechneKursMitNeigungsausgleich();
  Sollkurs = Kurs; // Der aktuelle Kurs ist der Sollkurs
  KupplungEin(); // Kupplung in Eingriff bringen
}


//**************************************************
// Programmschleife
//**************************************************
void loop() {
  // Sensorwerte aktualisieren und neuen Kurs sowie Kursdifferenz berechnen
  LadeUndGlaetteSensorDaten();
  berechneKursMitNeigungsausgleich();
  aktualisiereKursdifferenz();
  berechneMpwm();
  
  if (Kursdifferenz < 0)//Entscheidung ob Rudermaschine vorwaerts oder rueckwaerts
    vorwaerts(); //Rudermaschine dreht vorwaerts
  else
    rueckwaerts(); //Rudermaschine dreht rueckwaerts

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
}



//**************************************************
// Ab Hier: Berechnungsfunktionen
//**************************************************


//**************************************************
void vorwaerts() //Rudermaschine dreht vorwaerts
{
  digitalWrite(ENmotor, HIGH); // fuer echten Betrieb auf HIGH setzen
  digitalWrite(IN1motor, LOW);
  analogWrite(IN2motor, Mpwm);
  Mstrom = analogRead(A3);// read the input on analog pin 3:
  Serial.print("Vor");
}

//**************************************************
void rueckwaerts() //Rudermaschine dreht rueckwaerts
{
  digitalWrite(ENmotor, HIGH); // fuer echten Betrieb auf HIGH setzen
  digitalWrite(IN2motor, LOW);
  analogWrite(IN1motor, Mpwm);
  Mstrom = analogRead(A3);// read the input on analog pin 3:
  Serial.print("Rueck");
}


//**************************************************
void berechneMpwm() // Berechnen des erforderlichen Motordrehmoments
{
  Mpwm = abs(Kursdifferenz) * 10;
  if (Mpwm > 255) //Verhinderung von sinnlosen Werten ueber 255
  {
    Mpwm = 255;
  }
}

//**************************************************
void KupplungEin() // Kupplung in Eingriff bringen (Nur im Setup zu aktivieren)
{
  digitalWrite(ENkupplung, HIGH);
  digitalWrite(IN1kupplung, LOW);
  analogWrite(IN2kupplung, 255); //Voller PWM-Wert
}

//**************************************************
void LadeUndGlaetteSensorDaten()
{
  int loops = 10;
  // Summenvariablen zur Durchschnittsberechnung anlegen
  int sax = 0, say = 0, saz = 0; 
  int smx = 0, smy = 0, smz = 0; 
  int sgx = 0, sgy = 0, sgz = 0;
  for (int i = 0; i < loops; i++) {
    // Temporäre Variablen um den aktuellen Wert auszulesen
    int tax, tay, taz, tgx, tgy, tgz; 
    // aktuelle Sensorwerte auslesen
    MagnetometerRaw raw = compass.ReadRawAxis(); 
    accelgyro.getMotion6(&tax, &tay, &taz, &tgx, &tgy, &tgz);
    // Summen für den Durchschnitt berechnen
    smx += raw.XAxis;
    smy += raw.YAxis;
    smz += raw.ZAxis;
    sax += tax;
    say += tay;
    saz += taz;
    sgx += tgx;
    sgy += tgy;
    sgz += tgz;
  }
  // Durchschnitt berechnen & Sensorvariablen befüllen
  mx = smx / loops;
  my = smy / loops;
  mz = smz / loops;
  ax = sax / loops;
  ay = say / loops;
  az = saz / loops;
  gx = sgx / loops;
  gy = sgy / loops;
  gz = sgz / loops;
  //Nach dieser Routine sind die Sensorvariablen beruhigt
}


//**************************************************
void berechneKurs()
{
  Kurs = atan2(my, mx) / 0.0174532925;
  if (Kurs < 0)
    Kurs += 360;
  Kurs = 360 - Kurs;
}

//**************************************************
void berechneKursMitNeigungsausgleich() {
  float roll = asin(ay);
  float pitch = asin(ax);
  
  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78)
    return;

  // Vorabberechnung, da die Werte gleich mehrfach benötigt werden.
  float sinRoll = sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Neigungsausgleich
  float Xh = mx * cosPitch + mz * sinPitch;
  float Yh = mx * sinRoll * sinPitch + my * cos(roll) - mz * sinRoll * cosPitch;
  float aktuellerKurs = atan2(Yh, Xh);
  
  aktuellerKurs = aktuellerKurs * 180 / M_PI; // Umwandlung in Winkelmaß
  
  Kurs = aktuellerKurs; // Neuen Kurs speichern
}

//**************************************************
void aktualisiereKursdifferenz()
{
  Kursdifferenz = Kurs - Sollkurs;
  if (Kursdifferenz < -180) {
    Kursdifferenz = Kursdifferenz + 360;
  }
  if (Kursdifferenz > 180) {
    Kursdifferenz = Kursdifferenz - 360;
  }
}

// Ruderlage lesen und auf Anschlag testen
//**************************************************
void leseruderlage()
{
  ruderlage = analogRead(ruderlagepin);
  if (ruderlage > 1000) {
    digitalWrite(ENmotor, LOW);  // Motor aus
  }
  if (ruderlage < 10) {
    digitalWrite(ENmotor, LOW);  // Motor aus
  }
}
