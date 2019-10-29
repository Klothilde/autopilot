/*****************************************************
 *         CMPS 14 Library by Sven Beckmann          *
 *   for using the sensor in Serial Mode (NOT I2C)   *
 ****************************************************/
 
#include "CMPS14Serial.h"

/*
 *
 * For detailed Information about the Hardware 
 * have a look at the hardware documentation at:
 * https://robot-electronics.co.uk/files/cmps14.pdf
 * 
 */


/*
 * Constructor
 */
CMPS14::CMPS14(byte rx, byte tx) : _cmps(rx, tx) {
  _cmps.begin(9600);
}

/***********************************************
 *             PUBLIC METHODS                  *
 ***********************************************/

/*
 * Returns the Software Version
 */
uint8_t CMPS14::GetVersion() { 
  return Read8BitUnsigned(GET_VERSION);
}

/*
 * Returns the Bearing als single Byte 0-255
 */
uint8_t CMPS14::GetBearing8Bit() { 
  return Read8BitUnsigned(GET_BEARING_8_BIT);
}

/*
 * Returns the Bearing as 16 bit 0-3599 
 * 3599 = bearing of 359.9
 */
uint16_t CMPS14::GetBearing16Bit() { 
  return Read16BitUnsigned(GET_BEARING_16_BIT);
}

/*
 * Returns Pitch angle +/- 90 degree
 */
int8_t CMPS14::GetPitch() {
  return Read8Bit(GET_PITCH);
}

/*
 * Returns Roll angle +/- 90 degree
 */
int8_t CMPS14::GetRoll() { 
  return Read8Bit(GET_ROLL);
}

/*
 * Returns the Raw Data of the magnetic sensor
 */
void CMPS14::GetMagRaw(int16_t& X, int16_t& Y, int16_t& Z) { 
  ReadRawData(X, Y, Z, GET_MAG_RAW);  
}

/*
 * Returns the Raw Data of the accelerometer sensor
 */
void CMPS14::GetAccelRaw(int16_t& X, int16_t& Y, int16_t& Z) { 
  ReadRawData(X, Y, Z, GET_ACCEL_RAW);
}

/*
 * Returns the Raw Data of the gyro sensor
 */
void CMPS14::GetGyroRaw(int16_t& X, int16_t& Y, int16_t& Z) { 
  ReadRawData(X, Y, Z, GET_GYRO_RAW);
}

/*
 * Writes the current bearing16bit, pitch, roll in the giving parameters
 */
void CMPS14::GetAll(uint16_t& Bearing16Bit, int8_t& Pitch, int8_t& Roll)  { 
  unsigned char high_byte, low_byte;
  _cmps.write(GET_ALL);
  while(_cmps.available() < 4);
  high_byte = _cmps.read();
  low_byte = _cmps.read();
  Bearing16Bit = high_byte;
  Bearing16Bit <<= 8;
  Bearing16Bit += low_byte;
  while(_cmps.available() < 2);
  Pitch = _cmps.read();
  while(_cmps.available() < 1);
  Roll = _cmps.read();
}

/*
 * Bits 0 and 1 reflect the calibration status
 * 0 = uncalibrated
 * 3 = fully calibrated
 */
void CMPS14::GetCalibrationState(uint8_t& systemCalibration, uint8_t& gyroCalibration, uint8_t& accelCalibration, uint8_t& magnetCalibration)  { 
  /***********************************************************************************************************
   *                                                                                                         *
   *   Calibration Status Return of the CMPS14:                                                               *
   *   +-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+     *
   *   |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |     *
   *   +-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+     *
   *   |        System         |          Gyro         |        Accel          |        Magnet         |     *
   *   |      Calibration      |      Calibration      |      Calibration      |      Calibration      |     *
   *   +-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+     *
   *                                                                                                         *
   * ********************************************************************************************************/
  _cmps.write(GET_CALIBRATION_STATE);
  while(_cmps.available() < 1);
  uint8_t state = _cmps.read();
  magnetCalibration = (state & 3);
  accelCalibration = (state & 12) >> 2;
  gyroCalibration = (state & 48) >> 4;
  systemCalibration = (state & 192) >> 6;
}

/*
 * Returns the 16 bit Roll angle +/- 180 degree
 */
int16_t CMPS14::GetRoll180()  { 
  return Read16Bit(GET_ROLL_180);
}

/*
 * ATTENTION - NOT TESTET YET
 * Enters the calibration mode by sending the needed bytes
 */
bool CMPS14::EnterCalibrationMode() { 
  if(!SendCommand(CHANGE_CALIBRATION_CONFIG_BYTE_1, 0x55))
    return false;
  if(!SendCommand(CHANGE_CALIBRATION_CONFIG_BYTE_2, 0x55))
    return false;
  if(!SendCommand(CHANGE_CALIBRATION_CONFIG_BYTE_3, 0x55))
    return false;
  return true;
}

/*
 * ATTENTION - NOT TESTET YET
 * sets the current calibration config to the settings of the parameters
 */
bool CMPS14::SetCalibrationConfig(bool periodicAutoSaveEnabled, bool GyroCalibrationEnabled, 
                                        bool AccelCalibrationEnabled, bool MagCalibrationEnabled) { 
  /***********************************************************************************************************
   *                                                                                                         *
   *   CalibrationConfiguration                                                                              *
   *   +-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+     *
   *   |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |     *
   *   +-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+     *
   *   |  Always 1 |           |           |  Periodic |           |  GyroCal  | AccelCal  |  MagCal   |     *
   *   |           | Undefined | Undefined |    Save   | Undefined |  Enabled  |  Enabled  |  Enabled  |     *
   *   +-----------+-----------+-----------+-----------+-----------+-----------+-----------+-----------+     *
   *                                                                                                         *
   * ********************************************************************************************************/
  bool calibrationConfigBits[8] = {1, 0, 0, periodicAutoSaveEnabled, 0, GyroCalibrationEnabled, AccelCalibrationEnabled, MagCalibrationEnabled};
  byte calibrationConfig = 0;
  for(int i = 0; i < 8; i++)
  {
    // Bits um 1 nach links schieben
    calibrationConfig = calibrationConfig << 1;
    if(calibrationConfigBits[i])
      calibrationConfig++; // Wenn an der Stelle eine 1 kommt zähle sie hinzu
  }
  _cmps.write(calibrationConfig);
  return true;
}

/*
 * ATTENTION - NOT TESTET YET
 * Saves the current calibration to the CMPS14
 */
bool CMPS14::StoreCalibration() { 
  if(!SendCommand(STORE_CALIBRATION_BYTE_1, 0x55))
    return false;
  if(!SendCommand(STORE_CALIBRATION_BYTE_2, 0x55))
    return false;
  if(!SendCommand(STORE_CALIBRATION_BYTE_3, 0x55))
    return false;
  return true;
}

/*
 * ATTENTION - NOT TESTET YET
 * Erases the stored Calibration
 */
bool CMPS14::EraseStoredCalibration() { 
  if(!SendCommand(DELETE_CALIBRATION_BYTE_1, 0x55))
    return false;
  if(!SendCommand(DELETE_CALIBRATION_BYTE_2, 0x55))
    return false;
  if(!SendCommand(DELETE_CALIBRATION_BYTE_3, 0x55))
    return false;
  return true;
}

/*
 * Sets the Baud-Rate to 19200
 * ATTENTION: Its automatic set back to 9600 after restarting
 */
bool CMPS14::SetBaud19200() {
  _cmps.write(BAUD_19200);
  _cmps.end();
  _cmps.begin(19200);
  while(_cmps.available() < 1);
  byte responseByte = _cmps.read();
  if(responseByte != 0x55){
    _cmps.end();
    return false;
  }
  return true;
}

/*
 * Sets the Baud-Rate to 38400
 * ATTENTION: Its automatic set back to 9600 after restarting
 */
bool CMPS14::SetBaud38400() { 
  _cmps.write(BAUD_38400);
  _cmps.end();
  _cmps.begin(38400);
  while(_cmps.available() < 1);
  byte responseByte = _cmps.read();
  if(responseByte != 0x55){
    _cmps.end();
    return false;
  }
  return true;
}


/***********************************************
 *            PRIVATE METHODS                  *
 ***********************************************/

bool CMPS14::SendCommand(byte command, byte successResponseByte){
  _cmps.write(command);
  while(_cmps.available() < 1);
  byte responseByte = _cmps.read();
  if(responseByte != successResponseByte)
    return false;
  return true;
}

void CMPS14::ReadRawData(int16_t& X, int16_t& Y, int16_t& Z, byte address){
  unsigned char high_byte, low_byte;
  _cmps.write(address);
  while(_cmps.available() < 6);
  high_byte = _cmps.read();
  low_byte = _cmps.read();
  X = high_byte;
  X <<= 8;
  X += low_byte;
  while(_cmps.available() < 4);
  high_byte = _cmps.read();
  low_byte = _cmps.read();
  Y = high_byte;
  Y <<= 8;
  Y += low_byte;
  while(_cmps.available() < 2);
  high_byte = _cmps.read();
  low_byte = _cmps.read();
  Z = high_byte;
  Z <<= 8;
  Z += low_byte;
}

uint8_t CMPS14::Read8BitUnsigned(byte address){
  _cmps.write(address);
  while(_cmps.available() < 1);
  uint8_t val = _cmps.read();
  return val;
}

int8_t CMPS14::Read8Bit(byte address){
  _cmps.write(address);
  while(_cmps.available() < 1);
  int8_t val = _cmps.read();
  return val;
}

uint16_t CMPS14::Read16BitUnsigned(byte address){
  unsigned char high_byte, low_byte;
  _cmps.write(address);
  while(_cmps.available() < 2);
  high_byte = _cmps.read();
  low_byte = _cmps.read();
  uint16_t val = high_byte;
  val <<= 8;
  val += low_byte;
  return val;
}

int16_t CMPS14::Read16Bit(byte address){
  unsigned char high_byte, low_byte;
  _cmps.write(address);
  while(_cmps.available() < 2);
  high_byte = _cmps.read();
  low_byte = _cmps.read();
  int16_t val = high_byte;
  val <<= 8;
  val += low_byte;
  return val;
}
