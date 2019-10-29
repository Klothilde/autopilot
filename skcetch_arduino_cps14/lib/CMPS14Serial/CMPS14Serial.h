/*****************************************************
 *         CMPS 14 Library by Sven Beckmann          *
 *   for using the sensor in Serial Mode (NOT I2C)   *
 ****************************************************/
 
#ifndef CMPS14Serial
#define CMPS14Serial

#ifndef Arduino
#include <Arduino.h>
#endif

#include <SoftwareSerial.h>

#define GET_VERSION 0x11
#define GET_BEARING_8_BIT  0x12
#define GET_BEARING_16_BIT 0x13
#define GET_PITCH  0x14
#define GET_ROLL 0x15
#define GET_MAG_RAW 0x19
#define GET_ACCEL_RAW 0x20
#define GET_GYRO_RAW 0x21
#define GET_ALL 0x23
#define GET_CALIBRATION_STATE 0x24
#define GET_ROLL_180 0x26
#define CHANGE_CALIBRATION_CONFIG_BYTE_1 0x98
#define CHANGE_CALIBRATION_CONFIG_BYTE_2 0x95
#define CHANGE_CALIBRATION_CONFIG_BYTE_3 0x99
#define STORE_CALIBRATION_BYTE_1 0xF0
#define STORE_CALIBRATION_BYTE_2 0xF5
#define STORE_CALIBRATION_BYTE_3 0xF6
#define DELETE_CALIBRATION_BYTE_1 0xE0
#define DELETE_CALIBRATION_BYTE_2 0xE5
#define DELETE_CALIBRATION_BYTE_3 0xE2
#define BAUD_19200 0xA0
#define BAUD_38400 0xA1


class CMPS14 {

  public:
  
    // Constructor
    CMPS14(byte rx, byte tx);
    
    // Data Methods
    uint8_t GetVersion();
    uint8_t GetBearing8Bit();
    uint16_t GetBearing16Bit();
    int8_t GetPitch();
    int8_t GetRoll();
    int16_t GetRoll180();
    void GetMagRaw(int16_t& X, int16_t& Y, int16_t& Z);
    void GetAccelRaw(int16_t& X, int16_t& Y, int16_t& Z);
    void GetGyroRaw(int16_t& X, int16_t& Y, int16_t& Z);
    void GetAll(uint16_t& Bearing16Bit, int8_t& Pitch, int8_t& Roll);

    // Configuration Methods
    bool SetBaud19200();
    bool SetBaud38400();
    bool StoreCalibration();
    bool EraseStoredCalibration();
    void GetCalibrationState(uint8_t& systemCalibration, uint8_t& gyroCalibration, uint8_t& accelCalibration, uint8_t& magnetCalibration);
    bool EnterCalibrationMode();
    bool SetCalibrationConfig(bool periodicAutoSaveEnabled, bool GyroCalibrationEnabled, 
                                        bool AccelCalibrationEnabled, bool MagCalibrationEnabled);
    
  private:
    SoftwareSerial _cmps;
    uint8_t Read8BitUnsigned(byte address); 
    int8_t Read8Bit(byte address); 
    uint16_t Read16BitUnsigned(byte address);
    int16_t Read16Bit(byte address);
    void ReadRawData(int16_t& X, int16_t& Y, int16_t& Z, byte address);
    bool SendCommand(byte command, byte successResponseByte);
};
#endif
