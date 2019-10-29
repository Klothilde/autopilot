/***********************************************
 *     Core Module for the Yacht Autopilot     *
 *              by Sven Beckmann               *
 ***********************************************/

#ifndef MagAutopilotLib
#define MagAutopilotLib

#ifndef Arduino
#include <Arduino.h>
#endif

/************************************************************************************************************
 *                        A U T O P I L O T   S T A T E   M A C H I N E                                     *
 *                                                                                                          *
 *                                                                                                          *
 *                            +-------------------------------------------------+                           *
 *                            |                                                 | waited time x and the     *
 *                            |                                                 | headingTrackingArray      *
 *                            |                                                 | is not filled yet         *
 *                            v                                                 |                           *
 *     +-------+ true +--------------+ true  +-------------------+  true  +------------+                    *
 *     | Start |----->| Scan Heading |------>|   START WAITING   |------->|    WAIT    |                    *
 *     +-------+      +--------------+       +-------------------+        +------------+                    *
 *         ^                                           ^                        |                           *
 *         | User starts AP                            |                        | waited time x and the     *
 *         |                                           |                        | headingTrackingArray      *
 *     +-------+                                       |                        | is filled                 *
 *     |  Off  |                                       |                        |                           *
 *     +-------+                                       |                        |                           *
 *         ^       +----------------+     true         |                        |                           *
 *         |       |  Scan Heading  |------------------+                        |                           *
 *         +-------|       &        |                                           |                           *
 *     User turns  |  Move Rudder   |<------------------------------------------+                           *
 *     off the AP  +----------------+                                                                       *
 *                                                                                                          *
 *                                                                                                          *
 ************************************************************************************************************/ 
 
 /* 
  * States of the Autopilot
  */
#define AP_STATE_OFF 0
#define AP_STATE_START 1
#define AP_STATE_SCAN 2
#define AP_STATE_START_WAITING 3
#define AP_STATE_WAIT 4
#define AP_STATE_SCAN_AND_MOVE 5

#define AP_SMOOTHNESS_MODE_MEDIAN 0
#define AP_SMOOTHNESS_MODE_AVERAGE 1

/*
 * Boat Variables
 */
#define MAX_RUDDER_ANGLE 45

/*
 * Pin Settings
 * Currently not used
 * Needed later for connecting the real hardware
 */
#define CLUTCH_EN 0
#define CLUTCH_IN_1 0
#define CLUTCH_IN_2 0
#define CLUTCH_CS 0
#define CLUTCH_PWM 0
#define MOTOR_EN 0
#define MOTOR_IN_1 0
#define MOTOR_IN_2 0
#define MOTOR_CS 0
#define MOTOR_CURRENT 0


class MagAutopilot{

    public:
    MagAutopilot();
    void Start(uint8_t smoothnessMode, int8_t headingTrackingSize, uint16_t waitIntervalMilliseconds, int16_t targetHeading);
    void Stop();
    void Run(int16_t currentHeading);
    void SetSmoothnessMode(uint8_t smoothnessMode);
    uint16_t UserInputLeft(uint8_t degree);
    uint16_t UserInputRight(uint8_t degree);
    int16_t GetRudderAngle();
    uint16_t GetTargetHeading();

    private:
    unsigned long long int _waitStartedTime;
    uint16_t _waitMillis;
    int8_t _headingsmoothness;
    int16_t *_headingTrackingArray;
    uint16_t _targetHeading;
    int16_t _currentRudderAngle;
    uint8_t _currentState;
    uint8_t _currentSmoothnessMode;
    int16_t CalculateHeadingDelta();
    int16_t CalculateRudderAngle(int16_t headingDelta);
    int16_t CalculateSmoothedHeading();
    bool IsHeadingTrackingArrayFilled();
    void ScanHeading(uint16_t currentHeading);
    void QuickSort(int16_t arr[], int16_t left, int16_t right);
    void SetRudder(int16_t rudderAngle);
    int16_t GetCurrentRudderAngle(int16_t rudderAngle);

};
#endif

