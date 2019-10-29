/***********************************************
 *     Core Module for the Yacht Autopilot     *
 *              by Sven Beckmann               *
 ***********************************************/
#include "MagAutopilotLib.h"


/*
 * Constructor
 */
MagAutopilot::MagAutopilot(){
    _currentState = AP_STATE_OFF;
}

/***********************************************
 *             PUBLIC METHODS                  *
 ***********************************************/


/*
 * Starts the autopilot
 * 
 * SmoothnessMode can be set to Average or Median
 * HeadingTrackingSize: Number of Values for calculating average or median
 * WaitIntervalMilliseconds: Milliseconds between the heading lookups
 * targetHeading: Heading the autopilot wants to hold
 */
void MagAutopilot::Start(uint8_t smoothnessMode, int8_t headingTrackingSize, uint16_t waitIntervalMilliseconds, int16_t targetHeading){
    // waitMillis and headingsmoothness will be read
    // from Settings later
    _currentSmoothnessMode = smoothnessMode;
    _waitMillis = waitIntervalMilliseconds;
    _headingsmoothness = headingTrackingSize;
    _headingTrackingArray = (int16_t *)malloc(sizeof(uint16_t) * _headingsmoothness);

    _targetHeading = targetHeading;
    _currentState = AP_STATE_START;
}

/*
 * Stops the autopilot
 */
void MagAutopilot::Stop(){
    _currentState = AP_STATE_OFF;
    _currentRudderAngle = 0;
}

/*
 * User send the input to turn the target heading of the autopilot left
 */
uint16_t MagAutopilot::UserInputLeft(uint8_t degree){
    _targetHeading -= degree*10;
    return _targetHeading;
}

/*
 * User send the input to turn the target heading of the autopilot right
 */
uint16_t MagAutopilot::UserInputRight(uint8_t degree){
    _targetHeading += degree*10;
    return _targetHeading;
}

/*
 * Method that must be called from the arduino loop
 * if the autopilot is turned on
 * 
 * handles one operation of the autopilot based on the
 * autopilot state machine shown in the header file
 */
void MagAutopilot::Run(int16_t currentHeading){
    if(_currentState == AP_STATE_OFF){
        // do nothing
    }else if(_currentState == AP_STATE_START){
        // Initialize Tracking Array with -1 (not scanned value)
        for(int8_t i = 0; i<_headingsmoothness;i++){
            _headingTrackingArray[i] = -1;
        }
        _currentState = AP_STATE_SCAN;
    }else if(_currentState == AP_STATE_SCAN){
        ScanHeading(currentHeading);
        _currentState = AP_STATE_START_WAITING;
    }else if(_currentState == AP_STATE_START_WAITING){
        _waitStartedTime = millis();
        _currentState = AP_STATE_WAIT;
    }else if(_currentState == AP_STATE_WAIT){
        if(millis() - _waitStartedTime > _waitMillis){
            if(IsHeadingTrackingArrayFilled())
                _currentState = AP_STATE_SCAN_AND_MOVE;
            else
                _currentState = AP_STATE_SCAN;
        }
    }else if(_currentState == AP_STATE_SCAN_AND_MOVE){
        ScanHeading(currentHeading);
        int16_t delta = CalculateHeadingDelta();
        if(delta != 0){
            int16_t newRudderAngle = CalculateRudderAngle(delta);
            if(newRudderAngle != _currentRudderAngle)
                SetRudder(newRudderAngle);
        }
        _currentState = AP_STATE_START_WAITING;
    }
}

/*
 * Returns the current rudder angle
 */
int16_t MagAutopilot::GetRudderAngle(){
    return _currentRudderAngle/10;
}

/*
 * Sets the current used smoothnes mode
 */
void MagAutopilot::SetSmoothnessMode(uint8_t smoothnessMode){
    _currentSmoothnessMode = smoothnessMode;
}


/***********************************************
 *             PRIVATE METHODS                 *
 ***********************************************/


/*
 * Calculates the current delta to the smoothed Heading
 * 
 *    negative delta   TARGET   positiv delta
 * <----------------------|----------------------->
 */
int16_t MagAutopilot::CalculateHeadingDelta(){
    int16_t delta = CalculateSmoothedHeading() - _targetHeading;
    if(delta < -1800)
        return delta += 3600;
    if(delta > 1800)
        return delta -= 3600;
    return delta;
}

/*
 *   Calculates smoothed heading of the boat
 *     Average or Median based on the mode
 */
int16_t MagAutopilot::CalculateSmoothedHeading(){
    if(_currentSmoothnessMode == AP_SMOOTHNESS_MODE_MEDIAN){
        int16_t headings[_headingsmoothness];
        for(uint8_t i = 0; i<_headingsmoothness;i++){
            headings[i] = _headingTrackingArray[i];
        }
        QuickSort(headings, 0, _headingsmoothness-1);
        return headings[(_headingsmoothness/2)+1];
    }else if(_currentSmoothnessMode == AP_SMOOTHNESS_MODE_AVERAGE){
        uint16_t sum = 0;
        for(uint8_t i = 0; i<_headingsmoothness;i++){
            sum += _headingTrackingArray[i];
        }
        return sum/_headingsmoothness;
    }
    return 0;
}

/*
 * Calculates the angle the rudder needs to be set
 */
int16_t MagAutopilot::CalculateRudderAngle(int16_t headingDelta){
    return -headingDelta;
}

/*
 * Pushes the current Heading into the TrackingArray
 */
void MagAutopilot::ScanHeading(uint16_t currentHeading){
    for(int8_t i = _headingsmoothness-1;i>0;i--){
        _headingTrackingArray[i] = _headingTrackingArray[i-1];
    }
    _headingTrackingArray[0] = (int16_t)currentHeading;
}

/*
 * Method checks if the HeadingTrackingArray is filled
 * with valid entries
 */
bool MagAutopilot::IsHeadingTrackingArrayFilled(){
    for(int8_t i = 0;i<_headingsmoothness;i++){
        if(_headingTrackingArray[i] < 0){
            return false;
        }
    }
    return true;
}

/*
 * Method to send the commands to set the boat rudder to the given angle
 */
void MagAutopilot::SetRudder(int16_t rudderAngle){
    _currentRudderAngle = rudderAngle;
}

/*
 * Method to read the current rudder angle
 */
int16_t MagAutopilot::GetCurrentRudderAngle(int16_t rudderAngle){
    return 0;
}

/*
 *   Sorting an array
 */
void MagAutopilot::QuickSort(int16_t arr[], int16_t left, int16_t right) {
      int16_t i = left, j = right;
      int16_t tmp;
      int16_t pivot = arr[(left + right) / 2];
      while (i <= j) {
            while (arr[i] < pivot)
                  i++;
            while (arr[j] > pivot)
                  j--;
            if (i <= j) {
                  tmp = arr[i];
                  arr[i] = arr[j];
                  arr[j] = tmp;
                  i++;
                  j--;
            }
      };
      if (left < j)
            QuickSort(arr, left, j);
      if (i < right)
            QuickSort(arr, i, right);
}

