#include <Arduino.h>

#include "Servo.h"
#include <OneButton.h>
#include "CMPS14Serial.h"
#include "DisplayLib.h"
#include "MagAutopilotLib.h"

// PINS
#define BUTTON_1_PIN 7
#define BUTTON_2_PIN 8
#define BUTTON_3_PIN 12
#define BUTTON_4_PIN 13

#define MAIN_STATE_STANDBY 0
#define MAIN_STATE_AP_HEADING 1

CMPS14 cmps = CMPS14(2, 4);
Display myDisplay = Display();
MagAutopilot autopilot = MagAutopilot();
OneButton button1(BUTTON_1_PIN, false);
OneButton button2(BUTTON_2_PIN, false);
OneButton button3(BUTTON_3_PIN, false);
OneButton button4(BUTTON_4_PIN, false);


/*
 * Global Varialbes for holding the current Values from the Sensor
 * to avoid multiple scans in one loop
 */
uint16_t currentHeading = 0;
int8_t currentRoll = 0, currentPitch = 0;

/*
 * Called if Button 1 is clicked
 * Handles action of the button based on the current display state
 */
void Button1Clicked()
{
  switch(myDisplay.GetCurrentMode()){
    case DISPLAY_MODE_STANDBY:
    case DISPLAY_MODE_HEADING:
      myDisplay.EnterMenu();
      break;
    case DISPLAY_MODE_MENU:
      myDisplay.ExitMenu();
      break;
  }
}

/*
 * Called if Button 2 is clicked
 * Handles action of the button based on the current display state
 */
void Button2Clicked()
{
  switch(myDisplay.GetCurrentMode()){
    case DISPLAY_MODE_HEADING:
      myDisplay.AutopilotHeadingUpdate(autopilot.UserInputLeft(1));
      break;
      break;
  }
}

/*
 * Called if Button 3 is clicked
 * Handles action of the button based on the current display state
 */
void Button3Clicked()
{
  switch(myDisplay.GetCurrentMode()){
    case DISPLAY_MODE_HEADING:
      myDisplay.AutopilotHeadingUpdate(autopilot.UserInputRight(1));
      break;
    case DISPLAY_MODE_MENU:
      myDisplay.MenuUp();
      break;
  }
}

/*
 * Called if Button 4 is clicked
 * Handles action of the button based on the current display state
 */
void Button4Clicked(){
  switch(myDisplay.GetCurrentMode()){
    case DISPLAY_MODE_STANDBY:
      myDisplay.AutopilotOn(currentHeading);
      autopilot.Start(AP_SMOOTHNESS_MODE_AVERAGE, 15, 100, currentHeading);
      break;
    case DISPLAY_MODE_HEADING:
      myDisplay.AutopilotOff();
      autopilot.Stop();
      break;
    case DISPLAY_MODE_MENU:
      myDisplay.MenuDown();
  }
}

/*
 * Checks the current display mode and sends updates to the display if needed
 * For example: current heading
 */
void displayLoop()
{
  uint8_t cDM = myDisplay.GetCurrentMode();
  // Loop-Aktualisierungen für das Display

  if(cDM == DISPLAY_MODE_STANDBY){
    // Display im Standby Modus
    myDisplay.ShowHeading(currentHeading);
    myDisplay.ShowRollAndPitch(currentRoll, currentPitch);
  }else if(cDM == DISPLAY_MODE_HEADING){
    // Display im Heading Modus
    myDisplay.ShowHeading(currentHeading);
    myDisplay.ShowRollAndPitch(currentRoll, currentPitch);
    myDisplay.ShowRudder(autopilot.GetRudderAngle());
  }
  myDisplay.ShowNavbar();
}

/*
 * Setup method of arduino
 */
void setup()
{
  Serial.begin(9600);
  button1.attachClick(Button1Clicked);
  button2.attachClick(Button2Clicked);
  button3.attachClick(Button3Clicked);  
  button4.attachClick(Button4Clicked);
  myDisplay.Initialize();
}

/*
 * Loop method of arduino
 */
void loop()
{
  cmps.GetAll(currentHeading, currentPitch, currentRoll);
  if(myDisplay.IsAutopilotRunning()){
    autopilot.Run(currentHeading);
  }
  button1.tick();
  button2.tick();
  button3.tick();
  button4.tick();
  displayLoop();
}