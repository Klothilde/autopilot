/***********************************************
 *   Display Library for the Yacht Autopilot   *
 *              by Sven Beckmann               *
 *            based on the U8x8lib             *
 ***********************************************/

#include "DisplayLib.h"

   
/*
 * Constructor
 */
Display::Display(){}

/*
 * Initialize the display at program start
 */
void Display::Initialize(){
  U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
  u8x8.begin();
  _display = u8x8;
  _currentMode = DISPLAY_MODE_STANDBY;
}

/*
 * returns the current mode of the Display
 */
uint8_t Display::GetCurrentMode(){
  return _currentMode;
}

/*
 * Shows the navbar (footer) for the current display mode
 */
void Display::ShowNavbar(){
  _display.setFont(FONT_1x1);
  _display.drawString(0,6,"________________");
  switch(_currentMode){
    case DISPLAY_MODE_HEADING:
      _display.drawString(0,7,"Menu  << >>  OFF");      
      break;
    case DISPLAY_MODE_STANDBY:
      _display.drawString(0,7,"Menu          ON");
      break;
    case DISPLAY_MODE_MENU:
      _display.drawString(0,7,"Exit     UP DOWN");
      break;
  }
}

/*
 * Prints the line with Roll and Pitch information
 */
void Display::ShowRollAndPitch(int8_t roll, int8_t pitch){
  String rollString = (String)abs(roll);
  String pitchString = (String)abs(pitch);
  if(abs(roll) < 10)
    rollString += " ";
  if(abs(pitch) < 10)
    pitchString += " ";
  String displayString = "Roll "+rollString+" Pitch "+pitchString;
  _display.setFont(FONT_1x1);
  _display.drawString(0,UI_ROLL_LINE, displayString.c_str());
}

/*
 * Overrides the line of the input parameter with spaces
 * Afterwards the line on the Display is empty
 */
void Display::ClearRow(uint8_t row){
 _display.drawString(0,row,"                ");
}


/***********************************************
 *                Methods for:                 *
 *   Main display - controlling the autopilot  *
 ***********************************************/

/*
 * Turn on showing the autopilot settingss
 */
void Display::AutopilotOn(int16_t heading){
  _currentAutopilotHeading = heading;
  _currentMode = DISPLAY_MODE_HEADING;
  ShowAutopilotMode();
  ShowAutopilotTargetHeading();
}

/*
 * Turns off showing the autopilot settings
 */
void Display::AutopilotOff(){
  _currentMode = DISPLAY_MODE_STANDBY;
  // Clear AP Heading & Ruderanzeige
  HideAutopilotTargetHeading();
  ClearRow(UI_RUDDER_LINE);
  ShowAutopilotMode();
}

/*
 * Prints the current autopilot mode to the display
 */
void Display::ShowAutopilotMode(){
  switch(_currentMode){
    case DISPLAY_MODE_HEADING:
      _display.drawString(0,UI_MODE_LINE,"  HEADING MODE  ");
      break;
    case DISPLAY_MODE_STANDBY:
      _display.drawString(0,UI_MODE_LINE,"     STANDBY    ");
      ClearRow(UI_RUDDER_LINE);
      break;
  }
}

/*
 * Return if the autopilot is turned on or off
 * based on the current display state
 */
bool Display::IsAutopilotRunning(){
  if(_currentMode == DISPLAY_MODE_HEADING)
    return true;
  if(_currentMode == DISPLAY_MODE_MENU && _mainMode == DISPLAY_MODE_HEADING)
    return true;
  return false;
}

/*  
 * Shows the current Rudder position in the Display
 * Parameter "percent": Percent value of the Rudder angle
 * Positiv: rudder right | Negativ: rudder left
 */
void Display::ShowRudder(int8_t percent){
  _display.setFont(FONT_1x1);
  int8_t fields = abs(percent)/13;
  if(percent == 0){
    _display.setCursor(0,UI_RUDDER_LINE);
    _display.print("        0       ");
    return;
  }
  if(percent == 100){
    _display.drawString(0,UI_RUDDER_LINE,"      100");
    _display.setInverseFont(true);
    _display.drawString(9,UI_RUDDER_LINE,"       ");
    _display.setInverseFont(false);
    return;
  }
  if(percent == -100){
    _display.setInverseFont(true);
    _display.drawString(0,UI_RUDDER_LINE,"       ");
    _display.setInverseFont(false);
    _display.drawString(7,UI_RUDDER_LINE,"100      ");
    return;
  }
  if(fields == 0)
    fields = 1;
  String percString = (String)abs(percent);
  if(abs(percent)<10)
    percString = " "+percString;
  if(percent > 0){
    // Rudder right
    String displayString = "       "+percString;
    _display.drawString(0,UI_RUDDER_LINE,displayString.c_str());
    _display.setInverseFont(true);
    for(int i = 0; i<fields;i++){
      _display.drawString(9+i,UI_RUDDER_LINE," ");
    }
      _display.setInverseFont(false);
    for(int i = 0; i< 7-fields;i++){
      _display.drawString(9+fields+i,UI_RUDDER_LINE," ");
    }
  }else{
    // Rudder left
    for(int i = 0; i< 7-fields;i++){
      _display.drawString(i,UI_RUDDER_LINE," ");
    }
    _display.setInverseFont(true);
    for(int i = 0; i<fields;i++){
      _display.drawString(7-fields+i,UI_RUDDER_LINE," ");
    }
    _display.setInverseFont(false);

    String displayString = percString+"       ";
    _display.drawString(7,UI_RUDDER_LINE,displayString.c_str());
  }
}

/*
 * Prints the current Heading
 * If the autopilot is turned on: prints the heading of the autopilot too
 */ 
void Display::ShowHeading(uint16_t heading){
  String displayString;
  if(heading < 10)
    displayString = "    0";
  else if(heading < 100)
    displayString = "    "+(String)(heading/10);
  else if(heading < 1000)
    displayString = "   "+(String)(heading/10);
  else
    displayString = "  "+(String)(heading/10);
  _display.setFont(FONT_2x2);
  _display.drawString(0,0,displayString.c_str());
  _display.setCursor(10,0);
  _display.refreshDisplay();
}

/*
 * Updates the Heading the autopilot is following
 */
void Display::AutopilotHeadingUpdate(uint16_t heading){
  _currentAutopilotHeading = heading;
  ShowAutopilotTargetHeading();
}

/*
 * Shows the AutopilotTargetHeading on the Display
 */
void Display::ShowAutopilotTargetHeading(){
  _display.setFont(FONT_1x1);
  _display.drawString(10,1,("("+((String)(_currentAutopilotHeading/10))+")").c_str());
}

/*
 * Hids the AutopilotTargetHeading on the Display
 */
void Display::HideAutopilotTargetHeading(){
  _display.setFont(FONT_1x1);
  _display.drawString(10,1,"     ");
}

/***********************************************
 *                 Methods for:                *
 *                Menu controlling             *
 ***********************************************/

/*
 * Enters the menu
 */
void Display::EnterMenu(){
  _mainMode = _currentMode;
  _currentMode = DISPLAY_MODE_MENU;
  _currentMenuOption = 0;
  _display.clear();
  BuildMenu();
}

/*
 * One option up in the menu
 */
void Display::MenuUp(){
  if(_currentMenuOption > 0)
    _currentMenuOption--;
  BuildMenu();
}

/*
 * One option down in the menu
 */
void Display::MenuDown(){
  if(_currentMenuOption < 4)
    _currentMenuOption++;
  BuildMenu();
}

/*
 * Shows the menu in the display with the current selected value
 */
void Display::BuildMenu(){
  _display.setFont(FONT_1x1);
  for(uint8_t i = 0; i < 5; i++){
    String item = _menuItems[i];
    if(item.length() < 16) {
      for(uint8_t j = item.length(); j < 16;j++)
        item += " ";
    }else if(item.length() > 16){
      item = item.substring(0,15);
    }
    if(i == _currentMenuOption){
      _display.setInverseFont(true);
      _display.drawString(0,i,item.c_str());
      _display.setInverseFont(false);
    }else{
      _display.drawString(0,i,item.c_str());
    }
  }
}

/*
 * Leaving the menu - returns to the main display
 */
void Display::ExitMenu(){
  _display.clear();
  _currentMode = _mainMode;
  ShowAutopilotMode();
  if(IsAutopilotRunning())
    ShowAutopilotTargetHeading();
}





