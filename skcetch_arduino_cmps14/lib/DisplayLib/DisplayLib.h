/***********************************************
 *   Display Library for the Yacht Autopilot   *
 *              by Sven Beckmann               *
 *            based on the U8x8lib             *
 ***********************************************/

#ifndef DisplayLib
#define DisplayLib


#ifndef Arduino
#include <Arduino.h>
#endif

#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#define FONT_2x2 u8x8_font_profont29_2x3_n
#define FONT_1x1 u8x8_font_torussansbold8_r

/*
 * Definitions for sorting the lines in the main view
 */
#define UI_MODE_LINE 3
#define UI_RUDDER_LINE 4
#define UI_ROLL_LINE 5

/*
 * Definitions for the current displaymode
 */
#define DISPLAY_MODE_STANDBY 0
#define DISPLAY_MODE_HEADING 1
#define DISPLAY_MODE_MENU 2

class Display {

  public:
    // Constructor
    Display();

    // Control Methods for the Main Program
    void Initialize();
    uint8_t GetCurrentMode();
    void AutopilotOn(int16_t heading);
    void AutopilotOff();
    void ShowRollAndPitch(int8_t roll, int8_t pitch);
    void ShowHeading(uint16_t heading);
    void ShowRudder(int8_t percent);
    void MenuUp();
    void MenuDown();
    void AutopilotHeadingUpdate(uint16_t heading);
    bool IsAutopilotRunning();
    void EnterMenu();
    void ExitMenu();
    void ShowNavbar();

  private:
    U8X8_SH1106_128X64_NONAME_HW_I2C _display;
    uint8_t _currentMode;
    uint8_t _mainMode;
    uint8_t _currentMenuOption;
    uint8_t _currentMenuDepth;
    uint16_t _currentAutopilotHeading;
    String _menuItems[5] = {"Option 1", "Option 2", "Option 3", "Option 4", "Option 5"};

    void ShowAutopilotMode();
    void ShowAutopilotTargetHeading();
    void HideAutopilotTargetHeading();
    void ClearRow(uint8_t row);
    void BuildMenu();
};
#endif
