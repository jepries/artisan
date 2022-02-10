//FEATURES TO TEST
// 

//issues, 
//     Define a clean state
//     




// References:
//   https://github.com/end2endzone/SoftTimers
//   https://github.com/facts-engineering/P1AMCore
//   https://github.com/cotestatnt/YA_FSM
//   https://github.com/facts-engineering/P1AM
//   https://github.com/duinoWitchery/hd44780
//   https://github.com/arduino-libraries/RTCZero



#include <Wire.h> //allow us to communicate with the LCD panel
#include <hd44780.h> // to actually talk to the LCD panel in the language it understands
#include <hd44780ioClass/hd44780_I2Cexp.h> // i/o expander/backpack class
#include <RTCZero.h> // set/read a real time clock for convenience
#include <P1AM.h> // control the P1 microcontroller industrial IO boards
#include "commands.h" // commands we will accept on the console for date/etc
#include <SoftTimers.h> // real time timers, never use pause!
#include <YA_FSM.h> // state machine, states and transitions
//#include <string.h> //string libraries

#define DEV_PLATFORM //comment line out for PROD platform
//#define DEBUG_STATES //uncoment when done troubleshooting states

/*****************************************************************
   Chanel Labels, English Words for IO Pins
   The below is the PRODUCTION CONFIGURATION.
   _____  _____  _____  _____  _____  _____
  |  P  ||  P  ||  S  ||  S  ||  S  ||  S  |
  |  1  ||  1  ||  L  ||  L  ||  L  ||  L  |
  |  A  ||  A  ||  O  ||  O  ||  O  ||  O  |
  |  M  ||  M  ||  T  ||  T  ||  T  ||  T  |
  |  -  ||  -  ||     ||     ||     ||     |
  |  G  ||  1  ||  0  ||  0  ||  0  ||  0  |
  |  P  ||  0  ||  1  ||  2  ||  3  ||  4  |
  |  I  ||  0  ||     ||     ||     ||     |
  |  O  ||     ||     ||     ||     ||     |
   ¯¯¯¯¯  ¯¯¯¯¯  ¯¯¯¯¯  ¯¯¯¯¯  ¯¯¯¯¯  ¯¯¯¯¯

  P1AM-GPIO : to talk to LCD Screen
  P1AM-100  : your micro controller
  SLOT 1    : 8 toggle switches
  SLOT 2    : 8 toggle switches
  SLOT 3    : 15 Sourcing Digital Out - Level 1 and 2 valves
  SLOT 4    : 15 Sourcing Digital Out - Level 3 and 4 valves AND PUMP

  DEVELOPMENT PLATFORM (DEV_PLATFORM)
  P1AM-GPIO : to talk to LCD Screen
  P1AM-100  : your micro controller
  SLOT 1    : 8 toggle switches
  SLOT 2    : 8 output relay
  SLOT 3    : 15 Sourcing Digital Out - Level 1 and 2 valves

 ****************************************************************/

#ifdef DEV_PLATFORM
//this will compile for the NONPROD platform
channelLabel pump = {3, 15};
channelLabel pumpOverride = {1, 8};

// vent valve
channelLabel VV_1 = {2, 1};

// station disalble switches, FS = Flood Station, D = Disable
channelLabel FSA_D = {1, 1};
channelLabel FSB_D = {1, 2};
channelLabel FSC_D = {1, 3};
channelLabel FSD_D = {1, 4};
channelLabel FSE_D = {1, 5};
channelLabel FSF_D = {1, 6};

//valves, V=valve, S=station, O for overflow, D for drain
//valves, V=valve, L=Level, O for overflow, D for drain, Overflow is a MANUAL valve
channelLabel VL1_D = {3, 1};
channelLabel VSA_O = {3, 2};
channelLabel VSA_D = {3, 3};
channelLabel VSB_O = {3, 4};
channelLabel VSB_D = {3, 5};
channelLabel VSC_O = {3, 6};
channelLabel VSC_D = {3, 7};

channelLabel VL2_D = {3, 8};
channelLabel VSD_O = {3, 9};
channelLabel VSD_D = {3, 10};
channelLabel VSE_O = {3, 11};
channelLabel VSE_D = {3, 12};
channelLabel VSF_O = {3, 13};
channelLabel VSF_D = {3, 14};

channelLabel StationDrainValves[]      = { VSA_D, VSB_D, VSC_D, VSD_D, VSE_D, VSF_D };
channelLabel StationOverflowValves[]   = { VSA_O, VSB_O, VSC_O, VSD_O, VSE_O, VSF_O };
channelLabel StationDisabledSwitches[] = { FSA_D, FSB_D, FSC_D, FSD_D, FSE_D, FSF_D };
channelLabel LevelDrainValves[]        = { VL1_D, VL2_D };
const int numLevels = 2;
const int numStationsPerLevel = 3;
const int numStations = 6;
bool enabledStations[]   = { true, true, true, true, true, true };
bool previousStations[] =  { true, true, true, true, true, true };

#else
// THIS WILL COMPILE FOR THE PROD PLATFORM
channelLabel pump = {3, 15};
channelLabel pumpOverride = {2, 8};

// vent valve
channelLabel VV_1 = {4, 15};

// station disalble switches, FS = Flood Station, D = Disable
channelLabel FSA_D = {1, 1};
channelLabel FSB_D = {1, 2};
channelLabel FSC_D = {1, 3};
channelLabel FSD_D = {1, 4};
channelLabel FSE_D = {1, 5};
channelLabel FSF_D = {1, 6};

channelLabel FSG_D  = {2, 1};
channelLabel FSH_D  = {2, 2};
channelLabel FSI_D  = {2, 3};
channelLabel FSJ_D  = {2, 4};
channelLabel FSK_D  = {2, 5};
channelLabel FSL_D  = {2, 6};

//valves, V=valve, S=station, O for overflow, D for drain
//valves, V=valve, L=Level, O for overflow, D for drain, Overflow is a MANUAL valve
channelLabel VL1_D = {3, 1};
channelLabel VSA_O = {3, 2};
channelLabel VSA_D = {3, 3};
channelLabel VSB_O = {3, 4};
channelLabel VSB_D = {3, 5};
channelLabel VSC_O = {3, 6};
channelLabel VSC_D = {3, 7};

channelLabel VL2_D = {3, 8};
channelLabel VSD_O = {3, 9};
channelLabel VSD_D = {3, 10};
channelLabel VSE_O = {3, 11};
channelLabel VSE_D = {3, 12};
channelLabel VSF_O = {3, 13};
channelLabel VSF_D = {3, 14};

channelLabel VL3_D = {4, 1};
channelLabel VSG_O = {4, 2};
channelLabel VSG_D = {4, 3};
channelLabel VSH_O = {4, 4};
channelLabel VSH_D = {4, 5};
channelLabel VSI_O = {4, 6};
channelLabel VSI_D = {4, 7};

channelLabel VL4_D  = {4, 8};
channelLabel VSJ_O = {4, 9};
channelLabel VSJ_D = {4, 10};
channelLabel VSK_O = {4, 11};
channelLabel VSK_D = {4, 12};
channelLabel VSL_O = {4, 13};
channelLabel VSL_D = {4, 14};

channelLabel StationDrainValves[]      = { VSA_D, VSB_D, VSC_D, VSD_D, VSE_D, VSF_D, VSG_D, VSH_D, VSI_D, VSJ_D, VSK_D, VSL_D };
channelLabel StationOverflowValves[]   = { VSA_O, VSB_O, VSC_O, VSD_O, VSE_O, VSF_O, VSG_O, VSH_O, VSI_O, VSJ_O, VSK_O, VSL_O };
channelLabel StationDisabledSwitches[] = { FSA_D, FSB_D, FSC_D, FSD_D, FSE_D, FSF_D, FSG_D, FSH_D, FSI_D, FSJ_D, FSK_D, FSL_D };
channelLabel LevelDrainValves[]        = { VL1_D, VL2_D, VL3_D, VL4_D };
const int numLevels = 4;
const int numStationsPerLevel = 3;
const int numStations = 12;
bool enabledStations[]  = { true, true, true, true, true, true, true, true, true, true, true, true };
bool previousStations[] = { true, true, true, true, true, true, true, true, true, true, true, true };

#endif
//Station Labels
char stationLabels[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L'};
hd44780_I2Cexp lcd; // auto detect backpack and pin mappings
RTCZero rtc;  // Create an rtc object

// LCD geometry
const int LCD_COLS = 20;
const int LCD_ROWS = 4;

// FLOOD TIMES - Military Time, 800 = 0800, 1600 = 4pm,-1 is not set.
int floodTimes [] = {800, 1600, -1, -1}; // 8am, 4pm, nada, dada
int numFloodTimes = 4; //how many flood times will we allow?
int nextFloodTime = -1; //next flood time
bool triggerFlood = false; //has the flood triggered?

// Create new FSM
YA_FSM stateMachine;

// State Alias
enum State { Idle, Flood, IdlePauseTemp, IdlePause, IdlePumpOvr, IdlePumpOff, FldPauseTemp, FldPause, FldPumpOvr, FldPumpOff, Clean, ClnPumpOvr, ClnPumpOff };

// Helper for print labels instead integer when state change
const char* const stateName[] PROGMEM = { "Idle", "Flood", "Idle Pause Temp", "Idle Pause", "Pause Pump Override", "Pause Pump Off",
                                          "Flood Pause Temp", "Flood Pause", "Pause Pump Override", "Pause Pump Off", 
                                          "Clean", "Clean Pump Override", "Clean Pump Off"
                                        };


SoftTimer updateLcdTimer;

//Flood Timer
SoftTimer floodTimer; //millisecond timer
unsigned long floodTime = 300000; //Default 5 minutes

//Valves take 5 seconds to open, lets wait them out
SoftTimer valveTimer;
unsigned long valveTime = 5000; //5 seconds for valves to open

//1 second timer for updating seconds
SoftTimer secondTimer;

//5 second timer for dimming the backlight
SoftTimer backlightTimer;

bool updateStateNow = false;

boolean alreadyDimmed = false;
bool switchState = 0; //Variable to hold the current state of the switch
bool prevSwitchState = 0; //only update when state change detected
bool updateLine3 = true; // and only when it needs to be
bool pumpSwitchState = 0; //Varialbe to hold the current state of the pump override switch
int output = 0;
uint8_t prevPumpState = LOW; //store the last known state of the pump

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// SETUP ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/*****************************************
   our SETUP routine, only runs at startup
 *****************************************/
void setup() {
  // put your setup code here, to run once:
  int lcdstatus; //lcd status

  pinMode(LED_BUILTIN, OUTPUT); //Set our LED (Pin 32) to be an output
  pinMode(SWITCH_BUILTIN, INPUT); //Set our Switch (Pin 31) to be an input

  Serial.begin(115200);  //initialize serial communication at 115200 bits per second
  while (!P1.init()) {
    ; //Wait for Modules to Sign on
  }
  //watchdog, reset the controller if we don't pet the dog
  P1.configWD(5000, TOGGLE); //Pass in the timer value and TOGGLE will reset the CPU after 5000 milliseconds has passed.

  lcdstatus = lcd.begin(LCD_COLS, LCD_ROWS);
  if (lcdstatus) // non zero status means it was unsuccesful
  {
    // hd44780 has a fatalError() routine that blinks an led if possible
    // begin() failed so call fatalError() with the error code.
    hd44780::fatalError(lcdstatus); // does not return
  }
  turnOnLcd();

  rtc.begin(); // initialize RTC
  // Print a message to the LCD
  lcd.clear();
  lcd.setCursor(2, 0);
  lcd.print("ARTISAN GROWERS");

  Serial.println("ARTISAN GROWERS   !!!");

  //set up the next flood alarm
  int nextFloodTime = getNextFloodTime();
  Serial.print("next flood time: ");
  Serial.println(nextFloodTime);
  if ( !setNextFloodAlarm(nextFloodTime) ) {
    Serial.println("BAD FLOOD TIME");
    Serial.println(getNextFloodTime());
    hd44780::fatalError(-1); // does not return
  }
  
  floodTimer.setTimeOutTime(floodTime);
  floodTimer.reset();

  updateLcdTimer.setTimeOutTime(30000); //lets update the lcd once per minute, loop counter 2 times this
  updateLcdTimer.reset(); //start the lcd update timer

  secondTimer.setTimeOutTime(1000);
  valveTimer.setTimeOutTime(valveTime);
  backlightTimer.setTimeOutTime(10000);


  printMenu();

  //PUMP OFF
  prevPumpState = HIGH;
  controlPump(pump, LOW);
  //OPEN the Level Drain Valves
  controlLevelDrainValves(HIGH);
  //Open the Vent Valve
  controlVentValve(HIGH);
  //Open the Drain Vavles
  controlFloodStationDrainValves(HIGH);
  //Open the Overflow Valves
  controlFloodStationOverflowValves(HIGH);

  setupStateMachine();

}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// LCD ROUTINES ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//LINE 0: DATE/TIME
//LINE 1: Disabled Stations
//LINE 2: Current State and Time
//Line 3: Pump / Run

/*****************************************
   Clear a line on the LCD
******************************************/
void printDisabledStations(hd44780_I2Cexp &outlcd) {
  //01234567890123456789
  //Dis: ABCDEFGHIJKL
  clearLcdLine(outlcd, 1);
  outlcd.setCursor(0,1);
  outlcd.print("Dis: ");
  // enabledStations array contains true/false
  for (int i=0; i<numStations; i++) {
    if (!enabledStations[i]) {
      outlcd.print(stationLabels[i]);
    }
  }
    
}
void turnOnLcd() {
  lcd.backlight();
  alreadyDimmed = false;
  backlightTimer.reset();
}

/*****************************************
   Clear a line on the LCD
******************************************/
void clearLcdLine(hd44780_I2Cexp &outlcd, int line) {
  outlcd.setCursor(0, line);
  outlcd.print("                    ");
}

/*****************************************
   Print Pump Status
******************************************/
void printLcdPumpState(hd44780_I2Cexp &outlcd) {
  outlcd.setCursor(0,3);
  if (prevPumpState) 
    outlcd.print("PUMP ON ");
  else
    outlcd.print("PUMP OFF");
  updateLine3 = true;
}
/*****************************************
   Prints Current State with remaining time to LCD Screen.
 *****************************************/
void printState(hd44780_I2Cexp &outlcd, String stateLabel, unsigned long remainingmillis) {
  clearLcdLine(outlcd, 2);
  //Serial.println("printState");
  
  outlcd.setCursor(0, 2);
  outlcd.print(stateLabel);
  if (remainingmillis == 0) {
    for (int i = stateLabel.length(); i < LCD_COLS; i++)
      outlcd.write(' ');
  } else {
    outlcd.write(' ');
    int hours = remainingmillis / 3600000;
    int minutes = (remainingmillis % 3600000) / 60000;
    if (hours <= 9) outlcd.write('0');
    outlcd.print((int)hours);
    outlcd.write(':');
    if (minutes <= 9) outlcd.write('0');
    outlcd.print((int)minutes);

    if (remainingmillis < 60000) {
      outlcd.write(':');
      int seconds = ((remainingmillis % 3600000) % 60000) / 1000;
      if (seconds < 10) outlcd.write('0');
      outlcd.print((int)seconds);
      secondTimer.reset();
    }
  }
  updateStateNow = false;
}

/*****************************************
   Prints TIME and DATE to LCD Screen
 *****************************************/
void printTimeDate(hd44780_I2Cexp &outlcd) {
  // date will look like MM/DD/YY HH:MI
  outlcd.setCursor(0, 0);
  outlcd.write("   ");
  int month = rtc.getMonth();
  if ( month < 10 )
    outlcd.write('0');
  outlcd.print((int)month);
  lcd.write("/");
  int day = rtc.getDay();
  if ( day < 10 )
    outlcd.write('0');
  outlcd.print((int)day);
  outlcd.print('/');
  int year = rtc.getYear();
  if ( year < 10 )
    outlcd.write('0');
  outlcd.print((int)year);

  outlcd.write(" ");

  int hour = rtc.getHours();
  if ( hour < 10 )
    outlcd.write('0');
  outlcd.print((int)hour);
  outlcd.write(':');

  int min = rtc.getMinutes();
  if ( min < 10 )
    outlcd.write('0');
  outlcd.print((int)min);
  lcd.write("   ");
}



//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// SWITCH INPUTS ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


/*****************************************************************************
 * RUN SWITCH
 *****************************************************************************/
bool checkRunSwitch() {
  switchState = digitalRead(SWITCH_BUILTIN) == HIGH;//Read the state of the switch
  if (prevSwitchState != switchState || updateLine3) {
    prevSwitchState = switchState;
    updateLine3 = false;
    digitalWrite(LED_BUILTIN, switchState); //Update the LED
    if (switchState) {
      lcd.setCursor(13, 3);
      lcd.write("running");
    } else {
      lcd.setCursor(13, 3);
      lcd.write(" paused");
    }
    turnOnLcd();
  }
  return switchState;
}
bool notCheckRunSwitch() {
  return !checkRunSwitch();
}

/*****************************************************************************
 * PUMP SWITCH
 *****************************************************************************/
bool checkPumpSwitch() {
  pumpSwitchState = P1.readDiscrete(pumpOverride) == HIGH;
  return pumpSwitchState;
}
bool notCheckPumpSwitch() {
  return !checkPumpSwitch();
}

/*****************************************************************************
 * FLOOD STATION SWITCHES
 *****************************************************************************/
void checkDisabledFloodStations() {
  //true, the station is enabled
  //false, the station is disabled
  //INPUT, LOW the station is enabled.
  int stateChangeDetected = false;
  for (int i=0; i<numStations; i++) {
    enabledStations[i] = P1.readDiscrete(StationDisabledSwitches[i]) == LOW;
    //immediately turn off the station if we see this
    if (enabledStations[i] != previousStations[i] && !enabledStations[i]) {
      P1.writeDiscrete(LOW, StationOverflowValves[i]);
      P1.writeDiscrete(LOW, StationDrainValves[i]);
    }
    if (enabledStations[i] != previousStations[i] && enabledStations[i] //&&
        /*stateMachine.GetState() <= 1 */) { //and we are in idle or flood, turn valves back on
      P1.writeDiscrete(HIGH, StationOverflowValves[i]);
      P1.writeDiscrete(HIGH, StationDrainValves[i]);
    }
    if (enabledStations[i] != previousStations[i]) {
      //state change detected
      previousStations[i] = enabledStations[i];
      stateChangeDetected = true;
      turnOnLcd();
    }
  }
  if (stateChangeDetected)
    printDisabledStations(lcd);
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// CONSOLE INPUTS //////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/*****************************************
   print out our menut to the user
 *****************************************/
void printMenu() {
  Serial.println("");
  Serial.println("******************************************************");
  Serial.println("*  commands: y, m, d, h, M xx                        *");
  Serial.println("*  commands: D mm/dd/yy                              *");
  Serial.println("*  commands: T hh:MM                                 *");
  Serial.println("*  commands: I hh:MM (set idle timer)                *");
  Serial.println("*  commands: F MM:SS (set flood timer in minutes)    *");
  Serial.println("*  commands: L (List Schedule)                       *");
  Serial.println("*  commands: 1 800  (Set First Flood Slot to 8am)    *");
  Serial.println("*  commands: 2 1600 (Set Second Flood Slot to 4pm)   *");
  Serial.println("*  commands: 3 1900 (Set Third Flood Slot to 7pm)    *");
  Serial.println("*  commands: 4 -1   (DISABLE Fourth Flood Slot)      *");
  Serial.println("*  commands: N (Next Flood Time)                     *");
  Serial.println("*                                                    *");
  Serial.println("*  example to set year: y 22                         *");
  Serial.println("*  example to set full date: D 02/19/22              *");
  Serial.println("*  example to set full time: T 16:47                 *");
  Serial.println("*  example to set flood time for 4 minutes: F 4:0    *");
  Serial.println("*  example to set SCHEDULE 1: 1 800                  *");
  Serial.println("*  example to set SCHEDULE 2: 2 1600                 *");
  Serial.println("*  example to set SCHEDULE 3: 3 1800                 *");
  Serial.println("*  example to set SCHEDULE 4: 4 -1                   *");
  Serial.println("*  example to LIST SCHEDULE: L                       *");
  Serial.println("******************************************************");
  Serial.println("");

}

// A pair of varibles to help parse serial commands
int arg = 0;
int indexcmd = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[32];
char argv2[32];

// The arguments converted to integers
long arg1;
long arg2;
/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  indexcmd = 0;
}
/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i;
  char *p = argv1;
  char *str;
  int dt_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  String output;

  switch (cmd) {
    case SET_YEAR:
      Serial.println("setting year");
      rtc.setYear(arg1);
      break;
    case SET_MONTH:
      Serial.println("setting month");
      rtc.setMonth(arg1);
      break;
    case SET_DAY:
      Serial.println("setting day");
      rtc.setDay(arg1);
      break;
    case SET_HOUR:
      Serial.println("setting hour");
      rtc.setHours(arg1);
      break;
    case SET_MINUTE:
      Serial.println("setting minute");
      rtc.setMinutes(arg1);
      rtc.setSeconds(0);
      break;
    case SET_DATE:
      i = 0;
      while ((str = strtok_r(p, "/", &p)) != '\0') {
        dt_args[i] = atoi(str);
        i++;
      }
      Serial.println("setting date");
      rtc.setMonth(dt_args[0]);
      rtc.setDay(dt_args[1]);
      rtc.setYear(dt_args[2]);
      break;
    case SET_TIME:
      i = 0;
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        dt_args[i] = atoi(str);
        i++;
      }
      Serial.println("setting time");
      rtc.setHours(dt_args[0]);
      rtc.setMinutes(dt_args[1]);
      rtc.setSeconds(0);
      break;
    case SET_FLOOD_TIME:
      i = 0;
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        dt_args[i] = atoi(str);
        i++;
      }
      Serial.println("setting flood time");
      floodTime = dt_args[0] * 60000 + dt_args[1] * 1000;
      floodTimer.setTimeOutTime(floodTime);
      floodTimer.reset();
      updateStateNow = true;
      break;
    case SET_FLOOD_1:
      floodTimes[0] = (int)arg1;
      setNextFloodAlarm(getNextFloodTime());
      updateStateNow = true;
      break;
    case SET_FLOOD_2:
      floodTimes[1] = (int)arg1;
      setNextFloodAlarm(getNextFloodTime());
      updateStateNow = true;
      break;
    case SET_FLOOD_3:
      floodTimes[2] = (int)arg1;
      setNextFloodAlarm(getNextFloodTime());
      updateStateNow = true;
      break;
    case SET_FLOOD_4:
      floodTimes[3] = (int)arg1;
      setNextFloodAlarm(getNextFloodTime());
      updateStateNow = true;
      break;
    case LIST_FLOODS:
      Serial.print("Flood Times: ");
      Serial.print(floodTimes[0]);
      Serial.print(",");
      Serial.print(floodTimes[1]);
      Serial.print(",");
      Serial.print(floodTimes[2]);
      Serial.print(",");
      Serial.println(floodTimes[3]);
      break;
    case NEXT_FLOOD:
      Serial.print("Next Flood Time: ");
      Serial.println(rtc.getAlarmHours()*100 + rtc.getAlarmMinutes());
      break;
    case PRINT_MENU:
      printMenu();
      break;
    default:
      Serial.println("Invalid Command");
  }
  printTimeDate(lcd);
  turnOnLcd();
  //printMenu();
}

/*****************************************
   Check the serial console for input
 *****************************************/
void checkConsole() {
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[indexcmd] = NULL;
      else if (arg == 2) argv2[indexcmd] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[indexcmd] = NULL;
        arg = 2;
        indexcmd = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[indexcmd] = chr;
        indexcmd++;
      }
      else if (arg == 2) {
        argv2[indexcmd] = chr;
        indexcmd++;
      }
    }
  }

}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// TIME FUNCTIONS FOR FLOODING  ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
int currentMilTime() {
  return rtc.getHours()*100+rtc.getMinutes();
}
int getNextFloodTime() {
  for (int i=0; i<numFloodTimes; i++) {
    if (floodTimes[i] > currentMilTime()) {
      return floodTimes[i];
    }
  }
  //if we get here, lets loop back to first timestamp that is > 0
  for (int i=0; i<numFloodTimes; i++) {
    if (floodTimes[i] > 0) {
      return floodTimes[i];
    }
  }
  //if we get here, there are no flood times configured.
  return -1; //negative flood time is NO FLOOD TIME  
}
bool setNextFloodAlarm(int ft) {
  if (ft < 0) {
    Serial.println(" ##### NO FLOOD TIME AVAILABLE ###### ");
    rtc.disableAlarm();
    return false;
  }
  rtc.setAlarmTime((int)ft/100, (int)ft%100, 0);
  triggerFlood = false;
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
  rtc.attachInterrupt(alarmMatch);
  return true;
}
void alarmMatch() {
  triggerFlood = true;
}
long millisTillNextFlood() {
  int alarmHour = rtc.getAlarmHours();
  int alarmMin = rtc.getAlarmMinutes();
  int nowHour = rtc.getHours();
  int nowMin = rtc.getMinutes();
  int nowSec = rtc.getSeconds();

  //deal with rollover :(
  int alarmHM = alarmHour * 100 + alarmMin;
  int nowHM = nowHour * 100 + nowMin;
#ifdef DEBUG_STATES
  Serial.print("AlarmHM: ");
  Serial.print(alarmHM);
  Serial.print(", NowHM: ");
  Serial.println(nowHM);
#endif

  int diffMin = 0;
  int diffHour = 0;
  if (nowHM > alarmHM){
    //we are say at like 5pm (1700) and the next alarm is at 8am (0800)
    if (alarmMin == nowMin) {
      diffMin = 0;
      diffHour = 24 + alarmHour - nowHour;
    } else if (alarmMin < nowMin) {
      diffMin = 60 + alarmMin - nowMin - 1;
      diffHour = 24 - nowHour + alarmHour - 1;
      if (diffHour < 0) diffHour = 0;
    } else {
      if (alarmMin ==0){
        diffMin = 60 - nowMin;
        if (diffMin == 60) diffMin = 0;
      } else {
        diffMin = alarmMin - nowMin;
      }
      diffHour = 24 - nowHour + alarmHour;
    }
  } else {
    //next alarm is ahead of us in this day
    //first, subract the minutes....
    if (alarmMin == nowMin) {
      diffMin = 0;
      diffHour = alarmHour - nowHour;
    } else if (alarmMin < nowMin){
      //then we have to borrow 60 minutes from the hour
      diffMin = 60 + alarmMin - nowMin - 1;
      diffHour = alarmHour - nowHour - 1;
      if (diffHour < 0) diffHour = 0;
    } else {
      //math is easy
      if (alarmMin == 0) {
        diffMin = 60 - nowMin - 1;
        if (diffMin == 60) diffMin = 0;
      } else {
        diffMin = alarmMin - nowMin - 1;
      }
      diffHour = alarmHour - nowHour;
    }
  }
  return diffHour*3600000 + diffMin * 60000 + (60 - nowSec) * 1000;
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// LOOP  ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  P1.petWD();       //when called petWD will reset Watchdog timer to 0.
  checkConsole();
  checkRunSwitch();
  checkDisabledFloodStations();
  stateMachine.Update();
  

  // update the clock every minute
  unsigned long loopCount = updateLcdTimer.getLoopCount();
  static bool alreadyUpdated = false;
  if ((loopCount % 2) == 0 && !alreadyUpdated) { //every other 30 second loop count is 1 minute :)
    printTimeDate(lcd);
    alreadyUpdated = true;
  }
  if (loopCount % 2 == 1) alreadyUpdated = false;

#ifdef DEBUG_STATES
  delay(2000);
#endif
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// VALVE MANIPULATION //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void controlFloodStationDrainValves(uint8_t state) {
  for (int i = 0; i < numStations; i++) {
    if (enabledStations[i]) 
      P1.writeDiscrete(state, StationDrainValves[i]);
  }
}
void controlFloodStationOverflowValves(uint8_t state) {
  for (int i = 0; i < numStations; i++) {
    if (enabledStations[i]) 
      P1.writeDiscrete(state, StationOverflowValves[i]);
  }
}
void controlLevelDrainValves(uint8_t state) {
  for (int i = 0; i < numLevels; i++) {
    P1.writeDiscrete(state, LevelDrainValves[i]);
  }
}
void controlVentValve(uint8_t state) {
    P1.writeDiscrete(state, VV_1);
}
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// PUMP MANIPULATION ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void controlPump(channelLabel pump, uint8_t state) {
#ifdef DEBUG_STATES
  Serial.print("Prev Pump State: ");
  Serial.print(prevPumpState);
  Serial.print(" : Current Requested State: ");
  Serial.println(state);
#endif
  if (state != prevPumpState) { 
    P1.writeDiscrete(state, pump);
    prevPumpState = state;
    printLcdPumpState(lcd);   
    turnOnLcd();
  }
}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////// STATE MACHINE ///////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

/**********************************************************************
   IDLE
 **********************************************************************/
unsigned long idleResumeTimeLeft = 0; //use the setpoint for idle if less than 0
unsigned long floodResumeTimeLeft = 0; // to resume flood after a pause

void idle_onEnter() {
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif

  // IDLE, we want to
  // make sure level drains are open 
  // make sure the vent valve is open
  // make sure overflow and station drains are open 
  // pump is off (do this in entering)
  // Set the RTC Alarm for the next flood
  // use the lcd to indicate what is going on
  // turn off the lcd after a timeout (do this in state)

  //once we go into idle, we need to reset flood timer so it doesn't resume a short flood
  floodResumeTimeLeft = 0;
  
  //PUMP OFF
  controlPump(pump, LOW);
  //OPEN the Level Drain Valves
  controlLevelDrainValves(HIGH);
  //Open the Vent Valve
  controlVentValve(HIGH);
  //Open the Drain Vavles
  controlFloodStationDrainValves(HIGH);
  //Open the Overflow Valves
  controlFloodStationOverflowValves(HIGH);

  //printState(lcd, stateName[stateMachine.GetState()], idleTimer.getRemainingTime());
  turnOnLcd();
  printState(lcd, stateName[stateMachine.GetState()], 0);

}

void idle_onState() {
#ifdef DEBUG_STATES
  Serial.print("On State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif

  // indicate to user how much time is left
  unsigned long loopCount = updateLcdTimer.getLoopCount();
  static bool alreadyUpdated = false;
  unsigned long remainingTime = millisTillNextFlood();//idleTimer.getRemainingTime();
#ifdef DEBUG_STATES
  Serial.print("Remaining Time = ");
  Serial.println(remainingTime);
#endif
  // if it is time to update (loopcount), or we changed something (updatenow) or we are in seconds left:
  if ((((loopCount % 2) == 0) && !alreadyUpdated) || updateStateNow || (remainingTime < 60000 && secondTimer.hasTimedOut())) {  //every other 30 second loop count is 1 minute :)
    printState(lcd, stateName[stateMachine.GetState()], remainingTime);//idleTimer.getRemainingTime());
    alreadyUpdated = true;
  }
  if (loopCount % 2 == 1 ) alreadyUpdated = false;
  if (remainingTime < 60000 && alreadyDimmed) {
    lcd.backlight();
  }

#ifndef DEBUG_STATES
  if (backlightTimer.hasTimedOut() && !alreadyDimmed) {
    //lcd.noBacklight();
    alreadyDimmed = true;
  }
#endif

}

void idle_onExit() {
  //store the current time_left so we can "resume" where we left off on idle
#ifdef DEBUG_STATES
  Serial.print("Exiting State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
//TODO SET RTC ALARM FOR THE NEXT FLOOD CYCLE
//  if (!idleTimer.hasTimedOut()) {
//    idleResumeTimeLeft = idleTimer.getRemainingTime();
//  } else {
//    idleResumeTimeLeft = 0;
//  }
}
bool transition_5() {
  // idle to flood trigger
  //TODO BASE THIS ON A TIME, NOT A TIMER
#ifdef DEBUG_STATES
  Serial.print("transition_5: ");
  Serial.print(triggerFlood);
  Serial.println(";");
#endif
  if (triggerFlood) {
    triggerFlood = false;
    return true;
  } else {
    return false;
  }
}

/**********************************************************************
   FLOOD
 **********************************************************************/

bool transition_6() {
  // flood to idle trigger

#ifdef DEBUG_STATES
  Serial.print("transition_6: ");
  Serial.print(floodTimer.hasTimedOut());
  Serial.print(" : ");
  Serial.println(floodTimer.getRemainingTime());
#endif
  return floodTimer.hasTimedOut();
}

void flood_onEnter() {
  // FLOOD, we want to
  // make sure level drains are CLOSED (do this entering)
  // make sure overflow and station drains are open (do this in state)
  // pump is ON (do this in STATE after all valves opened)
  // reset or restart the timer
  // use the lcd to indicate what is going on

  //when we go into flood, we need to reset the idle resume timer so it doesn't return to a short idle
  idleResumeTimeLeft = 0;
  
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif

  //close the level drain valves
  controlLevelDrainValves(LOW);
  controlFloodStationDrainValves(HIGH);
  controlFloodStationOverflowValves(HIGH);
  controlVentValve(HIGH);
  valveTimer.reset(); //start the 5 second timer to allow the valves to open
  
  turnOnLcd();
  if (floodResumeTimeLeft == 0) {
    floodTimer.setTimeOutTime(floodTime);
  } else {
    floodTimer.setTimeOutTime(floodResumeTimeLeft);
  }
  floodTimer.reset(); //restart the idle timer
  printState(lcd, stateName[stateMachine.GetState()], floodTimer.getRemainingTime());
}

void flood_onState() {
#ifdef DEBUG_STATES
  Serial.print("On State: ");
  Serial.println(stateName[stateMachine.GetState()]);
  Serial.print("Valve Timer Remaining : ");
  Serial.println(valveTimer.getRemainingTime());
#endif


  if (valveTimer.hasTimedOut()) {
    //PUMP ON
    controlPump(pump, HIGH);
  }

  // indicate to user how much time is left
  static bool alreadyUpdated = false;
  unsigned long loopCount = updateLcdTimer.getLoopCount();
  unsigned long remainingTime = floodTimer.getRemainingTime();
  if (((loopCount % 2) == 0 && !alreadyUpdated) || updateStateNow || (remainingTime < 60000 && secondTimer.hasTimedOut())) { //every other 30 second loop count is 1 minute :)
    printState(lcd, stateName[stateMachine.GetState()], floodTimer.getRemainingTime());
    alreadyUpdated = true;
  }
  if (loopCount % 2 == 1 ) alreadyUpdated = false;
}

void flood_onExit() {
#ifdef DEBUG_STATES
  Serial.print("Exiting State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  //PUMP Off
  controlPump(pump, LOW);

  //store the current time_left so we can "resume" where we left off on idle
  if (!floodTimer.hasTimedOut()) {
    floodResumeTimeLeft = floodTimer.getRemainingTime();
  } else {
    floodResumeTimeLeft = 0;
  }

  //select the next flood time
   //set up the next flood alarm
  int nextFloodTime = getNextFloodTime();
  Serial.print("next flood time: ");
  Serial.println(nextFloodTime);
  if ( !setNextFloodAlarm(nextFloodTime) ) {
    Serial.println("BAD FLOOD TIME");
    Serial.println(getNextFloodTime());
    hd44780::fatalError(-1); // does not return
  } 
}

/**********************************************************************
   IDLE PAUSE TEMP
 **********************************************************************/

void ipt_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  printState(lcd, stateName[stateMachine.GetState()], 0);
}

/**********************************************************************
   IDLE PAUSE
 **********************************************************************/

void ip_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  printState(lcd, stateName[stateMachine.GetState()], 0);
  backlightTimer.reset();
}
void ip_onState() {
  if (backlightTimer.hasTimedOut() && !alreadyDimmed) {
    lcd.noBacklight();
    alreadyDimmed = true;
  }
}

/**********************************************************************
   IDLE PAUSE PUMP OVERRIDE
 **********************************************************************/

void ipovr_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, HIGH);
  printState(lcd, stateName[stateMachine.GetState()], 0);
  lcd.setCursor(0, 3);
  lcd.write("PUMP OVR");
}
void ipovr_onExit() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Exiting State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
}

/**********************************************************************
   IDLE PAUSE PUMP OFF
 **********************************************************************/

void ipoff_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  printState(lcd, stateName[stateMachine.GetState()], 0);
}

/**********************************************************************
   FLOOD PAUSE TEMP
 **********************************************************************/

void fpt_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  printState(lcd, stateName[stateMachine.GetState()], 0);
}

/**********************************************************************
   FLOOD PAUSE
 **********************************************************************/

void fp_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  printState(lcd, stateName[stateMachine.GetState()], 0);
  backlightTimer.reset();
}
void fp_onState() {
  if (backlightTimer.hasTimedOut() && !alreadyDimmed) {
    lcd.noBacklight();
    alreadyDimmed = true;
  }
}

/**********************************************************************
   FLOOD PAUSE PUMP OVERRIDE
 **********************************************************************/

void fpovr_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, HIGH);
  printState(lcd, stateName[stateMachine.GetState()], 0);
}
void fpovr_onExit() {
#ifdef DEBUG_STATES
  Serial.print("Exiting State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
}

/**********************************************************************
   FLOOD PAUSE OFF
 **********************************************************************/

void fpoff_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  printState(lcd, stateName[stateMachine.GetState()], 0);
}


/**********************************************************************
   CLEAN 
 **********************************************************************/

void clean_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  controlLevelDrainValves(HIGH);
  controlFloodStationDrainValves(LOW);
  controlFloodStationOverflowValves(LOW);
  controlVentValve(LOW);

  printState(lcd, stateName[stateMachine.GetState()], 0);
  backlightTimer.reset();
}

/**********************************************************************
   CLEAN PUMP OVERRIDE
 **********************************************************************/

void cpovr_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, HIGH);
  printState(lcd, stateName[stateMachine.GetState()], 0);
  lcd.setCursor(0, 3);
  lcd.write("PUMP OVR");
}
void cpovr_onExit() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Exiting State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
}

/**********************************************************************
   CLEAN  PUMP OFF
 **********************************************************************/

void cpoff_onEnter() {
  turnOnLcd();
#ifdef DEBUG_STATES
  Serial.print("Entering State: ");
  Serial.println(stateName[stateMachine.GetState()]);
#endif
  controlPump(pump, LOW);
  printState(lcd, stateName[stateMachine.GetState()], 0);
}

bool allStationsDisabled() {
  //true, the station is enabled
  //false, the station is disabled
  //INPUT, LOW the station is enabled.
  for (int i=0; i<numStations; i++) {
    if ( P1.readDiscrete(StationDisabledSwitches[i]) == LOW ) {
      return false; //at least 1 station is enabled, LOW = enabled.
    }
  }
  return true; // no stations were enabled, therefore all stations are disabled
}
bool notAllStationsDisabled() {
  return !allStationsDisabled();
}



/**********************************************************************
   STATE MACHINE DEFINITION
 **********************************************************************/

// Setup the State Machine
void setupStateMachine() {

  //const char* const stateName[] PROGMEM = { "Idle", "Flood", "Idle Pause Temp", "Idle Pause", "Pause Pump Override", "Pause Pump Off",
  //                                          "Flood Pause Temp", "Flood Pause", "Pause Pump Override", "Pause Pump Off" };

  // Follow the order of defined enumeration for the state definition (will be used as index)
  // Add States => name,timeout, onEnter cb, onState cb, onLeave cb
  stateMachine.AddState(stateName[Idle],             0, idle_onEnter,  idle_onState,  idle_onExit  );
  stateMachine.AddState(stateName[Flood],            0, flood_onEnter, flood_onState, flood_onExit );
  stateMachine.AddState(stateName[IdlePauseTemp], 2000, ipt_onEnter,   nullptr,       nullptr      );
  stateMachine.AddState(stateName[IdlePause],        0, ip_onEnter,    ip_onState,    nullptr      );
  stateMachine.AddState(stateName[IdlePumpOvr],      0, ipovr_onEnter, nullptr,       ipovr_onExit );
  stateMachine.AddState(stateName[IdlePumpOff],      0, ipoff_onEnter, nullptr,       nullptr      );
  stateMachine.AddState(stateName[FldPauseTemp],  2000, fpt_onEnter,   nullptr,       nullptr      );
  stateMachine.AddState(stateName[FldPause],         0, fp_onEnter,    nullptr,       nullptr      );
  stateMachine.AddState(stateName[FldPumpOvr],       0, fpovr_onEnter, nullptr,       ipovr_onExit );
  stateMachine.AddState(stateName[FldPumpOff],       0, fpoff_onEnter, nullptr,       nullptr      );

  stateMachine.AddState(stateName[Clean],            0, clean_onEnter, nullptr,       nullptr      );
  stateMachine.AddState(stateName[ClnPumpOvr],       0, cpovr_onEnter, nullptr,       cpovr_onExit );
  stateMachine.AddState(stateName[ClnPumpOff],       0, cpoff_onEnter, nullptr,       nullptr      );

  // Add transitions with related trigger input callback functions
  stateMachine.AddTransition(Idle,          Flood,         transition_5                            );
  stateMachine.AddTransition(Flood,         Idle,          transition_6                            );
  stateMachine.AddTransition(Idle,          IdlePauseTemp, notCheckRunSwitch                       );
  stateMachine.AddTransition(IdlePauseTemp, Flood,         checkRunSwitch                          );
  stateMachine.AddTransition(IdlePauseTemp, IdlePause, []() { return stateMachine.CurrentState()->timeout; } );
  stateMachine.AddTransition(IdlePause,     Idle,          checkRunSwitch                          );
  stateMachine.AddTransition(IdlePause,     IdlePumpOvr,   checkPumpSwitch                         );
  stateMachine.AddTransition(IdlePumpOvr,   IdlePumpOff,   notCheckPumpSwitch                      );
  stateMachine.AddTransition(IdlePumpOff,   IdlePause, []() { return true; }                       );

  stateMachine.AddTransition(IdlePause,     Clean,         allStationsDisabled                     );
  stateMachine.AddTransition(Clean,         IdlePause,     notAllStationsDisabled                  );
  stateMachine.AddTransition(Clean,         ClnPumpOvr,    checkPumpSwitch                         );
  stateMachine.AddTransition(ClnPumpOvr,    ClnPumpOff,    notCheckPumpSwitch                      );
  stateMachine.AddTransition(ClnPumpOff,    Clean,     []() { return true; }                       );


  stateMachine.AddTransition(Flood,         FldPauseTemp, notCheckRunSwitch                        );
  stateMachine.AddTransition(FldPauseTemp,  Idle,         checkRunSwitch                           );
  stateMachine.AddTransition(FldPauseTemp,  FldPause, []() { return stateMachine.CurrentState()->timeout; }  );
  stateMachine.AddTransition(FldPause,      FldPumpOvr,   checkPumpSwitch                          );
  stateMachine.AddTransition(FldPause,      Flood,        checkRunSwitch                           );
  stateMachine.AddTransition(FldPumpOvr,    FldPumpOff,   notCheckPumpSwitch                       );
  stateMachine.AddTransition(FldPumpOff,    FldPause, []() { return true; }                        );

}
