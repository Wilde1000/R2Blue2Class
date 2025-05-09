/*************************************************************************
 * ************************ BODY LIGHTS MEGA *****************************
 * ***********************************************************************/

/* This is the Lighting control program for the body of Blue Crew's mascot R2-Blue2.  This program is based on the original 
 *  Dataport/CBI program by Michael Erwin with additions from CuriousMarc, VAShadow, and S. Sloan. 
 * 
 * R2-Blue2  uses Michael Erwin's CBI and Dataport boards which utilize the MAX7219 chip to control LEDs.  In addition, R2 has three
 * team created light features using NeoPixels, the Large Dataport, the Coin Slots, and the Lightbar (on the DPL). These will be accessed
 * by using the FastLED.h and pixeltypes.h, its helper library
 * 
 *
*
We will be using a Jawa-Lite inspired technique to control all the MPU's in 
R2-Blue2.
The command structure is as follows:
    Element 0 - MPU Code - Single Character indicating the Microprocessor Unit
    Element 1, 2 - two digit Integer representing the device attached to the MPU
    Element 3 - one character command code
    Element 4 - the command option
    MPU Codes:
      
      A - Body Master Mega
      B - Lights Mega
      C - Drive Uno
      D - RFID Uno
      E - Lifts Mega
      F - Periscope Nano
      G - Teeces Micro
      H - CBI Nano
      I - Exp. Nano
    Device Codes:
      0-9 - Teeces
      10-19 - Body Master
      20-29 - Lights Mega
      30-39 - Drive Mega
      40-49 - RFID Nano
      50-59 - Lift Mega
      60-69 - Periscope Nano
      70-79 - CBI Nano
      80-99 - Exp. Nano
  A valid command would look like this:
    E51T102
  And would consist of the following:
  MPU code - E
  Device Address - 51
  Command - T
  Option - 102
*/



/*************************************************************************
 * ********************** INCLUDED LIBRARIES *****************************
 * ***********************************************************************/
#include <LedControl.h>         //Needed for MAX7219 driven leds
#include <FastLED.h>            // Needed for Neopixels
#include <pixeltypes.h>         //Helper Library for FastLED
#include <Adafruit_NeoPixel.h>  //Needed for neopixels
#include <Servo.h>              //Servo library for Data Panel

/*************************************************************************
 * ********************** MACRO DEFINITIONS ******************************
 * ***********************************************************************/
#define MPU 'B'       //  Body Lights Mega is MPU B
#define CBIDoorPin 3  //  Used for the CBI door open/close - Switch should be wired for Normally Closed
//#define DPLDoorPin 3          //  Used for DPL door open/close - Switch should be wired for Normally close
#define DPL_LOAD 4             //  DPL load pin for the MAX7219 chip.
#define DPL_CLOCK 5            //  DPL clock pin for the MAX7219 chip
#define DPL_DATA 6             //  DPL Data pin for the MAX7219 chip
#define CBI_LOAD 7             //  CBI load pin for the MAX7219 chip.
#define CBI_CLOCK 8            //  CBI clock pin for the MAX7219 chip
#define CBI_DATA 9             // CBI data pin for the MAX7219 chip
#define CS_PIN 11              // Coin slots FastLED signal pin
#define LDPL_PIN 10            // Large Data port FastLED signal pin
#define LB_PIN 12              // sets the data pin for the LightBar Fast LED
#define LIGHTBARSPEED 125      // speed for DPL Neopixel Light bar
#define TOPBLOCKSPEED 70       // speed for top left blocks on DPL
#define BOTTOMLEDSPEED 175     // speed for bottom white leds on DPL
#define REDLEDSPEED 250        // speed for two red leds on DPL
#define BLUELEDSPEED 200       // speed for blue leds on DPL
#define BARGRAPHSPEED 100      // speed for Right blocks on DPL
#define CBISPEED_RANDOM 50     // 50 for Curious Marc version
#define TESTDELAY 30           // Test routine for DPL and CBI delay
#define MAXGRAPH 2             // Used for Bar Graph display
#define LB_LEDS 16             // Num of leds in DPL Light Bar
#define LDPL_LEDS 15           // Number of leds in Large Data Port
#define CS_LEDS 18             // Number of leds in Coin Slots
#define DATAPORT 0             // device 1 is second in chain
#define CBI 0                  // device 0 is first in chain
#define NUMDEV 1               // One for the dataport, one for the battery indicator/CBI
#define DATAPORTINTENSITY 15   // 15 is max
#define CBIINTENSITY 8        // 15 is max
#define MAX_COMMAND_LENGTH 64  // Max size for a serial command
#define DP_DOOR 45             //Set the pin for the Dataport Door
#define DP_DOOR_MAX 1600       //Set the door open position
#define DP_DOOR_MIN 900        //Set the door close position

//Set this to which Analog Pin you use for the voltage in.
#define analoginput A0  //
#define greenVCC 13.0   // Green LED on if above this voltage
#define yellowVCC 12.0  // Yellow LED on if above this voltage
#define redVCC 11.5     // Red LED on if above this voltage

// For 12volts: R1=47k, R2=33k
// For 15volts: R1=47k, R2=24k
// For 30volts: R1=47k, R2=9.4k

#define R1 47000.0  // >> resistance of R1 in ohms << the more accurate these values are
#define R2 33000.0  // >> resistance of R2 in ohms << the more accurate the measurement will be

// uncomment this to test the LEDS one after the other at startup
//#define TEST

// If you are using the voltage monitor uncomment this
#define monitorVCC

// Uncomment this if you want an alternate effect for the blue LEDs, where it moves
// in sync with the bar graph
#define BLUELEDTRACKGRAPH

//========================End Macro Definitions ====================================================

//
/*************************************************************************
 * ********************** GLOBAL VARIABLES *******************************
 * ***********************************************************************/
Adafruit_NeoPixel cslotsLGHT(CS_LEDS, CS_PIN, NEO_GRB + NEO_KHZ800);    //Coin Slots
Adafruit_NeoPixel ldplLGHT(LDPL_LEDS, LDPL_PIN, NEO_GRB + NEO_KHZ800);  //LDPL

CRGB lb[LB_LEDS];      // Create a CRGB object for Data Panel Light Bar
CRGB ldpl[LDPL_LEDS];  // Create a CRGB object for Large Data Panel Logics
CRGB cs[CS_LEDS];      // Create a CRGB object for the Coin Slots
float vout = 0.0;      // for voltage out measured analog input
int value = 0;         // used to hold the analog value coming out of the voltage divider
float vin = 0.0;       // voltage calulcated... since the divider allows for 15 volts
char cmdStr0[MAX_COMMAND_LENGTH];
char cmdStr1[MAX_COMMAND_LENGTH];
char cmdStr2[MAX_COMMAND_LENGTH];
char cmdStr3[MAX_COMMAND_LENGTH];
// Currently, our droid uses two unconnected MAX7219 driven light systems - the DPL and the CBI
// Instantiate LedControl driver
LedControl cc = LedControl(CBI_DATA, CBI_CLOCK, CBI_LOAD, NUMDEV);  // CBI
LedControl dc = LedControl(DPL_DATA, DPL_CLOCK, DPL_LOAD, NUMDEV);  // Dataport
int displayEffect = 100;                                            // 100=no change, 4=whistle/heart sequence
int dev_addr, dev_opt;                                              // Create variables for device address and device option
char dev_cmd, dev_MPU;                                              // Create variable for the device command
int cs_State = 3;                                                   // Sets default Coin Slot State to off
int cs_Speed = 425;                                                 // Sets default Coin Slot speed to Medium
int cs_Tspeed = 10;                                                 // Sets default Coin Slot throb speed (0-99)
int cs_color = 5;                                                   // Sets default Coin Slot color to Blue
int ldpl_color = 5;                                                 // Sets default Large Data Port Logics color to Blue
int ldpl_State = 3;                                                 // Sets default Large Data Port Logics State to two Across
int ldpl_Speed = 200;                                               // Sets default Large Data Port Logics speed to Medium
int ldpl_Tspeed = 10;                                               // Sets default Large Data Port Logics throb speed (0-99)
int dpl_State = 0;                                                  // Set default Data Port Logics to off

//  Color Sequences stored in Program Memory
const uint16_t np_color[][3] PROGMEM = {
  { 0, 0, 0 },       //Off - 0
  { 255, 0, 0 },     //Red - 1
  { 255, 0, 128 },   //Rose - 2
  { 255, 0, 255 },   //Magenta - 3
  { 128, 0, 255 },   //Violet - 4
  { 0, 0, 255 },     //Blue - 5
  { 0, 128, 255 },   //Azure - 6
  { 0, 255, 255 },   //Cyan - 7
  { 0, 255, 128 },   //Spring Green - 8
  { 0, 255, 0 },     //Green - 9
  { 128, 255, 0 },   //Chartreuse - 10
  { 255, 255, 0 },   //Yellow - 11
  { 255, 128, 0 },   //Orange - 12
  { 255, 255, 255 }  //White - 13
};


Servo dp_door;  //Create servo object for Dataport Door
//========================End Global Variables ====================================================

/*************************************************************************
 * ********************** FUNCTION DECLARATIONS **************************
 * ***********************************************************************


//Updates the blocks on the right of the DPL
void bargraphDisplay(byte disp);
int buildCommand(char ch, char* output_str);   //builds command from serial input
int cal_Speed(int num);                        //returns a calculated speed based on passed num
void cbi();                                    //Handles the CBI and Voltage lights           
void checkSerial();                            //Checks for serial commands
int coinslot(int num);                         //Controls the neo-pixel based coinslot
void cs_singleUpDown(int num);                 //Displays single Up/Down routine on coin slots
void cs_updown(int num);                       //Displays up/down routine on Coin slots
int doTcommand(int addr, int option);          //Handles all T commands
int doScommand(int addr, int option);          //Handles all S commands
void dpl(int option);                          //Handles dataport interactions
void fillBar(byte disp, byte data, byte value, byte maxcol);  //Utility to light up a bar of leds based on a value
void getVCC();                                 //Updates the current voltage
void ldpl_double(int num);                     // Display double dot sequence on LDPL
void ldpl_single(int num);                     //Displays single dot sequence on LDPL
int parseCommand(char* input_str);             //Parses command into mpu, address, command, and option  
byte randomRow(byte randomMode);               //Utility to generate random LED patterns
void singleTest();                             //Tests every led in the CBI and DPL
int std_color(int num);                        //Returns a standard color based on passed num
byte updatebar(byte disp, byte* bargraphdata, byte maxcol);   //Utility to make bargraph look more realistic
void updateBlueLEDs();                         //Updates the blue LEDs on the DPL
void updatebottomLEDs();                       //Updates the bottom White leds in the DPL
void updateCBILEDs();                          //Updates the CBI leds
int updateLDPL(int num);                       //Controls the neo-pixel based LDPL
void updateLightBar();                         //Updates the DPL Light Bar
void updateRedLEDs();                          //Updates the two red LEDs on the DPL
void updateTopBlocks();                        //Updates the Top Green and Yellow Blocks on the DPL

//========================End Function Declarations ====================================================
*/

/*************************************************************************
 * *****************************  FUNCTIONS ******************************
 * ***********************************************************************/




// bargraph for the right column
// disp 0: Row 2 Col 5 to 0 (left bar) - 6 to 0 if including lower red LED,
// disp 1: Row 3 Col 5 to 0 (right bar)
void bargraphDisplay(byte disp) {
  static byte bargraphdata[MAXGRAPH];  // status of bars

  if (disp >= MAXGRAPH) return;

  // speed control
  static unsigned long previousDisplayUpdate[MAXGRAPH] = { 0, 0 };

  unsigned long currentMillis = millis();
  if (currentMillis - previousDisplayUpdate[disp] < BARGRAPHSPEED) return;
  previousDisplayUpdate[disp] = currentMillis;

  // adjust to max numbers of LED available per bargraph
  byte maxcol;
  if (disp == 0 || disp == 1) maxcol = 6;
  else maxcol = 3;  // for smaller graph bars, not defined yet

  // use utility to update the value of the bargraph  from it's previous value
  byte value = updatebar(disp, &bargraphdata[disp], maxcol);
  byte data = 0;
  // transform value into byte representing of illuminated LEDs
  // start at 1 so it can go all the way to no illuminated LED
  for (int i = 1; i <= value; i++) {
    data |= 0x01 << i - 1;
  }
  // transfer the byte column wise to the video grid
  fillBar(disp, data, value, maxcol);
  return;
}


//The buildCommand takes the current byte from the Serial1 buffer and builds a command for processing.  It returns a 0
//while in the building process and a 1 when the command is ready.
int buildCommand(char ch, char* output_str) {
  static int pos = 0;
  switch (ch) {
    case '\n':
    case '\r':
    case '\0':
      output_str[pos] = 13;
      pos = 0;
      return true;
      break;
    default:
      output_str[pos] = ch;
      if (pos <= MAX_COMMAND_LENGTH - 1) pos++;
      break;
  }
  return false;
}




//  cal_Speed returns the time in milliseconds for routine delays from the passed option
int cal_Speed(int num) {
  switch (num) {
    case 1: return 100;  // Fastest
    case 2: return 250;
    case 3: return 500;
    case 4: return 750;
    case 5: return 1000;  // Slowest
    default:
      if (num > 100 && num < 200) return num - 100;
  }
}




// The cbi function runs the Charging Bay Indicator Lights
void cbi() {
  updateCBILEDs();
  //getVCC();
}


//The checkSerial function is the first thread of seven threads in this program.  It checks the Serial0 buffer for incoming serial
//data and then sends it to be processed.
byte checkSerial() {
  char ch;
  byte cmd_Complete;
  if (Serial.available()) {
    ch = Serial.read();
    //Serial.print(ch);
    cmd_Complete = buildCommand(ch, cmdStr0);
    if (cmd_Complete) {
      return 1;
      //Serial.println();
    }
  }
  return 0;
}

//The checkSerial function is the first thread of seven threads in this program.  It checks the Serial0 buffer for incoming serial
//data and then sends it to be processed.
byte checkSerial1() {
  char ch;
  byte cmd_Complete;
  if (Serial1.available()) {
    ch = Serial1.read();
    //Serial.print(ch);
    cmd_Complete = buildCommand(ch, cmdStr1);
    if (cmd_Complete) {
      return 1;
    }
  }
  return 0;
}

//The checkSerial function is the first thread of seven threads in this program.  It checks the Serial0 buffer for incoming serial
//data and then sends it to be processed.
void checkSerial2() {
  char ch;
  byte cmd_Complete;
  if (Serial2.available()) {
    ch = Serial2.read();
    //Serial.print(ch);
    cmd_Complete = buildCommand(ch, cmdStr2);
    if (cmd_Complete) {
      return 1;
    }
  }
  return 0;
}

//The checkSerial function is the first thread of seven threads in this program.  It checks the Serial0 buffer for incoming serial
//data and then sends it to be processed.
byte checkSerial3() {
  char ch;
  byte cmd_Complete;
  if (Serial3.available()) {
    ch = Serial3.read();
    //Serial.print(ch);
    cmd_Complete = buildCommand(ch, cmdStr3);
    if (cmd_Complete) {
      parseCommand(cmdStr3);
      //Serial.println();
    }
  }
}




//Handles all interactions with the coin slot neopixels
int coinslot(int num) {
  int red = pgm_read_word(&(np_color[cs_color][0]));
  int green = pgm_read_word(&(np_color[cs_color][1]));
  int blue = pgm_read_word(&(np_color[cs_color][2]));
  //0 - Shut off coin slots
  if (num == 0) {
    for (int x = 0; x < CS_LEDS; x++) cslotsLGHT.setPixelColor(x, cslotsLGHT.Color(0, 0, 0));
    cslotsLGHT.show();
    return 0;
  }
  //1 - Runs Solid color
  if (num == 1) {
    for (int x = 0; x < CS_LEDS; x++) cslotsLGHT.setPixelColor(x, cslotsLGHT.Color(red, green, blue));
    cslotsLGHT.show();
    return 0;
  }

  static unsigned long timeLast = 0;
  unsigned long elapsed = millis();
  if ((elapsed - timeLast) < cs_Speed) return num;
  timeLast = elapsed;
  //timed routines
  //2 - cs_updown
  if (num == 2) {
    cs_updown();
    return 0;
  }
  //3 - singleUpDown
  if (num == 3) {
    cs_singleUpDown();
    return 0;
  }
  cslotsLGHT.show();
  return 0;
}


//Animates a single coin slot moving up and down
void cs_singleUpDown() {
  static int turn = 0;  //tracks which coinslot is being addressed
  static int dir = 0;   // 0 is up 1 is down
  int red = pgm_read_word(&(np_color[cs_color][0]));
  int green = pgm_read_word(&(np_color[cs_color][1]));
  int blue = pgm_read_word(&(np_color[cs_color][2]));
  if (turn <= 0 && dir == 1) {
    dir = 0;
    turn = 0;
  }
  if (turn > 16 && dir == 0) {
    dir = 1;
    turn = 15;
  }
  //First set all pixels to off
  for (int x = 0; x < CS_LEDS; x++) cslotsLGHT.setPixelColor(x, cslotsLGHT.Color(0, 0, 0));
  cslotsLGHT.setPixelColor(turn, cslotsLGHT.Color(red, green, blue));
  cslotsLGHT.setPixelColor(turn + 1, cslotsLGHT.Color(red, green, blue));
  cslotsLGHT.setPixelColor(turn + 2, cslotsLGHT.Color(red, green, blue));
  if (dir == 0) {
    turn += 3;
  } else {
    turn -= 3;
  }
  cslotsLGHT.show();
  return;
}

//  cs_updown displays the passed color one coin slot at a time starting with the top coin slot.
//  Once all are filled, they are cleared from the bottom up.
void cs_updown() {
  static int turn = 0;  //tracks which coinslot is being addressed
  static int dir = 0;   // 0 is up 1 is down
  int red = pgm_read_word(&(np_color[cs_color][0]));
  int green = pgm_read_word(&(np_color[cs_color][1]));
  int blue = pgm_read_word(&(np_color[cs_color][2]));
  if (turn <= 0 && dir == 1) {
    dir = 0;
    turn = 0;
  }
  if (turn > 16 && dir == 0) {
    dir = 1;
    turn = 15;
  }
  if (dir == 0) {
    cslotsLGHT.setPixelColor(turn, cslotsLGHT.Color(red, green, blue));
    cslotsLGHT.setPixelColor(turn + 1, cslotsLGHT.Color(red, green, blue));
    cslotsLGHT.setPixelColor(turn + 2, cslotsLGHT.Color(red, green, blue));
  } else {
    cslotsLGHT.setPixelColor(turn, cslotsLGHT.Color(0, 0, 0));
    cslotsLGHT.setPixelColor(turn + 1, cslotsLGHT.Color(0, 0, 0));
    cslotsLGHT.setPixelColor(turn + 2, cslotsLGHT.Color(0, 0, 0));
  }

  if (dir == 0) {
    turn += 3;
  } else {
    turn -= 3;
  }
  cslotsLGHT.show();
  return;
}



//The doTcommand handles all T commands sent from the parseCommand function
int doTcommand(int addr, int option) {
  //Serial.println("T command");
  switch (addr) {
    case 21:              // Address of Coin Slots is 21
      cs_State = option;  // Set the Coin Slot state to the option
      break;
    case 22:                // Address of Dataport is 22
      ldpl_State = option;  // Set the Dataport state to the option
      break;
    case 23:  // Address of Charging port is 23
      //Code for Charging Port
      break;
    case 24:  //Address of the Dataport is 24
      dpl_State = option;
      break;
  }
}

//The doScommand handles all T commands sent from the parseCommand function
int doScommand(int addr, int option) {
  //Serial.println("S command");
  switch (addr) {
    case 21:  // Address of Coin Slots is 21
      if (option > 0 && option <= 13) {
        cs_color = option;
        break;
      }
      if (option == 20) {
        if (cs_color < 13) cs_color++;
        else cs_color = 1;
        break;
      }
      if (option == 30) {
        if (cs_State < 3) cs_State++;
        else cs_State = 1;
        break;
      }
    case 22:  // Address of Dataport is 22
      if (option > 0 && option <= 13) {
        ldpl_color = option;
        break;
      }
      if (option == 20) {
        if (ldpl_color < 13) ldpl_color++;
        else ldpl_color = 1;
        break;
      }
      if (option == 30) {
        if (ldpl_State < 3) ldpl_State++;
        else ldpl_State = 1;
        break;
      }

      break;
    case 23:  // Address of Charging port is 33
      //cp_Speed=option;                           // set the Charging port to the option
      break;
  }
}



//The dpl function handles all actions concerning the Dataport. If the passed option is 0 - no change to the dataport
//if the option is 1 - Door is opened and lights are on, if 2 door is closed and lights go off.
void dpl(int option) {
  static bool door_open = false;
  static long unsigned lastTime = 0;
  int waitTime = 50;
  switch (option) {
    case 0:
      //dp_door.detach();
      return;
    case 1:
      if (!door_open) {
        //dp_door.attach(DP_DOOR);
        if (millis() - lastTime > waitTime) {
          lastTime = millis();
          dp_door.writeMicroseconds(DP_DOOR_MAX);
          door_open = true;
        }
      }
      updateTopBlocks();
      bargraphDisplay(0);
      updatebottomLEDs();
      updateRedLEDs();
      updateLightBar();
      break;
    case 2:
      if (door_open) {
        //dp_door.attach(DP_DOOR);
        if (millis() - lastTime > waitTime) {
          lastTime = millis();
          dp_door.writeMicroseconds(DP_DOOR_MIN);
          door_open = false;
          dc.setRow(DATAPORT, 1, 0);  // top yellow blocks
          dc.setRow(DATAPORT, 2, 0);  // top yellow blocks
          dc.setRow(DATAPORT, 3, 0);  // top yellow blocks
          dc.setRow(DATAPORT, 4, 0);  // top yellow blocks
          dc.setRow(DATAPORT, 5, 0);  // top green blocks
          dc.setRow(DATAPORT, 0, 0);  // blue LEDs
          for (int x = 0; x < 16; x++) lb[x] = CRGB::Black;
          FastLED.show();
          dpl_State = 0;
        }
      }

      break;
  }
}


// helper for lighting up a bar of LEDs based on a value
void fillBar(byte disp, byte data, byte value, byte maxcol) {
  byte row;

  // find the row of the bargraph
  switch (disp) {
    case 0:
      row = 2;
      break;
    case 1:
      row = 3;
      break;
    default:
      return;
      break;
  }

  for (byte col = 0; col < maxcol; col++) {
    // test state of LED
    byte LEDon = (data & 1 << col);
    if (LEDon) {
      //dc.setLed(DATAPORT,row,maxcol-col-1,true);  // set column bit
      dc.setLed(DATAPORT, 2, maxcol - col - 1, true);  // set column bit
      dc.setLed(DATAPORT, 3, maxcol - col - 1, true);  // set column bit
      //dc.setLed(DATAPORT,0,maxcol-col-1,true);      // set blue column bit
    } else {
      //dc.setLed(DATAPORT,row,maxcol-col-1,false); // reset column bit
      dc.setLed(DATAPORT, 2, maxcol - col - 1, false);  // reset column bit
      dc.setLed(DATAPORT, 3, maxcol - col - 1, false);  // reset column bit
      //dc.setLed(DATAPORT,0,maxcol-col-1,false);     // set blue column bit
    }
  }
#ifdef BLUELEDTRACKGRAPH
  // do blue tracking LED
  byte blueLEDrow = B00000010;
  blueLEDrow = blueLEDrow << value;
  dc.setRow(DATAPORT, 0, blueLEDrow);
#endif
  return;
}


// getVCC calculates the voltage based on the voltage coming off the voltage divider
void getVCC() {
  value = analogRead(analoginput);  // this must be between 0.0 and 5.0 - otherwise you'll let the blue smoke out of your arduino
  vout = (value * 5.0) / 1024.0;    //voltage coming out of the voltage divider
  vin = vout / (R2 / (R1 + R2));    //voltage to display
  cc.setLed(CBI, 6, 5, 0);
  cc.setLed(CBI, 5, 5, 0);
  cc.setLed(CBI, 4, 5, 0);
  //cc.setLed(CBI, 6, 5, (vin >= greenVCC));
  //cc.setLed(CBI, 5, 5, (vin >= yellowVCC));
  //cc.setLed(CBI, 4, 5, (vin >= redVCC));
  //Serial.print("Volt Out = ");                                  // DEBUG CODE
  //Serial.print(vout, 1);   //Print float "vin" with 1 decimal   // DEBUG CODE
  //Serial.print("\tVolts Calc = ");                             // DEBUG CODE
  //Serial.println(vin, 1);   //Print float "vin" with 1 decimal   // DEBUG CODE
  return;
}


//Animates two lit neopixels across the LDPL
void ldpl_double() {
  static unsigned long timeLast = 0;
  unsigned long elapsed = millis();
  if ((elapsed - timeLast) < ldpl_Speed) return;
  timeLast = elapsed;
  int red = pgm_read_word(&(np_color[ldpl_color][0]));
  int green = pgm_read_word(&(np_color[ldpl_color][1]));
  int blue = pgm_read_word(&(np_color[ldpl_color][2]));
  static int turn = 0;
  static int dir = 0;
  if (turn <= 0 && dir == 1) {
    dir = 0;
    turn = 0;
  }
  if (turn >= 14 && dir == 0) {
    dir = 1;
    turn = 14;
  }
  for (int x = 0; x < LDPL_LEDS; x++) ldplLGHT.setPixelColor(x, ldplLGHT.Color(0, 0, 0));
  ldplLGHT.setPixelColor(turn, ldplLGHT.Color(red, green, blue));
  ldplLGHT.setPixelColor(turn - 1, ldplLGHT.Color(red, green, blue));
  ldplLGHT.show();
  if (dir == 0) {
    turn++;
  } else {
    turn--;
  }
  return;
}



//ldpl_single animates a single lit pixel across the LDPL
void ldpl_single() {
  static unsigned long timeLast = 0;
  unsigned long elapsed = millis();
  if ((elapsed - timeLast) < ldpl_Speed) return;
  timeLast = elapsed;
  int red = pgm_read_word(&(np_color[ldpl_color][0]));
  int green = pgm_read_word(&(np_color[ldpl_color][1]));
  int blue = pgm_read_word(&(np_color[ldpl_color][2]));
  static int turn = 0;
  static int dir = 0;
  if (turn <= 0 && dir == 1) {
    dir = 0;
    turn = 0;
  }
  if (turn >= 14 && dir == 0) {
    dir = 1;
    turn = 14;
  }
  for (int x = 0; x < LDPL_LEDS; x++) ldplLGHT.setPixelColor(x, ldplLGHT.Color(0, 0, 0));
  ldplLGHT.setPixelColor(turn, ldplLGHT.Color(red, green, blue));
  ldplLGHT.show();
  if (dir == 0) {
    turn++;
  } else {
    turn--;
  }
  return;
}




//The parseCommand takes the command from the buildCommand function and parses into its component parts - MPU, Address, Command and Option
int parseCommand(char* input_str) {
  byte length = strlen(input_str);
  if (length < 2) return 1;  //not enough characters
  int mpu = input_str[0];    //MPU is the first character
  if (MPU != mpu) {          //if command is not for this MPU - send it on its way
    Serial.flush();
    for (int x = 0; x < length; x++) {
      Serial.write(input_str[x]);
    }
    Serial.write(13);
    return 0;
  }
  dev_MPU = mpu;
  // Now the address which should be the next two characters

  char addrStr[3];  //set up a char array to hold them (plus the EOS (end of String) character)
  addrStr[0] = input_str[1];
  addrStr[1] = input_str[2];
  addrStr[2] = '\0';
  dev_addr = atoi(addrStr);
  if (dev_addr < 20 || dev_addr > 29) return 1;
  dev_cmd = input_str[3];
  char optStr[4];
  optStr[0] = input_str[4];
  if (input_str[5] == 13) {
    optStr[1] = '\0';
  } else {
    optStr[1] = input_str[5];
    if (input_str[6] == 13) {
      optStr[2] = '\0';

    } else {
      optStr[3] = input_str[6];
      optStr[4] = '\0';
    }
  }


  dev_opt = atoi(optStr);  // that's the numerical argument after the command character
  // switch on command character
  switch (dev_cmd)  // 2nd or third char, should be the command char
  {
    case 'T':
      doTcommand(dev_addr, dev_opt);
      break;
    case 'S':
      doScommand(dev_addr, dev_opt);
      break;
    default:
      return 1;  // unknown command
      break;
  }
  return 0;
}





// Utility to generate random LED patterns
// Mode goes from 0 to 6. The lower the mode
// the less the LED density that's on.
// Modes 4 and 5 give the most organic feel
byte randomRow(byte randomMode) {
  switch (randomMode) {
    case 0:  // stage -3
      return (random(256) & random(256) & random(256) & random(256));
      break;
    case 1:  // stage -2
      return (random(256) & random(256) & random(256));
      break;
    case 2:  // stage -1
      return (random(256) & random(256));
      break;
    case 3:  // legacy "blocky" mode
      return random(256);
      break;
    case 4:  // stage 1
      return (random(256) | random(256));
      break;
    case 5:  // stage 2
      return (random(256) | random(256) | random(256));
      break;
    case 6:  // stage 3
      return (random(256) | random(256) | random(256) | random(256));
      break;
    default:
      return random(256);
      break;
  }
}


// Test LEDs, each Maxim driver row in turn
// Each LED blinks according to the col number
// Col 0 is just on
// Col 1 blinks twice
// col 2 blinks 3 times, etc...
//
void singleTest() {
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 7; col++) {
      delay(TESTDELAY);
      dc.setLed(DATAPORT, row, col, true);
      delay(TESTDELAY);
      for (int i = 0; i < col; i++) {
        dc.setLed(DATAPORT, row, col, false);
        delay(TESTDELAY);
        dc.setLed(DATAPORT, row, col, true);
        delay(TESTDELAY);
      }
    }
  }
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 5; col++) {
      delay(TESTDELAY);
      cc.setLed(CBI, row, col, true);
      delay(TESTDELAY);
      for (int i = 0; i < col; i++) {
        cc.setLed(CBI, row, col, false);
        delay(TESTDELAY);
        cc.setLed(CBI, row, col, true);
        delay(TESTDELAY);
      }
    }
  }
  cc.setLed(CBI, 4, 5, true);
  delay(TESTDELAY);
  cc.setLed(CBI, 5, 5, true);
  delay(TESTDELAY);
  cc.setLed(CBI, 6, 5, true);
  delay(TESTDELAY);
  return;
}

/*
//returns a Hue value to be used in HSV calculations
int std_color(int num) {
  int curr_Hue;  // a double integer to hold routine color value up to 256 cubed (256 for red, 256 for green, 256 for blue)
  switch (num) {

    case 1:
      curr_Hue = 0;  //Color is Red
      break;
    case 2:
      curr_Hue = 8;  //Color is Orange
      break;
    case 3:
      curr_Hue = 32;  //Color is Yellow
      break;
    case 4:
      curr_Hue = 96;  //Color is Green
      break;
    case 5:
      curr_Hue = 128;  //Color is Cyan
      break;
    case 6:
      curr_Hue = 160;  //Color is Blue
      break;
    case 7:
      curr_Hue = 192;  //Color is Violet
      break;
    case 8:
      curr_Hue = 224;  //Color is Pink
      break;
    case 9:
      curr_Hue = 255;  //Color is White
      break;
    case 10:
      curr_Hue = random(0, 255);  //Color is Random
      break;
  }
  return curr_Hue;
}
*/


// helper for updating bargraph values, to imitate bargraph movement
byte updatebar(byte disp, byte* bargraphdata, byte maxcol) {
  // bargraph values go up or down one pixel at a time
  int variation = random(0, 3);      // 0= move down, 1= stay, 2= move up
  int value = (int)(*bargraphdata);  // get the previous value
  //if (value==maxcol) value=maxcol-2; else      // special case, staying stuck at maximum does not look realistic, knock it down
  value += (variation - 1);  // grow or shring it by one step
#ifndef BLUELEDTRACKGRAPH
  if (value <= 0) value = 0;  // can't be lower than 0
#else
  if (value <= 1) value = 1;  // if blue LED tracks, OK to keep lower LED always on
#endif
  if (value > maxcol) value = maxcol;  // can't be higher than max
  (*bargraphdata) = (byte)value;       // store new value, use byte type to save RAM
  return (byte)value;                  // return new value
}


// This animates the blue LEDs
// Uses a random delay, which never exceeds BLUELEDSPEED
void updateBlueLEDs() {
  static unsigned long timeLast = 0;
  static unsigned long variabledelay = BLUELEDSPEED;
  unsigned long elapsed = millis();
  if ((elapsed - timeLast) < variabledelay) return;
  timeLast = elapsed;
  variabledelay = random(10, BLUELEDSPEED);

  /*********experimental, moving dots animation
  static byte stage=0;
  stage++;
  if (stage>7) stage=0;
  byte LEDstate=B00000011;
  // blue LEDs are row 0 col 0-5 
  dc.setRow(DATAPORT,0,LEDstate<<stage);
  *********************/

  // random
  dc.setRow(DATAPORT, 0, randomRow(4));
  return;
}



// This animates the bottom white LEDs
void updatebottomLEDs() {
  static unsigned long timeLast = 0;
  unsigned long elapsed = millis();
  if ((elapsed - timeLast) < BOTTOMLEDSPEED) return;
  timeLast = elapsed;
  // bottom LEDs are row 1,
  dc.setRow(DATAPORT, 1, randomRow(4));
  return;
}


// animates the CBI
//
void updateCBILEDs() {
  static unsigned long timeLast = 0;
  unsigned long elapsed;
  elapsed = millis();
  if ((elapsed - timeLast) < CBISPEED_RANDOM) return;
  timeLast = elapsed;
  cc.setRow(CBI, random(4), randomRow(random(4)));
  return;
}


//Handles all interactions with the class made Large Dataport Logics
int updateLDPL(int num) {
  int red = pgm_read_word(&(np_color[ldpl_color][0]));
  int green = pgm_read_word(&(np_color[ldpl_color][1]));
  int blue = pgm_read_word(&(np_color[ldpl_color][2]));

  //0 - Shut off Large Data Port
  if (num == 0) {
    for (int x = 0; x < LDPL_LEDS; x++) ldplLGHT.setPixelColor(x, ldplLGHT.Color(0, 0, 0));
    ldplLGHT.show();
    return 0;
  }
  //1 - Solid Color
  if (num == 1) {
    for (int x = 0; x < LDPL_LEDS; x++) ldplLGHT.setPixelColor(x, ldplLGHT.Color(red, green, blue));
    ldplLGHT.show();
    return 0;
  }
  //2 - Single
  if (num == 2) {
    ldpl_single();
    return 0;
  }
  //3 - Double
  if (num == 3) {
    ldpl_double();
    return 0;
  }
  return 1;
}




//updateLightBar controls the class made light bar in the DAtaport
void updateLightBar() {
  static unsigned long timeLast = 0;
  unsigned long elapsed = millis();
  if ((elapsed - timeLast) < LIGHTBARSPEED) return;
  timeLast = elapsed;
  int leftBar = random(0, 7);
  int rightBar = random(0, 7);
  for (int x = 0; x < 16; x++) lb[x] = CRGB::Black;
  //First do the left bar
  for (int x = 0; x <= leftBar; x++) {
    if (x < 4) lb[x] = CRGB::Green;
    if (x < 6 && x >= 4) lb[x] = CRGB::Yellow;
    if (x > 5) lb[x] = CRGB::Red;
  }

  //now do the right light bar
  for (int x = 0; x <= rightBar; x++) {
    int x2 = map(x, 0, 7, 15, 8);
    if (x < 4) lb[x2] = CRGB::Green;
    if (x < 6 && x >= 4) lb[x2] = CRGB::Yellow;
    if (x > 5) lb[x2] = CRGB::Red;
  }
  FastLED.show();
  return;
}



// This is for the two red LEDs
void updateRedLEDs() {
  static unsigned long timeLast = 0;
  unsigned long elapsed = millis();
  if ((elapsed - timeLast) < REDLEDSPEED) return;
  timeLast = elapsed;
  // red LEDs are row 2 and 3, col 6,
  dc.setLed(DATAPORT, 2, 6, random(0, 2));
  dc.setLed(DATAPORT, 3, 6, random(0, 2));
  return;
}


// animates the two top left blocks
// (green and yellow blocks)
void updateTopBlocks() {
  static unsigned long timeLast = 0;
  unsigned long elapsed;
  elapsed = millis();
  if ((elapsed - timeLast) < TOPBLOCKSPEED) return;
  timeLast = elapsed;

  dc.setRow(DATAPORT, 4, randomRow(4));  // top yellow blocks
  dc.setRow(DATAPORT, 5, randomRow(4));  // top green blocks
  return;
}

/*************************************************************************
 * ************************* SETUP FUNCTION ******************************
 * ***********************************************************************/


void setup() {
  Serial.begin(9600);  //Serial connection to Body Master
  //Serial1.begin(9600);           //Connected to Dome 25-pin Serial 2 - Not currently used
  //Serial2.begin(9600);           //Connected to CBI Nano - Not currently used
  Serial3.begin(9600);           //Connected to EXP Nano 
  FastLED.addLeds<WS2811, LB_PIN, GRB>(lb, LB_LEDS);        //Adds Data Panel Light Bar LEDs to FastLED array
  FastLED.addLeds<WS2811, CS_PIN, GRB>(cs, CS_LEDS);        //Adds Coin Slot LEDs to FastLED array
  FastLED.addLeds<WS2811, LDPL_PIN, GRB>(ldpl, LDPL_LEDS);  //Adds Large Data Panel LEDs to FastLED array
  // initialize Maxim driver chips
  dc.shutdown(DATAPORT, false);                  // take Data Port out of shutdown
  dc.clearDisplay(DATAPORT);                     // clear Data Port LEDs
  dc.setIntensity(DATAPORT, DATAPORTINTENSITY);  // set intensity of Data Port LEDs
  cc.shutdown(CBI, false);                       // take Charging Bay Indicator out of shutdown
  cc.clearDisplay(CBI);                          // clear CBI LEDs
  cc.setIntensity(CBI, CBIINTENSITY);            // set intensity of CBI LEDs
  //singleTest();                                  //Tests all Maxim connected leds in turn
  //delay(2000);
  pinMode(CBIDoorPin, INPUT);  //Pin on the Arduino Mini Breakout Board connected to left door switch HIGH=Door closed (NC when door closed) - S.Sloan
  pinMode(analoginput, INPUT);
  dp_door.attach(DP_DOOR);

  dp_door.writeMicroseconds(DP_DOOR_MIN);
  //delay(50);
  //dp_door.detach();
}
/*************************************************************************
 * ************************** LOOP FUNCTION ******************************
 * ***********************************************************************/

void loop() {
  if(checkSerial())parseCommand(cmdStr0);
  //if(checkSerial1())parseCommand(cmdStr1);
  //if(checkSerial2())parseCommand(cmdStr2);
  if(checkSerial3())parseCommand(cmdStr3);
  coinslot(cs_State);
  updateLDPL(ldpl_State);
  dpl(dpl_State);
  
  if (digitalRead(CBIDoorPin)==HIGH) cbi();
  else {
    cc.clearDisplay(CBI);
    cc.setLed(CBI, 6, 5, 0);
    cc.setLed(CBI, 5, 5, 0);
    cc.setLed(CBI, 4, 5, 0);
  }
}
