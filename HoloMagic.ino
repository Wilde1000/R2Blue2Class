/*************************************************************************
 ************************** HOLO MAGIC NANO ******************************
 *************************************************************************/

/* The Holo Magic Mega control the Holoprojectors and the Magic Panel.  In addition, it is connected to 
a MFRC 522 RFID Reader under R2's Radar Eye.  Commands from the RFID Reader not associated with the Holoprojectors or 
the Magic Panel are sent to the Dome Lift Mega for further processing.
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
      J - MP3 Trigger - Not a MPU - but treated as such in our code
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
 ************************* INCLUDED LIBRARIES ****************************
 *************************************************************************/
#include <Servo.h>              //Needed for the Holoprojector servos
#include <Adafruit_NeoPixel.h>  //Needed for Holoprojector and Magic Panel neopixels
#include <SPI.h>                //Needed for communication with the MFRC 522
#include <MFRC522.h>            //Needed to use the MFRC 522 RFID Reader

/*************************************************************************
 ************************* MACRO DEFINITIONS *****************************
 *************************************************************************/
#define SS_PIN 10  //SDA pin on the MFRC522
#define RST_PIN 6  //Reset pin on the MFRC522

#define MAX_RF_CMD 47       //Defines Maximum number of RF Commands
#define BUFFER_LEN 18       //Defines the Buffer length for a single block on a scanned card
#define MPU 'D'             //Define the current Microprocessor
#define CMD_MAX_LENGTH 63   //Define the maximum serial command length
#define HOLO_LED 1          //Define the Number of neopixel in the holoprojectors
#define MAGIC_PANEL_LED 24  //define the Number of neopixel in the Magic Panel
#define MAGIC_PANEL 2       //Define the pin the Magic Panel is attached to
#define TOP_HOLO1 A2        //Define the pin for the Top Holo Servo 1
#define TOP_HOLO_LGT 7      //Define the pin for the Top Holo light
#define TOP_HOLO2 A1        //Define the pin for the Top Holo Servo 2
#define FRT_HOLO1 3         //Define the pin for the Front Holo Servo 1
#define FRT_HOLO_LGT 8      //Define the pin for the Front Holo light  - 4
#define BCK_HOLO_LGT 4      //Define the pin for the Back Holo light
#define FRT_HOLO2 5         //Define the pin for the Front Holo Servo 2
#define BCK_HOLO1 A3        //Define the pin for the Back Holo Servo 1
#define BCK_HOLO2 9         //Define the pin for the Back Holo Servo 2

#define BH1_MAX 100  //Define the Max position for the Back Holo Servo 1
#define BH1_MID 60   //Define the Mid position for the Back Holo Servo 1
#define BH1_MIN 20   //Define the Min position for the Back Holo Servo 1
#define BH2_MAX 100  //Define the Max position for the Back Holo Servo 2
#define BH2_MID 70   //Define the Mid position for the Back Holo Servo 2
#define BH2_MIN 40   //Define the Min position for the Back Holo Servo 2

#define FH1_MAX 100  //Define the Max position for the Front Holo Servo 1
#define FH1_MID 70   //Define the Mid position for the Front Holo Servo 1
#define FH1_MIN 40   //Define the Min position for the Front Holo Servo 1
#define FH2_MAX 90   //Define the Max position for the Front Holo Servo 2
#define FH2_MID 70   //Define the Mid position for the Front Holo Servo 2
#define FH2_MIN 40   //Define the Min position for the Front Holo Servo 2

#define TH1_MAX 100  //Define the Max position for the Top Holo Servo 1
#define TH1_MID 70   //Define the Mid position for the Top Holo Servo 1
#define TH1_MIN 40   //Define the Min position for the Top Holo Servo 1
#define TH2_MAX 90   //Define the Max position for the Top Holo Servo 2
#define TH2_MID 70   //Define the Mid position for the Top Holo Servo 2
#define TH2_MIN 40   //Define the Min position for the Top Holo Servo 2


/*************************************************************************
 ************************** GLOBAL VARIABLES *****************************
 *************************************************************************/
//Neo-Pixel Objects
Adafruit_NeoPixel bHolo(HOLO_LED, BCK_HOLO_LGT, NEO_GRB + NEO_KHZ800);         //Back Holo Light
Adafruit_NeoPixel fHolo(HOLO_LED, FRT_HOLO_LGT, NEO_GRB + NEO_KHZ800);         //Front Holo Light
Adafruit_NeoPixel tHolo(HOLO_LED, TOP_HOLO_LGT, NEO_GRB + NEO_KHZ800);         //Top Holo Light
Adafruit_NeoPixel mPanel(MAGIC_PANEL_LED, MAGIC_PANEL, NEO_GRB + NEO_KHZ800);  //Magic Panel

byte nuidPICC[4];  //Holds the UID of scanned card
byte bufferLen = 18;
byte readBlockData[BUFFER_LEN];  //Holds the contents of a single block of data on scanned card


char cmdStr[64];              //Contains the incoming message from Serial0
char dev_MPU;                 //Contains the MPU code from incoming serial message
char dev_cmd;                 //Contains the Command code from incoming serial message
char rfData[MAX_RF_CMD][16];  //Contains the all the data from a scanned card

int blockNum = 2;                //Contains the current block number
int curr_holo_color = 13;        //Contains current color code for the holoprojectors
int curr_tholo_color = 5;        //Contains current color code for the top holoprojector
int current_mp_color = 6;        //Contains current color code for the Magic Panel
int dev_addr;                    //Device address received from Serial interface
int dev_opt;                     //Device option received from the Serial interface
int holo_speed;                  //Holds the current holo timeout speed.
int holo_state = 0;              //Contains current state for the holoprojectors
int mPanel_state = 0;            //Contains the current state for the Magic Panel
int sequencer_state = 0;         //Contains the current state for the sequencer
int rf_wait = 5;                 //Contains the Sequencer current wait state
long current_time = millis();    //Contains current time
long holo_timer = current_time;  //Holds the holo random movement timer
long mp_timer = current_time;    //Holds the magic panel timer

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create an instance of the RFID Reader
MFRC522::MIFARE_Key key;           // Create instance of MIFARE key
MFRC522::StatusCode status;        //Create instance of MIFARE status

unsigned long rf_timer = current_time;   //Holds the current rf timer for the sequencer
unsigned long pixelPrevious = 0;         // Previous Pixel Millis
unsigned long patternPrevious = 0;       // Previous Pattern Millis
int patternCurrent = 0;                  // Current Pattern Number
int patternInterval = 5000;              // Pattern Interval (ms)
int pixelInterval = 10;                  // Pixel Interval (ms)
int pixelQueue = 0;                      // Pattern Pixel Queue
int pixelCycle = 0;                      // Pattern Pixel Cycle
uint16_t pixelCurrent = 0;               // Pattern Current Pixel Number
uint16_t pixelNumber = MAGIC_PANEL_LED;  // Total Number of Pixels

//Servo Objects
Servo bh1;  //Back Holoprojector Servo 1
Servo bh2;  //Back Holoprojector Servo 2
Servo fh1;  //Front Holoprojector Servo 1
Servo fh2;  //Front Holoprojector Servo 2
Servo th1;  //Top Holoprojector Servo 1
Servo th2;  //Top Holoprojector Servo 2

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


/*************************************************************************
 ****************************** FUNCTIONS ********************************
 *************************************************************************/

//The buildCommand takes the current byte from the Serial1 buffer and builds a command for processing.  It returns a 0
//while in the building process and a 1 when the command is ready.
int buildCommand(char ch, char* output_str) {
  static int pos = 0;
  switch (ch) {
    case '\n':
    case '\r':
    case '\0':
      output_str[pos] = '\0';
      pos = 0;
      return true;
      break;
    default:
      output_str[pos] = ch;
      if (pos <= CMD_MAX_LENGTH - 1) pos++;
      break;
  }
  return false;
}


//The checkRFID function checks the MFRC522 to see if a card has been presented.  If so, it reads the
//entire card and starts the sequencer to process the card data.
void checkRFID() {
  if (!mfrc522.PICC_IsNewCardPresent()) return;  //No card - leave
  if (!mfrc522.PICC_ReadCardSerial()) return;    //If no card read - leave
  //Serial.println();
  blockNum = 1;
  for (int x = 0; x < MAX_RF_CMD; x++) {
    byte bufferLen = 18;
    byte readBlockData[18];
    for (byte i = 0; i < 6; i++) {
      key.keyByte[i] = 0xFF;
    }
    ReadDataFromBlock(blockNum, readBlockData);
    for (int j = 0; j < 16; j++) {
      rfData[x][j] = readBlockData[j];
    }
    blockNum++;
    if (blockNum % 4 == 3) blockNum++;
  }
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  sequencer_state = 1;
}


//The checkSerial function is the first thread of seven threads in this program.  It checks the Serial0 buffer for incoming serial
//data and then sends it to be processed.
void checkSerial() {
  char ch;
  byte cmd_Complete;
  if (Serial.available()) {
    ch = Serial.read();
    //Serial.print(ch);
    cmd_Complete = buildCommand(ch, cmdStr);
    if (cmd_Complete) {
      parseCommand(cmdStr);
      // Serial.println();
    }
  }
}


// colorWipe is a Magic Panel effect which slowly populates the Magic Panel with the passed color
void colorWipe(uint32_t color, int wait) {
  current_time = millis();
  static int i = 0;
  if (current_time - mp_timer > wait) {
    mp_timer = current_time;
    mPanel.setPixelColor(i, color);  //  Set pixel's color (in RAM)
    mPanel.show();
    i++;
    if (i == MAGIC_PANEL_LED) i = 0;
  }
}


//The doScommand handles all T commands sent from the parseCommand function
int doScommand(int addr, int opt) {
  //Serial.println("S command");
  switch (addr) {
    case 40:  //Holoprojectors
      if (opt < 14) curr_holo_color = opt;
      break;
    case 45:
      if (opt < 14) current_mp_color = opt;
      break;
  }
}


//The doTcommand handles all T commands sent from the parseCommand function
int doTcommand(int addr, int opt) {
  //Serial.println("T command");
  switch (addr) {
    case 40:
      holo_state = opt;
      break;
    case 45:
      mPanel_state = opt;
      break;
  }
}


//getColor is a helper function which returns a Magic Panel color
uint32_t getColor(int num) {
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  return mPanel.Color(red, green, blue);
}

//helper function for the Holos function
void holoRandom() {
  setHoloColor(curr_holo_color, 0);
  current_time = millis();
  if (current_time - holo_timer > holo_speed) {
    holo_timer = current_time;
    bh1.write(random(45, 135));
    bh2.write(random(45, 135));
    fh1.write(random(45, 135));
    fh2.write(random(45, 135));
    th1.write(random(45, 135));
    th2.write(random(45, 135));
  }
}


//The Holos function controls the three Holoprojectors on R2
void Holos(int opt) {
  static int servoPOS = 0;
  int sec;
  if (opt < 100) {
    switch (opt) {
      case 0:
        setHoloColor(0, 0);
        break;
      case 1:  //Turn on holos with current color
        setHoloColor(curr_holo_color, 0);
        // holo_speed = 2000;
        //holoRandom();

        break;
      case 2:  //Standard random motion
        holo_speed = 1000;
        holoRandom();
        break;
      case 3:  //fast random motion
        holo_speed = 500;
        holoRandom();
        break;
      case 4:
        //fh2.write(FH2_MIN);
        th1.write(TH1_MIN);
        holo_state = 0;
        break;
      case 5:
        th1.write(TH1_MAX);
        //fh2.write(FH2_MAX);

        holo_state = 0;
        break;
      case 6:
        th1.write(TH1_MID);
        th2.write(TH2_MID);
        //fh2.write(FH2_MAX);

        holo_state = 0;
        break;
      case 7:
        servoPOS += 10;
        th1.write(servoPOS);
        //Serial.print("Position is ");
        //Serial.println(servoPOS);
        holo_state = 0;
        break;
      case 8:
        servoPOS -= 10;
        th1.write(servoPOS);
        //Serial.print("Position is ");
        //Serial.println(servoPOS);
        holo_state = 0;
        break;
    }
  }
  if (opt >= 100 && opt < 200) {
    sec = opt - 100;
    if (HPBlink(sec)) {
      holo_state = 0;
    }
  } else if (opt >= 200 && opt < 300) {
    sec = opt - 200;
    if (HPFlicker(sec)) {
      holo_state = 0;
    }
  } else if (opt >= 300 && opt < 400) {
    sec = opt - 300;
    Serial.println("Here");
    if (HPFlicker1(sec)) holo_state = 0;
  }

  return;
}

int HPBlink(int seconds) {
  int static step = 1;
  long static routineTimer, blinkTimer;
  int static setTime = 0;
  int num = curr_holo_color;
  int interval = 250;
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  current_time = millis();
  if (!setTime) {
    setTime = 1;
    routineTimer = current_time;
    blinkTimer = current_time;
  }
  if (current_time - routineTimer < seconds * 1000) {
    if (current_time - blinkTimer > interval) {
      blinkTimer = current_time;
      if (step) {
        bHolo.setPixelColor(0, bHolo.Color(red, green, blue));
        fHolo.setPixelColor(0, fHolo.Color(red, green, blue));
        tHolo.setPixelColor(0, tHolo.Color(red, green, blue));
        step = 0;
      } else {
        bHolo.setPixelColor(0, bHolo.Color(0, 0, 0));
        fHolo.setPixelColor(0, fHolo.Color(0, 0, 0));
        tHolo.setPixelColor(0, tHolo.Color(0, 0, 0));
        step = 1;
      }
      bHolo.show();
      fHolo.show();
      tHolo.show();
    }
  } else {
    routineTimer = current_time;
    setTime = 0;
    bHolo.setPixelColor(0, bHolo.Color(0, 0, 0));
    fHolo.setPixelColor(0, fHolo.Color(0, 0, 0));
    tHolo.setPixelColor(0, tHolo.Color(0, 0, 0));
    return 1;
  }
  return 0;
}


int HPFlicker(int seconds) {
  byte flicker[10] = { 180, 30, 89, 23, 255, 200, 90, 150, 60, 230 };
  byte static setTime = 0;
  int num = curr_holo_color;
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  int static step = 0;
  long static rTimer, fTimer, cTimer;
  int interval = 50;
  cTimer = millis();
  if (!setTime) {
    setTime = 1;
    rTimer = cTimer;
    fTimer = cTimer;
  }

  if (cTimer - fTimer > interval) {
    fTimer = cTimer;
    bHolo.setPixelColor(0, bHolo.Color(red * flicker[step] / 255, green * flicker[step] / 255, blue * flicker[step] / 255));
    fHolo.setPixelColor(0, fHolo.Color(red * flicker[step] / 255, green * flicker[step] / 255, blue * flicker[step] / 255));
    tHolo.setPixelColor(0, tHolo.Color(red * flicker[step] / 255, green * flicker[step] / 255, blue * flicker[step] / 255));
    bHolo.show();
    fHolo.show();
    tHolo.show();
    if (step < 9) step++;
    else step = 0;
    return 0;
  }
  if (cTimer - rTimer > seconds * 1000) {
    rTimer = cTimer;
    bHolo.setPixelColor(0, bHolo.Color(0, 0, 0));
    fHolo.setPixelColor(0, fHolo.Color(0, 0, 0));
    tHolo.setPixelColor(0, tHolo.Color(0, 0, 0));
    bHolo.show();
    fHolo.show();
    tHolo.show();
    setTime = 0;
    return 1;
  }
}

int HPFlicker1(int seconds) {
  byte flicker[10] = { 180, 30, 89, 23, 255, 200, 90, 150, 60, 230 };
  byte static setTime = 0;
  int static step = 0;
  long static rTimer, fTimer, cTimer;
  cTimer = millis();
  int num = 13;
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  int interval = 50;
  if (!setTime) {
    setTime = 1;
    rTimer = cTimer;
    fTimer = cTimer;
  }
  if (cTimer - rTimer < seconds * 1000) {
    if (cTimer - fTimer > interval) {
      fTimer = cTimer;
      bHolo.setPixelColor(0, bHolo.Color(0, 0, 0));
      fHolo.setPixelColor(0, fHolo.Color(red * flicker[step] / 255, green * flicker[step] / 255, blue * flicker[step] / 255));
      tHolo.setPixelColor(0, tHolo.Color(0, 0, 0));
      bHolo.show();
      fHolo.show();
      tHolo.show();
      if (step < 9) step++;
      else step = 0;
      return 0;
    }
  } else {
    rTimer = cTimer;
    bHolo.setPixelColor(0, bHolo.Color(0, 0, 0));
    fHolo.setPixelColor(0, fHolo.Color(0, 0, 0));
    tHolo.setPixelColor(0, tHolo.Color(0, 0, 0));
    bHolo.show();
    fHolo.show();
    tHolo.show();
    setTime = 0;
    return 1;
  }
}


//The MagicPanel function handles all operations for the Magic Panel on R2
void MagicPanel(int opt) {
  int sec;
  switch (opt) {
    case 0:
      for (int x = 0; x < MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, 0);
      mPanel.show();
      break;
    case 1:
      for (int x = 0; x < MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, getColor(current_mp_color));
      mPanel.show();
      break;
    case 2:
      colorWipe(getColor(current_mp_color), 25);
      break;
    case 3:
      theaterChase(getColor(current_mp_color), 100);
      break;
    case 4:
      theaterChaseRainbow(100);
      break;
    case 5:
      rainbow(10);
      break;
    case 6:
      if (MPFlicker(4)) mPanel_state = 0;
      break;
    case 7:
      if (MPBlink(4)) mPanel_state = 0;
      break;
  }
  if (opt >= 100 && opt < 200) {
    sec = opt - 100;
    if (MPBlink(sec)) holo_state = 0;
  } else if (opt >= 200 && opt < 300) {
    sec = opt - 200;
    if (MPFlicker(sec)) holo_state = 0;
  }
  return;
}

int MPBlink(int seconds) {
  int static step = 1;
  long static rTimer, bTimer, cTimer;
  int static setTime = 0;
  int num = current_mp_color;
  int interval = 250;
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  cTimer = millis();
  if (!setTime) {
    setTime = 1;
    rTimer = cTimer;
    bTimer = cTimer;
  }

  if (cTimer - bTimer > interval) {
    bTimer = cTimer;
    if (step) {
      for (int x = 0; x < MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, mPanel.Color(red, green, blue));
      mPanel.show();
      step = 0;
    } else {
      for (int x = 0; x < MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, mPanel.Color(0, 0, 0));
      mPanel.show();
      step = 1;
    }
  }
  if (cTimer - rTimer > seconds * 1000) {
    rTimer = cTimer;
    setTime = 0;
    for (int x = 0; x < MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, mPanel.Color(0, 0, 0));
    mPanel.show();
    return 1;
  }
  return 0;
}


int MPFlicker(int seconds) {
  byte static timeSet = 0;  //Switch to set the timers
  long static rTimer, fTimer, cTimer;
  byte flicker[10] = { 180, 30, 89, 23, 255, 200, 90, 150, 60, 230 };
  int num = current_mp_color;
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  int static step = 0;
  int interval = 50;
  cTimer = millis();
  if (!timeSet) {
    timeSet = 1;
    rTimer = cTimer;
    fTimer = cTimer;
  }

  if (cTimer - rTimer < seconds * 1000) {
    if (cTimer - fTimer > interval) {
      fTimer = cTimer;
      for (int x = 0; x < MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, mPanel.Color(red * flicker[step] / 255, green * flicker[step] / 255, blue * flicker[step] / 255));
      mPanel.show();
      if (step < 9) step++;
      else step = 0;
    }
  } else {
    rTimer = cTimer;
    for (int x = 0; x < MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, 0, 0, 0);
    mPanel.show();
    timeSet = 0;
    return 1;
  }
  return 0;
}



//The parseCommand takes the command from the buildCommand function and parses into its component parts - MPU, Address, Command and Option
int parseCommand(char* input_str) {
  byte length = strlen(input_str);
  if (length < 2) goto deadCmd;  //not enough characters
  int mpu = input_str[0];        //MPU is the first character
  if (MPU != mpu) {              //if command is not for this MPU - send it on its way
    Serial.flush();
    for (int x = 0; x < length; x++) Serial.write(input_str[x]);
    Serial.write(13);
    return;
  }
  dev_MPU = mpu;
  // Now the address which should be the next two characters
  char addrStr[3];  //set up a char array to hold them (plus the EOS (end of String) character)
  addrStr[0] = input_str[1];
  addrStr[1] = input_str[2];
  addrStr[2] = '\0';
  dev_addr = atoi(addrStr);
  if (!length > 4) goto deadCmd;  //invalid, no command after address
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
      optStr[2] = input_str[6];
      optStr[3] = '\0';
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
      goto deadCmd;  // unknown command
      break;
  }
  return;
deadCmd:
  return;
}


// Rainbow cycle along whole mPanel. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  current_time = millis();
  static long firstPixelHue = 0;
  if (current_time - mp_timer > wait) {
    mp_timer = current_time;
    mPanel.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // mPanel.rainbow(firstPixelHue, 1, 255, 255, true);
    mPanel.show();  // Update strip with new contents
    firstPixelHue += 256;
    if (firstPixelHue >= 5 * 65536) firstPixelHue = 0;
  }
}



//The ReadDataFromBlock function reads a single block from the scanned card and verifies the encryption
void ReadDataFromBlock(int blockNum, byte readBlockData[]) {
  /* Authenticating the desired data block for Read access using Key A */
  byte status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, blockNum, &key, &(mfrc522.uid));

  if (status != MFRC522::STATUS_OK) {
    //Serial.print("Authentication failed for Read: ");
    //Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
  /* Reading data from the Block */
  status = mfrc522.MIFARE_Read(blockNum, readBlockData, &bufferLen);
  if (status != MFRC522::STATUS_OK) {
    //Serial.print("Reading failed: ");
    //Serial.println(mfrc522.GetStatusCodeName(status));
    return;
  }
}


//The sequencer() function takes the data from a scanned card and processes it line by line.
void sequencer() {
  static byte step = 0;

  if (rfData[step][0] == '@') {
    //Serial.println("Final Command");
    //final step is reached
    //Clear the rfData
    for (int x = 0; x > 10; x++) {
      for (int y = 0; y < 16; y++) {
        rfData[x][y] = 0;
      }
    }
    step = 0;
    sequencer_state = 0;
    return;
  }
  current_time = millis();
  if (current_time - rf_timer > rf_wait) {
    rf_timer = current_time;
    rf_wait = 3;  //reset wait time
    //Process current step

    char cmd[16];
    byte count = 0;
    while (rfData[step][count] != 35) {
      cmd[count] = rfData[step][count];
      count++;
    }
    cmd[count] = 13;
    int cmdLen = count;
    count = 0;
    step++;

    //for(int x=0; x<cmdLen; x++){
    //  Serial.print(cmd[x]);
    //}
    //Serial.println();
    if (cmd[0] == 'W') {

      // set the Wait state in seconds
      rf_wait = ((cmd[1] - 48) * 10 + (cmd[2] - 48)) * 1000;
      //Serial.println(rf_wait);
      return;
    } else if (cmd[0] >= 65 && cmd[0] <= 74) parseCommand(cmd);
    return;
  }
}

//The setHoloColor function sets the color of the three Holoprojectors in R2's dome.
void setHoloColor(int num, int holo) {
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  if (holo == 0) {
    bHolo.setPixelColor(0, bHolo.Color(red, green, blue));
    fHolo.setPixelColor(0, fHolo.Color(red, green, blue));
    tHolo.setPixelColor(0, tHolo.Color(red, green, blue));
    bHolo.show();
    fHolo.show();
    tHolo.show();
    return;
  } else if (holo == 1) {
    tHolo.setPixelColor(0, tHolo.Color(red, green, blue));
    tHolo.show();
    return;
  } else if (holo == 2) {
    fHolo.setPixelColor(0, fHolo.Color(red, green, blue));
    fHolo.show();
    return;
  } else {
    bHolo.setPixelColor(0, bHolo.Color(red, green, blue));
    bHolo.show();
    return;
  }
}


//theaterChase is a Magic Panel routine that animates every other neopixel like an old time movie theather
void theaterChase(uint32_t color, int wait) {
  current_time = millis();
  static int b = 0;
  if (current_time - mp_timer > wait) {
    mp_timer = current_time;
    mPanel.clear();  //   Set all pixels in RAM to 0 (off)
    // 'c' counts up from 'b' to end of strip in steps of 3...
    for (int c = b; c < mPanel.numPixels(); c += 3) {
      mPanel.setPixelColor(c, color);  // Set pixel 'c' to value 'color'
    }
    mPanel.show();  // Update strip with new contents
    b++;
    if (b == 3) b = 0;
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int static firstPixelHue = 0;  // First pixel starts at red (hue 0)
  int static b = 0;

  current_time = millis();
  if (current_time - mp_timer > wait) {
    mp_timer = current_time;
    mPanel.clear();  //   Set all pixels in RAM to 0 (off)
    // 'c' counts up from 'b' to end of strip in increments of 3...
    for (int c = b; c < mPanel.numPixels(); c += 3) {
      // hue of pixel 'c' is offset by an amount to make one full
      // revolution of the color wheel (range 65536) along the length
      // of the strip (mPanel.numPixels() steps):
      int hue = firstPixelHue + c * 65536L / mPanel.numPixels();
      uint32_t color = mPanel.gamma32(mPanel.ColorHSV(hue));  // hue -> RGB
      mPanel.setPixelColor(c, color);                         // Set pixel 'c' to value 'color'
    }
    mPanel.show();
    firstPixelHue += 65536 / 90;  // One cycle of color wheel over 90 frames
    b++;
    if (b == 3) b = 0;
  }
  return;
}


/*************************************************************************
 *************************** SETUP FUNCTION ******************************
 *************************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //Connection with controller Arduino
  SPI.begin();         //Initialize SPI bus
  mfrc522.PCD_Init();  //Initialize MFRC522 Module
  bHolo.begin();
  tHolo.begin();
  fHolo.begin();
  bHolo.clear();
  fHolo.clear();
  tHolo.clear();
  mPanel.begin();
  mPanel.clear();
  mPanel.setBrightness(100);
  bHolo.setBrightness(250);
  fHolo.setBrightness(250);
  tHolo.setBrightness(250);
  //th1.attach(TOP_HOLO1);
  //th2.attach(TOP_HOLO2);
  //fh1.attach(FRT_HOLO1);
  //fh2.attach(FRT_HOLO2);
  //bh1.attach(BCK_HOLO1);
  //bh2.attach(BCK_HOLO2);
}

/*************************************************************************
 **************************** LOOP FUNCTION ******************************
 *************************************************************************/

void loop() {
  checkSerial();      //Check Serial 0 for commands
  Holos(holo_state);  //Run Holo actions
  MagicPanel(mPanel_state);
  if (sequencer_state) sequencer();
  else checkRFID();
}
