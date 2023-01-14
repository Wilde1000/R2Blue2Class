


#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <time.h>
#include <stdlib.h>


#define MPU 'D'
#define CMD_MAX_LENGTH 63
#define HOLO_LED 1
#define MAGIC_PANEL_LED 24
#define MAGIC_PANEL 2
#define TOP_HOLO1 3
#define TOP_HOLO_LGT 4
#define TOP_HOLO2 5
#define FRT_HOLO1 6
#define FRT_HOLO_LGT 7
#define BCK_HOLO_LGT 8
#define FRT_HOLO2 9
#define BCK_HOLO1 10
#define BCK_HOLO2 11








/************************************************
*                Global Variables               *
*************************************************/
//Neo-Pixel Objects
Adafruit_NeoPixel bHolo(HOLO_LED, BCK_HOLO_LGT, NEO_GRB + NEO_KHZ800);  //Back Holo Light
Adafruit_NeoPixel fHolo(HOLO_LED, FRT_HOLO_LGT, NEO_GRB + NEO_KHZ800);  //Front Holo Light
Adafruit_NeoPixel tHolo(HOLO_LED, TOP_HOLO_LGT, NEO_GRB + NEO_KHZ800);  //Top Holo Light
Adafruit_NeoPixel mPanel(MAGIC_PANEL_LED, MAGIC_PANEL, NEO_GRB + NEO_KHZ800);
char cmdStr[64];  //Contains the incoming message from Serial0
char dev_MPU;     //Contains the MPU code from incoming serial message
char dev_cmd;
int curr_holo_color = 13;  //Contains current color code for the holoprojectors
int curr_tholo_color = 5;  //Contains current color code for the top holoprojector
int current_mp_color=1;  //
int dev_addr;              //Device address received from Serial interface
int dev_opt;               //Device option received from the Serial interface
int holo_speed;            //Holds the current holo timeout speed.
int holo_state = 0;        //Contains current state for the holoprojectors
int mPanel_state = 0;
long int current_time = millis();  //Contains current time

long int holo_timer = current_time;  //Holds the holo random movement timer
long int mp_timer = current_time;  //Holds the magic panel timer

//Servo Objects
Servo bh1;       //Back Holoprojector Servo 1
Servo bh2;       //Back Holoprojector Servo 2
Servo fh1;       //Front Holoprojector Servo 1
Servo fh2;       //Front Holoprojector Servo 2
Servo lfServo;   //Life Form Continous Rotation Servo
Servo perServo;  //Periscope Continous Rotation Servo
Servo th1;       //Top Holoprojector Servo 1
Servo th2;       //Top Holoprojector Servo 2

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  //Connection with controller Arduino
  bHolo.begin();
  tHolo.begin();
  fHolo.begin();
  bHolo.clear();
  fHolo.clear();
  tHolo.clear();
  mPanel.begin();
  mPanel.clear();
  mPanel.setBrightness(255);
  th1.attach(TOP_HOLO1);
  th2.attach(TOP_HOLO2);
  fh1.attach(FRT_HOLO1);
  fh2.attach(FRT_HOLO2);
  bh1.attach(BCK_HOLO1);
  bh2.attach(BCK_HOLO2);
}

void loop() {
  checkSerial();      //Check Serial 0 for commands
  Holos(holo_state);  //Run Holo actions
  MagicPanel(mPanel_state);
}

void MagicPanel(int opt){
  switch(opt){
    case 0:
     for(int x=0; x<MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, 0);
     mPanel.show();
     break;
    case 1:
     for(int x=0; x<MAGIC_PANEL_LED; x++) mPanel.setPixelColor(x, getColor(current_mp_color));
     mPanel.show();
     break;
    case 2:
     colorWipe(getColor(current_mp_color),50);
     break; 
        
  }
}

void colorWipe(uint32_t color, int wait) {
  static int i=0;
  if(current_time-mp_timer>wait){
    mp_timer=current_time;
    mPanel.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    mPanel.show();                          //  Update strip to match
    i++;                          
    if(i==MAGIC_PANEL_LED)i=0;
  }
  return;
}

void theaterChase(uint32_t color, int wait) {
  static int b=0;
  if(current_time-mp_timer>wait){
    mp_timer=current_time;
    mPanel.clear();
    for(int c=b; c<MAGIC_PANEL_LED; c += 3) {
        mPanel.setPixelColor(c, color); // Set pixel 'c' to value 'color'
    }
    mPanel.show();
    b++;
    if(b==3)b=0;
    
  }
}


uint32_t getColor(int num){
  int red = pgm_read_word(&(np_color[num][0]));
  int green = pgm_read_word(&(np_color[num][1]));
  int blue = pgm_read_word(&(np_color[num][2]));
  return mPanel.Color(red, green, blue);
}


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


//The checkSerial function is the first thread of seven threads in this program.  It checks the Serial0 buffer for incoming serial
//data and then sends it to be processed.
void checkSerial() {
  char ch;
  byte cmd_Complete;
  if (Serial.available()) {
    ch = Serial.read();
    Serial.print(ch);
    cmd_Complete = buildCommand(ch, cmdStr);
    if (cmd_Complete) {
      parseCommand(cmdStr);
      Serial.println();
    }
  }
}

//The parseCommand takes the command from the buildCommand function and parses into its component parts - MPU, Address, Command and Option
int parseCommand(char* input_str) {
  byte hasArgument = false;
  byte pos = 0;
  byte length = strlen(input_str);
  if (length < 2) goto deadCmd;  //not enough characters
  int mpu = input_str[pos];      //MPU is the first character
  if (MPU != mpu) {              //if command is not for this MPU - send it on its way
    Serial.flush();
    return;
  }
  if ((mpu > 64 && mpu < 71) || mpu == '@') dev_MPU = mpu;
  else goto deadCmd;  //Not a valid MPU - end command
  // Now the address which should be the next two characters
  char addrStr[3];  //set up a char array to hold them (plus the EOS (end of String) character)
  addrStr[0] = input_str[1];
  addrStr[1] = input_str[2];
  addrStr[2] = '\0';
  dev_addr = atoi(addrStr);
  if (!length > 3) goto deadCmd;  //invalid, no command after address
  dev_cmd = input_str[3];
  char optStr[4];
  for (int x = 0; x <= 2; x++) optStr[x] = input_str[x + 4];
  optStr[3] = '\0';
  dev_opt = atoi(optStr);  // that's the numerical argument after the command character
  hasArgument = true;
  // switch on command character
  switch (dev_cmd)  // 2nd or third char, should be the command char
  {
    case 'T':
      if (!hasArgument) goto deadCmd;  // invalid, no argument after command
      doTcommand(dev_addr, dev_opt);
      break;
    case 'S':
      if (!hasArgument) goto deadCmd;  // invalid, no argument after command
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


//The doTcommand handles all T commands sent from the parseCommand function
int doTcommand(int addr, int opt) {
  Serial.println("T command");
  switch (addr) {
    case 90:
      holo_state = opt;
      break;
    case 95:
      mPanel_state = opt;
      break;
  }
}

//The doScommand handles all T commands sent from the parseCommand function
int doScommand(int addr, int opt) {
  Serial.println("S command");
  switch (addr) {
    case 90:  //Holoprojectors
      if (opt < 14) curr_holo_color = opt;
      break;
  }
}

void Holos(int opt) {
  switch (opt) {
    case 0:
      setHoloColor(0, 0);
      break;
    case 1:  //Turn on holos with current color
      setHoloColor(curr_holo_color, 0);
      holo_speed = 2000;
      holoRandom();

      break;
    case 2:  //Standard random motion
      holo_speed = 1000;
      holoRandom();
      break;
    case 3:  //fast random motion
      holo_speed = 500;
      holoRandom();
      break;
  }
  return;
}
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
