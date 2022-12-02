/*  This program controls the periscope lights for the R2-Blue2 Dome Lift System.  It is designed to run
on an Arduino Nano.
The Periscope lights consist of 7 White LED's and eight neo-pixels.  
The 7 small white LEDs will be referred to as "Display" Lights 
Neo-pixels 0 and 2 are the right and left "Eyes"
Neo-pixel 1 is the "Status" Light
Neo-pixels 3 thru 6 are the "Main" Display 
The program will have the following threads:
    checkSerial()  -  Checks Serial0 for commands and executes the commands
    runDisplay()  - Runs any commands for the Display Lights
    runEyes()  -  Runs any commands for the Eyes
    runStatus() - Runs any commands for the Status Light
    runMain() - Runs any command for the Main Display
*/
#include <Adafruit_NeoPixel.h>
#define LED_PIN 10   //Pin NeoPixel attached to
#define LED_COUNT 12  //Number of NeoPixels
#define RED 1
#define ROSE 2
#define MAGENTA 3
#define VIOLET 4
#define BLUE 5
#define AZURE 6
#define CYAN 7
#define LTGREEN 8
#define GREEN 9
#define CHARTREUSE 10
#define YELLOW 11
#define ORANGE 12
#define WHITE 13

#define CMD_MAX_LENGTH 63
#define MPU 'F'

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)


long int curr_time = millis();
long int disp_time = curr_time;
long int eyes_time = curr_time;
long int stat_time = curr_time;
long int main_time = curr_time;
int disp_int = 200;
int stat_int = 444;
int eyes_int = 750;
int main_int = 2000;
int toggle = 0;
int disp_State = 4;
int eyes_State = 31;
int stat_State = 10;
int main_State = 1;
int seq_State = 0;
int dev_addr, dev_opt;
char dev_cmd, dev_MPU;
char cmdStr[64];

void setup() {
  Serial.begin(9600);
  for (int x = 3; x <= 9; x++) pinMode(x, OUTPUT);
  strip.begin();             // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();              // Turn OFF all pixels ASAP
  strip.setBrightness(255);  // Set BRIGHTNESS to about 1/5 (max = 255)
}


// loop() function -- runs repeatedly as long as board is on ---------------

void loop() {
  checkSerial();
  runDisplay(disp_State);
  runStatus(stat_State);
  runEyes(eyes_State);
  runMain(main_State);
}

void runMain(int option){
    switch(option){
      case 0:
        for(int x=3; x<12; x++)setColor(x,0);
        break;
      case 1:
        for(int x=3; x<12; x++)setColor(x,RED);
        break;
    }  
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
      Serial.println("Here");
      parseCommand(cmdStr);
      Serial.println();
    }
  }
}

//The buildCommand takes the current byte from the Serial0 buffer and builds a command for processing.  It returns a 0
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

//The parseCommand takes the command from the buildCommand function and parses into its component parts - MPU, Address, Command and Option
int parseCommand(char* input_str) {
  byte hasArgument = false;
  byte pos = 0;
  byte length = strlen(input_str);
  if (length < 2) goto deadCmd;  //not enough characters
  int mpu = input_str[pos];      //MPU is the first character
  if (!MPU == mpu) {             //if command is not for this MPU - send it on its way
    //transmitCMD(MPU,mpu);
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
      //doScommand(dev_addr, dev_opt);
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
    case 56:  //Display Lights
      disp_State = opt;
      break;
    case 57:  //Eyes
      eyes_State = opt;
      break;
    case 58:
      main_State = opt;
      break;
    case 59:
      stat_State = opt;
      break;
    case 60:
      seq_State = opt;
      break;
  }
}

void runStatus(int option) {
  switch (option) {
    case 0:
      return;
      break;
    case 1:
      statBlink(RED, GREEN);
      break;
    case 2:
      statBlink(RED, BLUE);
      break;
    case 3:
      statBlink(RED, WHITE);
      break;
    case 4:
      statBlink(GREEN, BLUE);
      break;
    case 5:
      statBlink(WHITE, GREEN);
      break;
    case 6:
      statBlink(WHITE, BLUE);
      break;
    case 7:
      setColor(1, RED);
      break;
    case 8:
      setColor(1, GREEN);
      break;
    case 9:
      setColor(1, BLUE);
      break;
    case 10:
      setColor(1, WHITE);
      break;
  }
}

void runDisplay(int option) {
  switch (option) {
    case 0:
      return;
      break;
    case 1:
      dispBlink();
      break;
    case 2:
      dispSeq1();
      break;
    case 3:
      dispSeq2();
      break;
    case 4:
      dispSeq3();
      break;
  }
}

void dispSeq1() {
  static int step = 0;
  curr_time = millis();
  if (curr_time - disp_time > disp_int) {
    disp_time = curr_time;

    if (step == 0) {
      digitalWrite(step + 3, HIGH);
      step++;
    } else if (step == 7) {
      digitalWrite(step + 2, LOW);
      step = 0;
    } else {
      digitalWrite(step + 3, HIGH);
      digitalWrite(step + 2, LOW);
      step++;
    }
  }
}
void dispSeq2() {
  static int step = 0;
  curr_time = millis();
  if (curr_time - disp_time > disp_int) {
    disp_time = curr_time;
    switch (step) {
      case 0:
        digitalWrite(3, HIGH);
        digitalWrite(9, HIGH);
        step++;
        break;
      case 1:
        digitalWrite(3, LOW);
        digitalWrite(9, LOW);
        digitalWrite(4, HIGH);
        digitalWrite(8, HIGH);
        step++;
        break;
      case 2:
        digitalWrite(4, LOW);
        digitalWrite(8, LOW);
        digitalWrite(5, HIGH);
        digitalWrite(7, HIGH);
        step++;
        break;
      case 3:
        digitalWrite(5, LOW);
        digitalWrite(7, LOW);
        digitalWrite(6, HIGH);
        step++;
        break;
      case 4:
        digitalWrite(6, LOW);
        digitalWrite(5, HIGH);
        digitalWrite(7, HIGH);
        step++;
        break;
      case 5:
        digitalWrite(5, LOW);
        digitalWrite(7, LOW);
        digitalWrite(4, HIGH);
        digitalWrite(8, HIGH);
        step++;
        break;
      case 6:
        digitalWrite(4, LOW);
        digitalWrite(8, LOW);
        step = 0;
        break;
    }
  }
}

void dispSeq3() {  //loading bar
  static int step = 3;
  static int state = 1;
  curr_time = millis();
  if (curr_time - disp_time > disp_int) {
    disp_time = curr_time;
    if (step == 9 && state == 1) {
      state = 0;
      digitalWrite(step, HIGH);
      step = 3;
    } else if (step == 9 && state == 0) {
      state = 1;
      digitalWrite(step, LOW);
      step = 3;
    } else if (state) {
      digitalWrite(step, HIGH);
      step++;
    } else {
      digitalWrite(step, LOW);
      step++;
    }
  }
}

void dispBlink() {
  int x;
  curr_time = millis();
  if (curr_time - disp_time > disp_int) {
    disp_time = curr_time;
    if (toggle) {
      for (x = 3; x < 10; x++) digitalWrite(x, HIGH);
      toggle = 0;
    } else {
      for (x = 3; x < 10; x++) digitalWrite(x, LOW);
      toggle = 1;
    }
  }
}

void setColor(int pixel, int color) {
  switch (color) {
    case 0:  //off
      strip.setPixelColor(pixel, strip.Color(0, 0, 0));
      break;
    case 1:  //Red
      strip.setPixelColor(pixel, strip.Color(255, 0, 0));
      break;
    case 2:  //Rose
      strip.setPixelColor(pixel, strip.Color(255, 0, 128));
      break;
    case 3:  //Magenta
      strip.setPixelColor(pixel, strip.Color(255, 0, 255));
      break;
    case 4:  //Violet
      strip.setPixelColor(pixel, strip.Color(128, 0, 255));
      break;
    case 5:  //Blue
      strip.setPixelColor(pixel, strip.Color(0, 0, 255));
      break;
    case 6:  //Azure
      strip.setPixelColor(pixel, strip.Color(0, 128, 255));
      break;
    case 7:  //Cyan
      strip.setPixelColor(pixel, strip.Color(0, 255, 255));
      break;
    case 8:  //Spring Green
      strip.setPixelColor(pixel, strip.Color(0, 255, 128));
      break;
    case 9:  //Green
      strip.setPixelColor(pixel, strip.Color(0, 255, 0));
      break;
    case 10:  //Chartreuse
      strip.setPixelColor(pixel, strip.Color(128, 255, 0));
      break;
    case 11:  //Yellow
      strip.setPixelColor(pixel, strip.Color(255, 255, 0));
      break;
    case 12:  //Orange
      strip.setPixelColor(pixel, strip.Color(255, 128, 0));
      break;
    case 13:  //White
      strip.setPixelColor(pixel, strip.Color(255, 255, 255));
      break;
  }
  strip.show();
}

void statBlink(int color1, int color2) {
  int x;
  static byte stoggle = 1;
  curr_time = millis();
  if (curr_time - stat_time > stat_int) {
    stat_time = curr_time;
    if (stoggle) {
      setColor(1, color1);
      stoggle = 0;
    } else {
      setColor(1, color2);
      stoggle = 1;
    }
    strip.show();
  }
}
void runEyes(int option) {
  switch (option) {
    case 0:
      return;
      break;
    case 1:  //Solid Red
      setColor(0, RED);
      setColor(2, RED);
      break;
    case 2:  //Solid Rose
      setColor(0, ROSE);
      setColor(2, ROSE);
      break;
    case 3:  //Solid Magenta
      setColor(0, MAGENTA);
      setColor(2, MAGENTA);
      break;
    case 4:  //Solid Violet
      setColor(0, VIOLET);
      setColor(2, VIOLET);
      break;
    case 5:  //Solid Blue
      setColor(0, BLUE);
      setColor(2, BLUE);
      break;
    case 6:  //Solid Azure
      setColor(0, AZURE);
      setColor(2, AZURE);
      break;
    case 7:  //Solid Cyan
      setColor(0, CYAN);
      setColor(2, CYAN);
      break;
    case 8:  //Solid Spring Green
      setColor(0, LTGREEN);
      setColor(2, LTGREEN);
      break;
    case 9:  //Solid Green
      setColor(0, GREEN);
      setColor(2, GREEN);
      break;
    case 10:  //Solid Chartreuse
      setColor(0, CHARTREUSE);
      setColor(2, CHARTREUSE);
      break;
    case 11:  //Solid Yellow
      setColor(0, YELLOW);
      setColor(2, YELLOW);
      break;
    case 12:  //Solid Orange
      setColor(0, ORANGE);
      setColor(2, ORANGE);
      break;
    case 13:  //Solid White
      setColor(0, WHITE);
      setColor(2, WHITE);
      break;
    case 14:  //Blink Red
      eyesBlink(RED);
      break;
    case 15:  //Blink Rose
      eyesBlink(ROSE);
      break;
    case 16:  //Blink Magenta
      eyesBlink(MAGENTA);
      break;
    case 17:  //Blink Violet
      eyesBlink(VIOLET);
      break;
    case 18:  //Blink Blue
      eyesBlink(BLUE);
      break;
    case 19:  //Blink Azure
      eyesBlink(AZURE);
      break;
    case 20:  //Blink Cyan
      eyesBlink(CYAN);
      break;
    case 21:  //Blink Light Green
      eyesBlink(LTGREEN);
      break;
    case 22:  //Blink Green
      eyesBlink(GREEN);
      break;
    case 23:  //Blink Chartreuse
      eyesBlink(CHARTREUSE);
      break;
    case 24:  //Blink Yellow
      eyesBlink(YELLOW);
      break;
    case 25:  //Blink Orange
      eyesBlink(ORANGE);
      break;
    case 26:  //Blink White
      eyesBlink(WHITE);
      break;
    case 27:  //Blink Red
      eyesAltBlink(RED);
      break;
    case 28:  //Blink Rose
      eyesAltBlink(ROSE);
      break;
    case 29:  //Blink Magenta
      eyesAltBlink(MAGENTA);
      break;
    case 30:  //Blink Violet
      eyesAltBlink(VIOLET);
      break;
    case 31:  //Blink Blue
      eyesAltBlink(BLUE);
      break;
    case 32:  //Blink Azure
      eyesAltBlink(AZURE);
      break;
    case 33:  //Blink Cyan
      eyesAltBlink(CYAN);
      break;
    case 34:  //Blink Light Green
      eyesAltBlink(LTGREEN);
      break;
    case 35:  //Blink Green
      eyesAltBlink(GREEN);
      break;
    case 36:  //Blink Chartreuse
      eyesAltBlink(CHARTREUSE);
      break;
    case 37:  //Blink Yellow
      eyesAltBlink(YELLOW);
      break;
    case 38:  //Blink Orange
      eyesAltBlink(ORANGE);
      break;
    case 39:  //Blink White
      eyesAltBlink(WHITE);
      break;
  }
}


void eyesBlink(int num) {
  int x;
  static byte etoggle = 1;
  curr_time = millis();
  if (curr_time - eyes_time > eyes_int) {
    eyes_time = curr_time;
    if (etoggle) {
      setColor(0, num);
      setColor(2, num);
      etoggle = 0;
    } else {
      setColor(0, 0);
      setColor(2, 0);
      etoggle = 1;
    }
    strip.show();
  }
}
void eyesAltBlink(int num) {
  int x;
  static byte etoggle = 1;
  curr_time = millis();
  if (curr_time - eyes_time > eyes_int) {
    eyes_time = curr_time;
    if (etoggle) {
      setColor(0, num);
      setColor(2, 0);
      etoggle = 0;
    } else {
      setColor(0, 0);
      setColor(2, num);
      etoggle = 1;
    }
    strip.show();
  }
}


void mainBlink() {
  int x;
  static byte mtoggle = 1;
  curr_time = millis();
  if (curr_time - main_time > main_int) {
    main_time = curr_time;
    if (mtoggle) {
      strip.fill(strip.Color(0, 0, 255), 3);
      mtoggle = 0;
    } else {
      strip.fill(0, 3);
      mtoggle = 1;
    }
    strip.show();
  }
}
