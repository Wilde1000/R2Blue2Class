/*  This program controls the periscope lights for the R2-Blue2 Dome Lift System.  It is designed to run
on an Arduino Nano.

The Periscope lights consist of 7 White LED's and eight neo-pixels.  

The 7 small white LEDs will be referred to as "Display" Lights 

Neo-pixels 0 and 2 are the right and left "Eyes"

Neo-pixel 1 is the "Status" Light

Neo-pixels 3 thru 11 are the "Main" Display 

The program will have the following threads:
    checkSerial()  -  Checks Serial0 for commands and executes the commands
    runDisplay()  - Runs any commands for the Display Lights
    runEyes()  -  Runs any commands for the Eyes
    runStatus() - Runs any commands for the Status Light
    runMain() - Runs any command for the Main Display

Display - device 56
Eyes    - device 57
Main    - device 58
Status  - device 59

Main states:
  000 - Disabled
  001 - Horizontal ping pong
  002 - Vertical ping pong
  003 - Diagonal ping pong
  004 - Alternate
  005 - Circle
  006 - Random
Eye states:
  000 - Disabled
  001 - Toggle on/off
  002 - Blue
  003 - Random
Status states:
  000 - Disabled
  001 - Alternate red & green
  002 - Green
  003 - Red
Display states:
  000 - Disabled
  001 - Ping pong, speeding up and down
  002 - Random
  003 - Alternate odds and evens

Example: F53T002 - Set the display's state to random.
*/

#include <Adafruit_NeoPixel.h>
#include <time.h>
#include <stdlib.h>

#define LEDPIN 10
#define NOOFLED 12
Adafruit_NeoPixel pixels(NOOFLED, LEDPIN, NEO_GRB + NEO_KHZ800);

#define CMD_MAX_LENGTH 63

#define LEFT_EYE 0
#define RIGHT_EYE 2

char cmdStr[64];

long int currentTime = millis();

#define MPU 'F'
char devMPU, devCmd;
int devAddr, devOpt;

int pingPongIntervalDirection = 1;

long int displayPause = currentTime;
int displayInterval = 5;
long int mainPause = currentTime;
int mainInterval = 250;
long int eyesPause = currentTime;
int eyesInterval = 250;
long int statusPause = currentTime;
int statusInterval = 500;

bool allOn = false;
int allState     = 0;
int eyesState    = 1;
int statusState  = 1;
int mainState    = 1;
int displayState = 1;
float mainPixelColors[10][3];
float eyesPixelColors[3][3];


void setup() {
  srand((unsigned int)time(NULL));

  for (int i = 3; i <= 9; i++) {
    pinMode(i, OUTPUT);
  }

  for (int i=0; i<=12 - 3; i++) {
    mainPixelColors[i][0] = 1;
    mainPixelColors[i][1] = 1;
    mainPixelColors[i][2] = 1;
  }
  for (int i=0; i<3; i++) {
    eyesPixelColors[i][0] = 0;
    eyesPixelColors[i][1] = 0;
    eyesPixelColors[i][2] = 0;
  }

  pixels.begin();
  Serial.begin(9600);
}

void loop() {
  checkSerial();

  pixels.clear();

  runAll();

  if (allOn)
  {
    runMain(mainState);
    runDisplay(displayState);
    runEyes(eyesState);
    runStatus(statusState);
  } else {
    runMain(0);
    runDisplay(0);
    runEyes(0);
    runStatus(0);
  }

  pixels.show();
}

void runAll() {
  switch(allState) {
    case 0:
      allOn = false;
      break;
    case 1:
      allOn = true;
      break;
    case 2:
      allOn = false;
      break;
  }
}

int parseCommand(char* inputStr) {
  byte hasArgument = false;
  byte pos = 0;
  byte length = strlen(inputStr);
  if (length < 2) goto deadCmd;  //not enough characters
  int mpu = inputStr[pos];      //MPU is the first character
  if (!MPU == mpu) {             //if command is not for this MPU - send it on its way
    //transmitCMD(MPU,mpu);
    return;
  }
  if (mpu >= '@' && mpu <= 'F') devMPU = mpu;
  else goto deadCmd;  //Not a valid MPU - end command
  // Now the address which should be the next two characters
  char addrStr[3];  //set up a char array to hold them (plus the EOS (end of String) character)
  addrStr[0] = inputStr[1];
  addrStr[1] = inputStr[2];
  addrStr[2] = '\0';
  devAddr = atoi(addrStr);
  if (!length > 3) goto deadCmd;  //invalid, no command after address
  devCmd = inputStr[3];
  char optStr[4];
  for (int x = 0; x <= 3; x++) optStr[x] = inputStr[x + 4];
  optStr[4] = '\0';
  devOpt = atoi(optStr);  // that's the numerical argument after the command character
  hasArgument = true;
  // switch on command character
  switch (devCmd)  // 2nd or third char, should be the command char
  {
    case 'T':
      if (!hasArgument) goto deadCmd;  // invalid, no argument after command
      doTcommand(devAddr, devOpt);
      break;
    case 'S':
      if (!hasArgument) goto deadCmd;  // invalid, no argument after command
      //doScommand(devAddr, devOpt);
      break;
    default:
      goto deadCmd;  // unknown command
      break;
  }
  return;
deadCmd:
  return;
}

int buildCommand(char ch, char* outputStr) {
  static int pos = 0;
  switch (ch) {
    case '\n':
    case '\r':
    case '\0':
      outputStr[pos] = '\0';
      pos = 0;
      return true;
    default:
      outputStr[pos] = ch;
      if (pos <= CMD_MAX_LENGTH - 1) pos++;
      break;
  }
  return false;
}

void checkSerial() {
  char ch;
  byte cmdComplete;

  if (Serial.available()) {
    ch = Serial.read();
    Serial.print(ch);
    cmdComplete = buildCommand(ch, cmdStr);
    if (cmdComplete) {
      Serial.println();
      parseCommand(cmdStr);
    }
  }
}


int doTcommand(int addr, int opt) {
  Serial.print("T command ");
  Serial.println(opt);
  switch (addr) {
    case 56:
      displayInterval = 1;
      displayState = opt;
      break;
    case 57:
      eyesState = opt;
      break;
    case 58:
      mainState = opt;
      break;
    case 59:
      statusState = opt;
      break;
    case 60:
      allState = opt;
      break;
  }
}

void runDisplay(int option) {
  switch (option) {
    case 0: // Disabled
      for (int i=3; i<=9; i++) {
        digitalWrite(i, LOW);
      }
      break;
    case 1:
      displaySequencePingPong();
      break;
    case 2:
      displayInterval = 250;
      displaySequenceRandom();
      break;
    case 3:
      displayInterval = 250;
      displaySequenceAlternate();
      break;
  }
}

void displaySequenceAlternate() {
  static int step = 0;
  currentTime = millis();

  if (currentTime - displayPause > displayInterval) {
    displayPause = currentTime;

    for (int i=3; i<=9; i++) {
      digitalWrite(i, (i + step) % 2 == 0 ? HIGH : LOW);
    }
    step++;
  }
}

#define ENABLE_LIGHT(idx, isEnabled) digitalWrite((idx), (isEnabled) ? HIGH : LOW)
void displaySequenceRandom() {
  
  currentTime = millis();

  if (currentTime - displayPause > displayInterval) {
    displayPause = currentTime;
    ENABLE_LIGHT(3, rand() % 2 == 0);
    ENABLE_LIGHT(4, rand() % 2 == 0);
    ENABLE_LIGHT(5, rand() % 2 == 0);
    ENABLE_LIGHT(6, rand() % 2 == 0);
    ENABLE_LIGHT(7, rand() % 2 == 0);
    ENABLE_LIGHT(8, rand() % 2 == 0);
    ENABLE_LIGHT(9, rand() % 2 == 0);
  }
}
#undef ENABLE_LIGHT
  
void displaySequencePingPong() {
  static int step = 0;
  static int totalSteps = 0;
  static int dir = 1;
  currentTime = millis();

  if (currentTime - displayPause > displayInterval) {
    displayPause = currentTime;

    if (step == 0) {
      digitalWrite(step + 3, HIGH);
      digitalWrite(step + 4, LOW);
      dir = 1;
      step++;
    } else if (step == 6) {
      digitalWrite(step + 3, HIGH);
      digitalWrite(step + 2, LOW);
      dir = 0;
      step--;
    } else if (dir) {
      digitalWrite(step + 3, HIGH);
      digitalWrite(step + 2, LOW);
      step++;
    } else {
      digitalWrite(step + 3, HIGH);
      digitalWrite(step + 4, LOW);
      step--;
    }
    totalSteps++;

    if (totalSteps % 7 == 0) // Increase/decrease the interval every 7 steps.
      displayInterval += pingPongIntervalDirection;
    if (totalSteps % 150 == 0) // Every 150 steps, reverse the direction.
      pingPongIntervalDirection *= -1; 
  }
}

float mainSequenceStepColors[3][3] = {
  { 1, 0, 0 },
  { 0, 1, 0 },
  { 0, 0, 1 }
};

void mainSequencePattern(int steps[][9], int stepCount, float colors[][3], int colorCount, bool pingPong) {
  static int step = 0;
  static int stepDir = 1;
  if (!pingPong) stepDir = 1;
  
  currentTime = millis();

  if (currentTime - mainPause > mainInterval) {
    mainPause = currentTime;
    step += stepDir;
    if (step > stepCount - 1 || step < 0) {
      if (pingPong) {
        stepDir *= -1;
        step += stepDir * 2; // Keeping it in bounds      
        if (step > stepCount - 1) step = stepCount - 1;
      } else {
        stepDir = 1;
        step = 0;
      }
    }
  }

  for (int i=0; i<9; i++) {
    bool isOn = steps[step][i];
    float* color = colors[step % colorCount];
    setColor(i + 3, color[0] * isOn, color[1] * isOn, color[2] * isOn);
  }
}

void mainSequenceRowsHorizontal() {
  int steps[3][9] = {
    { 1, 1, 1,
      0, 0, 0,
      0, 0, 0 },
    { 0, 0, 0,
      1, 1, 1,
      0, 0, 0 },
    { 0, 0, 0,
      0, 0, 0,
      1, 1, 1 }
  };
  
  mainSequencePattern(steps, 3, mainSequenceStepColors, 3, true);
}
void mainSequenceRowsVertical() {
  int steps[3][9] = {
    { 1, 0, 0,
      0, 0, 1,
      1, 0, 0 },
    { 0, 1, 0,
      0, 1, 0,
      0, 1, 0 },
    { 0, 0, 1,
      1, 0, 0,
      0, 0, 1 },
  };
  
  mainSequencePattern(steps, 3, mainSequenceStepColors, 3, true);
}
void mainSequenceRowsDiagonal() {
  int steps[][9] = {
    { 1, 0, 0,
      0, 0, 0,
      0, 0, 0 },
    { 0, 1, 0,
      0, 0, 1,
      0, 0, 0 },
    { 0, 0, 1,
      0, 1, 0,
      1, 0, 0 },
    { 0, 0, 0,
      1, 0, 0,
      0, 1, 0 },
    { 0, 0, 0,
      0, 0, 0,
      0, 0, 1 },
  };
  
  mainSequencePattern(steps, 5, mainSequenceStepColors, 3, true);
}

void mainSequenceAlernate() {
  int steps[][9] = {
    { 1, 0, 1,
      0, 1, 0,
      1, 0, 1 },
    { 0, 1, 0,
      1, 0, 1,
      0, 1, 0 }
  };

  float colors[][3] = {
    { 1, 1, 0 }, 
    { 1, 0, 0 }
  };
  
  mainSequencePattern(steps, 2, colors, 2, true);
}

void mainSequenceCircle() {
  int steps[8][9] = {
    { 1, 0, 0,
      0, 1, 0,
      0, 0, 0 },
    { 0, 1, 0,
      0, 1, 0,
      0, 0, 0 },
    { 0, 0, 1,
      0, 1, 0,
      0, 0, 0 },
    { 0, 0, 0,
      1, 1, 0,
      0, 0, 0 },
    { 0, 0, 0,
      0, 1, 0,
      0, 0, 1 },
    { 0, 0, 0,
      0, 1, 0,
      0, 1, 0 },
    { 0, 0, 0,
      0, 1, 0,
      1, 0, 0 },
    { 0, 0, 0,
      0, 1, 1,
      0, 0, 0 },
  };

  mainSequencePattern(steps, 8, mainSequenceStepColors, 2, false);
  setColor(7, 1, 1, 1);
}

void mainSequenceRandom() {
  currentTime = millis();

  if (currentTime - mainPause > mainInterval) {
    mainPause = currentTime;
    for (int i=0; i<=9; i++) {
      mainPixelColors[i][0] = ((float) (rand() % 255)) / 128; // Red
      mainPixelColors[i][1] = ((float) (rand() % 255)) / 128; // Green
      mainPixelColors[i][2] = ((float) (rand() % 255)) / 128; // Blue
    }
  }
  for (int i=3; i<=11; i++) {
    setColor(
        i, 
        mainPixelColors[i - 2][0],    
        mainPixelColors[i - 2][1],    
        mainPixelColors[i - 2][2]);    
  }
}

void runEyes(int option) {
  currentTime = millis();
  
  if (currentTime - eyesPause > eyesInterval) {
    eyesPause = currentTime;
    switch (option) {
      case 0:  // Disabled
        eyesPixelColors[LEFT_EYE][0] = 0;
        eyesPixelColors[LEFT_EYE][1] = 0;
        eyesPixelColors[LEFT_EYE][2] = 0;
        eyesPixelColors[RIGHT_EYE][0] = 0;
        eyesPixelColors[RIGHT_EYE][1] = 0;
        eyesPixelColors[RIGHT_EYE][2] = 0;
        break;
      case 1:
        static int blink = 0;
        blink = !blink;
        if (blink) {
          eyesPixelColors[LEFT_EYE][0] = 0;
          eyesPixelColors[LEFT_EYE][1] = 0;
          eyesPixelColors[LEFT_EYE][2] = 1;

          eyesPixelColors[RIGHT_EYE][0] = 0;
          eyesPixelColors[RIGHT_EYE][1] = 0;
          eyesPixelColors[RIGHT_EYE][2] = 0; 
        } else {
          eyesPixelColors[LEFT_EYE][0] = 0;
          eyesPixelColors[LEFT_EYE][1] = 0;
          eyesPixelColors[LEFT_EYE][2] = 0;

          eyesPixelColors[RIGHT_EYE][0] = 0;
          eyesPixelColors[RIGHT_EYE][1] = 0;
          eyesPixelColors[RIGHT_EYE][2] = 1;          
        }
        break; 
      case 2:  // Blue state
        eyesPixelColors[LEFT_EYE][0] = 0;
        eyesPixelColors[LEFT_EYE][1] = 0;
        eyesPixelColors[LEFT_EYE][2] = 1;
        eyesPixelColors[RIGHT_EYE][0] = 0;
        eyesPixelColors[RIGHT_EYE][1] = 0;
        eyesPixelColors[RIGHT_EYE][2] = 1;
        break;
      case 3: // Random state
        eyesPixelColors[LEFT_EYE][0]  = ((float) (rand() % 255)) / 128; // Red
        eyesPixelColors[LEFT_EYE][1]  = ((float) (rand() % 255)) / 128; // Green
        eyesPixelColors[LEFT_EYE][2]  = ((float) (rand() % 255)) / 128; // Blue
        
        eyesPixelColors[RIGHT_EYE][0] = ((float) (rand() % 255)) / 128; // Red
        eyesPixelColors[RIGHT_EYE][1] = ((float) (rand() % 255)) / 128; // Green
        eyesPixelColors[RIGHT_EYE][2] = ((float) (rand() % 255)) / 128; // Blue
        break;
         
    }
  }

  setColor(LEFT_EYE, 
      eyesPixelColors[LEFT_EYE][0], 
      eyesPixelColors[LEFT_EYE][1], 
      eyesPixelColors[LEFT_EYE][2]);
  setColor(RIGHT_EYE, 
      eyesPixelColors[RIGHT_EYE][0], 
      eyesPixelColors[RIGHT_EYE][1], 
      eyesPixelColors[RIGHT_EYE][2]);
}

void runStatus(int option) {
  switch (option) {
    case 0: // Disabled
      return;
    case 1:
      statusSequenceBlink();
      break;     
    case 2:
      setColor(1, 0, 1, 0);
      break;      
    case 3:
      setColor(1, 1, 0, 0);
      break; 
  }
}

void statusSequenceBlink() {
  static float r = 1;
  static float g = 0;
  
  currentTime = millis();

  if (currentTime - statusPause > statusInterval) {
    statusPause = currentTime;

    if (r) {
      r = 0;
      g = 1;
    } else if (g) {
      r = 1;
      g = 0;
    }    
  }

  setColor(1, r, g, 0);
}

void runMain(int option) {
  mainInterval = 250;
  switch (option) {
    case 0:  // Disabled
      return;
    case 1: 
      mainInterval = 75;
      mainSequenceRowsDiagonal();
      break;
    case 2: 
      mainInterval = 100;
      mainSequenceRowsHorizontal();
      break;
    case 3: 
      mainInterval = 100;
      mainSequenceRowsVertical(); 
      break;
    case 4: mainSequenceAlernate(); break;
    case 5: mainSequenceCircle();   break;
    case 6: mainSequenceRandom();   break;
  }
}

void setColor(int pixel, float r, float g, float b) {
#define COLOR_MAX 255  // Lowered from 255 to 32 because too bright
  pixels.setPixelColor(
    pixel,
    pixels.Color((int)r * COLOR_MAX, (int)g * COLOR_MAX, (int)b * COLOR_MAX));
}
#undef COLOR_MAX
