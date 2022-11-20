/*R2 Lift System program
Written by Gold FTC comp. Tech*/
/************************************************
*               Included Libraries              *
*************************************************/

#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
Adafruit_PWMServoDriver servoControl = Adafruit_PWMServoDriver();

/************************************************
*               Macro Definitions               *
*************************************************/
#define SERVO_FREQ 50
#define OSCIL_FREQ 27000000
#define MPU 'E'
#define CMD_MAX_LENGTH 63
//define Adafruit PWM servo Pins and Limits

#define Z_ROT 1
#define Z_RMAX 150
#define Z_RMIN 700
#define Z_EXT 0
#define Z_EMAX 125
#define Z_EMIN 300
#define Z_PIE 2
#define Z_PMAX 600
#define Z_PMIN 150
#define LS_PIE 3
#define LS_PMAX 600
#define LS_PMIN 150
#define BM_PIE 4
#define BM_PMAX 600
#define BM_PMIN 150
#define LF_PIE 5
#define LF_PMAX 600
#define LF_PMIN 150


//define Motor Pins
#define Z_IN1 2
#define Z_IN2 3
#define LS_IN1 4
#define LS_IN2 5
#define P_IN1 6
#define P_IN2 7
#define BM_IN1 8
#define BM_IN2 9
#define LF_IN1 10
#define LF_IN2 11
//define Cont. Rotation servo pins
#define P_ROT 12
#define LF_ROT 13
//define the Output Enable Pin for the Adafruit servo driver
#define OE_PIN 34
//define Bad Motivator 74hc595 pins
#define BM_DATA 36
#define BM_LATCH 37
#define BM_CLOCK 38
//define Zapper LED pin
#define Z_LED 39
//define HALL effect pins
#define P_HALL 40
#define LF_HALL 41
//#define the Limit Switch pins
#define Z_TOP 44
#define Z_BOT 45
#define LS_TOP 46
#define LS_BOT 47
#define P_TOP 48
#define P_BOT 49
#define BM_TOP 50
#define BM_BOT 51
#define LF_TOP 52
#define LF_BOT 53

/************************************************
*                Global Variables               *
*************************************************/
Servo perServo;
Servo lfServo;
//We will Be using a multidimensional to control the lift system
//The following elements are contained in each device array:
/*  Element 0 - Motor 1 pin
    Element 1 - Motor 2 pin
    Element 2 - Top Limit Switch pin
    Element 3 - Bottem Limit Switch pin
    Element 4 - Servo 1 Adafruit pin (-1 if none)
    Element 5 - Servo 2 Adafruit pin (-1 if none)
    Element 6 - Hall Effect  pin (-1 if none)
    Element 7 - Pie Servo  pin (-1 if none)   */


int zapper[8] = { Z_IN1, Z_IN2, Z_TOP, Z_BOT, Z_ROT, Z_EXT, -1, Z_PIE };
int lSaber[8] = { LS_IN1, LS_IN2, LS_TOP, LS_BOT, -1, -1, -1, LS_PIE };
int periscope[8] = { P_IN1, P_IN2, P_TOP, P_BOT, -1, -1, P_HALL, -1 };
int badMotive[8] = { BM_IN1, BM_IN2, BM_TOP, BM_BOT, -1, -1, -1, BM_PIE };
int lifeForm[8] = { LF_IN1, LF_IN2, LF_TOP, LF_BOT, -1, -1, LF_HALL, LF_PIE };
int lifts[6][8];
int servoLmt[16][3];
char cmdStr[64];
/*
In addition we will be tracking the following states for each device:
  0 - P_STATE - Pie state (0 - closed; 1 - open; 2 - Pie not used)
  1 - M_STATE - Motor state (0 - stopped; 1 - moving)
  2 - TLS_STATE - Top Limit Switch state (0 - switch closed; 1 switch open)
  3 - BLS_STATE - Bottom Limit Switch state (0 - switch closed; 1 switch open)
  4 - ROT_STATE - Rotation State (0 - home or stopped; 1 - fully rotated or rotating; 2 - not used)
  5 - EXT_STATE - Extention state (0 - home or stopped; 1 - fully extended or hall effect hit; 2 - not used)
  6 - L_STATE - Lights routine state (0 - lights off; 1 - lights on)

Tracking is necessary to ensure that each task is completed before the next command is issued. 
And since we will be trying to multi-thread the processor, we will be doing each task a bit at a 
time.

At the start of the program most of the states will be 0 (or 2 if not used) as the default state.
However, the TLS_STATE for all devices should be 1.  Both the periscope and Life form scanner
EXT_STATE should be set to 1 as that is the stored position for the hall effect sensor  
*/
byte states[6][7];
byte leds = 0;
long int current_millis = millis();
long int zPrev_millis = current_millis;
long int z_raise_millis = current_millis;
long int z_servo_pmillis = current_millis;
long int bmPrev_millis = current_millis;
int zInterval = 50;
int z_raise_int = 100;
int z_servo_int = 100;
int bmInterval = 75;
int zState = 0;
int zFlashCount = 0;
char dev_MPU, dev_cmd;
int dev_addr, dev_opt;
int allLifts_State = 0;
int zapper_State = 0;
int lSaber_State = 0;
int periscope_State = 0;
int badMotive_State = 0;
int lifeForm_State = 0;


void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  servoControl.begin();
  servoControl.setOscillatorFrequency(OSCIL_FREQ);
  servoControl.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  int x;
  for (x = 0; x <= 7; x++) lifts[1][x] = zapper[x];
  for (x = 0; x <= 7; x++) lifts[2][x] = lSaber[x];
  for (x = 0; x <= 7; x++) lifts[3][x] = periscope[x];
  for (x = 0; x <= 7; x++) lifts[4][x] = badMotive[x];
  for (x = 0; x <= 7; x++) lifts[5][x] = lifeForm[x];

  // set Motor pins as output
  for (x = 1; x <= 5; x++) {
    pinMode(lifts[x][0], OUTPUT);
    pinMode(lifts[x][1], OUTPUT);
  }
  //set Limit Switch pins to use internal resister to pull them HIGH
  for (x = 1; x <= 5; x++) {
    pinMode(lifts[x][2], INPUT_PULLUP);
    pinMode(lifts[x][3], INPUT_PULLUP);
  }
  //Attach the servo objects to pins
  perServo.attach(P_ROT);
  lfServo.attach(LF_ROT);
  //set the Adafruit Servo Driver OE pin to output
  pinMode(OE_PIN, OUTPUT);
  //set 74HC595 pins to output
  pinMode(BM_LATCH, OUTPUT);
  pinMode(BM_DATA, OUTPUT);
  pinMode(BM_CLOCK, OUTPUT);
  //set Zapper LED to output;
  pinMode(Z_LED, OUTPUT);
  //Set the OE_PIN to LOW to enable output on the Adafruit servo driver
  digitalWrite(OE_PIN, LOW);
  updateStates();
  leds = 0;
  updateShiftRegister();
  states[2][0] = 1;
}


void loop() {
  checkSerial();
  allLifts(allLifts_State);
  ZapLift(zapper_State);
  LSLift(lSaber_State);
  PLift(periscope_State);
  BMLift(badMotive_State);
}
//The allLifts function does three things depending on the opt option
//  0 - AllLifts is disabled (default); 1 - All lifts are moved up; 2 - All lifts are moved down;
void allLifts(int opt) {

  switch (opt) {
    case 0:  //allLifts disabled
      return;
      break;
    case 1:  //All Lifts up
      if(motorUp(1) && motorUp(2) && motorUp(3) && motorUp(4) && motorUp(5)){
          allLifts_State=0;
      }
      return;
      break;
    case 2:  //All lifts down
      if(motorDown(1) && motorDown(2) && motorDown(3) && motorDown(4) && motorDown(5)){
          allLifts_State=0;
      }
      return;
      break;
   
  }
}

//BM_Raise raises the Bad motivator and returns a 0 while rising and a 1 when raised
byte BM_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      moveServo(BM_PIE, BM_PMIN, BM_PMAX);
      states[4][0] = 1;
      step = 1;
      break;
    case 1:  //Move lift up
      if (motorUp(4)) step = 2;
      break;
    case 2:
      step = 0;
      return 1;
      break;
  }
  return 0;
}

//BM_Lower lowers the Bad Motivator and returns a 0 when lowering and a 1 when down.
byte BM_Lower() {
  bmLights(0);
  static int step = 0;
  switch (step) {
    case 0:  //Move Motor
      if (motorDown(4)) step = 1;
      break;
    case 1:  //Close the pie
      moveServo(BM_PIE, BM_PMAX, BM_PMIN);
      states[4][0] = 0;
      step = 2;
    case 2:              //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}


//BMLift is the sixth thread of seven threads in this program.  It handles all interactions with the 
//Bad Motivator.  It is controlled by the badMotive_State.
void BMLift(int option) {
  //Serial.println("Light saber");
  switch (option) {
    case 0:
      //default - No Action
      if(digitalRead(BM_BOT)) bmLights(1);
      return 0;
    case 1:  //Raise LSaber
      if(BM_Raise()==1) badMotive_State=0;
      break;
    case 2:  //Light Saber down
      if(BM_Lower()==1) badMotive_State=0;
      break;
  }
}


//The bmLights function controls the 8 leds in the Bad Motivator.  The BM uses a 74HC595 shift register chip to control
//the eight lights with three pins. Passing a 0 to the function turns the ligts off, a 1 turns them on.
void bmLights(int num) {
  if (num) {
    current_millis = millis();
    if (current_millis - bmPrev_millis > bmInterval) {
      bmPrev_millis = current_millis;
      leds = random(0, 255);
      updateShiftRegister();
    }
  } else {
    leds = 0;
    updateShiftRegister();
  }
  return;
}


//The buildCommand takes the current byte from the Serial buffer and builds a command for processing.  It returns a 0 
//while in the building process and a 1 when the command is ready.
int buildCommand(char ch, char* output_str) {
  static int pos = 0;
  switch (ch) {
    case '\n':
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



int doTcommand(int addr, int opt) {
  Serial.println("T command");
  switch (addr) {
    case 50:
      allLifts_State = opt;
      break;
    case 51:
      zapper_State = opt;
      break;
    case 52:
      lSaber_State = opt;
      break;
    case 53:
      periscope_State = opt;
      break;
    case 54:
      badMotive_State = opt;
      break;
    case 55:
      lifeForm_State = opt;
      break;
  }
}

byte LS_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      moveServo(LS_PIE, LS_PMIN, LS_PMAX);
      states[2][0] = 1;
      step = 1;
      break;
    case 1:  //Move lift up
      if (motorUp(2)) step = 2;
      break;
    case 2:
      step = 0;
      return 1;
      break;
  }
  return 0;
}

byte LS_Lower() {

  static int step = 0;
  switch (step) {
    case 0:  //Move Motor
      if (motorDown(2)) step = 1;
      break;
    case 1:  //Close the pie
      moveServo(LS_PIE, LS_PMAX, LS_PMIN);
      states[2][0] = 0;
      step = 2;
    case 2:              //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}

void LSLift(int option) {
  //Serial.println("Light saber");
  switch (option) {
    case 0:
      //default - No Action
      return 0;
    case 1:  //Raise LSaber
      if(LS_Raise()==1) lSaber_State=0;
      break;
    case 2:  //Light Saber down
      if(LS_Lower()==1) lSaber_State=0;
      break;
  }
}


// The loadServos function loads min and max values into the servoLmt array
void loadServos() {
  //list each servos min and max
  servoLmt[0][0] = Z_RMIN;
  servoLmt[0][1] = Z_RMAX;
  servoLmt[1][0] = Z_EMIN;
  servoLmt[1][1] = Z_EMAX;
  servoLmt[2][0] = Z_PMIN;
  servoLmt[2][1] = Z_PMAX;
  servoLmt[3][0] = LS_PMIN;
  servoLmt[3][1] = LS_PMAX;
  servoLmt[4][0] = BM_PMIN;
  servoLmt[4][1] = BM_PMAX;
  servoLmt[5][0] = LF_PMIN;
  servoLmt[5][1] = LF_PMAX;
  return;
}

//Moves the specified mtr until it reaches the top limit switch.
byte motorDown(int mtr) {
  if (digitalRead(lifts[mtr][3])) {
    analogWrite(lifts[mtr][1], 255);
    states[mtr][1] = 1;
    states[mtr][3] = 1;
    return 0;  //Motor is moving
  } else {
    digitalWrite(lifts[mtr][1], LOW);
    states[mtr][1] = 0;
    states[mtr][3] = 0;
    return 1;  //Motor is stopped
  }
}

//Moves the specified mtr until it reaches the top limit switch.
byte motorUp(int mtr) {
  if (digitalRead(lifts[mtr][2])) {
    analogWrite(lifts[mtr][0], 255);
    states[mtr][1] = 1;
    states[mtr][2] = 1;
    return 0;  //Motor is moving

  } else {
    digitalWrite(lifts[mtr][0], LOW);
    states[mtr][1] = 0;
    states[mtr][2] = 0;
    return 1;  //Motor is stopped
  }
  return;
}


void moveServo(int srvNo, int from, int to) {
  if (to > from) {
    for (int pulselen = from; pulselen < to; pulselen++) {
      servoControl.setPWM(srvNo, 0, pulselen);
    }
  } else {
    for (int pulselen = from; pulselen > to; pulselen--) {
      servoControl.setPWM(srvNo, 0, pulselen);
    }

    return;
  }
}

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
    /*case 'D':                           // D command is weird, does not need an argument, ignore if it has one
      doDcommand(address);
      break;
    case 'P':    
      if(!hasArgument) goto beep;       // invalid, no argument after command
      doPcommand(address, argument);
      break;
    case 'R':    
      if(!hasArgument) goto beep;       // invalid, no argument after command
      doRcommand(address, argument);
      break; */
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

byte P_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Move lift up
      if (motorUp(3)) step = 1;
      break;
    case 1:
      perServo.write(179);
      step=2;
      break;
    case 2:
      step = 0;
      return 1;
      break;
  }
  return 0;
}


byte P_Lower() {
  static int step = 0;
  switch (step) {
    case 0:  //Move Motor
      if (motorDown(3)) step = 1;
      break;
    case 1:              //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}

void PLift(int option) {
  //Serial.println("Light saber");
  switch (option) {
    case 0:
      //default - No Action
      if(digitalRead(P_BOT)) bmLights(1);
      return 0;
    case 1:  //Raise Periscope
      if(P_Raise()==1) periscope_State=0;
      break;
    case 2:  //Light Saber down
      if(P_Lower()==1) periscope_State=0;
      break;
  }
}


// The bad motivator uses a 74HC595 chip to control its 8 lights.  The updateShiftRegister
// function uses the binary value of leds to update the lights
void updateShiftRegister() {
  digitalWrite(BM_LATCH, LOW);
  shiftOut(BM_DATA, BM_CLOCK, LSBFIRST, leds);
  digitalWrite(BM_LATCH, HIGH);
}

//The updateStates function sets all states to their default values
void updateStates() {
  for (int x = 0; x < 6; x++) {
    for (int y = 0; y < 7; y++) states[x][y] = 0;
  }  // sets entire states array to zero;
  for (int x = 0; x < 6; x++) {
    states[x][3] = digitalRead(lifts[x][3]);  //sets the BLS_STATE to current state
    states[x][2] = digitalRead(lifts[x][2]);  //sets the TLS_STATE to current state
  }
  //Turn off States that will not be used
  states[2][4] = 2;  //ROT_STATE for light saber
  states[2][5] = 2;  //EXT_STATE for light saber
  states[2][6] = 2;  //L_STATE for light saber
  states[4][4] = 2;  //ROT_STATE for bad motivator
  states[4][5] = 2;  //EXT_STATE for bad moitvator
  states[5][6] = 2;  //L_STATE for Lift Form Scanner
  //Read current state of Hall Effect sensor
  states[3][5] = digitalRead(P_HALL);
  states[5][5] = digitalRead(LF_HALL);
  loadServos();
}


void ZapLift(int option) {
  static int step = 0;
  long int z_servo_pmillis;
  int z_servo_int = 100;
  if (option == 7) setTimer(1, 250);
  current_millis = millis();
  switch (option) {
    case 0:  //ZapLift state unchanged;
      step = 0;
      return;
      break;
    case 1:  //Raise lift
      if (Z_Raise()) {
        zapper_State = 0;
        return;
      }
      break;
    case 2:  //Move Zapper to stored position
      if (Z_Lower()) {
        zapper_State = 0;
        return;
      }
      break;
    case 3:  //Rotate Zapper Open
             //if (digitalRead(Z_TOP) == LOW) {
      moveServo(Z_ROT, Z_RMIN, Z_RMAX);
      //}
      zapper_State = 0;
      break;
    case 4:  //Rotate Zapper closed
             //if (digitalRead(Z_TOP) == LOW) {
      moveServo(Z_ROT, Z_RMAX, Z_RMIN);
      //}
      zapper_State = 0;
      break;
    case 5:  //Extend Zapper
      moveServo(Z_EXT, Z_EMIN, Z_EMAX);
      zapper_State = 0;
      break;
    case 6:  //Fold Zapper

      moveServo(Z_EXT, Z_EMAX, Z_EMIN);
      zapper_State = 0;
      break;
    case 7:  //Zap!
      if (ZapLights(1) == 1) {
        ZapLights(0);
        zapper_State = 0;
      }
      break;
  }
}



byte ZapLights(int num) {
  if (num) {
    switch (zState) {
      case 0:
        current_millis = millis();
        digitalWrite(Z_LED, HIGH);  // sets the led HIGH
        if (current_millis - zPrev_millis >= zInterval) {
          zPrev_millis = current_millis;
          zState = 1;
        }
        break;
      case 1:
        current_millis = millis();
        digitalWrite(Z_LED, LOW);  // sets the led LOW
        if (current_millis - zPrev_millis >= zInterval) {
          zPrev_millis = current_millis;
          zState = 2;
        }
        break;
      case 2:
        current_millis = millis();
        zFlashCount++;
        if (zFlashCount == 8) {
          zState = 0;
          zFlashCount = 0;
          return 1;
        } else {
          zState = 0;
        }
        break;
    }
  } else digitalWrite(Z_LED, LOW);
  return 0;
}

int setTimer(int num, int interval) {
  if (num) {
    current_millis = millis();
    z_raise_millis = current_millis;
    z_raise_int = interval;
    return 0;
  } else {
    current_millis = millis();
    if (current_millis - z_raise_millis > z_raise_int) return 1;
    else return 0;
  }
}
byte Z_Lower() {

  static int step = 0;
  switch (step) {
    case 0:  //Lights are off;
      ZapLights(0);
      step = 1;
      break;
    case 1:  //Zapper folded
      if (states[1][5]) {
        moveServo(Z_EXT, Z_EMAX, Z_EMIN);
        states[1][5] = 0;
        step = 2;
        setTimer(1, 150);  //Set timer for 150 milliseconds
      } else step = 3;
      break;
    case 2:  //Wait for Zapper to fold
      if (setTimer(0, 150)) step = 3;
      break;
    case 3:  //Rotate the Zapper
      if (states[1][4]) {
        moveServo(Z_ROT, Z_RMAX, Z_RMIN);
        states[1][4] = 0;
        step = 4;
        setTimer(1, 150);  //Set timer for 150 milliseconds
      } else step = 5;
      break;
    case 4:  //Wait for Zapper to rotate
      if (setTimer(0, 150)) step = 5;
      break;
    case 5:  //Move Motor
      if (motorDown(1)) step = 6;
      break;
    case 6:  //Close the pie
      moveServo(Z_PIE, Z_PMAX, Z_PMIN);
      states[1][0] = 0;
      step = 7;
      setTimer(1, 150);  //Set timer for 150 milliseconds
    case 7:              //Final cleanup and return 1 for a job well done
      step = 0;
      Serial.println("Here");
      return 1;
      break;
  }
  return 0;
}

byte Z_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      moveServo(Z_PIE, Z_PMIN, Z_PMAX);
      states[1][0] = 1;
      step = 1;
      break;
    case 1:  //Move lift up
      if (motorUp(1)) step = 2;
      break;
    case 2:  //Rotate the zapper
      moveServo(Z_ROT, Z_RMIN, Z_RMAX);
      states[1][4] = 1;
      step = 3;
      break;
    case 3:  //Extend the zapper
      moveServo(Z_EXT, Z_EMIN, Z_EMAX);
      states[1][5] = 1;
      step = 4;
      setTimer(1, 500);
      break;
    case 4:  //Wait for everything to settle
      if (setTimer(0, 500)) step = 5;
      break;
    case 5:  //Light the zapper
      if (ZapLights(1) == 1) {
        ZapLights(0);
        states[1][6] = 0;

        step = 0;
        return 1;
      }
      break;
  }



  return 0;
}
