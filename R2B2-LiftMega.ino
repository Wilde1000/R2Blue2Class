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
#define Z_RMID 425
#define Z_RMIN 700
#define Z_EXT 0
#define Z_EMAX 125
#define Z_EMIN 300
#define Z_EMID 213
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
char cmdStr1[64];

byte leds = 0;
long int current_millis = millis();
long int zPrev_millis = current_millis;
long int z_raise_millis = current_millis;
long int z_lights_pmillis = current_millis;
long int bmPrev_millis = current_millis;
int zInterval = 50;
int z_raise_int = 100;
int z_lights_int = 50;
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
  leds = 0;
  updateShiftRegister();
}


void loop() {
  checkSerial();  //Check Serial 0 for commands
  checkSerial1(); //Check Serial 1 for commands
  allLifts(allLifts_State);  //Run actions on all lifts if any
  ZapLift(zapper_State);  // Run actions on the Zapper if any
  LSLift(lSaber_State);  //Run actions on the Light Saber if any
  PLift(periscope_State);  //Run actions on the Periscope if any
  BMLift(badMotive_State); //Run actions on the Bad Motivator if any
  LFLift(lifeForm_State);  //Run actions on the Life Form Scanner if any
}

//The allLifts function does three things depending on the opt option
//  0 - AllLifts is disabled (default); 1 - All lifts are moved up; 2 - All lifts are moved down;
void allLifts(int opt) {
  switch (opt) {
    case 0:  //allLifts disabled
      return;
      break;
    case 1:  //All Lifts up
      for (int x = 1; x < 6; x++) {
        motorUp(x);
      }
      break;
    case 2:  //All lifts down
      for (int x = 1; x < 6; x++) {
        motorDown(x);
      }
      break;
    case 3:  //All lifts up sequentially
      if (motorUp(1) && motorUp(2) && motorUp(3) && motorUp(4) && motorUp(5)) allLifts_State = 0;
      break;
    case 4:  //All lifts down sequentially
      if (motorDown(1) && motorDown(2) && motorDown(3) && motorDown(4) && motorDown(5)) allLifts_State = 0;
      break;
  }

  return;
}

//BM_Raise raises the Bad motivator and returns a 0 while rising and a 1 when raised
byte BM_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      moveServo(BM_PIE, BM_PMIN, BM_PMAX);
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
      step = 2;
      break;
    case 2:  //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}


//BMLift is the sixth thread of seven threads in this program.  It handles all interactions with the
//Bad Motivator.  It is controlled by the badMotive_State.
void BMLift(int option) {
  switch (option) {
    case 0:
      //default - No Action
      if (digitalRead(BM_BOT)) bmLights(1);
      return 0;
      break;
    case 1:  //Raise LSaber
      if (BM_Raise() == 1) badMotive_State = 0;
      break;
    case 2:  //Light Saber down
      if (BM_Lower() == 1) badMotive_State = 0;
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


//The buildCommand takes the current byte from the Serial0 buffer and builds a command for processing.  It returns a 0
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

//The buildCommand1 takes the current byte from the Serial1 buffer and builds a command for processing.  It returns a 0
//while in the building process and a 1 when the command is ready.
int buildCommand1(char ch, char* output_str) {
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

//The checkSerial1 function is the second thread of seven threads in this program.  It checks the Serial1 buffer for incoming serial
//data and then sends it to be processed.
void checkSerial1() {
  char ch;
  byte cmd_Complete;
  if (Serial1.available()) {
    ch = Serial1.read();
    cmd_Complete = buildCommand1(ch, cmdStr1);
    if (cmd_Complete) {
      parseCommand(cmdStr1);
    }
  }
}


//The doTcommand handles all T commands sent from the parseCommand function
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

//Raises the Life Form Scanner - Return 0 while raising and 1 when raised
byte LF_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Move lift up
      if (motorUp(5)) step = 1;
      break;
    case 1:
      lfServo.write(180);
      step = 2;
      break;
    case 2:
      step = 0;
      return 1;
      break;
  }
  return 0;
}

//Lowers the Life Form Scanner - Returns 0 while raising and 1 when raised
byte LF_Lower() {
  static int step = 0;
  switch (step) {
    case 0:  //Move Motor
      if (motorDown(5)) step = 1;
      break;
    case 1:  //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}
//LFLift is the last thread of seven threads in this program.  It handles all interactions with the
//Life Form Scanner.  It is controlled by the lifeForm_State.
void LFLift(int option) {
  switch (option) {
    case 0:
      //default - No Action
      return 0;
    case 1:  //Raise Life Form Scanner
      if (LF_Raise() == 1) lifeForm_State = 0;
      break;
    case 2:  //lower Life Form Scanner
      if (LF_Lower() == 1) lifeForm_State = 0;
      break;
  }
}


//The LS_Raise function raises the Light Saber and returns a 0 while raising and a 1 when raised
byte LS_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      moveServo(LS_PIE, LS_PMIN, LS_PMAX);
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

//The LS_Lower function lowers the Light Saber and returns a 0 while lowering and a 1 when down
byte LS_Lower() {

  static int step = 0;
  switch (step) {
    case 0:  //Move Motor
      if (motorDown(2)) step = 1;
      break;
    case 1:  //Close the pie
      moveServo(LS_PIE, LS_PMAX, LS_PMIN);
      step = 2;
    case 2:  //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}
//LSLift is the fourth thread of seven threads in this program.  It handles all interactions with the
//Light Saber Lift.  It is controlled by the lSaber_State.
void LSLift(int option) {
  //Serial.println("Light saber");
  switch (option) {
    case 0:
      //default - No Action
      return 0;
    case 1:  //Raise LSaber
      if (LS_Raise() == 1) lSaber_State = 0;
      break;
    case 2:  //Light Saber down
      if (LS_Lower() == 1) lSaber_State = 0;
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

//Moves the specified mtr until it reaches the bottom limit switch. Returns a 0 when the motor is moving and a 1 when the limit switch is hit.
byte motorDown(int mtr) {
  if (digitalRead(lifts[mtr][3])) {
    analogWrite(lifts[mtr][1], 255);
    return 0;  //Motor is moving
  } else {
    digitalWrite(lifts[mtr][1], LOW);
    return 1;  //Motor is stopped
  }
}

//Moves the specified mtr until it reaches the top limit switch. Returns a 0 when the motor is moving and a 1 when the limit switch is hit.
byte motorUp(int mtr) {
  if (digitalRead(lifts[mtr][2])) {
    analogWrite(lifts[mtr][0], 255);
    return 0;  //Motor is moving
  } else {
    digitalWrite(lifts[mtr][0], LOW);
    return 1;  //Motor is stopped
  }
  return 0;
}

// Move servo from the passed from to the passed to
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
//Raises the periscope - Return 0 while raising and 1 when raised
byte P_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Move lift up
      if (motorUp(3)) step = 1;
      break;
    case 1:
      perServo.write(179);
      step = 2;
      break;
    case 2:
      step = 0;
      return 1;
      break;
  }
  return 0;
}

//Lowers the periscope - Returns 0 while raising and 1 when raised
byte P_Lower() {
  static int step = 0;
  switch (step) {
    case 0:  //Move Motor
      if (motorDown(3)) step = 1;
      break;
    case 1:  //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}


//PLift is the fifth thread of seven threads in this program.  It handles all interactions with the
//Periscope.  It is controlled by the periscope_State.
void PLift(int option) {
  //Serial.println("Light saber");
  switch (option) {
    case 0:
      //default - No Action
      if (digitalRead(P_BOT)) bmLights(1);
      return 0;
    case 1:  //Raise Periscope
      if (P_Raise() == 1) periscope_State = 0;
      break;
    case 2:  //lower Periscope
      if (P_Lower() == 1) periscope_State = 0;
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


//ZapLift is the third thread of seven threads in this program.  It handles all interactions with the
//Zapper.  It is controlled by the zapper_State.
void ZapLift(int option) {
  static int step = 0;
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
      moveServo(Z_ROT, Z_RMIN, Z_RMAX);
      zapper_State = 0;
      break;
    case 4:  //Rotate Zapper closed
      moveServo(Z_ROT, Z_RMAX, Z_RMIN);
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


//ZapLights operates the Zap light on the zapper. Pass a 0 to shut off and a positive number to run
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


int setZLTimer(int num, int interval) {
  if (num) {
    current_millis = millis();
    z_lights_pmillis = current_millis;
    z_lights_int = interval;
    return 0;
  } else {
    current_millis = millis();
    if (current_millis - z_lights_pmillis > z_lights_int) return 1;
    else return 0;
  }
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
      moveServo(Z_EXT, Z_EMAX, Z_EMIN);
      step = 2;
      break;
    case 2:  //Rotate the Zapper
      moveServo(Z_ROT, Z_RMAX, Z_RMIN);
      step = 3;
      break;
    case 3:  //Move Motor
      if (motorDown(1)) step = 4;
      break;
    case 4:  //Close the pie
      moveServo(Z_PIE, Z_PMAX, Z_PMIN);
      step = 5;
      break;
    case 5:  //Final cleanup and return 1 for a job well done
      step = 0;
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
      step = 1;
      break;
    case 1:  //Move lift up
      if (motorUp(1)) step = 2;
      break;
    case 2:  //Rotate the zapper
      moveServo(Z_ROT, Z_RMIN, Z_RMAX);
      step = 3;
      break;
    case 3:  //Extend the zapper
      moveServo(Z_EXT, Z_EMIN, Z_EMAX);
      step = 4;
      setTimer(1, 500);
      break;
    case 4:  //Wait for everything to settle
      if (setTimer(0, 500)) step = 5;
      break;
    case 5:  //Light the zapper
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 0;
        return 1;
      }
      break;
  }
  return 0;
}

byte zapSeq1(){
  static int step=0;
  switch(step){
    case 0:  //Open pie
      moveServo(Z_PIE, Z_PMIN, Z_PMAX);
      step=1;
      break;
    case 1: //Raise Zapper
      if(motorUp(1)) step=2;
      break;
    case 2: //Extend Zapper
      moveServo(Z_EXT, Z_EMIN, Z_EMAX);
      setTimer(1, 500);
      step=3;
      break;
    case 3:
      if(setTimer(0,500)) step=4;
      break;
    case 4:
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 5;
        setTimer(1,500);
      }
      break;
    case 4: //Move Servo to zap Position 2
      moveServo(Z_ROT, Z_RMIN, Z_RMID);
      moveServo(Z_EXT, Z_EMAX, Z_EMID);
      setTimer(1, 500);
      step = 5;
      break;
    case 5: //Wait for dust to settle
      if(setTimer(0,500)) step=6;
      break;
    case 6: //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 7;
        setTimer(1,500);
      }
      break;
    case 7: //Move to position three
      moveServo(Z_ROT, Z_RMID, Z_RMAX);
      moveServo(Z_EXT, Z_EMID, Z_EMAX);
      setTimer(1, 500);
      step = 8;
      break;
    case 8: //Wait for dust to settle
      if(setTimer(0,500)) step=9;
      break;
    case 9: //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 10;
        setTimer(1,500);
      }
      break;
    case 10: //Move to position four
      moveServo(Z_EXT, Z_EMAX, Z_EMID);
      setTimer(1, 500);
      step = 11;
      break;
    case 11: //Wait for dust to settle
      if(setTimer(0,500)) step=12;
      break;
    case 12: //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 13;
        setTimer(1,500);
      }
      break;
    case 13: //Move to position five
      moveServo(Z_ROT, Z_RMAX, Z_RMID);
      moveServo(Z_EXT, Z_EMID, Z_EMAX);
      setTimer(1, 500);
      step = 14;
      break;
    case 14: //Wait for dust to settle
      if(setTimer(0,500)) step=15;
      break;
    case 15: //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 16;
        setTimer(1,500);
      }
      break;
    case 16: //Move to position six
      moveServo(Z_ROT, Z_RMID, Z_RMIN);
      moveServo(Z_EXT, Z_EMAX, Z_EMID);
      setTimer(1, 500);
      step = 17;
      break;
    case 17: //Wait for dust to settle
      if(setTimer(0,500)) step=18;
      break;
    case 18: //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 19;
        setTimer(1,500);
      }
      break;
    case 19: //Move to open position 
      moveServo(Z_ROT, Z_RMID, Z_RMAX);
      moveServo(Z_EXT, Z_EMID, Z_EMAX);
      setTimer(1, 1000);
      step = 20;
      break;
    case 20: //Fold the Zapper and return to store position
      moveServo(Z_ROT, Z_RMAX, Z_RMIN);
      moveServo(Z_EXT, Z_EMAX, Z_EMIN);
      step = 21;
      break;
    case 21: //Move the Lift down
      if(motorDown(1)) step=22;
      break;
    case 22: //Close Pie 
      moveServo(Z_PIE, Z_PMAX, Z_PMIN);
      step=23;
      break;
    case 23:
      step=0;
      zapper_state=0;
      return 1;
      break;
  }
  return 0;
}
