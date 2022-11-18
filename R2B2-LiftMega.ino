/*R2 Lift System program
Written by Gold FTC comp. Tech*/
/************************************************
*               Included Libraries              *
*************************************************/

#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver servoControl = Adafruit_PWMServoDriver();

/************************************************
*               Macro Definitions               *
*************************************************/
#define SERVO_FREQ 50
#define OSCIL_FREQ 27000000

//define Adafruit PWM servo Pins and Limits

#define Z_ROT 0
#define Z_RMAX 4096
#define Z_RMIN 50
#define Z_EXT 1
#define Z_EMAX 4096
#define Z_EMIN 50
#define Z_PIE 2
#define Z_PMAX 4096
#define Z_PMIN 50
#define LS_PIE 3
#define LS_PMAX 4096
#define LS_PMIN 50
#define BM_PIE 4
#define BM_PMAX 4096
#define BM_PMIN 50
#define LF_PIE 5
#define LF_PMAX 4096
#define LF_PMIN 50


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
long int zPrev_millis = millis();
long int bmPrev_millis = millis();
int zInterval = 50;
int bmInterval = 75;
int zState = 0;
int zFlashCount = 0;


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




void bmLights(int num) {
 if(num){
  current_millis()=millis();
  if(current_millis-bmPrev_millis>bmInterval){
    bmPrev_millis=current_millis;
    leds = random(0, 255);
    updateShiftRegister();
  }
 }else{
    leds = 0;
    updateShiftRegister();
 }
  return;
}


// The bad motivator uses a 74HC595 chip to control its 8 lights.  The updateShiftRegister
// function uses the binary value of leds to update the lights
void updateShiftRegister() {
  digitalWrite(BM_LATCH, LOW);
  shiftOut(BM_DATA, BM_CLOCK, LSBFIRST, leds);
  digitalWrite(BM_LATCH, HIGH);
}


//Moves the specified mtr until it reaches the top limit switch.
void motorUp(int mtr) {
  if (digitalRead(lifts[mtr][2])) {
    analogWrite(lifts[mtr][0], 255);
    states[mtr][1] = 1;
    states[mtr][2] = 1;

  } else {
    digitalWrite(lifts[mtr][0], LOW);
    states[mtr][1] = 0;
    states[mtr][2] = 0;
  }
  return;
}

//Moves the specified mtr until it reaches the top limit switch.
void motorDown(int mtr) {
  if (digitalRead(lifts[mtr][3])) {
    analogWrite(lifts[mtr][1], 255);
    states[mtr][1] = 1;
    states[mtr][3] = 1;

  } else {
    digitalWrite(lifts[mtr][1], LOW);
    states[mtr][1] = 0;
    states[mtr][3] = 0;
  }
  return;
}

void setup() {
  Serial.begin(9600);
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
  //set 74HC595 pins to output
  pinMode(BM_LATCH, OUTPUT);
  pinMode(BM_DATA, OUTPUT);
  pinMode(BM_CLOCK, OUTPUT);
  //set Zapper LED to output;
  pinMode(Z_LED, OUTPUT);
  
  
  updateStates();
  //motorUp(1);
  //motorUp(2);
  //motorUp(3);
  //motorUp(4);
  //motorUp(5);
}


void loop() {
}

void ZapLights(int num) {
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
        if (zFlashCount == 80) {
          zState = 3;
          zFlashCount = 0;
        } else {
          zState = 0;
        }
        break;
    }
  }else digitalWrite(Z_LED, LOW);
  return;
}

void ZapLift(int option) {

  switch (option) {
    case 0:
      //Returns Zapper to stored closed position
      if (states[1][0] == 0) return;  //Pie is closed - Zapper should be stored
      else if (states[1][3] == 0) {   //Bottom limit switch is closed - Close the pie
        servoControl.setPWM(lifts[1][7], 0, servoLmt[lifts[1][7]][0]);
        states[1][0] = 0;
        return;                   //Zapper is stored
      } else if (states[1][6]) {  //Are the lights on?
        ZapLights(0);
        states[1][6] = 0;
        return;                   //Lights are off
      } else if (states[1][5]) {  //Is Zapper extended?
        servoControl.setPWM(lifts[1][5], 0, servoLmt[lifts[1][5]][0]);
        states[1][5] = 0;  //Zapper is folded
        return;
      } else if (states[1][4]) {  //Is zapper turned for lowering?
        servoControl.setPWM(lifts[1][4], 0, servoLmt[lifts[1][4]][0]);
        states[1][4] = 0;
        return;
      } else if (states[1][2] == 0) {  //Ready to start Motor?
        motorDown(1);                  //Start Motor
        states[1][2] = 1;
        
        return;
      } else if (states[1][1]) {  //Motor is moving
        motorDown(1);
        return;
      }
      break;
    case 1:                     //Raise the Zapper, rotate, extend and start zapping
      if (states[1][0] == 0) {  //Pie is closed - Zapper is stored
        //open the pie
        servoControl.setPWM(lifts[1][7], 0, servoLmt[lifts[1][7]][1]);
        states[1][0] = 1;  //Pie is open
        return;
      } else if (states[1][3] == 0) {  //Zapper is stored
        motorUp(1);
        states[1][3] = 1;
        // Motor is Moving
        return;
      } else if (states[1][1]) {
        //motor is still moving
        motorUp(1);
        return;
      } else if (states[1][4] == 0) {  //Rotate Zapper
        //turn the Rotate servo
        servoControl.setPWM(lifts[1][4], 0, servoLmt[lifts[1][4]][1]);
        states[1][4] = 1;  //Zapper is rotated
        return;
      } else if (states[1][5]) {  // Extend Zapper
        //turn the Extention servo
        servoControl.setPWM(lifts[1][5], 0, servoLmt[lifts[1][5]][1]);
        states[1][5] = 1;  //Zapper is extended
        return;
      } else if (states[1][6] == 0) {  //turn on Light routine
        ZapLights(1);
        states[1][6] = 1;
        return;
      } else {
        ZapLights(1);
        return;
      }
      break;
    case 2:  //Extend the Zapper
      //In order to safely extend the zapper, the zapper must be up and unextended.
      if (states[1][2] == 0 && states[1][5] == 0) {
        //Extend the zapper
        servoControl.setPWM(lifts[1][5], 0, servoLmt[lifts[1][5]][1]);
        states[1][5] = 1;
      }
      ZapLights(1);
      return;
      break;
    case 3:  //Retract the Zapper
      if (states[1][2] == 0 && states[1][5]) {
        //Retract the zapper
        servoControl.setPWM(lifts[1][5], 0, servoLmt[lifts[1][5]][0]);
        states[1][5] = 0;
      }
      ZapLights(0);
      return;
      break;
    case 4:  //Rotate the Zapper open
      if (states[1][2] == 0 && states[1][4] == 0) {
        //Rotate the zapper
        servoControl.setPWM(lifts[1][4], 0, servoLmt[lifts[1][4]][1]);
        states[1][4] = 1;
      }
      ZapLights(1);
      return;
      break;
    case 5:  //Rotate the Zapper closed
      if (states[1][2] == 0 && states[1][4]) {
        //Rotate the zapper
        servoControl.setPWM(lifts[1][4], 0, servoLmt[lifts[1][4]][0]);
        states[1][4] = 0;
      }
      ZapLights(0);
      return;
      break;
    case 6:  //Raise the Zapper - Zapper must be down and Pie open
      if (states[1][0] && states[1][3] == 0) {
        motorUp(1);
        states[1][3] = 1;
        return;
      } else if (states[1][1]) {
        motorUp(1);
        return;
      }
      return;
      break;
    case 7:  //Lower the Zapper - Zapper must be up, folded, and rotated
      if (states[1][2] == 0 && states[1][4] == 0 && states[1][5] == 0) {
        motorDown(1);
        states[1][2] = 1;
        ZapLights(0);
        states[1][6] = 0;
        return;
      } else if (states[1][1]) {
        motorDown(1);
        return;
      }
      return;
      break;
  }
}

void lsLift(int option){
    switch(option){
        case 0: // Light saber down and pie closed
            if(states[2][0]==0) return; //Pie is closed - LS is stored
            else if (digitalRead(LS_BOT)==0){ //Bottom Limit Hit
                servoControl.setPWM(LS_PIE, 0, LS_PMIN);
                states[2][0]=0;
                return;  //Pie is closed
            }else if(digitalRead(LS_TOP) == 0){
                motorDown(2);
                states[2][2]=1;
                return;
            }else if(states[2][1]){
                motorDown(2);
                return;
            }
            break;
         case 1:  //Light Saber up
            if(digitalRead(LS_TOP)==0) return;
            else if (states[2][0] == 0){  //Pie is closed
                servoControl.setPWM(LS_PIE, 0 LS_PMAX);
                states[2][0]=1;
                return;
            }else if(digitalRead(LS_BOT)==0 || (digitalRead(LS_BOT) && digitalRead(LS_TOP))){
                motorUp(2);
                states[2][3]=1;
                return;
            }else if(states[2][1]){
                motorUp(2);
                return;
            }
            
            
            
    }
    
}











