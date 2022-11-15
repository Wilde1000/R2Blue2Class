/*R2 Lift System program
Written by Gold FTC comp. Tech*/
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver servoCon = Adafruit_PWMServoDriver();
#define SERVO_FREQ 50
 
//define Adafruit PWM servo Pins and Limits
 
#define Z_ROT 0
#define Z_RMAX 4096
#define Z_RMIN 50
#define Z_ExT 1
#define Z_EMAX 4096
#define Z_EMIN 50
#define Z_PIE 2
#define Z_PMAX 4096
#define Z_PMIN 50
#define LS_PIE 3
#define LS_PMAX 4096
#define LS_PMIN 50
//Periscope Rotational servo is on Ardunio pin 12
//#define P_ROT 4
//#define P_RMAX 4096
//#define P_RMIN 50
#define BM_PIE 4
#define BM_PMAX 4096
#define BM_PMIN 50
//Life Form scanner rotational servo is on Digital 13
//#define LF_ROT 6
//#define LF_RMAX 4096
//#define LF_RMIN 50
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
//#define the Limit Swich pins
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
 
//We will Be using a multidimensional to control the lift system
//The fallowing elements are contained in each device array:
/*  element 0 - Motor 1 pin
    element 0 - Motor 2 pin
    element 0 - Top Limit Switch pin
    element 0 - Bottem Limit Switch pin
    element 0 - servo 1 maestor pin (-1 if none)
    element 0 - servo 2 maestor pin (-1 if none)
    element 0 - Hall Effect  pin (-1 if none)
    element 0 - Pie Servo  pin (-1 if none)   */
 
 
int zapper[8] = { Z_IN1, Z_IN2, Z_TOP, Z_BOT, Z_ROT, Z_ExT, -1, Z_PIE };
int lSaber[8] = { LS_IN1, LS_IN2, LS_TOP, LS_BOT, -1, -1, -1, LS_PIE };
int periscope[8] = { P_IN1, P_IN2, P_TOP, P_BOT, -1, -1, P_HALL, -1 };
int badMotive[8] = { BM_IN1, BM_IN2, BM_TOP, BM_BOT, -1, -1, -1, BM_PIE };
int lifeForm[8] = { LF_IN1, LF_IN2, LF_TOP, LF_BOT, -1, -1, LF_HALL, LF_PIE };
int lifts[6][8];
byte leds=0;
 
void updateShiftRegister() {
  digitalWrite(BM_LATCH, LOW);
  shiftOut(BM_DATA, BM_CLOCK, LSBFIRST, leds);
  digitalWrite(BM_LATCH, HIGH);
}
 
 
 
void motorForward(int mtr) {
  if (digitalRead(lifts[mtr][3]) == HIGH) {
    while (digitalRead(lifts[mtr][3])) {
      digitalWrite(lifts[mtr][0], LOW);
      analogWrite(lifts[mtr][1], 255);
    }
    digitalWrite(lifts[mtr][1], LOW);
  }
  while (digitalRead(lifts[mtr][2])) analogWrite(lifts[mtr][0], 255);
  digitalWrite(lifts[mtr][0], LOW);
}
void motorBack(int mtr) {
  if (digitalRead(lifts[mtr][2]) == HIGH) {
    while (digitalRead[mtr][2]) {
      digitalWrite(lifts[mtr][1], LOW);
      analogWrite(lifts[mtr][0], 255);
    }
    digitalWrite(lifts[mtr][0], LOW);
  }
  while (digitalRead(lifts[mtr][3])) analogWrite(lifts[mtr][1], 255);
  digitalWrite(lifts[mtr][1], LOW);
}
 
void setup() {
  // put your setup code here, to run once:
  pinMode(Z_IN1, OUTPUT);
  pinMode(Z_IN2, OUTPUT);
  pinMode(LS_IN1, OUTPUT);
  pinMode(LS_IN2, OUTPUT);
  pinMode(P_IN1, OUTPUT);
  pinMode(P_IN2, OUTPUT);
  pinMode(BM_IN1, OUTPUT);
  pinMode(BM_IN2, OUTPUT);
  pinMode(LF_IN1, OUTPUT);
  pinMode(LF_IN2, OUTPUT);
 
  pinMode(Z_TOP, INPUT_PULLUP);
  pinMode(Z_BOT, INPUT_PULLUP);
  pinMode(LS_TOP, INPUT_PULLUP);
  pinMode(LS_BOT, INPUT_PULLUP);
  pinMode(P_TOP, INPUT_PULLUP);
  pinMode(P_BOT, INPUT_PULLUP);
  pinMode(BM_TOP, INPUT_PULLUP);
  pinMode(BM_BOT, INPUT_PULLUP);
  pinMode(LF_TOP, INPUT_PULLUP);
  pinMode(LF_BOT, INPUT_PULLUP);
  pinMode(BM_LATCH, OUTPUT);
  pinMode(BM_DATA, OUTPUT);
  pinMode(BM_CLOCK, OUTPUT);  
  int x;
  for (x = 0; x <= 7; x++) lifts[1][x] = zapper[x];
  for (x = 0; x <= 7; x++) lifts[2][x] = lSaber[x];
  for (x = 0; x <= 7; x++) lifts[3][x] = periscope[x];
  for (x = 0; x <= 7; x++) lifts[4][x] = badMotive[x];
  for (x = 0; x <= 7; x++) lifts[5][x] = lifeForm[x];
 
  Serial.begin(9600);
  servoCon.begin();
  servoCon.setOscillatorFrequency(27000000);
  servoCon.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  //motorForward(1);
  //motorForward(2);
  //motorForward(3);
  //motorForward(4);
  //motorForward(5);
}
 
 
void loop() {
   
leds=random(0,255);
  updateShiftRegister();
  delay(100);
 
}
