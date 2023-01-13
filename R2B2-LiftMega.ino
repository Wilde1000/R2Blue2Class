/*R2 Lift System program
Written by Gold and Blue FTC comp. Tech*/
/************************************************
*               Included Libraries              *
*************************************************/

#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
Adafruit_PWMServoDriver servoControl = Adafruit_PWMServoDriver();
#include <Adafruit_NeoPixel.h>
#include <time.h>
#include <stdlib.h>



/************************************************
*               Macro Definitions               *
*************************************************/
#define SERVO_FREQ 50
//#define OSCIL_FREQ 27000000
#define MPU 'E'
#define CMD_MAX_LENGTH 63
#define HOLO_LED 1

//define Adafruit PWM servo Pins and Limits

#define Z_ROT 1
#define Z_RMAX 150
#define Z_RMID 425
#define Z_RMIN 700
#define Z_EXT 0
#define Z_EMAX 125
#define Z_EMIN 300
#define Z_EMID 150
#define Z_PIE 2
#define Z_PMAX 400
#define Z_PMIN 250
#define LS_PIE 3
#define LS_PMAX 250  //revisit
#define LS_PMIN 400  //revisit
#define BM_PIE 4
#define BM_PMAX 475
#define BM_PMIN 325
#define LF_PIE 5
#define LF_PMAX 475
#define LF_PMIN 325
#define DP1 6
#define DP1_MIN 270  //done
#define DP1_MAX 425  //done
#define DP2 7
#define DP2_MIN 300
#define DP2_MAX 400
#define DP3 8
#define DP3_MIN 275  //done
#define DP3_MAX 400  //done
#define DP4 9
#define DP4_MIN 240
#define DP4_MAX 375
#define DP5 10
#define DP5_MIN 340
#define DP5_MAX 425
#define DP6 11
#define DP6_MIN 305  //done
#define DP6_MAX 400  //done


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
//define Holoprojector servo pins
#define TOP_HOLO1 22
#define TOP_HOLO2 23
#define FRT_HOLO1 24
#define FRT_HOLO2 25
#define BCK_HOLO1 26
#define BCK_HOLO2 27

//define Holo Light pins
#define TOP_HOLO_LGT 29
#define FRT_HOLO_LGT 31
#define BCK_HOLO_LGT 28


#define MAGIC_PANEL 35

//define the Output Enable Pin for the Adafruit servo driver
#define OE_PIN 34
//define Bad Motivator 74hc595 pins
#define BM_DATA 36
#define BM_LATCH 37
#define BM_CLOCK 38
//define Zapper LED pin
#define Z_LED 39
//define HALL effect pins
#define P_HALL 41
#define LF_HALL 40
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
//Neo-Pixel Objects
Adafruit_NeoPixel bHolo(HOLO_LED, BCK_HOLO_LGT, NEO_GRB + NEO_KHZ800);  //Back Holo Light
Adafruit_NeoPixel fHolo(HOLO_LED, FRT_HOLO_LGT, NEO_GRB + NEO_KHZ800);  //Front Holo Light
Adafruit_NeoPixel tHolo(HOLO_LED, TOP_HOLO_LGT, NEO_GRB + NEO_KHZ800);  //Top Holo Light
byte leds = 0;                                                          //Contains LED pattern as a 8-bit number

char cmdStr[64];   //Contains the incoming message from Serial0
char cmdStr1[64];  //Contains the incoming message from Serial1
char dev_MPU;      //Contains the MPU code from incoming serial message
char dev_cmd;      //Contains the command code from the incoming serial message

int bad_motive_state = 0;   //Contains current state for Bad Motivator
int bm_int = 75;            //Delay interval for the Bad Motivator lights (keep under 100)
int curr_holo_color = 13;   //Contains current color code for the holoprojectors
int curr_tholo_color = 5;   //Contains current color code for the top holoprojector
int dev_addr;               //Device address received from Serial interface
int dev_opt;                //Device option received from the Serial interface
int holo_speed;             //Holds the current holo timeout speed.
int holo_state = 0;         //Contains current state for the holoprojectors
int life_form_state = 0;    //Contains current state for the life form scanner
int light_saber_state = 0;  //Contains current state for the Light Saber lift
int periscope_state = 0;    //Contains current state for the Periscope
int seq_state = 0;          //Contains current state for the Panel Sequencer
int z_flash_count = 0;      //Contains current number of Zapper pulses
int z_int = 50;             //Contains current zapper pulse timeout
int z_raise_int = 100;      //Contains the raise timeout for the Zapper
int z_state = 0;            //Contains the current step for the Zapper
int zapper_state = 0;       //Contains the current state for the Zapper



//integer Arrays
int badMotive[8] = { BM_IN1, BM_IN2, BM_TOP, BM_BOT, -1, -1, -1, BM_PIE };
int lifeForm[8] = { LF_IN1, LF_IN2, LF_TOP, LF_BOT, -1, -1, LF_HALL, LF_PIE };
int lifts[6][8];
int lSaber[8] = { LS_IN1, LS_IN2, LS_TOP, LS_BOT, -1, -1, -1, LS_PIE };
int periscope[8] = { P_IN1, P_IN2, P_TOP, P_BOT, -1, -1, P_HALL, -1 };
int servoLmt[16][3];
int zapper[8] = { Z_IN1, Z_IN2, Z_TOP, Z_BOT, Z_ROT, Z_EXT, -1, Z_PIE };


long int current_time = millis();       //Contains current time
long int z_timer = current_time;        //Holds the zapper flash timer
long int z_raise_timer = current_time;  //Holds the zapper raise timer
long int bm_timer = current_time;       //Holds the timer for bad motivator lights
long int p_timer = current_time;        //Holds the rotational timer for the Periscope
long int seq_timer = current_time;      //Holds the sequence step timer for the Panel Sequencer
long int holo_timer = current_time;     //Holds the holo random movement timer

//Servo Objects
Servo bh1;       //Back Holoprojector Servo 1
Servo bh2;       //Back Holoprojector Servo 2
Servo fh1;       //Front Holoprojector Servo 1
Servo fh2;       //Front Holoprojector Servo 2
Servo lfServo;   //Life Form Continous Rotation Servo
Servo perServo;  //Periscope Continous Rotation Servo
Servo th1;       //Top Holoprojector Servo 1
Servo th2;       //Top Holoprojector Servo 2

uint16_t seq_Timeout;  //Holds the sequence step timeout for the Panel Sequencer (must be a 16 bit integer)


/***********************************************
*    Panel Sequence Constants stored in Program memory  *
***********************************************/


const uint16_t panel_init[][11] PROGMEM = {
  { 20, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


const uint16_t panel_all_open[][11] PROGMEM = {
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 300, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 150, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

const uint16_t panel_all_open_long[][11] PROGMEM = {
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 1000, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 150, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

const uint16_t panel_wave[][11] PROGMEM = {
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MAX },
  { 30, Z_PMAX, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 30, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

const uint16_t panel_fast_wave[][11] PROGMEM = {
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MAX },
  { 15, Z_PMAX, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMAX, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MAX },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 15, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

const uint16_t panel_open_close_wave[][11] PROGMEM = {
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 80, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MAX, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MAX },
  { 20, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MAX },
  { 20, Z_PMIN, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

const uint16_t panel_dance[][11] PROGMEM = {
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MAX, DP5_MAX, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MAX, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MAX, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 45, Z_PMAX, LS_PMAX, BM_PMAX, LF_PMAX, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MAX, DP2_MAX, DP3_MAX, DP4_MAX, DP5_MAX, DP6_MAX },
  { 45, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


const uint16_t panel_marching_ants[][11] PROGMEM = {
  { 20, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MIN },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMAX, LS_PMIN, BM_PMAX, LF_PMIN, DP1_MAX, DP2_MIN, DP3_MAX, DP4_MIN, DP5_MAX, DP6_MIN },
  { 50, Z_PMIN, LS_PMAX, BM_PMIN, LF_PMAX, DP1_MIN, DP2_MAX, DP3_MIN, DP4_MAX, DP5_MIN, DP6_MAX },
  { 50, Z_PMIN, LS_PMIN, BM_PMIN, LF_PMIN, DP1_MIN, DP2_MIN, DP3_MIN, DP4_MIN, DP5_MIN, DP6_MIN },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

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
  Serial.begin(9600);   //Connection with controller Arduino
  Serial1.begin(9600);  //Connection with Body Arduino
  Serial2.begin(9600);  //Connection with Periscope
  Serial3.begin(2400);  //Teeces Connection
  servoControl.begin();
  //servoControl.setOscillatorFrequency(OSCIL_FREQ);
  servoControl.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  bHolo.begin();
  tHolo.begin();
  fHolo.begin();

  bHolo.clear();
  fHolo.clear();
  tHolo.clear();
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


  th1.attach(TOP_HOLO1);
  th2.attach(TOP_HOLO2);
  fh1.attach(FRT_HOLO1);
  fh2.attach(FRT_HOLO2);
  bh1.attach(BCK_HOLO1);
  bh2.attach(BCK_HOLO2);
  perServo.write(90);
  lfServo.write(90);


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
  checkSerial();              //Check Serial 0 for commands
  checkSerial1();             //Check Serial 1 for commands
  ZapLift(zapper_state);      // Run actions on the Zapper if any
  LSLift(light_saber_state);  //Run actions on the Light Saber if any
  PLift(periscope_state);     //Run actions on the Periscope if any
  BMLift(bad_motive_state);   //Run actions on the Bad Motivator if any
  LFLift(life_form_state);    //Run actions on the Life Form Scanner if any
  Sequencer(seq_state);       //Run panel sequences
  Holos(holo_state);          //Run Holo actions
}








//BM_Raise raises the Bad motivator and returns a 0 while rising and a 1 when raised
byte BM_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      servoControl.setPWM(BM_PIE, 0, BM_PMAX);
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
      servoControl.setPWM(BM_PIE, 0, BM_PMIN);
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
//Bad Motivator.  It is controlled by the bad_motive_state.
void BMLift(int option) {
  switch (option) {
    case 0:
      //default - No Action
      if (digitalRead(BM_BOT)) bmLights(1);
      return 0;
      break;
    case 1:  //Raise LSaber
      if (BM_Raise() == 1) bad_motive_state = 0;
      break;
    case 2:  //Light Saber down
      if (BM_Lower() == 1) bad_motive_state = 0;
      break;
  }
}


//The bmLights function controls the 8 leds in the Bad Motivator.  The BM uses a 74HC595 shift register chip to control
//the eight lights with three pins. Passing a 0 to the function turns the ligts off, a 1 turns them on.
void bmLights(int num) {
  if (num) {
    current_time = millis();
    if (current_time - bm_timer > bm_int) {
      bm_timer = current_time;
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

//The buildCommand1 takes the current byte from the Serial1 buffer and builds a command for processing.  It returns a 0
//while in the building process and a 1 when the command is ready.
int buildCommand1(char ch, char* output_str) {
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
    case 51:
      zapper_state = opt;
      break;
    case 52:
      light_saber_state = opt;
      break;
    case 53:
      periscope_state = opt;
      break;
    case 54:
      bad_motive_state = opt;
      break;
    case 55:
      life_form_state = opt;
      break;
    case 80:
      seq_state = opt;
      break;
    case 90:  //Holoprojectors
      holo_state = opt;
      break;
  }
}

//The doScommand handles all T commands sent from the parseCommand function
int doScommand(int addr, int opt) {
  Serial.println("S command");
  switch (addr) {
    case 80:
      seq_state = opt;
      break;
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


//Raises the Life Form Scanner - Return 0 while raising and 1 when raised
byte LF_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Move lift up
      servoControl.setPWM(LF_PIE, 0, LF_PMAX);
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
      if (digitalRead(LF_HALL) == LOW) {


        lfServo.write(90);
        step = 1;
      }
      break;
    case 1:
      if (motorDown(5)) step = 2;
      break;
    case 2:  //Final cleanup and return 1 for a job well done
      step = 0;
      servoControl.setPWM(LF_PIE, 0, LF_PMIN);
      return 1;
      break;
  }
  return 0;
}
//LFLift is the last thread of seven threads in this program.  It handles all interactions with the
//Life Form Scanner.  It is controlled by the life_form_state.
void LFLift(int option) {
  switch (option) {
    case 0:
      //default - No Action
      return 0;
    case 1:  //Raise Life Form Scanner
      if (LF_Raise() == 1) life_form_state = 0;
      break;
    case 2:  //lower Life Form Scanner
      if (LF_Lower() == 1) life_form_state = 0;
      break;
    case 3:  //lower Life Form Scanner
      LF_Alt_Raise();
      break;
  }
  return;
}
int LF_Alt_Raise() {
  static int step = 0;
  static byte dir = 0;
  static byte hall_hit = 0;
  servoControl.setPWM(LF_PIE, 0, LF_PMAX);
  switch (step) {
    case 0:  //Move lift up
      if (motorUp(5)) step = 1;
      break;
    case 1:
      if (dir) {
        lfServo.write(135);
      } else {
        lfServo.write(45);
      }
      step = 2;
      break;

    case 2:
      current_time = millis();
      if (current_time - p_timer > 1000) {
        p_timer = current_time;
        step = 3;
        hall_hit = 0;
      }
      break;
    case 3:
      if (digitalRead(LF_HALL) == LOW && hall_hit == 0) {
        hall_hit = 1;
        lfServo.write(90);
        dir = !dir;
        step = 4;
      }
      break;
    case 4:
      current_time = millis();
      if (current_time - p_timer > 200) {
        p_timer = current_time;
        step = 1;
      }
      break;
  }
  return 0;
}


//The LS_Raise function raises the Light Saber and returns a 0 while raising and a 1 when raised
byte LS_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      servoControl.setPWM(LS_PIE, 0, LS_PMAX);
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
      servoControl.setPWM(LS_PIE, 0, LS_PMIN);
      step = 2;
    case 2:  //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}
//LSLift is the fourth thread of seven threads in this program.  It handles all interactions with the
//Light Saber Lift.  It is controlled by the light_saber_state.
void LSLift(int option) {
  //Serial.println("Light saber");
  switch (option) {
    case 0:
      //default - No Action
      return 0;
    case 1:  //Raise LSaber
      if (LS_Raise() == 1) light_saber_state = 0;
      break;
    case 2:  //Light Saber down
      if (LS_Lower() == 1) light_saber_state = 0;
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
  if (MPU != mpu) {              //if command is not for this MPU - send it on its way
    if (mpu == 'G') {
      //Serial.println("HERE");
      Serial3.flush();

      for (int x = 1; x < length; x++) {
        Serial3.write(input_str[x]);
        Serial.print(input_str[x]);
      }
      Serial3.write(13);
      Serial.println();
    }
    if (mpu == 'F') {
      for (int x = 0; x < length; x++) {
        Serial2.write(input_str[x]);
      }
      Serial2.write(13);
    }
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
//Raises the periscope - Return 0 while raising and 1 when raised
byte P_Raise() {
  static int step = 0;
  switch (step) {
    case 0:  //Move lift up
      Serial2.write("F60T001\n");
      if (motorUp(3)) step = 1;
      break;
    case 1:
      perServo.write(135);
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
      if (digitalRead(P_HALL) == LOW) {
        perServo.write(90);
        step = 1;
      }
      break;
    case 1:
      if (motorDown(3)) step = 2;
      break;
    case 2:
      Serial2.write("F60T000\n");
      step = 3;
      break;
    case 3:  //Final cleanup and return 1 for a job well done
      step = 0;
      return 1;
      break;
  }
  return 0;
}


//PLift is the fifth thread of seven threads in this program.  It handles all interactions with the
//Periscope.  It is controlled by the periscope_state.
void PLift(int option) {
  static int plstate = 0;
  //Serial.println("Light saber");
  switch (option) {
    case 0:
      //default - No Action
      return;
      break;
    case 1:  //Raise Periscope
      pLights(1);
      if (P_Raise() == 1) periscope_state = 0;
      break;
    case 2:  //lower Periscope
      if (P_Lower() == 1) {
        periscope_state = 0;
        pLights(0);
      }
      break;
    case 3:  // Alternate rotation
      P_Alt_Raise();
      break;
  }
}

int P_Alt_Raise() {
  static int step = 0;
  static byte dir = 0;
  static byte hall_hit = 0;
  switch (step) {
    case 0:  //Move lift up
      Serial2.write("F60T001\n");
      if (motorUp(3)) step = 1;
      break;
    case 1:
      if (dir) {
        perServo.write(135);
      } else {
        perServo.write(45);
      }
      step = 2;
      break;

    case 2:
      current_time = millis();
      if (current_time - p_timer > 1000) {
        p_timer = current_time;
        step = 3;
        hall_hit = 0;
      }
      break;
    case 3:
      if (digitalRead(P_HALL) == LOW && hall_hit == 0) {
        hall_hit = 1;
        perServo.write(90);
        dir = !dir;
        step = 4;
      }
      break;
    case 4:
      current_time = millis();
      if (current_time - p_timer > 200) {
        p_timer = current_time;
        step = 1;
      }
      break;
  }
  return 0;
}

void pLights(int num) {
  switch (num) {
    case 0:  //Periscope Lights off
      Serial2.write("F60T002\n");
      return;
    case 1:  //Periscope Light on defalult value
      Serial2.write("F60T001\n");
      return;
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


//ZapLift is the third thread of seven threads in this program.  It handles all interactions with the
//Zapper.  It is controlled by the zapper_state.
void ZapLift(int option) {
  static int step = 0;
  if (option == 7) setTimer(1, 250);
  current_time = millis();
  switch (option) {
    case 0:  //ZapLift state unchanged;
      step = 0;
      return;
      break;
    case 1:  //Raise lift
      if (Z_Raise()) {
        zapper_state = 0;
        return;
      }
      break;
    case 2:  //Move Zapper to stored position
      if (Z_Lower()) {
        zapper_state = 0;
        return;
      }
      break;
    case 3:  //Rotate Zapper Open
      moveServo(Z_ROT, Z_RMIN, Z_RMAX);
      zapper_state = 0;
      break;
    case 4:  //Rotate Zapper closed
      moveServo(Z_ROT, Z_RMAX, Z_RMIN);
      zapper_state = 0;
      break;
    case 5:  //Extend Zapper
      moveServo(Z_EXT, Z_EMIN, Z_EMAX);
      zapper_state = 0;
      break;
    case 6:  //Fold Zapper
      moveServo(Z_EXT, Z_EMAX, Z_EMIN);
      zapper_state = 0;
      break;
    case 7:  //Zap!
      if (ZapLights(1) == 1) {
        ZapLights(0);
        zapper_state = 0;
      }
      break;
    case 8:  //Long Zap Seq
      if (zapSeq1() == 1) {
        zapper_state = 0;
      }
      break;
  }
}


//ZapLights operates the Zap light on the zapper. Pass a 0 to shut off and a positive number to run
byte ZapLights(int num) {
  if (num) {
    switch (z_state) {
      case 0:
        current_time = millis();
        digitalWrite(Z_LED, HIGH);  // sets the led HIGH
        if (current_time - z_timer >= z_int) {
          z_timer = current_time;
          z_state = 1;
        }
        break;
      case 1:
        current_time = millis();
        digitalWrite(Z_LED, LOW);  // sets the led LOW
        if (current_time - z_timer >= z_int) {
          z_timer = current_time;
          z_state = 2;
        }
        break;
      case 2:
        current_time = millis();
        z_flash_count++;
        if (z_flash_count == 8) {
          z_state = 0;
          z_flash_count = 0;
          return 1;
        } else {
          z_state = 0;
        }
        break;
    }
  } else digitalWrite(Z_LED, LOW);
  return 0;
}



int setTimer(int num, int interval) {
  if (num) {
    current_time = millis();
    z_raise_timer = current_time;
    z_raise_int = interval;
    return 0;
  } else {
    current_time = millis();
    if (current_time - z_raise_timer > z_raise_int) return 1;
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
    case 1:  //Rotate the Zapper
      moveServo(Z_ROT, Z_RMAX, Z_RMIN);
      step = 2;
      break;
    case 2:  //Zapper folded
      moveServo(Z_EXT, Z_EMAX, Z_EMIN);
      step = 3;
      break;
    case 3:  //Move Motor
      if (motorDown(1)) step = 4;
      break;
    case 4:  //Close the pie
      servoControl.setPWM(Z_PIE, 0, Z_PMIN);
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
      servoControl.setPWM(Z_PIE, 0, Z_PMAX);
      step = 1;
      break;
    case 1:  //Move lift up
      if (motorUp(1)) step = 2;
      break;
    case 2:  //Extend the zapper
      moveServo(Z_EXT, Z_EMIN, Z_EMAX);
      step = 3;
      setTimer(1, 500);
      break;
    case 3:  //Rotate the zapper
      moveServo(Z_ROT, Z_RMIN, Z_RMAX);
      step = 4;
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

byte zapSeq1() {
  static int step = 0;
  switch (step) {
    case 0:  //Open pie
      servoControl.setPWM(Z_PIE, 0, Z_PMAX);
      step = 1;
      break;
    case 1:  //Raise Zapper
      if (motorUp(1)) step = 2;
      break;
    case 2:  //Extend Zapper
      moveServo(Z_EXT, Z_EMIN, Z_EMAX);
      setTimer(1, 500);
      step = 3;
      break;
    case 3:
      if (setTimer(0, 500)) step = 4;
      break;
    case 4:
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 5;
        setTimer(1, 500);
      }
      break;
    case 5:  //Move Servo to zap Position 2
      moveServo(Z_ROT, Z_RMIN, Z_RMID);
      moveServo(Z_EXT, Z_EMAX, Z_EMID);
      setTimer(1, 500);
      step = 6;
      break;
    case 6:  //Wait for dust to settle
      if (setTimer(0, 500)) step = 7;
      break;
    case 7:  //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 8;
        setTimer(1, 500);
      }
      break;
    case 8:  //Move to position three
      moveServo(Z_ROT, Z_RMID, Z_RMAX);
      moveServo(Z_EXT, Z_EMID, Z_EMAX);
      setTimer(1, 500);
      step = 9;
      break;
    case 9:  //Wait for dust to settle
      if (setTimer(0, 500)) step = 10;
      break;
    case 10:  //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 11;
        setTimer(1, 500);
      }
      break;
    case 11:  //Move to position four
      moveServo(Z_EXT, Z_EMAX, Z_EMID);
      setTimer(1, 500);
      step = 12;
      break;
    case 12:  //Wait for dust to settle
      if (setTimer(0, 500)) step = 13;
      break;
    case 13:  //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 14;
        setTimer(1, 500);
      }
      break;
    case 14:  //Move to position five
      moveServo(Z_ROT, Z_RMAX, Z_RMID);
      moveServo(Z_EXT, Z_EMID, Z_EMAX);
      setTimer(1, 500);
      step = 15;
      break;
    case 15:  //Wait for dust to settle
      if (setTimer(0, 500)) step = 16;
      break;
    case 16:  //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 17;
        setTimer(1, 500);
      }
      break;
    case 17:  //Move to position six
      moveServo(Z_ROT, Z_RMID, Z_RMIN);
      moveServo(Z_EXT, Z_EMAX, Z_EMID);
      setTimer(1, 500);
      step = 18;
      break;
    case 18:  //Wait for dust to settle
      if (setTimer(0, 500)) step = 19;
      break;
    case 19:  //Zap
      if (ZapLights(1) == 1) {
        ZapLights(0);
        step = 20;
        setTimer(1, 500);
      }
      break;
    case 20:  //Move to open position
      moveServo(Z_ROT, Z_RMID, Z_RMAX);
      moveServo(Z_EXT, Z_EMID, Z_EMAX);
      setTimer(1, 1000);
      step = 21;
      break;
    case 21:  //Fold the Zapper and return to store position
      moveServo(Z_ROT, Z_RMAX, Z_RMIN);
      moveServo(Z_EXT, Z_EMAX, Z_EMIN);
      step = 22;
      break;
    case 22:  //Move the Lift down
      if (motorDown(1)) step = 23;
      break;
    case 23:  //Close Pie
      servoControl.setPWM(Z_PIE, 0, Z_PMIN);
      step = 24;
      break;
    case 24:
      step = 0;
      zapper_state = 0;
      return 1;
      break;
  }
  return 0;
}

void Sequencer(int opt) {
  switch (opt) {
    case 0:
      return;
      break;
    case 1:
      if (runSeq(panel_init)) seq_state = 0;
      break;
    case 2:
      if (runSeq(panel_all_open)) seq_state = 0;
      break;
    case 3:
      if (runSeq(panel_all_open_long)) seq_state = 0;
      break;
    case 4:
      if (runSeq(panel_wave)) seq_state = 0;
      break;
    case 5:
      if (runSeq(panel_fast_wave)) seq_state = 0;
      break;
    case 6:
      if (runSeq(panel_open_close_wave)) seq_state = 0;
      break;
    case 7:
      if (runSeq(panel_marching_ants)) seq_state = 0;
      break;
    case 8:
      if (runSeq(panel_dance)) seq_state = 0;
      break;
    case 9:                   //opens all lift systems
      zapper_state = 1;       // Run actions on the Zapper if any
      light_saber_state = 1;  //Run actions on the Light Saber if any
      periscope_state = 1;    //Run actions on the Periscope if any
      bad_motive_state = 1;   //Run actions on the Bad Motivator if any
      life_form_state = 1;    //Run actions on the Life Form Scanner if any
      seq_state = 0;
      break;
    case 10:                  //Closes all lift systems
      zapper_state = 2;       // Run actions on the Zapper if any
      light_saber_state = 2;  //Run actions on the Light Saber if any
      periscope_state = 2;    //Run actions on the Periscope if any
      bad_motive_state = 2;   //Run actions on the Bad Motivator if any
      life_form_state = 2;    //Run actions on the Life Form Scanner if any
      seq_state = 0;
      break;
    case 11:
      servoControl.setPWM(Z_PIE, 0, Z_PMAX);
      seq_state = 0;
      break;
    case 12:
      servoControl.setPWM(LS_PIE, 0, LS_PMAX);
      seq_state = 0;
      break;
    case 13:
      servoControl.setPWM(BM_PIE, 0, BM_PMAX);
      seq_state = 0;
      break;
    case 14:
      servoControl.setPWM(LF_PIE, 0, LF_PMAX);
      seq_state = 0;
      break;
    case 15:
      servoControl.setPWM(DP1, 0, DP1_MAX);
      seq_state = 0;
      break;
    case 16:
      servoControl.setPWM(DP2, 0, DP2_MAX);
      seq_state = 0;
      break;      
    case 17:
      servoControl.setPWM(DP3, 0, DP3_MAX);
      seq_state = 0;
      break;
    case 18:
      servoControl.setPWM(DP4, 0, DP4_MAX);
      seq_state = 0;
      break;
    case 19:
      servoControl.setPWM(DP5, 0, DP5_MAX);
      seq_state = 0;
      break;
    case 20:
      servoControl.setPWM(DP6, 0, DP6_MAX);
      seq_state = 0;
      break;      
    case 21:
      servoControl.setPWM(Z_PIE, 0, Z_PMIN);
      seq_state = 0;
      break;
    case 22:
      servoControl.setPWM(LS_PIE, 0, LS_PMIN);
      seq_state = 0;
      break;
    case 23:
      servoControl.setPWM(BM_PIE, 0, BM_PMIN);
      seq_state = 0;
      break;
    case 24:
      servoControl.setPWM(LF_PIE, 0, LF_PMIN);
      seq_state = 0;
      break;
    case 25:
      servoControl.setPWM(DP1, 0, DP1_MIN);
      seq_state = 0;
      break;
    case 26:
      servoControl.setPWM(DP2, 0, DP2_MIN);
      seq_state = 0;
      break;      
    case 27:
      servoControl.setPWM(DP3, 0, DP3_MIN);
      seq_state = 0;
      break;
    case 28:
      servoControl.setPWM(DP4, 0, DP4_MIN);
      seq_state = 0;
      break;
    case 29:
      servoControl.setPWM(DP5, 0, DP5_MIN);
      seq_state = 0;
      break;
    case 30:
      servoControl.setPWM(DP6, 0, DP6_MIN);
      seq_state = 0;
      break;      
          
    
  }
  return;
}

int runSeq(uint16_t const sequence_array[][11]) {
  static int seq_step = 0;
  static byte servo_moved = 0;
  seq_Timeout = pgm_read_word(&(sequence_array[seq_step][0])) * 10;  // restart timer with step time value
  if (seq_Timeout == 0) {
    servo_moved = 0;
    seq_step = 0;
    return 1;
  }
  if (!servo_moved) {
    Serial.print(seq_Timeout);
    Serial.print(", ");
    for (int x = 1; x <= 10; x++) {
      uint16_t servo_pos = pgm_read_word(&sequence_array[seq_step][x]);
      servoControl.setPWM(x + 1, 0, servo_pos);
      Serial.print(servo_pos);
      Serial.print(", ");
    }
    Serial.println();
    servo_moved = 1;
  }
  current_time = millis();
  if (current_time - seq_timer > seq_Timeout) {
    seq_timer = current_time;
    seq_step++;
    servo_moved = 0;
  }
  return 0;
}
