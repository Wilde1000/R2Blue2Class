/*************************************************************************
 * ********************** Body Master Mega  ******************************
 * ***********************************************************************/
/*
The Body Master Mega is used to route communications between the Dome Lift Mega, the Body Lights Mega
and the Drive Uno.  It is also used to control all the body servos except the Dataport door servo. It 
is also responisble for all sounds coming out of the droid as it is connected to the MP3 Trigger.
*/

/*
We will be using a Jawa-Lite inspired technique to control all the MPU's in 
R2-Blue2.

The command structure is as follows:

    Element 0 - MPU Code - Single Character indicating the Microprocessor Unit
    Element 1, 2 - two digit Integer representing the device attached to the MPU
    Element 3 - one character command code
    Element 4, 5, 6 - the command option

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
      J - MP3 Trigger - Not an Microprocessor, but treated as such for serial communications

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


//We have used (with slight modifications for our droid) most of the MP3sound.h and MP3sound.c
//functions with credit given below:


/***********************************************************
 *  MP3sound.h
 *	MarcDuino interface to play sounds from an MP3Trigger board
 *  Created on: Sep 17, 2013
 *  Author: Marc Verdiell
 *  Copyright © 2013 Marc Verdiell, All Rights Reserved
 *
 *  On the MP3, there are a maximum of 255 sound files
 *  They must be named NNN-xxxx.mp3
 *  Where NNN = 001 to 255
 *  The numbering ranges are predetermined, 25 sounds per
 *  bank category
 *       Bank 1: gen sounds, numbered 001 to 025
 *       Bank 2: chat sounds, numbered 026 to 050
 *       Bank 3: happy sounds, numbered 051 to 075
 *       Bank 4: sad sounds, numbered 076 to 100
 *       Bank 5: whistle sounds, numbered 101 to 125
 *       Bank 6: scream sounds, numbered 126 to 150
 *       Bank 7: Leia sounds, numbered 151 to 175
 *       Bank 8: sing sounds (deprecated, not used by R2 Touch)
 *       Bank 9: mus sounds, numbered 201 t0 225
 *
 *  The pre-cooked R2 sound library contains aonly a few non-copyrighted music sounds.
 *  Sound 202, 203 and 205 are beep placeholders, meant to be replaced with the
 *  original score of Star Wars, Empire March, and Cantina respectively
 *
 ***********************************************************/

/////////////COMMAND VOCABULARY///////////
// Play sound command by bank/sound numbers
// Jxyy
// x=bank number
// yy=sound number. If none, next sound is played in the bank
//
// Other commands
// Jc
// where c is a command character
// R - random from 4 first banks
// O - sound off
// L - Leia message (bank 7 sound 1)
// C - Cantina music (bank 9 sound 5)
// c - Beep cantina (bank 9 sound 1)
// S - Scream (bank 6 sound 1)
// F - Faint/Short Circuit (bank 6 sound 3)
// D - Disco (bank 9 sound 6)
// s - stop sounds
// + - volume up
// - - volume down
// m - volume mid
// f - volume max
// p - volume min
// W - Star Wars music (bank 9 sound 2)
// M - Imperial March (bank 9 sound 3)
//
///////////////////////////////////////////////



/*************************************************************************
 * ********************** INCLUDED LIBRARIES  ****************************
 * ***********************************************************************/

#include <Adafruit_PWMServoDriver.h>  //Needed for PCA9685 16 Servo Driver

/*************************************************************************
 * *********************** MACRO DEFINITIONS  ****************************
 * ***********************************************************************/

/*>>>>>>>>>>>>>>>>>>> Servo Macros <<<<<<<<<<<<<<<<<<<<*/

// Utility Arms

#define UA_TOP 0        //Utility Arm Top Servo PCA9685 pin
#define UA_TOP_MAX 200  //Utility Arm Top Servo (open position)
#define UA_TOP_MIN 500  //Utility Arm Top Servo (close position)
#define UA_BOT 1        //Utility Arm Bottom Servo PCA9685 pin
#define UA_BOT_MAX 200  //Utility Arm Bottom Servo (open position)
#define UA_BOT_MIN 450  //Utility Arm Bottom Servo (close position)
// Interface Arm
#define IA_DOR 2        //Interface Arm Door Servo PCA9685 pin
#define IA_DOR_MAX 275  //Interface Arm Door Servo (open position)
#define IA_DOR_MIN 375  //Interface Arm Door Servo (close position)
#define IA_LFT 3        //Interface Arm Lift Servo PCA9685 pin
#define IA_LFT_MAX 475  //Interface Arm Lift Servo (up position)
#define IA_LFT_MIN 150  //Interface Arm Lift Servo (down position)
#define IA_EXT 6        //Interface Arm Extension Servo PCA9685 pin
#define IA_EXT_MAX 375  //Interface Arm Extension (out position)
#define IA_EXT_MIN 200  //Interface Arm Extension (in position)
//Gripper Arm
#define GA_DOR 4        //Gripper Arm Door Servo PCA9685 pin
#define GA_DOR_MAX 300  //Gripper Arm Door Servo (open position)
#define GA_DOR_MIN 200  //Gripper Arm Door Servo (close position)
#define GA_LFT 5        //Gripper Arm Lift Servo PCA9685 pin
#define GA_LFT_MAX 150  //Gripper Arm Lift Servo (up position)
#define GA_LFT_MIN 450  //Gripper Arm Lift Servo (down position)
#define GA_EXT 7        //Gripper Arm Extension Servo PCA9685 pin
#define GA_EXT_MAX 200  //Gripper Arm Extension Servo (open gripper)
#define GA_EXT_MIN 375  //Gripper Arm Extension Servo (close gripper)
//Data Panel Door
//#define DP_DOR 8        //Data Panel Door Servo PCA9685 pin
//#define DP_DOR_MAX 425  //Data Panel Door Servo (open position)
//#define DP_DOR_MIN 200  //Data Panel Door Servo (close position)

#define OE_PIN 8       //Set low to enable servos - low to disable servos
#define SERVO_FREQ 50  //Standard servo frequency is 50Kz



/*>>>>>>>>>>>>>>>>>>>>>>>>> Music Macros <<<<<<<<<<<<<<<<<<<<<<<<<<<*/
#define USER_MUSIC_SOUNDS_NUMBER 5
#define MP3_MAX_BANKS 9             // nine banks
#define MP3_MAX_SOUNDS_PER_BANK 25  // no more than 25 sound in each
#define MP3_BANK_CUTOFF 4           // cutoff for banks that play "next" sound on $x

// for the random sounds, needs to know max sounds of first 5 banks
// only important for sounds below cutoff
#define MP3_BANK1_SOUNDS 19  // gen sounds, numbered 001 to 025
#define MP3_BANK2_SOUNDS 18  // chat sounds, numbered 026 to 050
#define MP3_BANK3_SOUNDS 7   // happy sounds, numbered 051 to 075
#define MP3_BANK4_SOUNDS 4   // sad sounds, numbered 076 to 100
#define MP3_BANK5_SOUNDS 3   // whistle sounds, numbered 101 to 125
// unless you change bank cutoff, these are ignored, so I set them to max
#define MP3_BANK6_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // scream sounds, numbered 126 to 150
#define MP3_BANK7_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // Leia sounds, numbered 151 to 175
#define MP3_BANK8_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // sing sounds (deprecated, not used by R2 Touch)
#define MP3_BANK9_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // mus sounds, numbered 201 t0 225

// this defines where the startup sound is
#define MP3_EMPTY_SOUND 254   // workaround, used to stop sounds
#define MP3_START_SOUND 255   // startup sound is number 255
#define MP3_LEIA 151 //Leia Sound number
#define MP3_FAINT 128
#define MP3_SCREAM 126
#define MP3_DISCO 181
#define MP3_MANAMA 179
#define MP3_CANTINA 180
#define SOUND_START_CHAR 'J'  //Lead character for sound commands
#define MP3_VOLUME_MID 50     // guessing mid volume 32 is right in-between...
#define MP3_VOLUME_MIN 100    // doc says anything below 64 is inaudible, not true, 100 is. 82 is another good value
#define MP3_VOLUME_MAX 0      // doc says max is 0
#define MP3_VOLUME_STEPS 20   // R2 Touch app has 20 steps from min to max
#define MP3_VOLUME_OFF 254    // to turn it off... 255 gets a buzz.

#define MP3_PLAY_CMD 't'               // command to play sound file on the MP3 trigger
#define MP3_VOLUME_CMD 'v'             // command to play sound file on the MP3 trigger
#define MP3_STOP_CMD 'O'               // command to stop/play  - not used
#define MP3_MIN_RANDOM_PAUSE 10000     // min wait on random sounds
#define MP3_MAX_RANDOM_PAUSE 20000     // max wait on random sounds
#define MP3_MAX_PAUSE_ON_RESUME 30000  // default wait to resume random. Works for short sound. Set mp3_random_timer manually for long ones.
#define CMD_MAX_LENGTH 64              //Defines max command Length - same as serial buffer
#define MPU 'A'                        //Defines the MPU code for the program

/*************************************************************************
 *************************  GLOBAL VARIABLES  ****************************
 *************************************************************************/

Adafruit_PWMServoDriver servoControl = Adafruit_PWMServoDriver();

char dev_MPU, dev_command;
char cmdStr0[CMD_MAX_LENGTH];
char cmdStr1[CMD_MAX_LENGTH];
char cmdStr2[CMD_MAX_LENGTH];
char cmdStr3[CMD_MAX_LENGTH];

int dev_option, dev_address;
unsigned long current_time = millis();
unsigned long door_time = current_time;
unsigned long lift_time = current_time;
unsigned long ext_time = current_time;
unsigned long mp3_random_timer = millis();
long door_int = 1500;
long lift_int = 2000;
long ext_int = 2000;
int mp3_random_int = random(MP3_MIN_RANDOM_PAUSE, MP3_MAX_RANDOM_PAUSE);
static uint8_t mp3_bank_indexes[MP3_MAX_BANKS];
static const uint8_t mp3_max_sounds[] = {
  MP3_BANK1_SOUNDS,
  MP3_BANK2_SOUNDS,
  MP3_BANK3_SOUNDS,
  MP3_BANK4_SOUNDS,
  MP3_BANK5_SOUNDS,
  MP3_BANK6_SOUNDS,
  MP3_BANK7_SOUNDS,
  MP3_BANK8_SOUNDS,
  MP3_BANK9_SOUNDS,
};
static uint8_t mp3_volume = MP3_VOLUME_MAX;
static uint8_t saveflag;
const char strSoundCmdError[] PROGMEM = "Invalid MP3Trigger Sound Command";
uint8_t mp3_random_mode_flag = 1;  // the switch to check random sound mode
byte ua_State = 0;
byte ia_State = 0;
byte ga_State = 0;
byte dp_State = 0;



/*************************************************************************
 ***********************  FUNCTION DEFINITIONS  **************************
 *************************************************************************/
/*


//The buildCommand takes the output from the checkSerial functions and builds a command
byte buildCommand(char ch, char* output_str);  
byte checkSerial();              // Checks serial0 for commands and processes them
byte checkSerial1();              // Checks serial1 for commands and processes them
byte checkSerial2();              // Checks serial2 for commands and processes them
byte checkSerial3();              // Checks serial3 for commands and processes them
void doTcommand(int address, int argument);   //Processes T commands for this mpu
void doScommand(int address, int argument);   //Processes S commands for this mpu
void gripper(int option);         //Controls the Gripper Arm
void interfaceArm(int option);    //Controls the Interface Arm
void liftInterface();             //Raises the Interface Arm
byte parseCommand(char* input_str);           //Parses command for processing
void safeReset();                 //Safely resets all devices to stored position
void utilityArms(int option);     //Controls the Utility Arms



//Sound functions from MP3Sound.h and MP3Sound.c
// public
void mp3_init();  // wait at least 3s after mp3trigger power up before calling
void mp3_parse_command(char* commandstr);     //Parses a music command for execution
void mp3_do_random();  // need to be called in the main loop for random sounds to work

// utilities
void mp3_sound(uint8_t bank, uint8_t sound);  // Plays sound from bank
void mp3_stop();                              // Stops sounds
void mp3_playstartsound();                    // Plays starting sound
void mp3_random();                            // Plays a random sound
void mp3_volumeup();                          // Turns the MP3 Volume up
void mp3_volumedown();                        // Turns the MP3 Volume down
void mp3_volumemid();                         // Sets MP3 volume to mid 
void mp3_volumeoff();                         // Sets MP3 volume to off
void mp3_volumemax();                         // Sets MP3 volume to max
void mp3_volumemin();                         // Sets MP3 volume to min

// private
void mp3_send_command_byte(char command);    //sends command byte to MP3 Trigger
void mp3_setvolume(uint8_t vol);             //sets volume on MP3 Trigger
void mp3_suspend_random();                   //suspends random noises
void mp3_resume_random();                    //resumes random noises
void mp3_stop_random();                      //stops random noises
void mp3_start_random();                     //starts random noises
void mp3_check_timer();                      //helper function to check timeouts
*/


/*************************************************************************
 *****************************  FUNCTIONS  *******************************
 *************************************************************************/


//The buildCommand takes the output from the checkSerial functions and builds a command
byte buildCommand(char ch, char* output_str) {
  static int pos = 0;
  switch (ch) {
    case '\r':  //end character reached
    case '\n':
    case '\0':
      output_str[pos] = 13;
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


//The checkSerial() function takes the serial data from Serial0 and sends it to the
//buildCommand function for further processing.
byte checkSerial() {
  if (Serial.available()) {
    char ch;                                       //create a character to hold the current byte from Serial stream
    byte command_complete;                         //Establish a flag value to indicate a complete command
    ch = Serial.read();                            //Read a byte from the Serial Stream
    command_complete = buildCommand(ch, cmdStr0);  //Build the command string
    if (command_complete) {                        //if complete return 1 to start the processing
      return 1;
    }
  }
  return 0;
}

//The checkSerial1() function takes the serial data from Serial1 and sends it to the
//buildCommand function for further processing.
byte checkSerial1() {
  if (Serial1.available()) {
    char ch;                                       //create a character to hold the current byte from Serial stream
    byte command_complete;                         //Establish a flag value to indicate a complete command
    ch = Serial1.read();                           //Read a byte from the Serial Stream
    command_complete = buildCommand(ch, cmdStr1);  //Build the command string
    if (command_complete) {                        //if complete return 1 to start the processing
      return 1;
    }
  }
  return 0;
}


//The checkSerial2() function takes the serial data from Serial2 and sends it to the
//buildCommand function for further processing.
byte checkSerial2() {
  if (Serial2.available()) {
    char ch;                                       //create a character to hold the current byte from Serial stream
    byte command_complete;                         //Establish a flag value to indicate a complete command
    ch = Serial2.read();                           //Read a byte from the Serial Stream
    command_complete = buildCommand(ch, cmdStr2);  //Build the command string
    if (command_complete) {                        //if complete return 1 to start the processing
      return 1;
    }
  }
  return 0;
}


//The checkSerial3() function takes the serial data from Serial3 and sends it to the
//buildCommand function for further processing.
byte checkSerial3() {
  if (Serial3.available()) {
    char ch;                                       //create a character to hold the current byte from Serial stream
    byte command_complete;                         //Establish a flag value to indicate a complete command
    ch = Serial3.read();                           //Read a byte from the Serial Stream
    command_complete = buildCommand(ch, cmdStr3);  //Build the command string
    if (command_complete) {                        //if complete return 1 to start the processing
      return 1;
    }
  }
  return 0;
}



//The doTcommand processes all action commands rececived from the parseCommand function.
void doTcommand(int address, int argument) {
  switch (address) {
    case 10:
      ua_State = argument;
      break;
    case 11:
      ia_State = argument;
      break;
    case 12:
      ga_State = argument;
      break;
  }
}


//The doScommand processes all settings commands rececived from the parseCommand function.
void doScommand(int address, int argument) {
  switch (argument) {
  }
}

//The gripper function handles all actions for the gripper arm
void gripper(int option) {
  static int step = 0;
  static int count = 0;
  switch (option) {

    current_time = millis();
    case 0:
      return;
    case 1:
      switch (step) {
        case 0:
          servoControl.setPWM(GA_DOR, 0, GA_DOR_MAX);
          step++;
          break;
        case 1:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(GA_LFT, 0, GA_LFT_MAX);
            step++;
          }
          break;
        case 2:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(GA_EXT, 0, GA_EXT_MAX);
            step = 3;
          }
          break;
        case 3:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(GA_EXT, 0, GA_EXT_MIN);
            step = 2;
            count++;
            if (count == 3) {
              step = 0;
              count = 0;
              ga_State = 0;
            }
          }

          break;
      }
      break;
    case 2:
      switch (step) {
        case 0:
          servoControl.setPWM(GA_EXT, 0, GA_EXT_MIN);
          step = 1;
          break;
        case 1:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(GA_LFT, 0, GA_LFT_MIN);
            step = 2;
          }
          break;
        case 2:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(GA_DOR, 0, GA_DOR_MIN);
            step = 0;
            ga_State = 0;
          }
          break;
      }
      break;
  }
}


//The interfaceArm function handles all actions for the interface arm
void interfaceArm(int option) {
  static int step = 0;
  static int count = 0;
  current_time = millis();

  switch (option) {
    case 0:
      return;
    case 1:
      liftInterface();
      break;
    case 2:
      switch (step) {
        case 0:
          servoControl.setPWM(IA_EXT, 0, IA_EXT_MIN);
          step = 1;
          break;
        case 1:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(IA_LFT, 0, IA_LFT_MIN);
            step = 2;
          }
          break;
        case 2:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(IA_DOR, 0, IA_DOR_MIN);
            step = 0;
            ia_State = 0;
          }
          break;
      }
      break;
  }
}


//The liftInterface function raises the interface arm.
void liftInterface() {
  static int step = 0;
  static int count = 0;
  //Serial.println(step);
  switch (step) {
    case 0:
      servoControl.setPWM(IA_DOR, 0, IA_DOR_MAX);
      step = 1;  // Lift thingy
      break;
    case 1:
      if (current_time - ext_time > ext_int) {
        ext_time = current_time;
        servoControl.setPWM(IA_LFT, 0, IA_LFT_MAX);
        step = 2;  // Extend thingy
      }
      break;
    case 2:
      if (current_time - ext_time > ext_int) {
        ext_time = current_time;
        servoControl.setPWM(IA_EXT, 0, IA_EXT_MAX);
        step = 3;  // Unextend thingy
      }
      break;
    case 3:
      if (current_time - ext_time > ext_int) {
        ext_time = current_time;
        servoControl.setPWM(IA_EXT, 0, IA_EXT_MIN);
        step = 2;  // Extend thingy again
        count++;

        // Stop
        if (count == 3) {
          step = 0;
          count = 0;
          ia_State = 0;
        }
      }
      break;
  }
}


//The mp3_do_random functions allows R2 to make random sounds from the first 4 banks
void mp3_do_random() {
  if (!mp3_random_mode_flag) return;
  if (millis() - mp3_random_timer < mp3_random_int) return;
  mp3_random_timer = millis();
  // play a random sound
  mp3_random();
  // set the timer for next sound
  mp3_random_int = random(MP3_MIN_RANDOM_PAUSE, MP3_MAX_RANDOM_PAUSE);
}


//The mp3_init() function initiates the MP3 Trigger
void mp3_init() {
  for (uint8_t i = 0; i < MP3_MAX_BANKS; i++) {
    mp3_bank_indexes[i] = 0;
  }

  // set volume somewhere in the middle
  mp3_volumemax();

  // play the startup sound (this function return immediately)
  mp3_playstartsound();
  return;
}


//The mp3_parse_command parse all Sound commands and sends the appropriate commands to the MP3 Trigger
void mp3_parse_command(char* commandstr) {
  ////////////////////////////////////////////////
  // Play sound command by bank/sound numbers
  // Jxyy
  // x=bank number
  // yy=sound number. If none, next sound is played in the bank
  //
  // Other commands
  // Jc
  // where c is a command character
  // R - random from 4 first banks
  // O - sound off
  // L - Leia message (bank 7 sound 1)
  // C - Cantina music (bank 9 sound 5)
  // c - Beep cantina (bank 9 sound 1)
  // S - Scream (bank 6 sound 1)
  // F - Faint/Short Circuit (bank 6 sound 3)
  // D - Disco (bank 9 sound 6)
  // s - stop sounds
  // + - volume up
  // - - volume down
  // m - volume mid
  // f - volume max
  // p - volume min
  // W - Star Wars music (bank 9 sound 2)
  // M - Imperial March (bank 9 sound 3)
  //
  ///////////////////////////////////////////////


  uint8_t len = strlen(commandstr);
  // should have between 2 and 4 characters
  if (len < 2 || len > 5) {
    //Serial1.write(strSoundCmdError);
    return;
  }

  char cmdch = commandstr[1];

  // if the command character is a digit, this is a sound play command
  if (isdigit(cmdch)) {
    mp3_stop_random();                   // any manual sound command stops random automatically
    uint8_t bank = (uint8_t)cmdch - 48;  // cheap ASCII to number conversion
    uint8_t sound = 0;
    if (len > 2) {
      sound = atoi(commandstr + 2);
    }
    mp3_sound(bank, sound);
    return;
  }

  // the command is a character
  switch (cmdch) {
    case '+':  // + - volume up
      mp3_volumeup();
      break;
    case '-':  // - - volume down
      mp3_volumedown();
      break;
    case 'C':  // C - Cantina music (bank 9 sound 5)
      mp3_stop_random();
      mp3_sound(0, MP3_CANTINA);
      break;
    case 'D':  // D - Disco (bank 9 sound 6)
      mp3_stop_random();
      mp3_sound(0, MP3_DISCO);
      break;
    case 'F':  // F - Faint/Short Circuit (bank 6 sound 3)
      mp3_stop_random();
      mp3_sound(0, MP3_FAINT);
      break;
    case 'L':  // L - Leia message (bank 7 sound 1)
      mp3_stop_random();		// so long (34s), just stop random?
      mp3_sound(0, MP3_LEIA);
      break;
    case 'M':             // M - Imperial March (bank 9 sound 3)
      mp3_stop_random();  // so long, just stop random
      //mp3_suspend_random();
      mp3_sound(0, MP3_MANAMA);
      //mp3_resume_random();
      break;
    case 'O':  // O - sound off
      mp3_stop_random();
      mp3_volumeoff();
      break;
    case 'R':              // R - random from 4 first banks
      //mp3_start_random();  // keep firing random sounds
      mp3_random();		// this is just a one shot
      break;
    case 'S':  // S - Scream (bank 6 sound 1)
      mp3_stop_random();
      mp3_sound(0, MP3_SCREAM);
      break;
    case 'W':             // W - Star Wars music (bank 9 sound 2)
      mp3_stop_random();  // so long, just stop random
      //mp3_suspend_random();
      mp3_sound(8, 2);
      //mp3_resume_random();
      break;
    case 'c':  // c - Beep cantina (bank 9 sound 1)
      mp3_stop_random();
      mp3_sound(8, 1);
      break;
    case 'f':  // f - volume max
      mp3_volumemax();
      break;
    case 'm':  // m - volume mid
      mp3_volumemid();
      break;
    case 'p':  // p - volume min
      mp3_volumemin();
      break;
    case 'r':              // R - random from 4 first banks
      mp3_start_random();  // keep firing random sounds
      //mp3_random();		// this is just a one shot
      break;
    case 's':  // s - stop sounds
      mp3_stop_random();
      mp3_stop();
      break;
    default:
      //Serial1.write(strSoundCmdError);
      break;
  }
  return;
}

//The mp3_playstartsound() function plays the start sound
void mp3_playstartsound() {
  // access the start sound directly through the global bank 0
  mp3_sound(0, MP3_START_SOUND);
}

//The mp3_random() function plays a random sound from the first 4 banks
void mp3_random() {
  uint8_t num;
  // Plays a random sound from the first 5 banks only
  num = random(1, MP3_BANK1_SOUNDS + MP3_BANK2_SOUNDS + MP3_BANK3_SOUNDS + MP3_BANK4_SOUNDS + MP3_BANK5_SOUNDS);
  if (num <= MP3_BANK1_SOUNDS) {
    mp3_sound(1, num);
    return;
  }
  num -= MP3_BANK1_SOUNDS;
  if (num <= MP3_BANK2_SOUNDS) {
    mp3_sound(2, num);
    return;
  }
  num -= MP3_BANK2_SOUNDS;
  if (num <= MP3_BANK3_SOUNDS) {
    mp3_sound(3, num);
    return;
  }
  num -= MP3_BANK3_SOUNDS;
  if (num <= MP3_BANK4_SOUNDS) {
    mp3_sound(4, num);
    return;
  }
  num -= MP3_BANK4_SOUNDS;
  if (num <= MP3_BANK5_SOUNDS) {
    mp3_sound(5, num);
    return;
  }
}

//The mp3_resume_random() resumes the random sounds after a call to mp3_suspend_random()
void mp3_resume_random() {
  mp3_random_mode_flag = saveflag;
}

// Sends the hardware command, assumes MP3 is connected to suart2 on the MarcDuino
void mp3_send_command_byte(char command) {
  // sends a single byte at a time
  Serial1.write(command);
}


//Set volume for the MP3 Trigger
void mp3_setvolume(uint8_t vol) {
  mp3_send_command_byte(MP3_VOLUME_CMD);
  mp3_send_command_byte(vol);
}

// play sound from bank. If sound=0, plays next sound in bank
// for bank 1 to 4, and first sound in bank for bank 5-9
// Bank 0 can access any sound
void mp3_sound(uint8_t bank, uint8_t sound) {
  uint8_t filenum;

  if (bank > MP3_MAX_BANKS) return;
  if (bank != 0 && sound > MP3_MAX_SOUNDS_PER_BANK) return;

  // if bank=0 play the sound number provided
  if (bank == 0) filenum = sound;

  else if (sound != 0) {
    // calculate actual file number on the MP3 memory card
    filenum = (bank - 1) * MP3_MAX_SOUNDS_PER_BANK + sound;
    // also adjust last sound played index for the next sound command
    // make sure not to go past max sounds
    if (sound > mp3_max_sounds[bank])
      mp3_bank_indexes[bank] = mp3_max_sounds[bank];
    else
      mp3_bank_indexes[bank] = sound;
  }
  // sound "0", play first or next sound depending on bank
  else {
    if (bank <= MP3_BANK_CUTOFF) {
      // advance index, rewind to first sound if at end
      if ((++mp3_bank_indexes[bank]) > mp3_max_sounds[bank])
        mp3_bank_indexes[bank] = 1;
      // we'll play the new indexed sound
      sound = mp3_bank_indexes[bank];
    } else {
      // for banks that always play the first sound
      sound = 1;
    }
    filenum = (bank - 1) * MP3_MAX_SOUNDS_PER_BANK + sound;
  }

  // send a 't'nnn number where nnn=file number
  mp3_send_command_byte(MP3_PLAY_CMD);
  mp3_send_command_byte(filenum);
}


//helper function to start the random sounds
void mp3_start_random() {
  mp3_random_timer = 0;
  mp3_random_mode_flag = 1;
}

//Stops the MP3 Trigger
void mp3_stop() {
  // this doesn't work as this is a start/stop combined
  //mp3_send_command_byte(MP3_STOP_CMD);
  // instead go to an empty sound
  mp3_sound(0, MP3_EMPTY_SOUND);
  //#ifdef _MP3_DEBUG_MESSAGES_
  //	printstr("MP3 stop: ");
  //	printlnchar(MP3_STOP_CMD);
  //#endif
}

// Stops the random sounds
void mp3_stop_random() {
  mp3_random_mode_flag = 0;
  mp3_random_timer = 0;
}


//Suspends random sounds
void mp3_suspend_random() {
  mp3_random_timer = MP3_MAX_PAUSE_ON_RESUME;
  saveflag = mp3_random_mode_flag;
  mp3_random_mode_flag = 0;
}


//Turns the volume down
void mp3_volumedown() {
  uint8_t step = (MP3_VOLUME_MIN - MP3_VOLUME_MAX) / MP3_VOLUME_STEPS;
  // volume was set to off, or ended up too low
  if (mp3_volume > MP3_VOLUME_MIN) mp3_volume = MP3_VOLUME_MIN;
  else {
    // the step would be too bit, peg to minimum
    if (MP3_VOLUME_MIN - mp3_volume < step)
      mp3_volume = MP3_VOLUME_MIN;
    // go up one step (volume goes inverse with value)
    else
      mp3_volume += step;
  }
  mp3_setvolume(mp3_volume);
}

//Sets the volume to Max
void mp3_volumemax() {
  mp3_volume = MP3_VOLUME_MAX;
  mp3_setvolume(mp3_volume);
}

//Sets the volume to mid-level
void mp3_volumemid() {
  mp3_volume = MP3_VOLUME_MID;
  mp3_setvolume(mp3_volume);
}

//Sets volume to minimum audible
void mp3_volumemin() {
  mp3_volume = MP3_VOLUME_MIN;
  mp3_setvolume(mp3_volume);
}

//Turns the volume off
void mp3_volumeoff() {
  mp3_volume = MP3_VOLUME_OFF;
  mp3_setvolume(mp3_volume);
}


//Turns the volume up
void mp3_volumeup() {
  uint8_t step = (MP3_VOLUME_MIN - MP3_VOLUME_MAX) / MP3_VOLUME_STEPS;
  // volume was at max or too high
  if (mp3_volume <= MP3_VOLUME_MAX) mp3_volume = MP3_VOLUME_MAX;
  else {
    // the step would be too big, peg to maximum
    if (mp3_volume - MP3_VOLUME_MAX < step)
      mp3_volume = MP3_VOLUME_MAX;
    // go up down step (volume goes inverse with value)
    else
      mp3_volume -= step;
  }
  mp3_setvolume(mp3_volume);
}


//Parses serial command and routes it on its way
byte parseCommand(char* input_str) {
  //At this point we have a command from one of the serial interfaces
  //The first step is to determine if it is for this MPU or needs to sent to another MPU
  byte length = strlen(input_str);
  if (length < 2) goto deadCmd;  //not enough characters
  char mpu = input_str[0];
  if (MPU != mpu) {  //if command is not for this MPU - send it on its way
    switch (mpu) {
      
      case 'B':
        Serial2.flush();
        for (int i = 0; i < length; i++) Serial2.write(input_str[i]);
        Serial2.write(13);
        break;
      case 'C':
        Serial3.flush();
        for (int i = 0; i < length; i++) Serial3.write(input_str[i]);
        Serial3.write(13);
        break;
      case 'D':
      case 'E':
      case 'F':
      case 'G':
        Serial.flush();
        for (int i = 0; i < length; i++) Serial.write(input_str[i]);
        Serial.write(13);
        break;
      case 'H':
      case 'I':
        Serial2.flush();
        for (int i = 0; i < length; i++) Serial2.write(input_str[i]);
        Serial2.write(13);
        break;
      case 'J':
        mp3_parse_command(input_str);
        break;
    }
    return;
  }
  //At this stage, the command is for this MPU which means that if it as valid
  //command then it should have a minimum length of 5 and a maximum length of 7.
  //It should start with an 'A'; followed by a number between 10 and 19, an 'T' or an 'S',
  //and a 1 to 3 digit option.
  dev_MPU = mpu;  //Should be an 'A'
  char addrStr[3];  //char array to hold address
  addrStr[0] = input_str[1];
  addrStr[1] = input_str[2];
  addrStr[2] = '\0';
  dev_address = atoi(addrStr);
  if (dev_address < 9 || dev_address > 19) goto deadCmd;  //invalid address
  if (!(length > 4)) goto deadCmd;                      //invalid, no command after address
  dev_command = input_str[3];
  char optStr[4];
  optStr[0] = input_str[4];
  if(input_str[5] == 13){
    optStr[1]='\0';
    dev_option = atoi(optStr);
  }else {
    optStr[1]=input_str[5];
    if(input_str[6] == 13){
      optStr[2]='\0';
      dev_option = atoi(optStr);  
    }else{
      optStr[3]=input_str[6];
      optStr[4]='\0';
      dev_option = atoi(optStr);
    }
  }
  // switch on command character
  switch (dev_command)  // 2nd char, should be the command char
  {
    case 'T':
      doTcommand(dev_address, dev_option);
      break;
    case 'S':
      doScommand(dev_address, dev_option);
      break;
    default:
      goto deadCmd;  // unknown command
      break;
  }

  return;


deadCmd:

  return;
}

//Safely reset all body devices to their starting configuration.
void safeReset() {
  servoControl.setPWM(UA_TOP, 0, UA_TOP_MIN);
  servoControl.setPWM(UA_BOT, 0, UA_BOT_MIN);
  servoControl.setPWM(IA_EXT, 0, IA_EXT_MIN);
  servoControl.setPWM(GA_EXT, 0, GA_EXT_MIN);
  delay(2000);
  servoControl.setPWM(IA_LFT, 0, IA_LFT_MIN);
  servoControl.setPWM(GA_LFT, 0, GA_LFT_MIN);
  delay(2000);
  servoControl.setPWM(IA_DOR, 0, IA_DOR_MIN);
  servoControl.setPWM(GA_DOR, 0, GA_DOR_MIN);
}


//Handles all interactions with the utility arms
void utilityArms(int option) {
  switch (option) {
    case 0:
      return;
    case 1:
      servoControl.setPWM(UA_TOP, 0, UA_TOP_MAX);
      servoControl.setPWM(UA_BOT, 0, UA_BOT_MAX);
      ua_State = 0;
      break;
    case 2:
      servoControl.setPWM(UA_TOP, 0, UA_TOP_MIN);
      servoControl.setPWM(UA_BOT, 0, UA_BOT_MIN);
      ua_State = 0;
      break;
  }
}


/*************************************************************************
 **************************  SETUP FUNCTION  *****************************
 *************************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);   //Serial Connection with Dome Lift Mega
  Serial1.begin(9600);  //Serial Connection with MP3 Trigger
  Serial2.begin(9600);  //Serial Connection with Lights Mega
  Serial3.begin(9600);  //Serial Connection with Xbox Uno
  //Wait 3 seconds before playing startup sound
  servoControl.begin();
  servoControl.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW);
  safeReset();
  mp3_init();
}

/*************************************************************************
 ***************************  LOOP FUNCTION  *****************************
 *************************************************************************/
void loop() {
  mp3_do_random();
  // put your main code here, to run repeatedly:
  if (checkSerial()) parseCommand(cmdStr0);
  if (checkSerial3()) parseCommand(cmdStr3);
  utilityArms(ua_State);
  interfaceArm(ia_State);
  gripper(ga_State);
}

