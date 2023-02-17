/*
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
// $xyy
// x=bank number
// yy=sound number. If none, next sound is played in the bank
//
// Other commands
// $c
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

#include <time.h>
#include <stdlib.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver servoControl = Adafruit_PWMServoDriver();


#define UA_TOP 0
#define UA_TOP_MAX 200
#define UA_TOP_MIN 500
#define UA_BOT 1
#define UA_BOT_MAX 200
#define UA_BOT_MIN 450
#define IA_DOR 2
#define IA_DOR_MAX 275
#define IA_DOR_MIN 400
#define IA_LFT 3
#define IA_LFT_MAX 475
#define IA_LFT_MIN 150
#define IA_EXT 4
#define IA_EXT_MAX 200
#define IA_EXT_MIN 400
#define GA_DOR 5
#define GA_DOR_MAX 200
#define GA_DOR_MIN 400
#define GA_LFT 6
#define GA_LFT_MAX 200
#define GA_LFT_MIN 400
#define GA_EXT 7
#define GA_EXT_MAX 200
#define GA_EXT_MIN 400
#define DP_DOR 8
#define DP_DOR_MAX 200
#define DP_DOR_MIN 400
#define OE_PIN 8


/////////////////////////////////////////////////////////////////////
// Adjust your total number of music sounds you put on the card here
/////////////////////////////////////////////////////////////////////
#define USER_MUSIC_SOUNDS_NUMBER 5

#define SERVO_FREQ 50
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
#define SOUND_START_CHAR '$'  //Lead character for sound commands
#define MP3_VOLUME_MID 50     // guessing mid volume 32 is right in-between...
#define MP3_VOLUME_MIN 100    // doc says anything below 64 is inaudible, not true, 100 is. 82 is another good value
#define MP3_VOLUME_MAX 0      // doc says max is 0
#define MP3_VOLUME_STEPS 20   // R2 Touch app has 20 steps from min to max
#define MP3_VOLUME_OFF 254    // to turn it off... 255 gets a buzz.

#define MP3_PLAY_CMD 't'              // command to play sound file on the MP3 trigger
#define MP3_VOLUME_CMD 'v'            // command to play sound file on the MP3 trigger
#define MP3_STOP_CMD 'O'              // command to stop/play  - not used
#define MP3_MIN_RANDOM_PAUSE 600      // min wait on random sounds
#define MP3_MAX_RANDOM_PAUSE 1000     // max wait on random sounds
#define MP3_MAX_PAUSE_ON_RESUME 1200  // default wait to resume random. Works for short sound. Set mp3_random_timer manually for long ones.
#define CMD_MAX_LENGHT 64             //Defines max command Length - same as serial buffer
#define MPU 'A'                       //Defines the MPU code for the program


// public
void mp3_init();  // wait at least 3s after mp3trigger power up before calling
void mp3_parse_command(char* commandstr);
void mp3_do_random();  // need to be called in the main loop for random sounds to work

// utilities
void mp3_sound(uint8_t bank, uint8_t sound);
void mp3_stop();
void mp3_playstartsound();
void mp3_random();
void mp3_volumeup();
void mp3_volumedown();
void mp3_volumemid();
void mp3_volumeoff();
void mp3_volumemax();
void mp3_volumemin();

// private
void mp3_send_command_byte(char command);
void mp3_setvolume(uint8_t vol);
void mp3_suspend_random();
void mp3_resume_random();
void mp3_stop_random();
void mp3_start_random();
void mp3_check_timer();



int dev_option, dev_address;
char dev_MPU, dev_command;
char cmdStr0[CMD_MAX_LENGHT];
char cmdStr1[CMD_MAX_LENGHT];
char cmdStr2[CMD_MAX_LENGHT];
char cmdStr3[CMD_MAX_LENGHT];
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
static uint8_t mp3_volume = MP3_VOLUME_MID;
static uint8_t saveflag;
const char strSoundCmdError[] PROGMEM = "Invalid MP3Trigger Sound Command";
uint8_t mp3_random_mode_flag = 0;  // the switch to check random sound mode
byte ua_State = 0;
byte ia_State = 0;
byte ga_State = 0;
byte dp_State = 0;

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
//

byte parseCommand(char* input_str) {
  byte hasArgument = false;
  int argument;
  int address;
  Serial.println("parse");
  byte pos = 0;
  byte length = strlen(input_str);
  if (length < 2) goto deadCmd;  //not enough characters
  int mpu = input_str[pos];
  if (MPU != mpu) {  //if command is not for this MPU - send it on its way
    switch (mpu) {
      case '$':
        mp3_parse_command(input_str);
        break;
      case 'B':
        for (int i = 0; i < length; i++) Serial2.write(input_str[i]);
        Serial2.write(13);
        break;
      case 'C':
        for (int i = 0; i < length; i++) Serial3.write(input_str[i]);
        Serial3.write(13);
        break;
      case 'D':
      case 'E':
      case 'F':
      case 'G':
        for (int i = 0; i < length; i++) Serial.write(input_str[i]);
        Serial.write(13);
        break;
      case 'H':
      case 'I':
        for (int i = 0; i < length; i++) Serial2.write(input_str[i]);
        Serial2.write(13);
        break;
    }
    return;
  }
  pos++;
  if (mpu > 64 && mpu < 73) dev_MPU = mpu;
  else goto deadCmd;  //Not a valid MPU - end command
  char addrStr[3];
  //next we need to get the device address which could be 1 or two characters
  if (!isdigit(input_str[pos])) goto deadCmd;  //Invalid as first char not a digit
  addrStr[pos - 1] = input_str[pos];
  pos++;
  if (isdigit(input_str[pos])) {
    addrStr[pos - 1] = input_str[pos];
    pos++;
  }
  addrStr[pos - 1] = '\0';
  dev_address = atoi(addrStr);
  if (!length > pos) goto deadCmd;  //invalid, no command after address
                                    //check for the special case message command 'M'
  dev_command = input_str[pos];
  pos++;                                   // need to increment in order to peek ahead of command char
  if (!length > pos) hasArgument = false;  // end of string reached, no arguments
  else {
    for (byte i = pos; i < length; i++) {
      if (!isdigit(input_str[i])) goto deadCmd;  // invalid, end of string contains non-numerial arguments
    }
    dev_option = atoi(input_str + pos);  // that's the numerical argument after the command character
    hasArgument = true;
  }
  // switch on command character
  switch (dev_command)  // 2nd or third char, should be the command char
  {
    case 'T':
      if (!hasArgument) goto deadCmd;  // invalid, no argument after command
      doTcommand(dev_address, dev_option);
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
    case 13:
      dp_State = argument;
      break;
  }
}



void doScommand(int address, int argument) {
  switch (argument) {
  }
}

byte buildCommand(char ch, char* output_str) {
  static int pos = 0;
  switch (ch) {
    case '\r':  //end character reached
    case '\n':
    case '\0':
      output_str[pos] = '\0';
      pos = 0;
      return true;
      break;
    default:
      output_str[pos] = ch;
      if (pos <= CMD_MAX_LENGHT - 1) pos++;
      break;
  }
  return false;
}


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

  servoControl.setPWM(UA_TOP, 0, UA_TOP_MIN);
  servoControl.setPWM(UA_BOT, 0, UA_BOT_MIN);

  //servoControl.setPWM(IA_EXT, 0, IA_EXT_MIN);
  //servoControl.setPWM(IA_LFT, 0, IA_LFT_MIN);
  //servoControl.setPWM(IA_DOR, 0, IA_DOR_MAX);
  delay(3000);
  mp3_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (checkSerial()) parseCommand(cmdStr0);
  if (checkSerial1()) parseCommand(cmdStr1);
  if (checkSerial2()) parseCommand(cmdStr2);
  if (checkSerial3()) parseCommand(cmdStr3);
  utilityArms(ua_State);
  interfaceArm(ia_State);
  gripper(ga_State);
  dataport(dp_State);
}

void dataport(int option){
  switch(option){
    case 0:
      return;
    case 1:
      Serial2.write("B")
      servoControl.setPWM(DP_DOR, 0, DP_DOR_MAX);

  }
}



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
          if (current_time - door_time > door_int) {
            door_time = current_time;
            servoControl.setPWM(GA_LFT, 0, GA_LFT_MAX);
            step++;
          }
          break;
        case 2:
          if (current_time - lift_time > lift_int) {
            lift_time = current_time;
            servoControl.setPWM(GA_EXT, 0, GA_EXT_MAX);
            step = 3;
          }
          count++;
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
      switch(step){
        case 0:
          servoControl.setPWM(GA_EXT, 0, GA_EXT_MIN);
          step=1;
          break;
        case 1:
          if(current_time-ext_time>ext_int){
            ext_time=current_time;
            servoControl.setPWM(GA_LFT, 0, GA_LFT_MIN);
            step=2;
          }
          break;
        case 2:
          if(current_time-door_time>door_int){
            ext_time=current_time;
            servoControl.setPWM(GA_DOR, 0, GA_DOR_MIN);
            step=0;
            ga_State=0;
          }
          break;
      }    
      break;
  }
}

void interfaceArm(int option) {
  static int step = 0;
  static int count = 0;
  switch (option) {

    current_time = millis();
    case 0:
      return;
    case 1:
      switch (step) {
        case 0:
          servoControl.setPWM(IA_DOR, 0, IA_DOR_MAX);
          step++;
          break;
        case 1:
          if (current_time - door_time > door_int) {
            door_time = current_time;
            servoControl.setPWM(IA_LFT, 0, IA_LFT_MAX);
            step++;
          }
          break;
        case 2:
          if (current_time - lift_time > lift_int) {
            lift_time = current_time;
            servoControl.setPWM(IA_EXT, 0, IA_EXT_MAX);
            step = 3;
          }
          count++;
          break;
        case 3:
          if (current_time - ext_time > ext_int) {
            ext_time = current_time;
            servoControl.setPWM(IA_EXT, 0, IA_EXT_MIN);
            step = 2;
            count++;
            if (count == 3) {
              step = 0;
              count = 0;
              ia_State = 0;
            }
          }

          break;
      }
      break;
    case 2:
      switch(step){
        case 0:
          servoControl.setPWM(IA_EXT, 0, IA_EXT_MIN);
          step=1;
          break;
        case 1:
          if(current_time-ext_time>ext_int){
            ext_time=current_time;
            servoControl.setPWM(IA_LFT, 0, IA_LFT_MIN);
            step=2;
          }
          break;
        case 2:
          if(current_time-door_time>door_int){
            ext_time=current_time;
            servoControl.setPWM(IA_DOR, 0, IA_DOR_MIN);
            step=0;
            ia_State=0;
          }
          break;
      }    
      break;
  }
}



void utilityArms(int option) {
  switch (option) {
    case 0:
      return;
    case 1:
      servoControl.setPWM(UA_TOP, 0, UA_TOP_MAX);
      servoControl.setPWM(UA_BOT, 0, UA_BOT_MAX);
      break;
    case 2:
      servoControl.setPWM(UA_TOP, 0, UA_TOP_MIN);
      servoControl.setPWM(UA_BOT, 0, UA_BOT_MIN);
      break;
  }
}


// MP3 Trigger Routines

void mp3_init() {
  for (uint8_t i = 0; i < MP3_MAX_BANKS; i++) {
    mp3_bank_indexes[i] = 0;
  }

  // set volume somewhere in the middle
  mp3_volumemid();

  // play the startup sound (this function return immediately)
  mp3_playstartsound();
}


void mp3_do_random() {
  if (!mp3_random_mode_flag) return;
  if (millis() - mp3_random_timer < mp3_random_int) return;
  mp3_random_timer = millis();
  // play a random sound
  mp3_random();
  // set the timer for next sound
  mp3_random_int = random(MP3_MIN_RANDOM_PAUSE, MP3_MAX_RANDOM_PAUSE);
}

void mp3_volumemid() {
  mp3_volume = MP3_VOLUME_MID;
  mp3_setvolume(mp3_volume);
}

void mp3_setvolume(uint8_t vol) {
  mp3_send_command_byte(MP3_VOLUME_CMD);
  mp3_send_command_byte(vol);
}

// Sends the hardware command, assumes MP3 is connected to suart2 on the MarcDuino
void mp3_send_command_byte(char command) {
  // sends a single byte at a time
  Serial1.write(command);
}

void mp3_playstartsound() {
  // access the start sound directly through the global bank 0
  mp3_sound(0, MP3_START_SOUND);
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

void mp3_parse_command(char* commandstr) {
  ////////////////////////////////////////////////
  // Play sound command by bank/sound numbers
  // $xyy
  // x=bank number
  // yy=sound number. If none, next sound is played in the bank
  //
  // Other commands
  // $c
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
  // check the start character
  if (commandstr[0] != SOUND_START_CHAR) {
    Serial1.write(strSoundCmdError);
    return;
  }

  // should have between 2 and 4 characters
  if (len < 2 || len > 4) {
    Serial1.write(strSoundCmdError);
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
    case 'R':              // R - random from 4 first banks
      mp3_start_random();  // keep firing random sounds
      //mp3_random();		// this is just a one shot
      break;
    case 'O':  // O - sound off
      mp3_stop_random();
      mp3_volumeoff();
      break;
    case 'L':  // L - Leia message (bank 7 sound 1)
      //mp3_stop_random();		// so long (34s), just stop random?
      mp3_suspend_random();
      mp3_sound(7, 1);
      mp3_random_timer = 4400;  // 34s + 10s extra long delay
      mp3_resume_random();
      break;
    case 'C':  // C - Cantina music (bank 9 sound 5)
      //mp3_stop_random();		// so long, just stop random
      mp3_suspend_random();
      mp3_sound(8, 5);
      mp3_random_timer = 5600;  // extra long delay
      mp3_resume_random();
      break;
    case 'c':  // c - Beep cantina (bank 9 sound 1)
      mp3_suspend_random();
      mp3_sound(8, 1);
      mp3_random_timer = 2700;  // extra long delay
      mp3_resume_random();
      break;
    case 'S':  // S - Scream (bank 6 sound 1)
      mp3_suspend_random();
      mp3_sound(6, 1);
      mp3_resume_random();
      break;
    case 'F':  // F - Faint/Short Circuit (bank 6 sound 3)
      mp3_suspend_random();
      mp3_sound(6, 3);
      mp3_resume_random();
      break;
    case 'D':  // D - Disco (bank 9 sound 6)
      mp3_suspend_random();
      mp3_sound(8, 6);
      mp3_random_timer = 39600;  // 6:26 +10s min extra long delay
      mp3_resume_random();
      break;
    case 's':  // s - stop sounds
      mp3_stop_random();
      mp3_stop();
      break;
    case '+':  // + - volume up
      mp3_volumeup();
      break;
    case '-':  // - - volume down
      mp3_volumedown();
      break;
    case 'm':  // m - volume mid
      mp3_volumemid();
      break;
    case 'f':  // f - volume max
      mp3_volumemax();
      break;
    case 'p':  // p - volume min
      mp3_volumemin();
      break;
    case 'W':             // W - Star Wars music (bank 9 sound 2)
      mp3_stop_random();  // so long, just stop random
      //mp3_suspend_random();
      mp3_sound(8, 2);
      //mp3_resume_random();
      break;
    case 'M':             // M - Imperial March (bank 9 sound 3)
      mp3_stop_random();  // so long, just stop random
      //mp3_suspend_random();
      mp3_sound(8, 3);
      //mp3_resume_random();
      break;
    default:
      Serial1.write(strSoundCmdError);
      break;
  }
}

void mp3_start_random() {
  mp3_random_timer = 0;
  mp3_random_mode_flag = 1;
}

void mp3_stop_random() {
  mp3_random_mode_flag = 0;
  mp3_random_timer = 0;
}

void mp3_suspend_random() {
  mp3_random_timer = MP3_MAX_PAUSE_ON_RESUME;
  saveflag = mp3_random_mode_flag;
  mp3_random_mode_flag = 0;
}

void mp3_resume_random() {
  mp3_random_mode_flag = saveflag;
}

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

void mp3_volumeoff() {
  mp3_volume = MP3_VOLUME_OFF;
  mp3_setvolume(mp3_volume);
}

void mp3_volumemax() {
  mp3_volume = MP3_VOLUME_MAX;
  mp3_setvolume(mp3_volume);
}

void mp3_volumemin() {
  mp3_volume = MP3_VOLUME_MIN;
  mp3_setvolume(mp3_volume);
}
