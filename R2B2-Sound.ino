

/*************************************************************************
 * ********************** Sound Nano  ******************************
 * ***********************************************************************/
/*
The Sound Nano is responisble for all sounds coming out of the droid as it is connected to the MP3 Trigger.  
Since the nano has only one Serial port, we will connect to the MP3 Trigger using software Serial.
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
#include <SoftwareSerial.h>

SoftwareSerial MP3Serial(2, 3);
/*************************************************************************
 * *********************** MACRO DEFINITIONS  ****************************
 * ***********************************************************************/

/*>>>>>>>>>>>>>>>>>>> Servo Macros <<<<<<<<<<<<<<<<<<<<*/

#define USER_MUSIC_SOUNDS_NUMBER 5
#define MP3_MAX_BANKS 9             // nine banks
#define MP3_MAX_SOUNDS_PER_BANK 25  // no more than 25 sound in each
#define MP3_BANK_CUTOFF 5           // cutoff for banks that play "next" sound on $x

// for the random sounds, needs to know max sounds of first 5 banks
// only important for sounds below cutoff
#define MP3_BANK1_SOUNDS 25  // gen sounds, numbered 001 to 025
#define MP3_BANK2_SOUNDS 25  // chat sounds, numbered 026 to 050
#define MP3_BANK3_SOUNDS 25  // happy sounds, numbered 051 to 075
#define MP3_BANK4_SOUNDS 25  // sad sounds, numbered 076 to 100
#define MP3_BANK5_SOUNDS 25   // whistle sounds, numbered 101 to 125
// unless you change bank cutoff, these are ignored, so I set them to max
#define MP3_BANK6_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // scream sounds, numbered 126 to 150
#define MP3_BANK7_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // Leia sounds, numbered 151 to 175
#define MP3_BANK8_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // sing sounds (deprecated, not used by R2 Touch)
#define MP3_BANK9_SOUNDS MP3_MAX_SOUNDS_PER_BANK  // mus sounds, numbered 201 t0 225

// this defines where the startup sound is
#define MP3_EMPTY_SOUND 254  // workaround, used to stop sounds
#define MP3_START_SOUND 255  // startup sound is number 255
#define MP3_LEIA 151         //Leia Sound number
#define MP3_FAINT 128
#define MP3_SCREAM 126
#define MP3_DISCO 181
#define MP3_MANAMA 179
#define MP3_CANTINA 176
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
#define MP3_MAX_RANDOM_PAUSE 30000     // max wait on random sounds
#define MP3_MAX_PAUSE_ON_RESUME 30000  // default wait to resume random. Works for short sound. Set mp3_random_timer manually for long ones.
#define CMD_MAX_LENGTH 64              //Defines max command Length - same as serial buffer
#define MPU 'H'                        //Defines the MPU code for the program

/*************************************************************************
 *************************  GLOBAL VARIABLES  ****************************
 *************************************************************************/

char dev_MPU, dev_command;
char cmdStr[CMD_MAX_LENGTH];

int dev_option, dev_address;
unsigned long current_time = millis();
unsigned long mp3_random_timer = millis();
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



/*************************************************************************
 ***********************  FUNCTION DEFINITIONS  **************************
 *************************************************************************/
/*



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

//The checkSerial function is the first thread of seven threads in this program.  It checks the Serial0 buffer for incoming serial
//data and then sends it to be processed.
void checkSerial() {
  char ch;
  byte cmd_Complete;
  if (Serial.available()) {
    ch = Serial.read();
    //Serial.print(ch);
    cmd_Complete = buildCommand(ch, cmdStr);
    if (cmd_Complete) {
      parseCommand(cmdStr);
      //Serial.println();
    }
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
    //MP3Serial.write(strSoundCmdError);
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
    case 'L':             // L - Leia message (bank 7 sound 1)
      mp3_stop_random();  // so long (34s), just stop random?
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
    case 'R':  // R - random from 4 first banks
      mp3_start_random();  // keep firing random sounds
      //mp3_random();  // this is just a one shot
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
      //mp3_start_random();  // keep firing random sounds
      mp3_random();		// this is just a one shot
      break;
    case 's':  // s - stop sounds
      mp3_stop_random();
      mp3_stop();
      break;
    default:
      //MP3Serial.write(strSoundCmdError);
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
  MP3Serial.write(command);
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
  if (input_str[0] == 'J') mp3_parse_command(input_str);
  Serial.flush();
  return;
}





/*************************************************************************
 **************************  SETUP FUNCTION  *****************************
 *************************************************************************/

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     //Serial Connection with Body Lights Mega
  MP3Serial.begin(9600);  //Serial Connection with MP3 Trigger
  //Wait 3 seconds before playing startup sound
  delay(4000);
  mp3_init();
}

/*************************************************************************
 ***************************  LOOP FUNCTION  *****************************
 *************************************************************************/
void loop() {
  mp3_do_random();
  // put your main code here, to run repeatedly:
  checkSerial();
}
