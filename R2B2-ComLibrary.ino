/*************************************************************************
 * ********************** Body Master Mega  ******************************
 * ***********************************************************************/
/*
The Body Master Mega is used to route communications between the Dome Lift Mega, the Body Lights Mega
and the Drive Uno.  It is also used to control all the body servos except the Dataport door servo. 
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



#define CMD_MAX_LENGTH 64  //Defines max command Length - same as serial buffer
#define MPU 'A'            //Defines the MPU code for the program

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

        Serial1.flush();
        for (int i = 0; i < length; i++) Serial1.write(input_str[i]);
        Serial1.write(13);
        break;
      case 'I':
        Serial2.flush();
        for (int i = 0; i < length; i++) Serial2.write(input_str[i]);
        Serial2.write(13);
        break;
      case 'J':
        Serial1.flush();
        for (int i = 0; i < length; i++) Serial1.write(input_str[i]);
        Serial1.write(13);
        break;
    }
    return;
  }
  //At this stage, the command is for this MPU which means that if it as valid
  //command then it should have a minimum length of 5 and a maximum length of 7.
  //It should start with an 'A'; followed by a number between 10 and 19, an 'T' or an 'S',
  //and a 1 to 3 digit option.
  dev_MPU = mpu;    //Should be an 'A'
  char addrStr[3];  //char array to hold address
  addrStr[0] = input_str[1];
  addrStr[1] = input_str[2];
  addrStr[2] = '\0';
  dev_address = atoi(addrStr);
  if (dev_address < 9 || dev_address > 19) goto deadCmd;  //invalid address
  if (!(length > 4)) goto deadCmd;                        //invalid, no command after address
  dev_command = input_str[3];
  char optStr[4];
  optStr[0] = input_str[4];
  if (input_str[5] == 13) {
    optStr[1] = '\0';
    dev_option = atoi(optStr);
  } else {
    optStr[1] = input_str[5];
    if (input_str[6] == 13) {
      optStr[2] = '\0';
      dev_option = atoi(optStr);
    } else {
      optStr[3] = input_str[6];
      optStr[4] = '\0';
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
  Serial1.begin(9600);  //Serial Connection with Sound Nano
  Serial2.begin(9600);  //Serial Connection with Lights Mega
  Serial3.begin(9600);  //Serial Connection with Xbox Uno
  //Wait 3 seconds before playing startup sound
  servoControl.begin();
  servoControl.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW);
  safeReset();
  
}

/*************************************************************************
 ***************************  LOOP FUNCTION  *****************************
 *************************************************************************/
void loop() {
  
  // put your main code here, to run repeatedly:
  if (checkSerial()) parseCommand(cmdStr0);
  if (checkSerial3()) parseCommand(cmdStr3);
  utilityArms(ua_State);
  interfaceArm(ia_State);
  gripper(ga_State);
}
