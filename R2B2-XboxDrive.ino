/*************************************************************************
 ************************** XBOX DRIVE UNO *******************************
 *************************************************************************/

/* The Xbox Drive Uno controls the right and left drive wheels and the Dome Rotation Motor.  All 
other commands are sent to the Body Master Mega for routing.   R2 is powered by two 12V Neo Brushless 
Motors attached to Spark Max motor controllers.  We will be controlling the Spark Max using PWM.  DO 
NOT use pins 5 and 6 with the Spark Max controller as 5 and 6 are 980 Hz PWM pin and will burn out the
Spark Max (Ask me how I know this!!!).



/*************************************************************************
 * ********************** INCLUDED LIBRARIES *****************************
 * ***********************************************************************/

#include <XBOXRECV.h>  //Needed for XBox Tranceiver
#include <Servo.h>     //Needed for controlling Spark MAX motor controller
#include <SPI.h>       //Needed for XBOXRECV.h

/*************************************************************************
 * ********************** MACRO DEFINITIONS ******************************
 * ***********************************************************************/

#define CONTROLLER 0          //Set the XBox Controller number to 0
#define DEAD_ZONE 8000        //Set the Dead zone for the Joysticks on Xbox controller
#define TRIGGER_DEAD_ZONE 50  //Set the Trigger dead zone
#define LEFT_PWM 3            //Set the left motor controller to pin 3 - do not use 5 & 6
#define RIGHT_PWM 9           //Set the right motor controller to pin 9  (3 & 9 are 490 Hz, 5 & 6 are 980 Hz)
#define CMD_MAX_LENGTH 12     //Set the max command length for this program
#define DOME_ENABLE 4         //Set the Enable pin for the Dome motor
#define IN1_DOME_MOTOR 2      //Set the INPUT1 pin for the Dome motor
#define IN2_DOME_MOTOR 5      //Set the INPUT2 pin for the Dome motor

#define UA_ON "A10T1\r"   //Utility Arm on command
#define UA_OFF "A10T2\r"  //Utility arm off command
#define IA_ON "A11T1\r"   //Interface Arm on command
#define IA_OFF "A11T2\r"  //Interface arm off command
#define GA_ON "A12T1\r"   //Gripper Arm on command
#define GA_OFF "A12T2\r"  //Gripper arm off command
#define DP_ON "B24T1\r"   //Dataport on command
#define DP_OFF "B24T2\r"  //Dataport off command
#define ZP_ON "E51T8\r"   //Zapper on command
#define ZP_OFF "E51T8\r"  //Zapper off command
#define LS_ON "E52T1\r"   //Light Saber on command
#define LS_OFF "E52T2\r"  //Light Saber off command
#define PS_ON "E53T1\r"   //Periscope on command
#define PS_OFF "E53T2\r"  //Periscope off command
#define BM_ON "E54T1\r"   //Bad Motivator on command
#define BM_OFF "E54T2\r"  //Bad Motivator off command
#define LF_ON "E55T1\r"   //Life Form on command
#define LF_OFF "E55T2\r"  //Life Form off command
#define BEEP01 "J01\r"  //General Beep 1
#define BEEP02 "J02\r"  //General Beep 2
#define BEEP03 "J03\r"  //General Beep 3
#define BEEP04 "J04\r"  //General Beep 4
#define BEEP05 "J11\r"  //Happy Beep 1
#define BEEP06 "J12\r"  //Happy Beep 2
#define BEEP07 "J13\r"  //Happy Beep 3
#define BEEP08 "J14\r"  //Happy Beep 4
#define BEEP09 "J21\r"  //Sad Beep 1
#define BEEP10 "J22\r"  //Sad Beep 2
#define BEEP11 "J23\r"  //Sad Beep 3
#define BEEP12 "J24\r"  //Sad Beep 4
#define BEEP13 "J31\r"  //Chatty Beep 1
#define BEEP14 "J32\r"  //Chatty Beep 2
#define BEEP15 "J33\r"  //Chatty Beep 1
#define BEEP16 "J34\r"  //Chatty Beep 2
#define ROUTSCREAM "E50T31\r" //Scream Routine  
#define ROUTWAVE "E50T32\r" //Wave Routine  
#define ROUTWAVE1 "E50T33\r" //Moody Wave Routine  
#define ROUTWAVE2 "E50T34\r" //Open Wave Routine  
#define ROUTFAINT "E50T35\r" //Faint/Short Circuit Routine  
#define ROUTCANTINA "E50T36\r" //Cantina Routine  
#define ROUTLEIA "E50T37\r"   //Leia Routine  
#define ROUTDISCO "E50T38\r"  //Disco Routine


/*************************************************************************
 * ******************************* STRUCTS *******************************
 * ***********************************************************************/

//Create a Command struct containing two char arrays and one bool value
typedef struct {
  /// The command to run if it needs to turn on.
  char onCommand[CMD_MAX_LENGTH];
  /// The command to run if it needs to turn off.
  char offCommand[CMD_MAX_LENGTH];
  /// Whether or not this command is on or off.
  bool isOn;
} Command;

/*************************************************************************
 ************************** FUNCTION DEFINITIONS *************************
 *************************************************************************/
Command createCommand(const char* onCommand, const char* offCommand);
Command runCommand(Command* command);

/*************************************************************************
 * ********************** GLOBAL VARIABLES *******************************
 * ***********************************************************************/


//determines whether drive wheels are enabled
bool driveEnabled = false;
byte comSet = 1;

USB Usb;              //Creates a USB object
XBOXRECV Xbox(&Usb);  //Creates a XBOXRECV object called Xbox and attached to the USB object
Servo LFoot;          //Create a servo object for left foot
Servo RFoot;          //Create a servo object for right foot
Command utility = createCommand(UA_ON, UA_OFF);
Command interface = createCommand(IA_ON, IA_OFF);
Command gripper = createCommand(GA_ON, GA_OFF);
Command dataport = createCommand(DP_ON, DP_OFF);
Command zapper = createCommand(ZP_ON, ZP_OFF);
Command lightSaber = createCommand(LS_ON, LS_OFF);
Command periscope = createCommand(PS_ON, PS_OFF);
Command motivator = createCommand(BM_ON, BM_OFF);
Command lifeForm = createCommand(LF_ON, LF_OFF);
Command sound1 = createCommand(BEEP01, BEEP02);
Command sound2 = createCommand(BEEP03, BEEP04);
Command sound3 = createCommand(BEEP05, BEEP06);
Command sound4 = createCommand(BEEP07, BEEP08);
Command sound5 = createCommand(BEEP09, BEEP10);
Command sound6 = createCommand(BEEP11, BEEP12);
Command sound7 = createCommand(BEEP13, BEEP14);
Command sound8 = createCommand(BEEP15, BEEP16);
Command holos = createCommand("D40T1\r", "D40T0\r");
Command magicpanel = createCommand("D45T1\r", "D45T0\r");
Command coinslots = createCommand("B21T5\r","B21T0\r");
bool mtrsEnable = 0;




/*************************************************************************
 ******************************* FUNCTIONS *******************************
 *************************************************************************/

/// Creates a command struct.
Command createCommand(const char* onCommand, const char* offCommand) {
  Command newCommand;
  newCommand.isOn = false;
  strcpy(newCommand.onCommand, onCommand);
  strcpy(newCommand.offCommand, offCommand);
  return newCommand;
}

/// Runs the correct command in a `Command` struct.
Command runCommand(Command* command) {
  auto commandString = !command->isOn ? command->onCommand : command->offCommand;
  Serial.write(commandString);
  command->isOn = !command->isOn;
}


/*************************************************************************
 ***************************** SETUP FUNCTION ****************************
 *************************************************************************/
void setup() {
  Serial.begin(9600);  //Connection with MPU A - Body Master


  if (Usb.Init() == -1) {
    //Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  //halt
  }
  LFoot.attach(LEFT_PWM);
  LFoot.writeMicroseconds(1500);
  RFoot.attach(RIGHT_PWM);
  RFoot.writeMicroseconds(1500);
  LFoot.detach();
  RFoot.detach();
  pinMode(DOME_ENABLE, OUTPUT);
  pinMode(IN1_DOME_MOTOR, OUTPUT);
  pinMode(IN2_DOME_MOTOR, OUTPUT);
}




/*************************************************************************
 ***************************** LOOP FUNCTION *****************************
 *************************************************************************/
void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    if (Xbox.Xbox360Connected[CONTROLLER]) {
      //Check the Left Joystick "LeftHat" Y axis for input
      if (Xbox.getAnalogHat(LeftHatY) > DEAD_ZONE || Xbox.getAnalogHat(LeftHatY) < -DEAD_ZONE) {
        int lMtrSpeed;  //tracks left motor  speed
        //Check for positive stick
        if (Xbox.getAnalogHat(LeftHatY) > DEAD_ZONE) {
          lMtrSpeed = map(Xbox.getAnalogHat(LeftHatY), DEAD_ZONE, 32767, 1500, 1750);
          LFoot.writeMicroseconds(lMtrSpeed);
        }
        //Check for negative stick
        if (Xbox.getAnalogHat(LeftHatY) < -DEAD_ZONE) {
          lMtrSpeed = map(Xbox.getAnalogHat(LeftHatY), -DEAD_ZONE, -32768, 1500, 1250);
          LFoot.writeMicroseconds(lMtrSpeed);
        }
      }
      //if the stick is in the DEAD_ZONE - shut down the motor
      if (Xbox.getAnalogHat(LeftHatY) < DEAD_ZONE && Xbox.getAnalogHat(LeftHatY) > -DEAD_ZONE) {
        LFoot.writeMicroseconds(1500);
      }
      //Check the Right Joystick "RightHat" Y axis for input
      if (Xbox.getAnalogHat(RightHatY) > DEAD_ZONE || Xbox.getAnalogHat(RightHatY) < -DEAD_ZONE) {
        int rMtrSpeed;
        //Check for positive stick
        if (Xbox.getAnalogHat(RightHatY) > DEAD_ZONE) {
          rMtrSpeed = map(Xbox.getAnalogHat(RightHatY), DEAD_ZONE, 32767, 1500, 1250);
          RFoot.writeMicroseconds(rMtrSpeed);
        }
        //Check for negative stick
        if (Xbox.getAnalogHat(RightHatY) < -DEAD_ZONE) {
          rMtrSpeed = map(Xbox.getAnalogHat(RightHatY), -DEAD_ZONE, -32768, 1500, 1750);
          RFoot.writeMicroseconds(rMtrSpeed);
        }
      }
      //if the stick is in the DEAD_ZONE - shut down the motor
      if (Xbox.getAnalogHat(RightHatY) < DEAD_ZONE && Xbox.getAnalogHat(RightHatY) > -DEAD_ZONE) {
        RFoot.writeMicroseconds(1500);
      }


      //Check the Left and Right triggers
      if (Xbox.getButtonPress(L2) > TRIGGER_DEAD_ZONE) {

        analogWrite(DOME_ENABLE, Xbox.getButtonPress(L2));
        digitalWrite(IN1_DOME_MOTOR, LOW);
        digitalWrite(IN2_DOME_MOTOR, HIGH);
      }
      if (Xbox.getButtonPress(R2) > TRIGGER_DEAD_ZONE) {
        analogWrite(DOME_ENABLE, Xbox.getButtonPress(R2));
        digitalWrite(IN1_DOME_MOTOR, HIGH);
        digitalWrite(IN2_DOME_MOTOR, LOW);
      }
      if (Xbox.getButtonClick(XBOX)) {
        mtrsEnable = !mtrsEnable;
        if (mtrsEnable) {
          Xbox.setLedMode(ROTATING);
          LFoot.attach(LEFT_PWM);
          LFoot.writeMicroseconds(1500);
          RFoot.attach(RIGHT_PWM);
          RFoot.writeMicroseconds(1500);
        } else {
          Xbox.setLedOff();
          Xbox.setLedOn((LEDEnum)comSet);
          LFoot.detach();
          RFoot.detach();
        }
      }
      if (Xbox.getButtonClick(L3)) {
        if(comSet>1)comSet--;
        else comSet=4;
        Xbox.setLedOn((LEDEnum)comSet);
      }
      if (Xbox.getButtonClick(R3)){
        if(comSet<4)comSet++;
        else comSet=1;
        Xbox.setLedOn((LEDEnum)comSet);
      } 
      if (Xbox.getButtonClick(SYNC)) {
        //Serial.println(F("Sync"));
        Xbox.disconnect(CONTROLLER);
      }
      switch (comSet) {
        case 1:  //activates servo and lights
          if (Xbox.getButtonClick(UP)) runCommand(&utility);
          if (Xbox.getButtonClick(DOWN)) runCommand(&dataport);
          if (Xbox.getButtonClick(LEFT)) runCommand(&holos);
          if (Xbox.getButtonClick(RIGHT)) runCommand(&magicpanel);
          if (Xbox.getButtonClick(START)) runCommand(&zapper);
          if (Xbox.getButtonClick(BACK)) runCommand(&coinslots);
          if (Xbox.getButtonClick(L1)) runCommand(&gripper);
          if (Xbox.getButtonClick(R1)) runCommand(&interface);
          if (Xbox.getButtonClick(A)) runCommand(&lightSaber);
          if (Xbox.getButtonClick(B)) runCommand(&periscope);
          if (Xbox.getButtonClick(X)) runCommand(&motivator);
          if (Xbox.getButtonClick(Y)) runCommand(&lifeForm);
          break;
        case 2:  //Prepackaged Routines - Full Animation and sound
          if (Xbox.getButtonClick(UP)) Serial.write(ROUTSCREAM);
          if (Xbox.getButtonClick(DOWN)) Serial.write(ROUTWAVE);
          if (Xbox.getButtonClick(LEFT)) Serial.write(ROUTWAVE1);
          if (Xbox.getButtonClick(RIGHT)) Serial.write(ROUTWAVE2);
          if (Xbox.getButtonClick(A)) Serial.write(ROUTCANTINA);
          if (Xbox.getButtonClick(B)) Serial.write(ROUTDISCO);
          if (Xbox.getButtonClick(X)) Serial.write(ROUTFAINT);
          if (Xbox.getButtonClick(Y)) Serial.write(ROUTLEIA);
          break;
        case 3:
          if (Xbox.getButtonClick(UP)) runCommand(&sound1);
          if (Xbox.getButtonClick(DOWN)) runCommand(&sound2);
          if (Xbox.getButtonClick(LEFT)) runCommand(&sound3);
          if (Xbox.getButtonClick(RIGHT)) runCommand(&sound4);
          if (Xbox.getButtonClick(A)) runCommand(&sound5);
          if (Xbox.getButtonClick(B)) runCommand(&sound6);
          if (Xbox.getButtonClick(X)) runCommand(&sound7);
          if (Xbox.getButtonClick(Y)) runCommand(&sound8);
          break;

      }
    }
  }
}
