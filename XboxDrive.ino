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

#define UA_ON "A10T1"         //Utility Arm on command
#define UA_OFF "A10T2"        //Utility arm off command
#define IA_ON "A11T1"         //Interface Arm on command
#define IA_OFF "A11T2"        //Interface arm off command
#define GA_ON "A12T1"         //Gripper Arm on command
#define GA_OFF "A12T2"        //Gripper arm off command
#define DP_ON "B24T1"         //Dataport on command
#define DP_OFF "B24T2"        //Dataport off command
#define ZP_ON  "E51T8"        //Zapper on command
#define ZP_OFF "E51T8"        //Zapper off command
#define LS_ON  "E52T1"        //Light Saber on command
#define LS_OFF "E52T2"        //Light Saber off command
#define PS_ON  "E53T1"        //Periscope on command
#define PS_OFF "E53T2"        //Periscope off command
#define BM_ON  "E54T1"        //Bad Motivator on command
#define BM_OFF "E54T2"        //Bad Motivator off command
#define LF_ON  "E55T1"        //Life Form on command
#define LF_OFF "E55T2"        //Life Form off command
#define SF_ON  "$S"           //Scream command
#define SF_OFF "$F"           //Faint command                              
#define LW_ON  "$L"           //Leia command
#define LW_OFF "$W"           //Star Wars Music
#define MU_ON  "$D"           //Disco Music
#define MU_OFF "$C"            //Cantina music

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
USB Usb;     //Creates a USB object
XBOXRECV Xbox(&Usb);  //Creates a XBOXRECV object called Xbox and attached to the USB object
Servo LFoot;   //Create a servo object for left foot
Servo RFoot;   //Create a servo object for right foot
Command utility = createCommand(UA_ON, UA_OFF);
Command interface = createCommand(IA_ON, IA_OFF);
Command gripper = createCommand(GA_ON, GA_OFF);
Command dataport = createCommand(DP_ON, DP_OFF);
Command zapper = createCommand(ZP_ON, ZP_OFF);
Command lightSaber = createCommand(LS_ON, LS_OFF);
Command periscope = createCommand(PS_ON, PS_OFF);
Command motivator = createCommand(BM_ON, BM_OFF);
Command lifeForm = createCommand(LF_ON, LF_OFF);
Command scream = createCommand(SF_ON, SF_OFF);
Command leia = createCommand(LW_ON, LW_OFF);
Command music = createCommand(MU_ON, MU_OFF);
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
  byte length = strlen(commandString);
  for (int x = 0; x < length; x++) Serial.write(commandString[x]);
  Serial.write(13);
  command->isOn = !command->isOn;
  return;
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
          digitalWrite(IN1_DOME_MOTOR, HIGH);
          digitalWrite(IN2_DOME_MOTOR, LOW);
        
      }
      if (Xbox.getButtonPress(R2) > TRIGGER_DEAD_ZONE) {
          analogWrite(DOME_ENABLE, Xbox.getButtonPress(R2));
          digitalWrite(IN1_DOME_MOTOR, LOW);
          digitalWrite(IN2_DOME_MOTOR, HIGH);
        
      }



      if (Xbox.getButtonClick(UP)) {
        runCommand(&scream);
      }
      if (Xbox.getButtonClick(DOWN)) {
        runCommand(&leia);
      }
      if (Xbox.getButtonClick(LEFT)) {
        runCommand(&music);
      }
      if (Xbox.getButtonClick(RIGHT)) {
        runCommand(&zapper);
      }


      if (Xbox.getButtonClick(START)) {
        //Xbox.setLedMode(ALTERNATING);
        runCommand(&interface);
      }
      if (Xbox.getButtonClick(BACK)) {
        //Xbox.setLedBlink(ALL);
        runCommand(&gripper);
      }
      if (Xbox.getButtonClick(L1))
      //runCommand(&zapper);
      //if (Xbox.getButtonClick(R1)) runCommand(&utility);

      if (Xbox.getButtonClick(L3))
        runCommand(&dataport);
      if (Xbox.getButtonClick(R3))
        runCommand(&utility);
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
          Xbox.setLedOn(1);
          LFoot.detach();
          RFoot.detach();
        }
        //Serial.print(F("Xbox (Battery: "));
        //Serial.print(Xbox.getBatteryLevel(CONTROLLER));  // The battery level in the range 0-3
        //Serial.println(F(")"));
      }
      if (Xbox.getButtonClick(SYNC)) {
        //Serial.println(F("Sync"));
        Xbox.disconnect(CONTROLLER);
      }


      if (Xbox.getButtonClick(A))
        runCommand(&lightSaber);
      if (Xbox.getButtonClick(B))
        runCommand(&periscope);
      if (Xbox.getButtonClick(X))
        runCommand(&motivator);
      if (Xbox.getButtonClick(Y))
        runCommand(&lifeForm);
    }
  }
}



