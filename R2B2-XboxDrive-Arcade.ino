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
#define CMD_MAX_LENGTH 10     //Set the max command length for this program
#define DOME_ENABLE 4         //Set the Enable pin for the Dome motor
#define IN1_DOME_MOTOR 2      //Set the INPUT1 pin for the Dome motor
#define IN2_DOME_MOTOR 5      //Set the INPUT2 pin for the Dome motor

#define UA_ON "A10T1\r"         //Utility Arm on command
#define UA_OFF "A10T2\r"        //Utility arm off command
#define IA_ON "A11T1\r"         //Interface Arm on command
#define IA_OFF "A11T2\r"        //Interface arm off command
#define GA_ON "A12T1\r"         //Gripper Arm on command
#define GA_OFF "A12T2\r"        //Gripper arm off command
#define DP_ON "B24T1\r"         //Dataport on command
#define DP_OFF "B24T2\r"        //Dataport off command
#define ZP_ON "E51T8\r"         //Zapper on command
#define ZP_OFF "E51T8\r"        //Zapper off command
#define LS_ON "E52T1\r"         //Light Saber on command
#define LS_OFF "E52T2\r"        //Light Saber off command
#define PS_ON "E53T1\r"         //Periscope on command
#define PS_OFF "E53T2\r"        //Periscope off command
#define BM_ON "E54T1\r"         //Bad Motivator on command
#define BM_OFF "E54T2\r"        //Bad Motivator off command
#define LF_ON "E55T1\r"         //Life Form on command
#define LF_OFF "E55T2\r"        //Life Form off command
#define BEEP01 "J01\r"          //General Beep 1
#define BEEP02 "J02\r"          //General Beep 2
#define BEEP03 "J03\r"          //General Beep 3
#define BEEP04 "J04\r"          //General Beep 4
#define BEEP05 "J05\r"          //Happy Beep 1
#define BEEP06 "J06\r"          //Happy Beep 2
#define BEEP07 "J07\r"          //Happy Beep 3
#define BEEP08 "J08\r"          //Happy Beep 4
#define BEEP09 "J09\r"          //Sad Beep 1
#define BEEP10 "J010\r"          //Sad Beep 2
#define BEEP11 "J011\r"          //Sad Beep 3
#define BEEP12 "J012\r"          //Sad Beep 4
#define BEEP13 "J013\r"          //Chatty Beep 1
#define BEEP14 "J014\r"          //Chatty Beep 2
#define BEEP15 "J015\r"          //Chatty Beep 1
#define BEEP16 "J016\r"          //Chatty Beep 2
#define ROUTSCREAM "E50T31\r"   //Scream Routine
#define ROUTWAVE "E50T32\r"     //Wave Routine
#define ROUTWAVE1 "E50T33\r"    //Moody Wave Routine
#define ROUTWAVE2 "E50T34\r"    //Open Wave Routine
#define ROUTFAINT "E50T35\r"    //Faint/Short Circuit Routine
#define ROUTCANTINA "E50T36\r"  //Cantina Routine
#define ROUTLEIA "E50T37\r"     //Leia Routine
#define ROUTDISCO "E50T38\r"    //Disco Routine
#define PIE1_O "E50T11\r"         //Zapper Pie Open
#define PIE1_C "E50T21\r"         //Zapper Pie Close
#define PIE2_O "E50T12\r"         //Light Saber Pie Open
#define PIE2_C "E50T22\r"         //Light Saber Pie Close
#define PIE3_O "E50T13\r"         //Bad Motivator Pie Open
#define PIE3_C "E50T23\r"         //Bad Motivator Pie Close
#define PIE4_O "E50T14\r"         //Life Form Pie Open
#define PIE4_C "E50T24\r"         //Life Form Pie Close
#define DP1_O  "E50T15\r"         //Dome Panel 1 Open
#define DP1_C  "E50T25\r"         //Dome Panel 1 Close
#define DP2_O  "E50T16\r"         //Dome Panel 2 Open
#define DP2_C  "E50T26\r"         //Dome Panel 2 Close
#define DP3_O  "E50T17\r"         //Dome Panel 3 Open
#define DP3_C  "E50T27\r"         //Dome Panel 3 Close
#define DP4_O  "E50T18\r"         //Dome Panel 4 Open
#define DP4_C  "E50T28\r"         //Dome Panel 4 Close
#define DP5_O  "E50T19\r"         //Dome Panel 5 Open
#define DP5_C  "E50T29\r"         //Dome Panel 5 Close
#define DP6_O  "E50T20\r"         //Dome Panel 6 Open
#define DP6_C  "E50T30\r"         //Dome Panel 6 Close



#define HP_COLOR "D40S"

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

int HP_current_color = 13;
int mp_current_color = 6;
int LDPL_current_color = 6;
int Coin_current_color = 6;

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
Command coinslots = createCommand("B21T6\r", "B21T0\r");
Command domePie1 = createCommand(PIE1_O, PIE1_C);
Command domePie2 = createCommand(PIE2_O, PIE2_C);
Command domePie3 = createCommand(PIE3_O, PIE3_C);
Command domePie4 = createCommand(PIE4_O, PIE4_C);
Command panel1 = createCommand(DP1_O, DP1_C);
Command panel2 = createCommand(DP2_O, DP2_C);
Command panel3 = createCommand(DP3_O, DP3_C);
Command panel4 = createCommand(DP4_O, DP4_C);
Command panel5 = createCommand(DP5_O, DP5_C);
Command panel6 = createCommand(DP6_O, DP6_C);
bool mtrsEnable = 0;

// Drive behavior tuning
const int NEUTRAL_PULSE   = 1500;
const int PULSE_RANGE     = 250;   // +/- range from neutral (1250–1750 µs)
const int RAMP_STEP       = 5;     // max µs step per loop for software ramp

bool  slowMode    = false;
const float SLOW_SCALE = 0.5f;     // 50%% speed in slow mode

// Drive mode selection
enum DriveMode {
  DRIVE_ARCADE = 0,
  DRIVE_TANK   = 1
};
DriveMode driveMode = DRIVE_ARCADE;

// Current and target servo pulses for ramping
int currentLeftPulse  = NEUTRAL_PULSE;
int currentRightPulse = NEUTRAL_PULSE;
int targetLeftPulse   = NEUTRAL_PULSE;
int targetRightPulse  = NEUTRAL_PULSE;





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

/************************************************
 * *************** DRIVE HELPER FUNCTIONS ********
 * ***********************************************/

float hatToNorm(int value) {
  if (value > DEAD_ZONE) {
    return (float)(value - DEAD_ZONE) / (32767.0f - (float)DEAD_ZONE);
  } else if (value < -DEAD_ZONE) {
    return (float)(value + DEAD_ZONE) / (-32768.0f + (float)DEAD_ZONE);
  } else {
    return 0.0f;
  }
}

float fAbs(float x) {
  return (x < 0.0f) ? -x : x;
}

int normToPulse(float cmd) {
  if (cmd > 1.0f)  cmd = 1.0f;
  if (cmd < -1.0f) cmd = -1.0f;
  float pulse = (float)NEUTRAL_PULSE + cmd * (float)PULSE_RANGE;
  return (int)pulse;
}

int scalePulse(int pulse, bool slow) {
  if (!slow) return pulse;
  float offset = (float)pulse - (float)NEUTRAL_PULSE;
  offset *= SLOW_SCALE;
  return (int)((float)NEUTRAL_PULSE + offset);
}

void stepToward(int &current, int target) {
  if (current < target - RAMP_STEP)      current += RAMP_STEP;
  else if (current > target + RAMP_STEP) current -= RAMP_STEP;
  else                                   current  = target;
}

bool sticksInDeadZone() {
  int lx = Xbox.getAnalogHat(LeftHatX);
  int ly = Xbox.getAnalogHat(LeftHatY);
  int rx = Xbox.getAnalogHat(RightHatX);
  int ry = Xbox.getAnalogHat(RightHatY);

  return (abs(lx) < DEAD_ZONE &&
          abs(ly) < DEAD_ZONE &&
          abs(rx) < DEAD_ZONE &&
          abs(ry) < DEAD_ZONE);
}

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
      // Drive control: arcade or tank mode with software ramp
      if (mtrsEnable) {
        int rawLeftY  = Xbox.getAnalogHat(LeftHatY);
        int rawRightY = Xbox.getAnalogHat(RightHatY);
        int rawRightX = Xbox.getAnalogHat(RightHatX);

        float leftCmd  = 0.0f;
        float rightCmd = 0.0f;

        if (driveMode == DRIVE_ARCADE) {
          // Arcade: Left stick Y = throttle, Right stick X = turn
          float throttle = hatToNorm(rawLeftY);
          float turn     = hatToNorm(rawRightX);

          leftCmd  = throttle + turn;
          rightCmd = throttle - turn;

          float maxMag = fAbs(leftCmd);
          if (fAbs(rightCmd) > maxMag) maxMag = fAbs(rightCmd);
          if (maxMag > 1.0f) {
            leftCmd  /= maxMag;
            rightCmd /= maxMag;
          }
        } else {
          // Tank: Left stick Y = left wheel, Right stick Y = right wheel
          leftCmd  = hatToNorm(rawLeftY);
          rightCmd = hatToNorm(rawRightY);
        }

        int leftPulse  = normToPulse(leftCmd);
        int rightPulse = normToPulse(rightCmd);

        leftPulse  = scalePulse(leftPulse,  slowMode);
        rightPulse = scalePulse(rightPulse, slowMode);

        targetLeftPulse  = leftPulse;
        targetRightPulse = rightPulse;

        stepToward(currentLeftPulse,  targetLeftPulse);
        stepToward(currentRightPulse, targetRightPulse);

        LFoot.writeMicroseconds(currentLeftPulse);
        RFoot.writeMicroseconds(currentRightPulse);
      } else {
        // Motors disabled: keep drive outputs neutral
        currentLeftPulse  = NEUTRAL_PULSE;
        currentRightPulse = NEUTRAL_PULSE;
        targetLeftPulse   = NEUTRAL_PULSE;
        targetRightPulse  = NEUTRAL_PULSE;
      }

      //Check the Left and Right triggers
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
        if (mtrsEnable) {
          // Slow mode toggle only when drive enabled
          slowMode = !slowMode;
        } else {
          // When drive disabled, L3 cycles command sets backward
          if (comSet > 1) comSet--;
          else comSet = 4;
          Xbox.setLedOn((LEDEnum)comSet);
        }
      }
      if (Xbox.getButtonClick(R3)) {
        if (mtrsEnable) {
          // Toggle drive mode (arcade <-> tank) only if sticks are centered
          if (sticksInDeadZone()) {
            if (driveMode == DRIVE_ARCADE) driveMode = DRIVE_TANK;
            else driveMode = DRIVE_ARCADE;

            // Reset ramp targets to neutral when switching modes
            targetLeftPulse  = NEUTRAL_PULSE;
            targetRightPulse = NEUTRAL_PULSE;
          }
        } else {
          // When drive disabled, R3 cycles command sets forward
          if (comSet < 4) comSet++;
          else comSet = 1;
          Xbox.setLedOn((LEDEnum)comSet);
        }
      }

      if (Xbox.getButtonClick(SYNC)) {
        //Serial.println(F("Sync"));
        Xbox.disconnect(CONTROLLER);
      }
      if (Xbox.getButtonClick(L1)) runCommand(&gripper);
      if (Xbox.getButtonClick(R1)) runCommand(&interface);
      switch (comSet) {
        case 1:  //activates servo and lights
          if (Xbox.getButtonClick(UP)) runCommand(&utility);
          if (Xbox.getButtonClick(DOWN)) runCommand(&dataport);
          if (Xbox.getButtonClick(LEFT)) runCommand(&holos);
          if (Xbox.getButtonClick(RIGHT)) runCommand(&magicpanel);
          if (Xbox.getButtonClick(START)) runCommand(&zapper);
          if (Xbox.getButtonClick(BACK)) runCommand(&coinslots);

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
        case 4:
          if (Xbox.getButtonClick(UP)) runCommand(&domePie1);
            
          if (Xbox.getButtonClick(DOWN)) runCommand(&domePie2);
          if (Xbox.getButtonClick(LEFT)) runCommand(&domePie3);
          if (Xbox.getButtonClick(RIGHT)) runCommand(&domePie4);
          if (Xbox.getButtonClick(A)) runCommand(&panel1);
          if (Xbox.getButtonClick(B)) runCommand(&panel2);
          if (Xbox.getButtonClick(X)) runCommand(&panel3);
          if (Xbox.getButtonClick(Y)) runCommand(&panel4);
          if (Xbox.getButtonClick(START)) runCommand(&panel5);
          if (Xbox.getButtonClick(BACK)) runCommand(&panel6);
          
          break;
      }
    }
  }
}
