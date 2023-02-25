#include <XBOXRECV.h>
#include <Servo.h>
#include <SPI.h>


#define CONTROLLER 0
#define DEAD_ZONE 8000
#define TRIGGER_DEAD_ZONE 50


#define LEFT_PWM 3
#define RIGHT_PWM 9
#define CMD_MAX_LENGTH 16

#define DOME_ENABLE 2
#define IN1_DOME_MOTOR 4
#define IN2_DOME_MOTOR 6


/*
3, 5 - left/right pwm
2, 4, 6 - enable for dome, input 1 for dome motor, input 2 for dome motor
*/


bool domeEnabled = false;


USB Usb;
XBOXRECV Xbox(&Usb);
Servo LFoot;
Servo RFoot;


typedef struct {
  /// The command to run if it needs to turn on.
  char onCommand[CMD_MAX_LENGTH];
  /// The command to run if it needs to turn off.
  char offCommand[CMD_MAX_LENGTH];
  /// Whether or not this command is on or off.
  bool isOn;
} Command;


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
Command utility = createCommand("A10T1", "A10T2");
Command interface = createCommand("A11T1", "A11T2");
Command gripper = createCommand("A12T1", "A12T2");
Command dataport = createCommand("B24T1", "B24T2");
Command coinslot = createCommand("B21T16", "B21T0");
Command ldpl = createCommand("B22T16", "B22T0");
Command zapper = createCommand("E51T8", "E51T2");
Command lightSaber = createCommand("E52T1", "E52T2");
Command periscope = createCommand("E53T1", "E53T2");
Command motivator = createCommand("E54T1", "E54T2");
Command lifeForm = createCommand("E55T1", "E55T2");
Command holos = createCommand("D90T1", "D90T2");
Command magic = createCommand("D95T1", "D95T2");
Command scream = createCommand("$S   ", "$F   ");
Command leia = createCommand("$L   ", "$W   ");
Command music = createCommand("$C   ", "$D   ") bool mtrsEnable = 0;

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
        if (Xbox.getButtonPress(R2) > TRIGGER_DEAD_ZONE) {
          digitalWrite(IN1_DOME_MOTOR, LOW);
          digitalWrite(IN2_DOME_MOTOR, LOW);
        } else {
          analogWrite(DOME_ENABLE, Xbox.getButtonPress(L2));
          digitalWrite(IN1_DOME_MOTOR, HIGH);
          digitalWrite(IN2_DOME_MOTOR, LOW);
        }
      }
      if (Xbox.getButtonPress(R2) > TRIGGER_DEAD_ZONE) {
        if (Xbox.getButtonPress(L2) > TRIGGER_DEAD_ZONE) {
          digitalWrite(IN1_DOME_MOTOR, LOW);
          digitalWrite(IN2_DOME_MOTOR, LOW);
        } else {
          analogWrite(DOME_ENABLE, Xbox.getButtonPress(R2));
          digitalWrite(IN1_DOME_MOTOR, HIGH);
          digitalWrite(IN2_DOME_MOTOR, LOW);
        }
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
        runCommand(&holos);
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
        runCommand(&zapper);
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
