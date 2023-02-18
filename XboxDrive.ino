#include <XBOXRECV.h>
#include <Servo.h>
#include <SPI.h>


#define CONTROLLER 0
#define DEAD_ZONE 8000
#define TRIGGER_DEAD_ZONE 50


#define LEFT_PWM 3
#define RIGHT_PWM 9


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
  char onCommand[64];
  /// The command to run if it needs to turn off.
  char offCommand[64];
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
  for(int x=0; x<length; x++) Serial.write(commandString[x]);
  Serial.write(13);
  command->isOn = !command->isOn;
  return;
}
Command utility = createCommand("A10T001", "A10T002");
Command interface = createCommand("A11T001", "A11T002");
Command gripper = createCommand("A12T001", "A12T002");
Command dataport = createCommand("A13T001", "A13T002");
Command coinslot = createCommand("B21T016", "B21T000");
Command ldpl = createCommand("B22T016", "B22T000");
Command zapper = createCommand("E51T008", "E51T008");
Command lightSaber = createCommand("E52T001", "E52T002");
Command periscope = createCommand("E53T001", "E53T002");
Command motivator = createCommand("E54T001", "E54T002");
Command lifeForm = createCommand("E55T001", "E55T002");
Command holos = createCommand("D90T001", "D90T002");
Command magic = createCommand("D95T001", "D95T002");

void setup() {
  Serial.begin(9600);


  if (Usb.Init() == -1) {
    //Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  //halt
  }
  LFoot.attach(LEFT_PWM);
  LFoot.writeMicroseconds(1500);
  RFoot.attach(RIGHT_PWM);
  RFoot.writeMicroseconds(1500);
  pinMode(DOME_ENABLE, OUTPUT);
  pinMode(IN1_DOME_MOTOR, OUTPUT);
  pinMode(IN2_DOME_MOTOR, OUTPUT);
}

void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    if (Xbox.Xbox360Connected[CONTROLLER]) {
      //Check the Left Joystick "LeftHat" Y axis for input
      if (Xbox.getAnalogHat(LeftHatY, CONTROLLER) > DEAD_ZONE || Xbox.getAnalogHat(LeftHatY, CONTROLLER) < -DEAD_ZONE) {
        int lMtrSpeed;  //tracks left motor  speed
        //Check for positive stick
        if (Xbox.getAnalogHat(LeftHatY, CONTROLLER) > DEAD_ZONE) {
          lMtrSpeed = map(Xbox.getAnalogHat(LeftHatY, CONTROLLER), DEAD_ZONE, 32767, 1500, 1750);
          LFoot.writeMicroseconds(lMtrSpeed);
        }
        //Check for negative stick
        if (Xbox.getAnalogHat(LeftHatY, CONTROLLER) < -DEAD_ZONE) {
          lMtrSpeed = map(Xbox.getAnalogHat(LeftHatY, CONTROLLER), -DEAD_ZONE, -32768, 1500, 1250);
          LFoot.writeMicroseconds(lMtrSpeed);
        }
      }
      //if the stick is in the DEAD_ZONE - shut down the motor
      if (Xbox.getAnalogHat(LeftHatY, CONTROLLER) < DEAD_ZONE && Xbox.getAnalogHat(LeftHatY, CONTROLLER) > -DEAD_ZONE) {
        LFoot.writeMicroseconds(1500);
      }
      //Check the Right Joystick "RightHat" Y axis for input
      if (Xbox.getAnalogHat(RightHatY, CONTROLLER) > DEAD_ZONE || Xbox.getAnalogHat(RightHatY, CONTROLLER) < -DEAD_ZONE) {
        int rMtrSpeed;
        //Check for positive stick
        if (Xbox.getAnalogHat(RightHatY, CONTROLLER) > DEAD_ZONE) {
          rMtrSpeed = map(Xbox.getAnalogHat(RightHatY, CONTROLLER), DEAD_ZONE, 32767, 1500, 1250);
          RFoot.writeMicroseconds(rMtrSpeed);
        }
        //Check for negative stick
        if (Xbox.getAnalogHat(RightHatY, CONTROLLER) < -DEAD_ZONE) {
          rMtrSpeed = map(Xbox.getAnalogHat(RightHatY, CONTROLLER), -DEAD_ZONE, -32768, 1500, 1750);
          RFoot.writeMicroseconds(rMtrSpeed);
        }
      }
      //if the stick is in the DEAD_ZONE - shut down the motor
      if (Xbox.getAnalogHat(RightHatY, CONTROLLER) < DEAD_ZONE && Xbox.getAnalogHat(RightHatY, CONTROLLER) > -DEAD_ZONE) {
        RFoot.writeMicroseconds(1500);
      }
    }


    if (Xbox.getButtonPress(L2, CONTROLLER) || Xbox.getButtonPress(R2, CONTROLLER)) {
      auto direction = 0;


      if (Xbox.getButtonPress(L2, CONTROLLER) > TRIGGER_DEAD_ZONE)
        direction -= 1;
      if (Xbox.getButtonPress(R2, CONTROLLER) > TRIGGER_DEAD_ZONE)
        direction += 1;


      digitalWrite(IN1_DOME_MOTOR, LOW);
      digitalWrite(IN2_DOME_MOTOR, LOW);

      if (direction > 0) {
        digitalWrite(IN1_DOME_MOTOR, HIGH);
        digitalWrite(IN2_DOME_MOTOR, LOW);
      } else if (direction < 0) {
        digitalWrite(IN1_DOME_MOTOR, LOW);
        digitalWrite(IN2_DOME_MOTOR, HIGH);
      }


      //Serial.println(direction);
    }





    if (Xbox.getButtonClick(UP, CONTROLLER)) {
      runCommand(&coinslot);
    }
    if (Xbox.getButtonClick(DOWN, CONTROLLER)) {
      runCommand(&ldpl);
    }
    if (Xbox.getButtonClick(LEFT, CONTROLLER)) {
      runCommand(&magic);
    }
    if (Xbox.getButtonClick(RIGHT, CONTROLLER)) {
      runCommand(&holos);
    }


    if (Xbox.getButtonClick(START, CONTROLLER)) {
      //Xbox.setLedMode(ALTERNATING, CONTROLLER);
      runCommand(&zapper);
    }
    if (Xbox.getButtonClick(BACK, CONTROLLER)) {
      //Xbox.setLedBlink(ALL, CONTROLLER);
      runCommand(&utility);
    }


    if (Xbox.getButtonClick(L3, CONTROLLER))
      runCommand(&dataport);
    if (Xbox.getButtonClick(R3, CONTROLLER))
      runCommand(&gripper);
    if (Xbox.getButtonClick(XBOX, CONTROLLER)) {
      Xbox.setLedMode(ROTATING, CONTROLLER);
      //Serial.print(F("Xbox (Battery: "));
      //Serial.print(Xbox.getBatteryLevel(CONTROLLER));  // The battery level in the range 0-3
      //Serial.println(F(")"));
    }
    if (Xbox.getButtonClick(SYNC, CONTROLLER)) {
      //Serial.println(F("Sync"));
      Xbox.disconnect(CONTROLLER);
    }


    if (Xbox.getButtonClick(A, CONTROLLER))
      runCommand(&lightSaber);
    if (Xbox.getButtonClick(B, CONTROLLER))
      runCommand(&periscope);
    if (Xbox.getButtonClick(X, CONTROLLER))
      runCommand(&motivator);
    if (Xbox.getButtonClick(Y, CONTROLLER))
      runCommand(&lifeForm);
  }
}
