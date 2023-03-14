# R2Blue2Class  

This repository is for the code for our R2-D2 Droid, R2-Blue2. 

Currently the droid has seven microprocessor units or MPU's.  Each MPU is connected serially to at least one other MPU so that commands originated in one MPU can be distributed to the appropriate MPU for execution.  

The seven microprocessors are as follows:

A - Body Master Mega - Acts as router for the body and controls the sounds and servos for the body.

B - Body Lights Mega - Controls all lighting systems on the body and is the gateway to the two expansion Nanos.

C - Drive Uno - Controls the drive and dome rotation motors and passes all other commands to the Body Master Mega for execution.

D - RFID Nano - Accepts RFID signals and conntrols the Holoprojectors and Magic Panel. 

E - Dome Lift Mega - Controls the dome lift mechanisms and acts as a router for the dome.

F - Periscope Nano - Controls the lights on the Periscope.

G - Teeces Mini  - Controls the Power State Indicators, the front and rear panel logics.

In addition, two more Nanos (H and I) are available on the electronics board.

R2B2-ComLib.ino is the program for the Body Master.  It connects to the Dome Lift Mega through Serial0, to the MP3 Trigger through Serial1, to the Body Lights Mega through Serial2, and the Drive Uno through Serial3.  In addition, it is connected through I2C to a PCA9685 16-Servo Driver.

BodyLights.ino is the program for the Body Lights Mega.  It connects to the Body Master through Serial 0; it is wired to the dome 25 pin connector on Serial1 (though not used); it is wired to the CBI Expansion board on Serial2; and is connected to the EXP Expansion Board on Serial3.  The program handles all body lighting systems (CBI, DPL, LDPL, Coinslots).

XboxDrive.ino is the program for the Drive Uno.  It is connected to the Body Master Mega through Serial0.

HoloMagic.ino is the program for the RFID Nano. It is connected to Dome Lift Mega through Serial0.

R2B2-LiftMega.ino is the program from the Dome Lift system.  It accepts commands from Serial0 and 
 from the Body Master MPU, and commands from Serial1 from the Dome Master MPU. All commands will be in the Jawa-Lite style (MPU/Address/Command/Option).  

