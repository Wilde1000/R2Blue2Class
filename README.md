# R2Blue2Class  

This repository is for the code for our R2-D2 Droid, R2-Blue2. 

Currently the droid has seven microprocessor units or MPU's.  Each MPU is connected serially to at least one other MPU so that commands originated in one MPU can be distributed to the appropriate MPU for execution.  

R2B2-LiftMega.ino is the program from the Dome Lift system.  It accepts commands from Serial0 from the Body Master MPU, and commands from Serial1 from the Dome Master MPU. All commands will be in the Jawa-Lite sytyle (MPU/Address/Command/Option).  

