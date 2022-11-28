/*  This program controls the periscope lights for the R2-Blue2 Dome Lift System.  It is designed to run
on an Arduino Nano.

The Periscope lights consist of 7 White LED's and eight neo-pixels.  

The 7 small white LEDs will be referred to as "Display" Lights 

Neo-pixels 0 and 2 are the right and left "Eyes"

Neo-pixel 1 is the "Status" Light

Neo-pixels 3 thru 6 are the "Main" Display 

The program will have the following threads:
    checkSerial()  -  Checks Serial0 for commands and executes the commands
    runDisplay()  - Runs any commands for the Display Lights
    runEyes()  -  Runs any commands for the Eyes
    runStatus() - Runs any commands for the Status Light
    runMain() - Runs any command for the Main Display

*/

