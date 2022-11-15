/*
We will be using a Jawa-Lite inspired technique to control all the MPU's in 
R2-Blue2.

The command structure is as follows:

    Element 0 - MPU Code
    Element 1, 2 - two digit Integer representing the device
    Element 3 - one char command code
    Element 4 - ?  option

    MPU Codes:
      @ - Teeces Micro
      A - Body Master Mega
      B - Lights Mega
      C - Drive Mega
      D - Dome Master Mega
      E - Lifts Mega
      F - Periscope Nano

    Device Codes:
      0-9 - Teeces
      10-19 - Body Master
      20-29 - Lights Mega
      30-39 - Drive Mega
      40-49 - Dome Master Mega
      50-59 - Lift Mega
      60-69 - Periscope

  A valid command would look like this:
    E51T102
*/

#define CMD_MAX_LENGHT 64
#define MPU E

int dev_option, dev_address;
char dev_MPU, dev_command;
char cmdStr[CMD_MAX_LENGHT];

void checkSerial() {
  if (Serial.available()) {
    char ch;  //create a character to hold the current byte from Serial stream
    byte command_complete;
    ch = Serial.read();
    Serial.print(ch);
    command_complete = buildCommand(ch, cmdStr);
    if (command_complete) {
      processCMD(cmdStr);
      Serial.println();
    }
  }
  return;
}

byte processCMD(char* input_str) {
  byte hasArgument = false;
  int argument;
  int address;
  byte pos = 0;
  byte length = strlen(input_str);
  if (length < 2) goto deadCmd;  //not enough characters
  int mpu = input_str[pos];
  pos++;
  if ((mpu > 64 && mpu < 71) || mpu == '@') dev_MPU = mpu;
  else goto deadCmd;  //Not a valid MPU - end command
  char addrStr[3];
  //next we need to get the device address which could be 1 or two characters
  if (!isdigit(input_str[pos])) goto deadCmd;  //Invalid as first char not a digit
  addrStr[pos - 1] = input_str[pos];
  pos++;
  if (isdigit(input_str[pos])) {
    addrStr[pos - 1] = input_str[pos];
    pos++;
  }
  addrStr[pos - 1] = '\0';
  dev_address = atoi(addrStr);
  if(!length>pos) goto deadCmd;  //invalid, no command after address
//check for the special case message command 'M'
  if(inputStr[pos]=='M'){
    pos++;
    if(!length>pos)goto deadCmd;  //invalid, no message argument
    doMcommand(dev_)    
  }


deadCmd:

  return;
}

byte buildCommand(char ch, char* output_str) {
  static int pos = 0;
  switch (ch) {
    case '\r':  //end character reached
      output_str[pos] = '\0';
      pos = 0;
      return true;
      break;
    default:
      output_str[pos] = ch;
      if (pos <= CMD_MAX_LENGHT - 1) pos++;
      break;
  }
  return false;
}


void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
