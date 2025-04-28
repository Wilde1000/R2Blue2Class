/*****
 * 
 * Astropixels example sketch. 
 * 
 * This sketch will get the basic Astropixel setup by Darren Poulson working. Pinouts are designed
 * to be used with the Astropixel ESP32 breakout board.
 * 
 * 
 */
#include "ReelTwo.h"
#include "dome/Logics.h"
#include "dome/LogicEngineController.h"
#include "dome/HoloLights.h"
#include "dome/NeoPSI.h"
#include "i2c/I2CReceiver.h"
#include "ServoDispatchDirect.h"
#include "ServoSequencer.h"
#include "core/Marcduino.h"
#define COMMAND_SERIAL Serial1
I2CReceiver i2cReceiver(0x0a);
#define CMD_MAX_LENGTH 64
#define MPU 'G'
char cmdStr0[CMD_MAX_LENGTH];
char dev_MPU;                 //Contains the MPU code from incoming serial message
char dev_cmd;                 //Contains the Command code from incoming serial message
int dev_addr;              //Device address received from Serial interface
int dev_opt;               //Device option received from the Serial interface
char mess[CMD_MAX_LENGTH-4];
AstroPixelRLD<> RLD(LogicEngineRLDDefault, 3);
AstroPixelFLD<> FLD(LogicEngineFLDDefault, 1);
AstroPixelFrontPSI<> frontPSI(LogicEngineFrontPSIDefault, 4);
AstroPixelRearPSI<> rearPSI(LogicEngineRearPSIDefault, 5);
HoloLights frontHolo(25, HoloLights::kRGB);
HoloLights rearHolo(26, HoloLights::kRGB);
HoloLights topHolo(27, HoloLights::kRGB);  

//CommandEventSerial<> commandSerial(COMMAND_SERIAL);
/*#define PIE_PANEL          0x0008
#define TOP_PIE_PANEL      0x0010

const ServoSettings servoSettings[] PROGMEM = {
    { 2,  1250, 1900, PIE_PANEL },      //0: pie panel 1 
    { 3,  1075, 1700, PIE_PANEL },      //1: pie panel 2 
    { 4,  1200, 2000, PIE_PANEL },      //2: pie panel 3 
    { 5,   750, 1450, PIE_PANEL },      //3: pie panel 4 
    { 6,  1250, 1850, TOP_PIE_PANEL },  //4: dome top panel 
};

ServoDispatchDirect<SizeOfArray(servoSettings)> servoDispatch(servoSettings);
ServoSequencer servoSequencer(servoDispatch);
AnimationPlayer player(servoSequencer);
MarcduinoSerial<> marcduinoSerial(COMMAND_SERIAL, player);

// Marcduino command starting with '*RT' followed by Reeltwo command
MARCDUINO_ACTION(DirectCommand, *RT, ({
    // Direct ReelTwo command
    CommandEvent::process(Marcduino::getCommand());
}))
*/








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
//The parseCommand takes the command from the buildCommand function and parses into its component parts - MPU, Address, Command and Option
int parseCommand(char* input_str) {
  byte length = strlen(input_str);
  if (length < 2) return 1;  //not enough characters
  int mpu = input_str[0];    //MPU is the first character
  if (MPU != mpu) {          //if command is not for this MPU - send it on its way
    Serial.flush();
    for (int x = 0; x < length; x++) Serial.write(input_str[x]);
    Serial.write(13);
    return 1;
  }
  dev_MPU = mpu;
  // Now the address which should be the next two characters
  char addrStr[3];  //set up a char array to hold them (plus the EOS (end of String) character)
  addrStr[0] = input_str[1];
  addrStr[1] = input_str[2];
  addrStr[2] = '\0';
  dev_addr = atoi(addrStr);
  if (!length > 4) return 0;  //invalid, no command after address
  dev_cmd = input_str[3];
  if(dev_cmd=='M'){
    for(int x=0; x<length-4; x++)mess[x]=input_str[4+x];
    switch (dev_addr){
        case 0:
           FLD.selectScrollTextLeft(mess, LogicEngineRenderer::kPurple, 2, 25);
           RLD.selectScrollTextLeft(mess, LogicEngineRenderer::kPink, 2, 25);
           break;
        case 1:
           FLD.selectScrollTextLeft(mess, LogicEngineRenderer::kYellow, 2, 25); 
           break;
        case 2:
           RLD.selectScrollTextLeft(mess, LogicEngineRenderer::kPink, 2, 25);
           break;
        case 3:
          resetSequence();
          break;
        case 4:
          redRW();
          break;
        case 5:
          orangeRW();
          break;
        case 6:
          yellowRW();
          break;
        case 7:
          greenRW();
          break;
        case 8:
          cyanRW();
          break;
        case 9:
          blueRW();
          break;
        case 10:
          purpleRW();
          break;
        case 11:
          magentaRW();
          break;
        case 12:
          pinkRW();
    }
    return 1;
  }
  
  return 0;
}

void redRW(){
  
  CommandEvent::process(F(
    
    "LE1051000|15\n"
    "LE3051000|15\n"
    "LE4051000|15\n"
    "LE5051000|15\n"
    "HPA0021|30\n"
  ));
  
}

void orangeRW(){
  CommandEvent::process(F(
    "LE1052000|0\n"
    "LE3052000|0\n"
    "LE4052000|0\n"
    "LE5052000|0\n"
    "HPA0027|0\n"
  ));
}

void cyanRW(){
  CommandEvent::process(F(
    "LE1055000|0\n"
    "LE3055000|0\n"
    "LE4055000|0\n"
    "LE5055000|0\n"
    "HPA0024|0\n"
  ));
}

void blueRW(){
  CommandEvent::process(F(
    "LE1056000|0\n"
    "LE3056000|0\n"
    "LE4056000|0\n"
    "LE5056000|0\n"
    "HPA0025|0\n"
  ));
}

void purpleRW(){
  CommandEvent::process(F(
    "LE1057000|0\n"
    "LE3057000|0\n"
    "LE4057000|0\n"
    "LE5057000|0\n"
    "HPA0028|0\n"
  ));
}

void magentaRW(){
  CommandEvent::process(F(
    "LE1058000|0\n"
    "LE3058000|0\n"
    "LE4058000|0\n"
    "LE5058000|0\n"
    "HPA0026|0\n"
  ));
}

void pinkRW(){
  CommandEvent::process(F(
    "LE1059000|0\n"
    "LE3059000|0\n"
    "LE4059000|0\n"
    "LE5059000|0\n"
    "HPA0021|0\n"
  ));
}

void yellowRW(){
  CommandEvent::process(F(
    "LE1053000|0\n"
    "LE3053000|0\n"
    "LE4053000|0\n"
    "LE5053000|0\n"
    "HPA0022|0\n"
  ));
}

void greenRW(){
  CommandEvent::process(F(
    "LE1054000|0\n"
    "LE3054000|0\n"
    "LE4054000|0\n"
    "LE5054000|0\n"
    "HPA0023|0\n"
  ));
}

void shortCircuitRW(){
  CommandEvent::process(F(
    "LE1020000|0\n"
    "LE3020000|0\n"
    "LE4020000|0\n"
    "LE5020000|0\n"
    "HPA007|0\n"
  ));
}
void marchRW(){
  CommandEvent::process(F(
    "LE1040000|0\n"
    "LE3040000|0\n"
    "LE4040000|0\n"
    "LE5040000|0\n"
    "HPA0021|0\n"
  ));
}
    
  void resetSequence()
{
    CommandEvent::process(F(
        "LE1000000|0\n" // LogicEngine devices to normal
        "LE3000000|0\n" // LogicEngine devices to normal
        "LE4231000|0\n" // LogicEngine devices to normal
        "LE5233000|0\n" // LogicEngine devices to normal
        "HPA000|0\n"   // Holo Projectors to Normal
        )); 
}





    


void setup()
{
    Serial.begin(9600);
    //Serial1.begin(9600);
    Wire.begin();
    REELTWO_READY();
    SetupEvent::ready();

    RLD.selectScrollTextLeft("... Blue Crew Robotics #6153  ....", LogicEngineRenderer::kBlue, 2, 25);
    FLD.selectScrollTextLeft("... R2-Blue2 at your service...", LogicEngineRenderer::kGreen, 2, 25);
    //FLD.selectScrollTextBottom("... Bottom Text  ", LogicEngineRenderer::kGreen, 2, 15); // Sets text and color for the bottom FLD
    
}



void loop()
{

    if (checkSerial()) parseCommand(cmdStr0);
    AnimatedEvent::process();
}