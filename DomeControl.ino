/***********************************************************
Dome Control Program which uses a nRF24L01 Wifi module to receive commands from our custom
transmitter and forwards the commands by serial interface to the Dome Lift Mega and the Body
Master Mega.
*/
#include <SPI.h>  //the nRF24 use the SPI interface to communicate
#include <nRF24L01.h> //Specific Library for the Wifi module
#include <RF24.h>  //Standard Radio Frequency Library

struct radioData{
  char MPU;
  byte device;
  char command;
  uint16_t option;
  char endData;
};
/**************************************************
*             Global Variables                    *
**************************************************/
const byte address[6] = "WR2B2";  //Establish an address for communication
radioData radioMess;  //Radio message buffer
RF24 radio(7, 8); // Set up the radio using the CE and CSN pins

void setup() {
  Serial.begin(9600);  //Open serial0 at 9600 baud
  radio.begin();  //Initialize the radio
  radio.openReadingPipe(0, address);  //Open a reading Pipe - This is the receiver
  radio.setPALevel(RF24_PA_HIGH);  //Set the Power Amplifier mode to HIGH
  radio.startListening();  //Stop all transmissions and start listening for traffic
}

void loop() {
  if (radio.available()) {
    radio.read(&radioMess, sizeof(radioMess));
    process_Radio_Mess(radioMess);
  }
}

void process_Radio_Mess(radioData rMess){
  if(rMess.MPU=='E'||rMess.MPU=='F'||rMess.MPU=='@'){
    Serial.write(rMess.MPU);
    Serial.write(rMess.device);
    Serial.write(rMess.command);
    Serial.write(rMess.option);
    Serial.write('\n');
    return;
  }
  if(rMess.MPU=='D'||rMess.MPU=='C'||rMess.MPU=='D'){
    Serial1.write(rMess.MPU);
    Serial1.write(rMess.device);
    Serial1.write(rMess.command);
    Serial1.write(rMess.option);
    Serial1.write('\n');
    return;
  }
  
  
}
