#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 6


const byte kids_Can[][16] PROGMEM = {
{"G0T100#          "},  //Turns All logic displays to text mode
{"G1MKIDS#         "},  //Add 12 (max) characters to Top FLD (enter between the 'M' and '#')
{"G2M  CAN DO#     "},  //Add 12 (max) characters to Bottom FLD (enter between the 'M' and '#')
{"G3M  ANYTHING#   "},  //Add 12 (max) characters to Rear FLD (enter between the 'M' and '#')
{"D40S1#           "},  //Set Color of Holoprojectors to Red
{"D45S1#           "},  //Set Color of Magic Panel to Red
{"B21S1#           "},  //Change Coin Slots to Red
{"B22S1#           "},  //Change LDPL to Red
{"A10T1#           "},  //Open Body Implement
{"E54T1#           "},  //Open Dome Implement
{"D45T1#           "},  //Turn on Magic Panel
{"D40T1#           "},  //Turn on Holoprojectors
{"J025#             "},  //Play Sound at Bank 0, track 25 
{"W15#             "},  //Wait 15 secs
{"E54T2#           "},  //Lower Motivator
{"G0T1#            "},  //Return FLD to Normal
{"B21T16#          "},  //Change coin slots to Blue Single
{"B22T16#          "},  //Change LDPL to Blue Two Across
{"B24T2#           "},  //Close the Dataport Door
{"D40T0#           "},  //Shut off Holoprojectors
{"D45T0#           "},  //Shut off Magic Panel
{"D45S5#           "},  //Reset Magic Panel back to Blue
{"D40S5#           "},  //Reset Holoprojector back to Blue
{"W03#             "},  //Wait 3 Seconds
{"@@@@@@#          "}   //End program
    
};



MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

int block = 1;

byte data[16] = { "@@@@@@#            " };
byte read_data[18];

void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("Scanning...");
}

void loop() {
  for (byte i = 0; i < 6; i++) {
    key.keyByte[i] = 0xFF;
  }
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return;
  }


  if (!mfrc522.PICC_ReadCardSerial()) {
    return;
  }
  Serial.println("Card is selected successfully!");
  for (int x = 0; x < 47; x++) {
    byte read_data[18];
    for (int y = 0; y < 16; y++) {
      data[y] = pgm_read_byte(&(kids_Can[x][y]));
    }
    writeBlock(block, data);
    readBlock(block, read_data);

    Serial.print("Reading block ");
    for (int j = 0; j < 16; j++) {
      Serial.write(read_data[j]);
    }
    Serial.println("");
    block++;
    if (block % 4 == 3) block++;
  }
}

int writeBlock(int blockNumber, byte arrayAddress[]) {
  int largestModulo4Number = blockNumber / 4 * 4;
  int trailerBlock = largestModulo4Number + 3;
  if (blockNumber > 2 && (blockNumber + 1) % 4 == 0) {
    Serial.print(blockNumber);
    Serial.println(" is a trailer block");
    return 2;
  }
  Serial.print(blockNumber);
  Serial.println(" is a data block");


  byte status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
  if (status != MFRC522::STATUS_OK) {
    Serial.print("PCD_Authenticate() failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return 3;
  }

  status = mfrc522.MIFARE_Write(blockNumber, arrayAddress, 16);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("MIFARE_Write() failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return 4;
  }
  Serial.println("Block has been written!");
}



int readBlock(int blockNumber, byte arrayAddress[]) {
  int largestModulo4Number = blockNumber / 4 * 4;
  int trailerBlock = largestModulo4Number + 3;

  byte status = mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));

  if (status != MFRC522::STATUS_OK) {
    Serial.print("PCD_Authenticate() failed (read): ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return 3;
  }

  byte buffersize = 18;
  status = mfrc522.MIFARE_Read(blockNumber, arrayAddress, &buffersize);
  if (status != MFRC522::STATUS_OK) {
    Serial.print("MIFARE_read() failed: ");
    Serial.println(mfrc522.GetStatusCodeName(status));
    return 4;
  }
  Serial.println("Block read successfully!");
}