#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 6
const byte demo_card[][16] PROGMEM = {
  { "D90S1#           " },
  { "D95S1#           " },
  { "D90T1#           " },
  { "D95T3#           " },
  { "G3T100#          " },
  { "G3MZAPPER#       " },
  { "E51T8#           " },
  { "W13#             " },
  { "D90S2#           " },
  { "D95S2#           " },
  { "G3MLIGHT SABER#  " },
  { "E52T1#           " },
  { "W10#             " },
  { "E52T2#           " },
  { "W02#             " },
  { "D90S3#           " },
  { "D95S3#           " },
  { "G3MPERISCOPE#    " },
  { "E53T1#           " },
  { "F58T005#         " },
  { "F57T002#         " },
  { "W06#             " },
  { "E53T2#           " },
  { "W03#             " },
  { "D90S4#           " },
  { "D95S4#           " },
  { "G3MMOTIVATOR#    " },
  { "E54T1#           " },
  { "W05#             " },
  { "E54T2#           " },
  { "W02#             " },
  { "D90S5#           " },
  { "D95S5#           " },
  { "G3MLIFE FORM#    " },
  { "E55T1#           " },
  { "W05#             " },
  { "E55T2#           " },
  { "G0T1#            " },
  { "D90T0#           " },
  { "D95T0#           " },
  { "@@@@@@#          " }
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
  for (int x = 0; x < 41; x++) {
    byte read_data[18];
    for(int y=0; y<16; y++){
    data[y] = pgm_read_byte(&(demo_card[x][y]));
    }
    writeBlock(block, data);
    readBlock(block, read_data);

    Serial.print("Reading block ");
    for (int j = 0; j < 16; j++) {
      Serial.write(read_data[j]);
    }
    Serial.println("");
    block++;
    if(block%4==3)block++;
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
