#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN 10
#define RST_PIN 6

const byte panel_open_close_wave_card[][16] PROGMEM = {
{"G3T100#          "},
{"E80T6#           "},
{"G3MOPEN CLOSE WV#"},
{"W05#             "},
{"G0T1#            "},
{"@@@@@@#          "}  
    
};

const byte team_9055[][16] PROGMEM = {
{"G1T100#          "},
{"G2T100#          "},
{"E54T1#           "},
{"B24T1#           "},
{"B21T1#           "},
{"B22T1#           "},
{"D40S1#           "},
{"D45S1#           "},
{"D45T1#           "},
{"D40T1#           "},
{"JS#              "},
{"G1MTEAM#         "},
{"G2M9055#         "},
{"W15#             "},
{"E54T2#           "},
{"G0T1#            "},
{"B21T16#          "},
{"B22T16#          "},
{"B24T2#           "},
{"D40T0#           "},
{"D45T0#           "},
{"D45S5#           "},
{"D40S5#           "},
{"W03#             "},
{"@@@@@@#          "}  
    
};



const byte all_tools_card[][16] PROGMEM = {
{"E51T1#           "},
{"W03#             "},
{"E52T1#           "},
{"W03#             "},
{"E53T1#           "},
{"W03#             "},
{"E54T1#           "},
{"W03#             "},
{"E55T1#           "},
{"W10#             "},
{"E51T2#           "},
{"W03#             "},
{"E52T2#           "},
{"W03#             "},
{"E53T2#           "},
{"W03#             "},
{"E54T2#           "},
{"W03#             "},
{"E55T2#           "},
{"@@@@@@#          "}
};


const byte panel_all_open_card[][16] PROGMEM = {
{"G3T100#          "},
{"E80T2#           "},
{"G3MALL OPEN#     "},
{"W05#             "},
{"G0T1#            "},
{"@@@@@@#          "}  
    
};

const byte panel_wave_card[][16] PROGMEM = {
{"G3T100#          "},
{"E80T4#           "},
{"G3MWAVE#         "},
{"W05#             "},
{"G0T1#            "},
{"@@@@@@#          "}  
    
};

const byte panel_all_open_long_card[][16] PROGMEM = {
{"G3T100#          "},
{"E80T3#           "},
{"G3MALL OPEN LONG#"},
{"W05#             "},
{"G0T1#            "},
{"@@@@@@#          "}  
    
};

const byte panel_fast_wave_card[][16] PROGMEM = {
{"G3T100#          "},
{"E80T5#           "},
{"G3MFAST WAVE#    "},
{"W05#             "},
{"G0T1#            "},
{"@@@@@@#          "}  
    
};
const byte lightsaber_demo_card[][16] PROGMEM = {
  { "G3T100#          " },
  { "E52T1#           " },
  { "G3MLIGHT SABER#  " },
  { "W05#             " },
  { "E52T2#           " },
  { "G0T1#            " },
  { "@@@@@@#          " }
};
const byte zapper_demo_card[][16] PROGMEM = {
  { "G3T100#          " },
  { "E51T8#           " },
  { "G3MZAPPER#       " },
  { "W05#             " },
  { "G0T1#            " },
  { "@@@@@@#          " }
};
const byte periscope_quick_card[][16] PROGMEM = {
  { "G3T100#          " },
  { "E53T1#           " },
  { "G3MZAPPER#       " },
  { "W05#             " },
  { "E53T2#           " },
  { "G0T1#            " },
  { "@@@@@@#          " }
};
const byte motivator_demo_card[][16] PROGMEM = {
  { "G3T100#          " },
  { "E54T1#           " },
  { "G3MMOTIVATOR#    " },
  { "W05#             " },
  { "E54T2#           " },
  { "G0T1#            " },
  { "@@@@@@#          " }
};

const byte lifeform_demo_card[][16] PROGMEM = {
  { "G3T100#          " },
  { "E55T1#           " },
  { "G3MLIFE FORM#    " },
  { "W05#             " },
  { "E55T2#           " },
  { "G0T1#            " },
  { "@@@@@@#          " }
};

const byte periscope_demo_card[][16] PROGMEM = {
  { "E53T1#           " },
  { "G3T100#          " },
  { "G3MPERISCOPE#    " },
  { "W03#             " },
  { "F56T001#         " },
  { "G3MDISPLAY 1#    " },
  { "W03#             " },
  { "F56T002#         " },
  { "G3MDISPLAY 2#    " },
  { "W03#             " },
  { "F56T003#         " },
  { "G3MDISPLAY 3#    " },
  { "W03#             " },
  { "F57T001#         " },
  { "G3MEYES 1#       " },
  { "W03#             " },
  { "F57T002#         " },
  { "G3MEYES 2#       " },
  { "W03#             " },
  { "F57T003#         " },
  { "G3MEYES 3#       " },
  { "W03#             " },
  { "F58T001#         " },
  { "G3MAIN 1#        " },
  { "W03#             " },
  { "F58T002#         " },
  { "G3MAIN 2#        " },
  { "W03#             " },
  { "F58T003#         " },
  { "G3MAIN 3#        " },
  { "W03#             " },
  { "F58T004#         " },
  { "G3MAIN 4#        " },
  { "W03#             " },
  { "F58T005#         " },
  { "G3MAIN 5#        " },
  { "W03#             " },
  { "F58T006#         " },
  { "G3MAIN 6#        " },
  { "W03#             " },
  { "E53T2#           " },
  { "G0T1#            " },
  { "@@@@@@#          " }
};
const byte tools_demo_card[][16] PROGMEM = {
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
  for (int x = 0; x < 20; x++) {
    byte read_data[18];
    for (int y = 0; y < 16; y++) {
      data[y] = pgm_read_byte(&(all_tools_card[x][y]));
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
