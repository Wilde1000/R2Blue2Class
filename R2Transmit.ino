#define CMD_MAX_LENGTH 63
char cmdStr[64];

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  checkSerial();
}

bool buildCommand(char ch, char* output_str) {
  static int pos = 0;
  switch (ch) {
    case '\n':
      output_str[pos] = '\0';
      pos = 0;
      return true;
    default:
      output_str[pos] = ch;
      if (pos <= CMD_MAX_LENGTH - 1) pos++;
      break;
  }
  return false;
}

void checkSerial() {
  bool is_cmd_complete = false;
  if (Serial.available()) {
    char ch = Serial.read(); 
    Serial.print(ch);

    is_cmd_complete = buildCommand(ch, cmdStr);
  }

  for (int i=0;cmdStr[i]!='\0';i++) {
    Serial1.write(cmdStr[i]);
  }
}