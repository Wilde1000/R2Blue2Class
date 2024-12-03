#include <SoftwareSerial.h>

// Define software serial pins
#define RX_PIN 4
#define TX_PIN 5

// Create a SoftwareSerial object
SoftwareSerial mySerial(RX_PIN, TX_PIN);

void setup() {
  // Start the hardware serial for communication with the Serial Monitor
  Serial.begin(9600);
  // Start the software serial for communication with the Mega
  mySerial.begin(9600);

  Serial.println("Nano ready. Enter text to send to the Mega:");
}

void loop() {
  // Check if data is available from the Serial Monitor
  if (Serial.available() > 0) {
    // Read the input from the Serial Monitor
    String input = Serial.readStringUntil('\n');
    // Send the input to the Mega
    mySerial.println(input);
    Serial.println("Sent to Mega: " + input);
  }

  
}
