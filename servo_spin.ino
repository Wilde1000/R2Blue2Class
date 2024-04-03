const int stepPin = 2;
const int dirPin = 3;
const int stepsPerRev=200;
int numofSteps = 3100;
int pulseWidthMicros = 100; 	// microseconds
int millisBtwnSteps = 100;

void Pololu_A4988_Example()
{  
  delay(500);   // delay for Controller startup
  digitalWrite(dirPin, LOW);     // Turn left
  int delayUs = 1000;
  for(double i = 0; i < numofSteps; i++)  //5000 steps in one direction
  {   
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayUs);
    if(delayUs > 300) delayUs-=5;
  } 
  /*delay(500);   // delay for Controller startup
  digitalWrite(9, LOW);     // Turn left
  for(double i = 0; i < 5000; i++)  //5000 steps in one direction
  {   
    digitalWrite(10, HIGH);
    delay(1);
    digitalWrite(10, LOW);
    delay(1);
  } 
  delay(1000);
  */
} 
void setup() {
 	Serial.begin(9600);
 	pinMode(stepPin, OUTPUT);
 	pinMode(dirPin, OUTPUT);
 	Serial.println(F("A4988 Initialized"));

  Pololu_A4988_Example();

}
void loop() {
 
}