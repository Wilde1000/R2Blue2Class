const int stepPin1 = 4;
const int dirPin1 = 3;
const int stepPin2 = 6;
const int dirPin2 = 5;
const int button = 7;
const int lservo1 = 8;
const int lservo2 = 9;

const int stepsPerRev=200;
int numofSteps = 3000;
int pulseWidthMicros = 100; 	// microseconds
int millisBtwnSteps = 100;
int liftState=LOW;

void moveArm(int direct)
{  
  delay(50);   // delay for Controller startup
  if(direct){
    digitalWrite(dirPin1, LOW);     // Turn left
    digitalWrite(dirPin2, LOW);
  }else {
    digitalWrite(dirPin1, HIGH);     // Turn left
    digitalWrite(dirPin2, HIGH);
  }
  int delayUs = 1000;
  for(double i = 0; i < numofSteps; i++)  //5000 steps in one direction
  {   
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(delayUs);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(delayUs);
    if(delayUs > 300) delayUs-=5;
  } 
  return;
} 
void setup() {
 	Serial.begin(9600);
 	pinMode(stepPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
 	pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(button, INPUT_PULLUP);
 
}

void loop() {
 if(digitalRead(button)==0){
    liftState=!liftState;
    moveArm(liftState);
    delay(50);
 } 
}
