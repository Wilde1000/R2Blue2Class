#define red 12            //Arduino pin attached to the red tail lights
#define white 11          //Arduino pin attached to the white head lights
#define trig 5            //Arduino pin attached to the TRIG pin on the Ultrasonic Sensor
#define echo 6            //Arduino pin attached to the ECHO pin on the Ultrasonic Sensor
#define in1 A2            //Arduino pin attached to A1 on red motor controller board (Motor 1)
#define in2 A3            //Arduino pin attached to A2 on red motor controller board (Motor 1)
#define in3 A4            //Arduino pin attached to B1 on red motor controller board (Motor 2)
#define in4 A5            //Arduino pin attached to B1 on red motor controller board (Motor 2)

long duration;            //Variable to hold the time (in microseconds) the sensor needed to receive the echo
long buTimer = millis();  //Timer to hold the back up timer
long turnTimer = millis();  //Timer to hold the turning timer
long buInt = 50;          //Interval for backing up
long turnInt = 50;        //Interval for turning
int sDistance = 6;        //Variable to hold the safe distance - if less than this - evasive action 
int mSpeed = 200;         //Variable to hold the default speed of the motors

void lights(int status){
  digitalWrite(white, HIGH);
  if(status){
    digitalWrite(red, HIGH);
  }else{
    digitalWrite(red, LOW);
  }
  return;
}
void drive(int mode){
  if(!mode){
    forward();
    return;
  }
  halt();         //Stop the car
  back();         //Back the car up
  delay(buInt);   //Wait for the backup
  halt();         //Stop the car
  turn();         //Turn the car
  delay(turnInt); //Wait for the car to turn
  halt();         //Stop the car
  return;
}

void forward(){
  analogWrite(in1, mSpeed);
  digitalWrite(in2, LOW);
  analogWrite(in3, mSpeed);
  digitalWrite(in4, LOW);
  lights(0);
  return;
}

void halt(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  lights(1);
  return;
}

void back(){
  analogWrite(in2, mSpeed);
  digitalWrite(in1, LOW);
  analogWrite(in4, mSpeed);
  digitalWrite(in3, LOW);
  lights(1);
  return;
}

void turn(){
  analogWrite(in1, mSpeed);
  digitalWrite(in2, LOW);
  analogWrite(in4, mSpeed);
  digitalWrite(in3, LOW);
  lights(0);
}

int sensor(){
  
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);

  // The echo pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  duration = pulseIn(echo, HIGH);
  int inches = duration / 74 / 2;
  if(inches<sDistance) return 1;
  else return 0;

}



void setup() {
  // put your setup code here, to run once:
  pinMode(red, OUTPUT);
  pinMode(white, OUTPUT);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  drive(sensor());
}
