/*
Motor Control Demo Sketch
MTE380 Project - Group 4
Author: Taylor Robertson
*/

// Motor Output Pins
#define dir_L 4
#define pwm_L 5
#define pwm_R 6
#define dir_R 7

// Motor Directions
#define fwd_R HIGH
#define rev_R LOW
#define fwd_L LOW
#define rev_L HIGH

// Motor Speed
int speed = 50; 

int signalFromSerial;

void setup() {
   
  // PIN CONFIG
  pinMode(dir_R,OUTPUT);
  pinMode(dir_L,OUTPUT);
  pinMode(pwm_R,OUTPUT); 
  pinMode(pwm_L,OUTPUT);
  
  //TODO check baud rate
  Serial.begin(9600); //opens serial port & sets data rate
  
  // Start driving forward
  digitalWrite(dir_R, fwd_R);
  digitalWrite(dir_L, fwd_L);
  
  serialFlush();
}

void loop() {
   // Check for signal from pi
   // For demo will follow a series of movements after detecting wall
  if(Serial.available()){
    signalFromSerial = Serial.read();
    if(signalFromSerial == '1'){
      seeWall();
    }
    else if(signalFromSerial == '2'){
      stop();
    }
    else if(signalFromSerial == '3'){
      analogWrite(pwm_R, speed);
      analogWrite(pwm_L, speed);
    }
  }
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  

void seeWall(){
  analogWrite(pwm_R, speed);
  analogWrite(pwm_L, speed);

  // Stop
  stop();
  delay(1000);

  // Reverse from wall
  digitalWrite(dir_R, rev_R);
  digitalWrite(dir_L, rev_L);
  analogWrite(pwm_R, speed);
  analogWrite(pwm_L, speed);
  delay(3000);

  // Stop
  stop();
  delay(1000);
  
  //Turn to right
  digitalWrite(dir_R, fwd_R);
  digitalWrite(dir_L, fwd_L);
  speed = 25;
  analogWrite(pwm_R, speed);
  speed = 110;
  analogWrite(pwm_L, speed);
  delay(2000);
  
  //Start driving straight again
  speed = 50;
  analogWrite(pwm_R, speed);
  analogWrite(pwm_L, speed);
  delay(2000);
  stop();
  // Wait forever - end of demo
  while(true){};
}

void stop(){
  analogWrite(pwm_R, 0);
  analogWrite(pwm_L, 0);
}
