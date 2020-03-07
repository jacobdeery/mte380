/*
Motor Control
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
float pwm_value = 0; 

// Motor Speed Array & Parameters
bool serialConfigComplete = false;
byte serialData[8];
const int noDataPoints = 1;
int speedToExecute = 0;
union {
  byte asBytes[4];
  float asFloat;
} speed;
float leftMotorSpeeds[noDataPoints] = {0};
float rightMotorSpeeds[noDataPoints] = {0};

void setup() {
  // PIN CONFIG
  pinMode(dir_R,OUTPUT);
  pinMode(dir_L,OUTPUT);
  pinMode(pwm_R,OUTPUT); 
  pinMode(pwm_L,OUTPUT);
  
  Serial.begin(9600); //opens serial port & sets data rate
  while(!Serial){
    
  }
  digitalWrite(dir_R, fwd_R);
  digitalWrite(dir_L, fwd_L);
}

void loop() {
   //Receive array of bytes
   //Find number of speeds
   //seperate array and convert sets of 4 bytes to values
   //assign speeds to array of floats
   //write speeds to controller until new plan arrives
   if(Serial.available() && !serialConfigComplete){
    int signalFromSerial = Serial.read();
    if(signalFromSerial == '1'){
       serialConfigComplete = true;
       analogWrite(pwm_L, 75);
       delay(1000);
       analogWrite(pwm_L, 0);
    }
   }
  if(serialConfigComplete){
    if(Serial.available()){
      analogWrite(pwm_R, 75);
    delay(1000);
    analogWrite(pwm_R, 0);
    while(true);
    Serial.readBytes(serialData, 8);
    serialFlush();
    //Fill left speed array
    int j=0;
    for(int i = 0; i < noDataPoints; i++){
      speed.asBytes[0] = serialData[j];
      speed.asBytes[1] = serialData[j+1];
      speed.asBytes[2] = serialData[j+2];
      speed.asBytes[3] = serialData[j+3];
      leftMotorSpeeds[i] = speed.asFloat;
      j=j+4;
    }
    
    // Fill right speed array
    for(int i = 0; i < noDataPoints; i++){
      speed.asBytes[0] = serialData[j];
      speed.asBytes[1] = serialData[j+1];
      speed.asBytes[2] = serialData[j+2];
      speed.asBytes[3] = serialData[j+3];
      rightMotorSpeeds[i] = speed.asFloat;
      j=j+4;
    }
    
    // Reset motor speed pointer
    speedToExecute = 0;
    }
    
  }
  Serial.print(speedToExecute);
  Serial.print("\n");
  // Set left motor speed to next in plan
  pwm_value = leftMotorSpeeds[speedToExecute];
  Serial.print(pwm_value);
  Serial.print(" left \n");
  digitalWrite(dir_L, fwd_L);
  if(pwm_value < 0){
    pwm_value = -pwm_value;
    digitalWrite(dir_L, rev_L);
  }
  
  analogWrite(pwm_L, pwm_value);
  
  // Set right motor speed to next in plan
  pwm_value = rightMotorSpeeds[speedToExecute];
  Serial.print(pwm_value);
  Serial.print(" right \n");
  digitalWrite(dir_R, fwd_R);
  if(pwm_value < 0){
    pwm_value = -pwm_value;
    digitalWrite(dir_R, rev_R);
  }
  
  analogWrite(pwm_R, pwm_value);
  speedToExecute++;
  
  // If run through whole plan - stop
  if(speedToExecute == noDataPoints){
    Serial.print("stop \n");
//    analogWrite(pwm_L, 0);
//    analogWrite(pwm_R, 0);
    speedToExecute = 0;
  }
  
  // Hold speed for 1s
  delay(10);
}

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}  
