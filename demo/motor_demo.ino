/*
 * Construction Check Motor Demo
 *  
 * Group 4 Robotics
 * MTE 380 Project
 * Winter 2020
 */

// Motor output pins
#define pwm_R 5
#define pwm_L 6
#define dir_R 4
#define dir_L 7

// Motor directions -- TODO: check these are correct
#define forward_L LOW
#define reverse_L HIGH
#define forward_R HIGH
#define reverse_R LOW

// Motor speed
int pwm_value = 75; 

void setup() {

  // PIN CONFIG
  pinMode(pwm_R,OUTPUT);
  pinMode(pwm_L,OUTPUT);
  pinMode(dir_R,OUTPUT); 
  pinMode(dir_L,OUTPUT);

  // DEMO
  drive("forward", 3000);
  stop(3000);
  drive("right", 3000);
  stop(3000);
  drive("left", 3000);
  stop(3000);
  drive("reverse", 3000);
}

void drive(String direction, int duration){
 
  // Set motor directions
  if (direction == "forward"){
    digitalWrite(dir_L, forward_L);
    digitalWrite(dir_R, forward_R);
  } else if (direction == "reverse"){
    digitalWrite(dir_L, reverse_L);
    digitalWrite(dir_R, reverse_R);
  } else if (direction == "right"){
    digitalWrite(dir_L, forward_L);
    digitalWrite(dir_R, reverse_R);
  } else if (direction == "left"){
    digitalWrite(dir_L, reverse_L);
    digitalWrite(dir_R, forward_R);
  } else {
    Serial.println("Invalid direction!");
    digitalWrite(dir_L, forward_L);
    digitalWrite(dir_R, forward_R);
  }

  // Drive
  analogWrite(pwm_R,pwm_value);
  analogWrite(pwm_L,pwm_value);
  delay(duration);
}

void stop(int duration){
  analogWrite(pwm_R,0);
  analogWrite(pwm_L,0);
  delay(duration);
}

void loop() {}
