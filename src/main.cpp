#include <Arduino.h>
#include <avr/interrupt.h>
#include "encoders.h"
#include"motors.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "config.h"
#include <Ticker.h>
#include <Servo.h>
#include "sensors.h"


Servo left_arm; //left arm
Servo right_arm; //right arm
Servo upper; //platform
Servo elbow; //rotate
Servo base;


// put function declarations here:
Motors motors;
Encoders encoders;
Sensors sensors;


void func() {
  encoders.update();
  sensors.update();
//motors.update(200,0,0);
motors.update(-speed,0,correction);
  }

Ticker ticker1(func, 20, 0, MILLIS);

void waitMillis(unsigned long interval) {
    unsigned long start = millis();
    while (millis() - start < interval) {
        ticker1.update();  // keep Ticker alive
    }
}

void  line_follow(){
  int ir_error = sensors.line_follow_error();
  // PID calculations
  integral += ir_error;
  float derivative = ir_error - lastError;
  float ir_correction = (Kp * ir_error) + (Ki * integral) + (Kd * derivative);
  
  lastError = ir_error;


  if (abs(ir_error) >= 2) {
    speed = 0;
    correction = -ir_correction/200;
  } else {
    speed = 150;
    correction =0;
  }

}


void rotate_ninety() {
  Serial.println(1111);
  while (rotate_ir>=threshold){
    line_follow();
    ticker1.update();
  }

  float ini_angle = encoders.robotAngle();

  while (abs(encoders.robotAngle()-ini_angle) <=90){
    correction = 1;
    ticker1.update();
  }
  correction = 0;
  waitMillis(2000);

}

void avoid_obstacal(){
   
}

void task_1(){
  int column = 1;
  while (column <= 7){
    line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
    ticker1.update();
  }
    column++;

  }
  Serial.println(column);
  ticker1.update();
  Serial.println(column);
    
  }
speed=0;correction=0;
  waitMillis(500);
  rotate_ninety();
  while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
    ticker1.update();
  }
  rotate_ninety();
}

unsigned long readFrequency(bool fs2, bool fs3);
//char getDominantColor();


void setup() {
  motors.begin();
  encoders.begin();
  encoders.reset();
  sensors.begin();
  ticker1.start();
 // ticker1.start();
  Serial.begin(9600);

  Wire.begin(); 

  left_arm.attach(33);  
  right_arm.attach(31);
  upper.attach(32); 
  elbow.attach(34);
  base.attach(30); 

  elbow.write(180);
  upper.write(0);
  base.write(0);
  
pinMode(trigger, OUTPUT);
  pinMode(eco, INPUT);

  
  motors.enable_controllers();
  
  //task_1();

  
}

void moveSmooth(Servo &servo, int fromAngle, int toAngle, int stepwaitMillis) {
  if (fromAngle < toAngle) {
    for (int a = fromAngle; a <= toAngle; a++) {
      servo.write(a);
      waitMillis(stepwaitMillis);
    }
  } else {
    for (int a = fromAngle; a >= toAngle; a--) {
      servo.write(a);
      waitMillis(stepwaitMillis);
    }
  }
}

void movebothSmooth(Servo &left, Servo &right, int startL, int startR, int endL, int endR, int stepwaitMillis) {
  int steps = max(abs(endL - startL), abs(endR - startR));
  for (int i = 0; i <= steps; i++) {
    int currentL = startL + (endL - startL) * i / steps;
    int currentR = startR + (endR - startR) * i / steps;
    left.write(currentL);
    right.write(currentR);
    waitMillis(stepwaitMillis);
  }

}

void boxpickup() {

  moveSmooth(base, 0, 0, 10);
  upper.write(0);
  left_arm.write(0);
  right_arm.write(90);
 
  waitMillis(1200);
  moveSmooth(elbow, 180, 120, 10);
  waitMillis(1200);
  //sensors.color();
  movebothSmooth(left_arm, right_arm, 0, 90, 75, 20, 10);
  waitMillis(1200);
  moveSmooth(elbow, 120, 180, 10);
 
}

void ballpickup() {
  //moveSmooth(upper, 0, 90, 15);
  //waitMillis(1200);
  
  //waitMillis(1200);

  //waitMillis(1200);
  //moveSmooth(base, 0, 90, 15);
  left_arm.write(0);
  right_arm.write(90);
  waitMillis(1200);
  //waitMillis(1200);
  moveSmooth(elbow, 180, 121, 15);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 0, 90, 80, 20, 10);
  waitMillis(1200);
  moveSmooth(elbow, 120, 180, 15);
  waitMillis(3000);
  //moveSmooth(base, 90, 0, 15);
  //waitMillis(1200);
  moveSmooth(elbow, 180, 120, 15);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 80, 20, 0, 90, 10);
  waitMillis(1200);
  moveSmooth(elbow, 121, 180, 15);
  waitMillis(1200);
  
}

void boxdrop() {
  moveSmooth(elbow, 180, 104, 10);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 85, 65, 0, 105, 10);
  waitMillis(1200);
  moveSmooth(elbow, 104, 180, 10);
  
}

void wall_following(){
  //uint16_t dist = sensor.readRangeSingleMillimeters();

  digitalWrite(trigger_r, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger_r, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_r, LOW);

  long duration = pulseIn(eco_r, HIGH, 20000);
  
  if (duration != 0) distance =duration * 0.034 / 2;// 999;
  //else distance = duration * 0.034 / 2;

  float error = distance - 5;
  Serial.println(duration);
  if (abs(error)>=2) {
    error =constrain(error,-2,2);
    speed = 0;
    correction = -error/1.2;
  }else{
    speed =100;
    correction=0;
  }
  ticker1.update();
    };

void pick_object(){
  boxpickup();
  waitMillis(1000);
  



}

void object(){
     
  //uint16_t dist = sensor.readRangeSingleMillimeters();

  line_follow();

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  long duration = pulseIn(eco, HIGH, 20000);
  
  if (duration != 0) distance =duration * 0.034 / 2;// 999;
  Serial.println(distance);
  //else distance = duration * 0.034 / 2;
  if (distance<=10){
    correction = 0;
    speed = 0;
    
    pick_object();

  }
  }

void loop() {

    // Always update ticker
  ticker1.update();

    // Always run line follow
    line_follow();

    //boxpickup();
    // task_1();
    //ll_following();
  //sensors.color();
  //delay(500);
  //ballpickup();
  //object();
  //feedforwardPWM(1);
  //feedforwardPWM(2);
  //Serial.println(encoders.leftRPS());
  //Serial.println(encoders.leftRPS());


    // -------- BLOCKING SERVO ARM CONTROL ------------
    //  left_arm.write(90);
    //  waitMillis(1500);
    //  left_arm.write(0);
    //  waitMillis(1500);
 }
