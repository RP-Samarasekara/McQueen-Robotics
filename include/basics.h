#pragma once
#include <Arduino.h>
#include "config.h"
#include <Servo.h>
#include "sensors.h"

void waitMillis(unsigned long ms);

class Basics;

extern Basics basics;

class Basics {
    private:
    Servo left_arm; //left arm
    Servo right_arm; //right arm
    Servo upper; //platform
    Servo elbow; //rotate
    Servo base;


    public:

    void begin(){
        left_arm.attach(33);  
        right_arm.attach(31);
        upper.attach(32); 
        elbow.attach(34);
        base.attach(30);

        elbow.write(180);
  upper.write(75);
  base.write(3);
    left_arm.write(0);  
  right_arm.write(90);

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
  upper.write(65);
  left_arm.write(0);
  right_arm.write(90);
 
  waitMillis(1200);
  moveSmooth(elbow, 110, 115, 10);
  waitMillis(1200);
  //sensors.color();
  //left_arm.write(110);
//  right_arm.write(10);

  movebothSmooth(left_arm, right_arm, 0, 90, 110, 10, 10);
  waitMillis(1200);
  moveSmooth(elbow, 115, 180, 10);
 
}

void ballpickup() {
  //moveSmooth(upper, 0, 90, 15);
  //waitMillis(1200);
  
  //waitMillis(1200);

  //waitMillis(1200);
  //moveSmooth(base, 0, 90, 15);
  left_arm.write(0);
  right_arm.write(90);
  // waitMillis(1200);
  // //waitMillis(1200);
  // moveSmooth(elbow, 180, 121, 15);
  // waitMillis(1200);
  // movebothSmooth(left_arm, right_arm, 0, 90, 80, 20, 10);
  // waitMillis(1200);
  // moveSmooth(elbow, 120, 180, 15);
  // waitMillis(3000);
  // //moveSmooth(base, 90, 0, 15);
  // //waitMillis(1200);
  // moveSmooth(elbow, 180, 120, 15);
  // waitMillis(1200);
  // movebothSmooth(left_arm, right_arm, 80, 20, 0, 90, 10);
  // waitMillis(1200);
  // moveSmooth(elbow, 121, 180, 15);
  // waitMillis(1200);
  
}

void boxdrop() {
  moveSmooth(elbow, 180, 104, 10);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 85, 65, 0, 105, 10);
  waitMillis(1200);
  moveSmooth(elbow, 104, 180, 10);
  
}

void check(){
    left_arm.write(0);
    right_arm.write(90);
    base.write(3);



    moveSmooth(elbow, 180, 115,10);
}

void take_back(){
    moveSmooth(elbow, 115, 180,10);
}



//line & wall

void  line_follow(){
  int ir_error = sensors.line_follow_error();
  // PID calculations
  integral += ir_error;
  float derivative = ir_error - lastError;
  float ir_correction = (Kp * ir_error) + (Ki * integral) + (Kd * derivative);
  
  lastError = ir_error;


  if (abs(ir_error) >= 2) {
    speed = 0;
    correction = -ir_correction/170;
  } else {
    speed = 150;
    correction =0;
  }

}

void wall_following(){
  Serial.print("d");
  //uint16_t dist = sensor.readRangeSingleMillimeters();

  long duration = sensors.r_ultrasonic();
  //Serial.print(duration);
  
  if (duration != 0) distance =duration * 0.034 / 2;// 999;
  //else distance = duration * 0.034 / 2;
  Serial.print(distance);

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
  //ticker1.update();
    };
};