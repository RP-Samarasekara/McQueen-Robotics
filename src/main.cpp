#include <Arduino.h>
#include <avr/interrupt.h>
#include "encoders.h"
#include"motors.h"
#include <Wire.h>
#include <VL53L0X.h>
#include "config.h"
#include <Ticker.h>
#include <Servo.h>


Servo left_arm; //left arm
Servo right_arm; //right arm
Servo upper; //platform
Servo elbow; //rotate
Servo base;


VL53L0X sensor;
// put function declarations here:
Motors motors;
Encoders encoders;


void func() {
  encoders.update();
//motors.update(100,0,0);
  motors.update(-speed,0,-correction);
  }

Ticker ticker1(func, 20, 0, MILLIS);

void waitMillis(unsigned long interval) {
    unsigned long start = millis();
    while (millis() - start < interval) {
        ticker1.update();  // keep Ticker alive
        //line_follow();     // keep line-following alive
    }
}
void  line_follow(){
  int s_L2 = analogRead(IR_L2);
  int s_L1 = analogRead(IR_L1);
  int s_R1 = analogRead(IR_R1);
  int s_R2 = analogRead(IR_R2);
  //int s_M = analogRead(IR_M);

  int ir_error = 0;  // If error is Negative = line is on left, Positive = line is on right
  
  // error calculation
  if (s_L1 >= threshold && s_L2 <= threshold) {
    ir_error += 1;
  }
  if (s_R1 >= threshold && s_R2 <= threshold) {
    ir_error -= 1;
  }
  if (s_L1 >= threshold && s_L2 >= threshold) {
    ir_error += 2;
  }
  if (s_R1 >= threshold && s_R2 >= threshold) {
    ir_error -= 2;
  }
  if (s_L1 <= threshold && s_L2 >= threshold) {
    ir_error += 3;
  }
  if (s_R1 <= threshold && s_R2 >= threshold ) {
    ir_error -= 3;
  }
  
  /*if (ir_error == 0) {
    if (s_M <= threshold && abs(last_nonzero_error) == 3) {
      ir_error = last_nonzero_error;
    }
  }*/

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

void setup() {
  motors.begin();
  encoders.begin();
  encoders.reset();
  ticker1.start();
  ticker1.start();
  Serial.begin(9600);

  pinMode(IR_L2, INPUT);
    pinMode(IR_L1, INPUT);
    pinMode(IR_R1, INPUT);
    pinMode(IR_R2, INPUT);
    pinMode(IR_M, INPUT);

    pinMode(rotate_ir, INPUT);

  pinMode(XSHUT_PIN_WALL_L, OUTPUT);
  digitalWrite(XSHUT_PIN_WALL_L, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN_WALL_L, HIGH);
  delay(10);

  Wire.begin(); 

  sensor.init();
  sensor.setTimeout(500);

  left_arm.attach(31);  
  right_arm.attach(30);
  upper.attach(32); 
  elbow.attach(34);
  base.attach(33); 

  elbow.write(180);
  upper.write(180);
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
  upper.write(180);
  left_arm.write(90);
  right_arm.write(0);
 
  waitMillis(1200);
  moveSmooth(elbow, 180, 104, 10);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 90, 60, 0, 105, 10);
  waitMillis(1200);
  moveSmooth(elbow, 104, 180, 10);
 
}

void ballpickup() {
  moveSmooth(upper, 0, 90, 15);
  waitMillis(1200);
  
  waitMillis(1200);

  waitMillis(1200);
  moveSmooth(base, 0, 90, 15);
  waitMillis(1200);
  moveSmooth(elbow, 180, 122, 15);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 10, 100, 120, 40, 10);
  waitMillis(1200);
  moveSmooth(elbow, 122, 180, 15);
  waitMillis(1200);
  moveSmooth(base, 90, 0, 15);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 85, 65, 10, 105, 10);
  waitMillis(1200);
  moveSmooth(upper, 90, 0, 15);
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

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  long duration = pulseIn(eco, HIGH, 20000);
  
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


  // element = dist - WALL_dist;

  // if (element>=max_wall_error){
  //   element = max_wall_error;
  // } else if (element<=-max_wall_error){
  //   element = -max_wall_error;
  // }

  // Serial.println(dist);
  
  // return element;
    };
void pick_object(){
  boxpickup();
  waitMillis(1000);
  



}

void object(){
     
  //uint16_t dist = sensor.readRangeSingleMillimeters();

  //line_follow();

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  long duration = pulseIn(eco, HIGH, 20000);
  
  if (duration != 0) distance =duration * 0.034 / 2;// 999;
  //else distance = duration * 0.034 / 2;
  if (distance<=10){
    correction = 0;
    speed = 0;
    pick_object();

  }
  }




unsigned long lastServoTime = 0;
int servoState = 0;

void feedforwardPWM(int motor, int step = 10, int delay_ms = 300) {
    // motor: 1 -> left, 2 -> right
    for (int i = -100; i <= 100; i += step) {
      
        if (motor == 1) {
            motors.set_left_motor_percentage(i);
            motors.set_right_motor_percentage(0);
            if (i==-100) delay(100); // stop the other motor
        } else if (motor == 2) {
            motors.set_right_motor_percentage(i);
            motors.set_left_motor_percentage(0); 
            if (i==-100) delay(100);// stop the other motor
        }

        if (abs(i)==30){
          delay(10000);
        }

        delay(delay_ms);           // wait for motor to stabilize
        encoders.update();         // update encoder readings
        float speed = 2 * encoders.robot_speed(); // mm/s

        // Print the motor, PWM, and measured speed
        if (motor == 1) Serial.print("Left,");
        else Serial.print("Right,");
        Serial.print(i);
        Serial.print(",");
        Serial.println(speed);
    }
}

void loop() {

    // Always update ticker
  ticker1.update();

    // Always run line follow
    line_follow();
    // task_1();
  //  wall_following();
  //object();
  //feedforwardPWM(1);
  //feedforwardPWM(2);
  Serial.println(encoders.leftRPS());
  //Serial.println(encoders.leftRPS());


    // -------- BLOCKING SERVO ARM CONTROL ------------
     /*elbow.write(180);
     waitMillis(1500);
     elbow.write(160);
     waitMillis(1500);*/
 }
