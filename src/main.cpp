#include <Arduino.h>
//#include "motors.h"
#include "Ticker.h"
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

//Ticker sendTicker;
//Ticker controlTicker;
// void updateFunctions() {
//     encoders.update();
//     motors.update(speed, 0, correction);
// }
// ISR(TIMER1_COMPA_vect) {
//   encoders.update();
//  // motors.update(0,0,2);
//   motors.update(speed,0,correction);
void func() {
  encoders.update();
  //motors.update(speed,0,correction);
  }

Ticker ticker1(func, 20, 0, MILLIS);


float  line_follow(){
  int s_L2 = analogRead(IR_L2);
  int s_L1 = analogRead(IR_L1);
  int s_R1 = analogRead(IR_R1);
  int s_R2 = analogRead(IR_R2);
  int s_M = analogRead(IR_M);

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
    correction = -ir_correction/150;
  } else {
    speed = 200;
    correction =0;
  }

}



// void setupTimer1() {

//     cli();  // Stop interrupts

//     TCCR1A = 0;            // Normal operation
//     TCCR1B = 0;

//     // Set compare value for 10ms interval
//     OCR1A = 2499;           // (16MHz / (64*100)) - 1

//     TCCR1B |= (1 << WGM12); // CTC mode
//     TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64

//     TIMSK1 |= (1 << OCIE1A); // Enable interrupt

//     sei();  // Enable interrupts
// }



void rotate_ninety() {

  float ini_angle = encoders.robotAngle();

  while (abs(encoders.robotAngle()-ini_angle) <=90){
    correction = 1;
  }
  correction = 0;
  //waitMillis(2000);

}

void task_1(){
  int column = 1;
  while (column <= 9){
    line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
  }
    column++;

  }
  Serial.println(column);
    
  }
  rotate_ninety();
}

float wall_following(){
  uint16_t dist = sensor.readRangeSingleMillimeters();


  element = dist - WALL_dist;

  if (element>=max_wall_error){
    element = max_wall_error;
  } else if (element<=-max_wall_error){
    element = -max_wall_error;
  }

  Serial.println(dist);
  
  return element;
  };


unsigned long lastServoTime = 0;
int servoState = 0;

void waitMillis(unsigned long interval) {
    unsigned long start = millis();
    while (millis() - start < interval) {
        ticker1.update();  // keep Ticker alive
        line_follow();     // keep line-following alive
    }
}
//.............................................Arm Movements....................................//

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

// ---- Action functions ----
void boxpickup() {

  moveSmooth(base, 0, 0, 10);
  upper.write(0);
  waitMillis(1200);
  moveSmooth(elbow, 180, 104, 10);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 0, 105, 65, 70, 10);
  waitMillis(1200);
  moveSmooth(elbow, 104, 180, 10);
 
}

void ballpickup() {
  moveSmooth(upper, 0, 90, 15);
  waitMillis(1200);
  moveSmooth(base, 0, 90, 15);
  waitMillis(1200);
  moveSmooth(elbow, 180, 116, 15);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 0, 105, 80, 60, 15);
  waitMillis(1200);
  moveSmooth(elbow, 116, 180, 15);
  waitMillis(1200);
  moveSmooth(base, 90, 0, 15);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 80, 60, 0, 105, 15);
  waitMillis(1200);
  moveSmooth(upper, 90, 0, 15);
  waitMillis(3000);
  
}

void boxdrop() {
  moveSmooth(elbow, 180, 104, 10);
  waitMillis(1200);
  movebothSmooth(left_arm, right_arm, 65, 70, 0, 105, 10);
  waitMillis(12);
  moveSmooth(elbow, 104, 180, 10);
  
}







//.................................void_setup....................................//
void setup() {
  motors.begin();
  encoders.begin();
  encoders.reset();
  ticker1.start();
  ticker1.start();
 // setupTimer1();
  Serial.begin(9600);

  pinMode(IR_L2, INPUT);
    pinMode(IR_L1, INPUT);
    pinMode(IR_R1, INPUT);
    pinMode(IR_R2, INPUT);
    pinMode(IR_M, INPUT);

  pinMode(XSHUT_PIN_WALL_L, OUTPUT);
  digitalWrite(XSHUT_PIN_WALL_L, LOW);
  waitMillis(10);
  digitalWrite(XSHUT_PIN_WALL_L, HIGH);
  waitMillis(10);

  Wire.begin(); 

  sensor.init();
  sensor.setTimeout(500);

  //task_1();
  //rotate_ninety();

  left_arm.attach(31);  
  right_arm.attach(30);
  upper.attach(32); 
  elbow.attach(34);
  base.attach(33); 

  elbow.write(180);
  upper.write(0);
  left_arm.write(0);
  right_arm.write(105);
  base.write(0);

  motors.enable_controllers();
 }


//.................................void_loop....................................//
void loop() {

    // Always update ticker
    ticker1.update();

    // Always run line follow
    //line_follow();

    boxpickup();
    waitMillis(5000);
    boxdrop();
    waitMillis(5000);
    ballpickup();
    waitMillis(5000);
    boxdrop();
    waitMillis(5000);
}
