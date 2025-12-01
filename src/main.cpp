#include <Arduino.h>
//#include "motors.h"
#include "Ticker.h"
#include <avr/interrupt.h>
#include "encoders.h"
#include"motors.h"
#include <Wire.h>
#include <VL53L0X.h>
//#include <Servo.h>

/*Servo left_arm; //left arm
Servo right_arm; //right arm
Servo upper; //platform
Servo elbow; //rotate
Servo base;*/


VL53L0X sensor;
// put function declarations here:
Motors motors;
Encoders encoders;

//Ticker sendTicker;
//Ticker controlTicker;

ISR(TIMER1_COMPA_vect) {
  encoders.update();
  motors.update(-200,0,0);
 // motors.update(200,wall_error,0);
}

float line_follow(){
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
  
  if (ir_error == 0) {
    if (s_M <= threshold) {
      ir_error = last_nonzero_error;
    }
  }

  // PID calculations
  integral += ir_error;
  float derivative = ir_error - lastError;
  float ir_correction = (Kp * ir_error) + (Ki * integral) + (Kd * derivative);
  
  lastError = ir_error;

  if (ir_error != 0) {
    last_nonzero_error = ir_error;
  }

  return(ir_correction);
}
   

void setupTimer1() {

    cli();  // Stop interrupts

    TCCR1A = 0;            // Normal operation
    TCCR1B = 0;

    // Set compare value for 10ms interval
    OCR1A = 2499;           // (16MHz / (64*100)) - 1

    TCCR1B |= (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64

    TIMSK1 |= (1 << OCIE1A); // Enable interrupt

    sei();  // Enable interrupts
}

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
void setup() {
  motors.begin();
  encoders.begin();
  encoders.reset();
  setupTimer1();
  Serial.begin(9600);

  pinMode(IR_L2, INPUT);
    pinMode(IR_L1, INPUT);
    pinMode(IR_R1, INPUT);
    pinMode(IR_R2, INPUT);
    pinMode(IR_M, INPUT);

  pinMode(XSHUT_PIN_WALL_L, OUTPUT);
  digitalWrite(XSHUT_PIN_WALL_L, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN_WALL_L, HIGH);
  delay(10);

  Wire.begin(); 

  sensor.init();
  sensor.setTimeout(500);

  /*left_arm.attach(31);  
  right_arm.attach(30);
  upper.attach(32); 
  elbow.attach(34);
  base.attach(33); */

  //elbow.write(180);

 


  // controlTicker.attach(0.005,[](){
  //     encoders.update();
  //     motion.update();
  //     motors.update(motion.velocity(), motion.omega(), sensors.get_steering_feedback());

  //});

  //sendTicker.attach(0.02, [](){

      // encoders.update();
      // motion.update();
      // motors.update(motion.velocity(), motion.omega(), sensors.get_steering_feedback());
      //communications.send("HI THERE");
      //communications.send("IRSENSORS",sensors.all_IR_readings, NUM_SENSORS+2);
      // communications.send_velocity();
      //communications.check(); 
    //  sensors.update();


    //  });
  


  //delay(5000);
  //encoders.update();
  //motors.set_right_motor_pwm(500);
  //motors.set_left_motor_pwm(500);
  // // delay(1000);
  motors.enable_controllers();
 
  // motors.omega = 0;
  // motors.speed = 0;

  
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



void loop() {
  // motion.reset_drive_system();
  // robot.move_straight(0.1);
  // delay(1000);
  //encoders.update();
  //Serial.println(encoders.robot_speed());

  irCorrection = line_follow()/0.5;

  //Serial.println(irCorrection);
  
  // feedforwardPWM(1); // Test left motor
  // delay(1000);
  // feedforwardPWM(2); // Test right motor
  delay(10);
  /*wall_error = wall_following()*5;
  Serial.print(wall_error);
  Serial.print(".....................");
  Serial.print(encoders.leftRPS());
  Serial.print(".....................");
  Serial.print(encoders.rightRPS());*/


}