#pragma once

#include <Arduino.h>
#include "config.h"
#include "encoders.h"


class Motors;

extern Motors motors;

class Motors
{  
public:
float speed = 0;
float omega = 0;
volatile float fwdKp = FWD_KP;
volatile float fwdKd = FWD_KD;
volatile float fwdKi = FWD_KI;
volatile float rotKp = ROT_KP;
volatile float rotKd = ROT_KD;
volatile float rotKi = ROT_KI;

void begin()
  {//set the pins
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
//set all the motor pins low
    digitalWrite(LEFT_MOTOR_PWM, 0);
    digitalWrite(LEFT_MOTOR_IN1, 0);
    digitalWrite(LEFT_MOTOR_IN2, 0);
    digitalWrite(RIGHT_MOTOR_IN2, 0);
    digitalWrite(RIGHT_MOTOR_IN1, 0);
    digitalWrite(RIGHT_MOTOR_PWM, 0);
    //calls setupPWM to initiate PWM
    //setupPWM();
  }
    void reset_controllers()
  {//reset all the PID values
    m_fwd_error = 0;
    m_rot_error = 0;
    m_previous_fwd_error = 0;
    m_previous_rot_error = 0;
  }
  float position_controller()
  {//take the error from desired and actual movement diff
    float increment = m_velocity * encoders.loopTime_s();
    m_fwd_error += increment - encoders.robot_fwd_change();
    float diff = m_fwd_error - m_previous_fwd_error;
    m_previous_fwd_error = m_fwd_error;
    acc_fwd_error += m_fwd_error;

    // change them to config kp kd later
    float output = fwdKp * m_fwd_error + fwdKd * diff+fwdKi*acc_fwd_error;
    return output;
  }

  float angle_controller(float steering_adjustment)
  {
    float increment = m_omega * encoders.loopTime_s();
    m_rot_error += increment - encoders.robot_rot_change();
    float diff = m_rot_error - m_previous_rot_error;

    m_previous_rot_error = m_rot_error;
    m_rot_error -= steering_adjustment;
    acc_rot_error += m_rot_error;

    // changethis kp kd to config kp kd later
    float output = rotKp * m_rot_error + rotKd * diff+rotKi*acc_rot_error;
    return output;
  }

  void update(float velocity, float omega, float steering)
  {
    m_velocity = velocity;
    m_omega = omega;

    float pos_output = position_controller();
    float rot_output = angle_controller(steering);
    float left_output = 0;
    float right_output = 0;

    left_output = pos_output - rot_output;
    right_output = pos_output + rot_output;

    float tangent_speed = m_omega * ROBOT_RADIUS * RADIANS_PER_DEGREE;
    float left_speed = m_velocity - tangent_speed;
    float right_speed = m_velocity + tangent_speed;
    float left_ff = left_feed_forward_percentage(left_speed);
    float right_ff = right_feed_forward_percentage(right_speed);
    if (m_feedforward_enabled)
    {
      left_output += left_ff;
      right_output += right_ff;
    }
    if (m_controller_output_enabled)
    {
      set_left_motor_percentage(left_output);
      set_right_motor_percentage(right_output);
    }
  }

  float left_feed_forward_percentage(float left_feed_velocity)
  {
    ///////give the percentage required to acheive a given velocity--- |v|<500
    //int l_rps = (left_feed_velocity * PULSES_PER_ROTATION) / MM_PER_ROTATION;
    float v = left_feed_velocity;//     /2 is due to a mistake in calculations
    float l_feed_percentage;
    if (v>=0){


    l_feed_percentage = (1.31699 * pow(10, -10)) * v * v * v * v
                 + (2.309 * pow(10, -7)) * v * v * v
                 - (2.7260 * pow(10, -4)) * v * v
                 + 0.1325 * v
                 + 11.4404;
    }
    else {
      l_feed_percentage =(4.07046e-7) * v * v * v
         + 0.000281453 * v * v
         + 0.106593 * v
         - 2.66851;
    }
    return l_feed_percentage;
  }

  float right_feed_forward_percentage(float right_feed_velocity)
  {
    ///////give the percentage required to acheive a given velocity--- |v|<500
    //int r_rps = (left_feed_velocity * PULSES_PER_ROTATION) / MM_PER_ROTATION;
    float v = right_feed_velocity;//     /2 is due to a mistake in calculations
    //Serial.print("  right feed  ");
    //Serial.print(v);
    float r_feed_percentage;
    if (v>0){
      r_feed_percentage =(1.67333e-10) * v * v * v * v
         + (1.56361e-7) * v * v * v
         - 0.000222106 * v * v
         + 0.117029 * v
         + 12.43975;
    }
    else {
      r_feed_percentage = -(1.00097e-9) * v * v * v * v
         - 0.00000124135 * v * v * v
         - 0.000575011 * v * v
         - 0.0524384 * v
         - 11.32181;
    }
    
    return r_feed_percentage;
  }
  void stop()
  {
    set_left_motor_percentage(0);
    set_right_motor_percentage(0);
  }
    void set_left_motor_percentage(float percentage)
  {
    percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
    
    int left_pwm = calculate_pwm(percentage);
    

    set_left_motor_pwm(left_pwm);
  }
    void set_right_motor_percentage(float percentage)
  {
    percentage = constrain(percentage, -maxMotorPercentage, maxMotorPercentage);
    
    m_right_motor_percentage = percentage;
    int right_pwm = calculate_pwm(percentage);

    
    
    set_right_motor_pwm(right_pwm);
  };


void set_left_motor_pwm(int pwm) {
    pwm = MOTOR_LEFT_POLARITY * pwm;
    if (pwm < 0) {
        pwm = -pwm + M_BALNCE_PWM;
        digitalWrite(LEFT_MOTOR_IN1, HIGH);
        digitalWrite(LEFT_MOTOR_IN2, LOW);
    } else {
        pwm = pwm + M_BALNCE_PWM;
        digitalWrite(LEFT_MOTOR_IN1, LOW);
        digitalWrite(LEFT_MOTOR_IN2, HIGH);
    }
    pwm = constrain(pwm, 0, 255);  // Mega PWM max 255
    analogWrite(LEFT_MOTOR_PWM, pwm);
}

void set_right_motor_pwm(int pwm) {
    pwm = MOTOR_RIGHT_POLARITY * pwm;
    if (pwm < 0) {
        pwm = -pwm - M_BALNCE_PWM;
        digitalWrite(RIGHT_MOTOR_IN1, HIGH);
        digitalWrite(RIGHT_MOTOR_IN2, LOW);
    } else {
        pwm = pwm - M_BALNCE_PWM;
        digitalWrite(RIGHT_MOTOR_IN1, LOW);
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    }
    pwm = constrain(pwm, 0, 255);
    analogWrite(RIGHT_MOTOR_PWM, pwm);
}

//THIS IS WHERE THE NEW MAPPING IS DONE

    int calculate_pwm(float desired_percentage)
  {
    int pwm = (int)(desired_percentage * 255.0 / 100.0); // 0-100% -> 0-255
    return constrain(pwm, -255, 255);
}
  
void enable_controllers()
  {
    m_controller_output_enabled = true;
  }

  void disable_controllers()
  {
    m_controller_output_enabled = false;
  }
  void calibrate_motors(){
    int i = 150;
     for (int j=1;j<25;j++){
      int speed = 20*j;
      int start = millis();
      int avg_PWM = 0;
      int avg_speed = 0;
      double count = 0;
      while (millis()-start<8000){
        motors.set_left_motor_pwm(i);
        if (encoders.robot_speed()*2<speed)
          i=i+1;

        if (encoders.robot_speed()*2>speed)
          i=i-1;

        
        encoders.update();
        
        delay(5);
        avg_PWM += i;
        avg_speed += encoders.robot_speed()*2;
        count++;
    }
   
    }
  }
private:
  float m_left_motor_percentage;
  float m_right_motor_percentage;

  float m_previous_fwd_error;
  float m_previous_rot_error;
  float m_fwd_error;
  float m_rot_error;
  float acc_fwd_error;
  float acc_rot_error;

  float m_velocity;
  float m_omega;

  bool m_feedforward_enabled = true;
  bool m_controller_output_enabled;
  unsigned long i = 0;
};