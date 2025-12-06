#pragma once
#include <Arduino.h>
#include "config.h"

class Sensors;

extern Sensors sensors;

class Sensors {
private:
    float s_L2;
    float s_L1;
    float s_M;
    float s_R1;
    float s_R2;

public:

    void begin()
  {//set the pins
    pinMode(IR_L2, INPUT);
    pinMode(IR_L1, INPUT);
    pinMode(IR_R1, INPUT);
    pinMode(IR_R2, INPUT);
    pinMode(IR_M, INPUT);
    pinMode(rotate_ir, INPUT);

    pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(color_out, INPUT);

    digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  }

    void update() {
        s_L2 = analogRead(IR_L2);
        s_L1 = analogRead(IR_L1);
        s_R1 = analogRead(IR_R1);
        s_R2 = analogRead(IR_R2);
        s_M = analogRead(IR_M);
    }

    int line_follow_error(){
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
        }*/
        
        return ir_error;

    }

    unsigned long readFrequency(bool fs2, bool fs3) {
  digitalWrite(s2, fs2);
  digitalWrite(s3, fs3);
  delay(40);
  return pulseIn(color_out, LOW);
}


char getDominantColor() {
  unsigned long red   = readFrequency(LOW, LOW);   // RED filter
  unsigned long blue  = readFrequency(LOW, HIGH);  // BLUE filter
  unsigned long green = readFrequency(HIGH, HIGH); // GREEN filter

  float sum = (float)red + (float)green + (float)blue;
  if (sum == 0) return 'N'; // No detection

  float Rn = red   / sum;
  float Gn = green / sum;
  float Bn = blue  / sum;
  Serial.println(sum);

  if (sum <=38) return 'W';

  else if (Rn < Gn && Rn < Bn) return 'R';
  else if (Gn < Rn && Gn < Bn) return 'G';
  
  else if (Bn < Gn && Bn < Rn) return 'B';
  //else if (sum<=50) return 'W';
   
  }


    int color() {
      color_value = 0;
  //eft_arm.write(0);
  //right_arm.write(90);

  char c = getDominantColor();

  if (c == 'R') {
    Serial.println("RED");
    color_value = 1;
   // moveSmooth(elbow, 120, 180, 10);
   // waitMillis(400000);
  } 
  else if (c == 'G') {
    Serial.println("GREEN");
    color_value = 3;
    //moveSmooth(elbow, 120, 180, 10);
   // waitMillis(400000);
  } 
  else if (c == 'B') {
    Serial.println("BLUE");
    color_value = 2;
  } 
  else if (c== 'W'){
    Serial.print("obstacle");
  }

  
  else {
    Serial.println("White");
    //moveSmooth(elbow, 120, 180, 10);
    //waitMillis(400000);
  }
}
};