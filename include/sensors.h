#pragma once
#include <Arduino.h>
#include "config.h"

class SensorArray {
private:
    int pins[5];        // {LM, L, M, R, RM}
    int values[5];      // Latest readings
    int S_L2, S_L1, S_M, S_R1, S_R2;

public:
    // Constructor with your exact mapping
    SensorArray() {
        pins[0] = A0;   // LeftMost
        pins[1] = A1;   // Left
        pins[2] = A2;   // Middle
        pins[3] = A7;   // Right
        pins[4] = A3;   // RightMost
    }
//....................................................Initialize sensor pins................................................//
    void begin() {
        for (int i = 0; i < 5; i++)
            pinMode(pins[i], INPUT);
    }
//......................................................Read all sensors.....................................................//
    void readSensors() {
        for (int i = 0; i < 5; i++)
            values[i] = analogRead(pins[i]);
    }

    //........................................................Getter functions....................................................//
    int getLeftMost()  { return values[0]; }
    int getLeft()      { return values[1]; }
    int getMiddle()    { return values[2]; }
    int getRight()     { return values[3]; }
    int getRightMost() { return values[4]; }

    //.................................................... Print for debugging.................................................//
    void print() {
        assign_values();
        Serial.print("LM: "); Serial.print(S_L2);
        Serial.print("  L: "); Serial.print(S_L1);
        Serial.print("  M: "); Serial.print(S_M);
        Serial.print("  R: "); Serial.print(S_R1);
        Serial.print("  RM: "); Serial.println(S_R2);
    }
    //......................................Used to assign the used values from the analog reads..................................//
    void assign_values(){
        S_L2 =  getLeftMost();
        S_L1=getLeft() ;
        S_M = getMiddle();
        S_R1 = getRight();
        S_R2= getRightMost();
    }
//..............................................Calculate the line position error..........................................//
    int calculate_error(){
        assign_values();        
        int ir_error1 = 0;  // If error is Negative = line is on left, Positive = line is on right
        if (S_L1 >= threshold && S_L2 <= threshold) {
            ir_error1 += 1;
        }
        if (S_R1 >= threshold && S_R2 <= threshold) {
            ir_error1 -= 1;
        }
        if (S_L1 >= threshold && S_L2 >= threshold) {
            ir_error1 += 2;
        }
        if (S_R1 >= threshold && S_R2 >= threshold) {
            ir_error1 -= 2;
        }
        if (S_L1 <= threshold && S_L2 >= threshold) {
            ir_error1 += 3;
        }
        if (S_R1 <= threshold && S_R2 >= threshold ) {
            ir_error1 -= 3;
  }
  return ir_error1;
}
};