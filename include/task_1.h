#pragma once
#include <Arduino.h>
#include "config.h"
#include "basics.h"
#include "encoders.h"
#include <Ticker.h>


extern Ticker ticker1;

void waitMillis(unsigned long ms);

class Task_1;

extern Task_1 task_1;

class Task_1{
    private:

    public:
    void go(int row,int start,int stop){
        int i = start;

    while (i < stop){
    basics.line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    basics.line_follow();
    ticker1.update();
  }
    i++;

  }
//   Serial.println(column);
  ticker1.update();
//   Serial.println(column);

  int increment = object(row,i,0);
  i=i+increment;
    
 }

    }

    void endd(int c){
while(c<=8){
  basics.line_follow();
  ticker1.update();
  
}
go_to_end();
speed =0; correction=0;ticker1.update();
  waitMillis(500);
rotate_ninety(-1);
speed =0; correction=0;ticker1.update();
  waitMillis(500);
go_to_end();
speed =0; correction=0;ticker1.update();
  waitMillis(500);
rotate_ninety(1);
speed =0; correction=0;ticker1.update();
  waitMillis(500);
go_to_end();
speed =0; correction=0;ticker1.update();
  waitMillis(500);
rotate_ninety(1);
speed =0; correction=0;ticker1.update();
  waitMillis(500);
  speed =100; correction=0;ticker1.update();
  waitMillis(1000);
  speed =0; correction=0;ticker1.update();
  waitMillis(500);
  task_2();





    }

void go_back(){
  if(analogRead(rotate_IR_L)>=threshold_2&& analogRead(rotate_IR_L)>=threshold_2){
    Serial.println(analogRead(rotate_IR_L));
  while(analogRead(rotate_IR_L)>=threshold_2 && analogRead(rotate_IR_L)>=threshold_2) {
    speed = -150;correction=0;
    ticker1.update();
  }}
  speed =0;correction=0;
  waitMillis(500);
}

void go_to_end(){
  if (analogRead(rotate_IR_R)<=threshold_2 || analogRead(rotate_IR_L)<=threshold_2){
  while(analogRead(rotate_IR_R)<=threshold_2 || analogRead(rotate_IR_L)<=threshold_2){
    basics.line_follow();
    //speed=150;correction=0;
    ticker1.update();
  }}
    //speed=120;correction=0;ticker1.update();
   // waitMillis(200);
    while(analogRead(rotate_IR_R)>=threshold_2 && analogRead(rotate_IR_L)>=threshold_2){
      basics.line_follow();
      ticker1.update();
      Serial.println(111);
      Serial.println(analogRead(rotate_IR_L));
      Serial.println(analogRead(rotate_IR_R));
    }

    while(analogRead(rotate_IR_R)<=threshold_2 || analogRead(rotate_IR_L)<=threshold_2 ){
    basics.line_follow();
      ticker1.update();
      Serial.println(222222);
    }
  
}





void rotate_ninety(int d) {
  Serial.println(1111);
  /*while (rotate_ir>=threshold){
    line_follow();
    ticker1.update();
  }*/

  float ini_angle = encoders.robotAngle();

  while (abs(encoders.robotAngle()-ini_angle) <=90){
     correction = d;
    ticker1.update();
  }
  correction = 0;
  waitMillis(500);

}

void rotate_oneeighty(){
float ini_angle = encoders.robotAngle();

  while (abs(encoders.robotAngle()-ini_angle) <=180){
     correction = 1;
    ticker1.update();
  }
  correction = 0;
  waitMillis(500);
}

void next_row(int d,int r){
  
  go_to_end();
  speed=0;correction=0;
  waitMillis(500);
  rotate_ninety(d);
  waitMillis(500);
  object(r,9,10);

  go_to_end();
  speed=0;correction=0;
  waitMillis(500);
  rotate_ninety(d);
  waitMillis(500);
}   

void avoid_obstacal(int r){
   speed =0; correction=0;
  waitMillis(500);

  go_back();
  waitMillis(500);

  rotate_ninety(r);
  speed =0; correction=0;
  waitMillis(500);

  go_to_end();
  speed =0; correction=0;
  waitMillis(500);

  rotate_ninety(-r);
speed =0; correction=0;
  waitMillis(500);

  /*speed =150; correction=0;
  waitMillis(1450);
  speed =0; correction=0;
  waitMillis(500);*/

  //waitMillis(500);
  go_to_end();
  go_to_end();
  speed =0; correction=0;
  waitMillis(500);

  rotate_ninety(-r);
speed =0; correction=0;
  waitMillis(500);

  go_to_end();
  speed =0; correction=0;
  waitMillis(500);

  rotate_ninety(r);
  speed =0; correction=0;
  waitMillis(500);

  speed =-120;correction=0;
  waitMillis(500);
  speed =0; correction=0;
  waitMillis(500);
  //speed =150; correction=0;
  //waitMillis(1450);
  /*speed =0; correction=0;
  waitMillis(500);

  //waitMillis(500);
  rotate_ninety(r);*/
}

void drop_object( int colour,int r, int c) {

  speed=-120;correction=0;ticker1.update();waitMillis(500);
   speed =0; correction=0;
  waitMillis(500);

  //int c1 = c; int r1=r;
int pos=colour;
  int rotation = 0;

  if (r%2==0) rotation =1;
  else rotation =-1;
 // int row =1;
 //if (c==9) rotation=-rotation;

  //while (row<=8){
   //column = c;

   
 
 

if (pos>=10){
colour-=10;
}else{
go_to_end();
go_to_end();
 speed=0;correction=0;
 waitMillis(500);

rotate_ninety(rotation);
 speed=0;correction=0;
 waitMillis(500);
}

 go(-r,r,8);

 go_to_end();
 speed=0;correction=0;
 waitMillis(500);


 int end;
int pp ;
 if (r%2==0){
  if (colour==1) end=3;
 else if (colour==2) end=7;
 else end =5;
 pp=9-end;
 }
 else{
if (colour==1) end=7;
 else if (colour==2) end=3;
 else end =5;
pp=end;
 }
 

int distance = end-(c+1);
int rotation2;
int turn;

if (r%2==0){
  if (distance<0) {rotation2=1;turn=1; }
else { rotation2=-1;turn=1 ;}

 }
 else{
if (distance<0) {rotation2=-1;turn=-1; }
else { rotation2=1;turn=-1 ;}

 }
 


 rotate_ninety(rotation2);
 speed=0;correction=0;
 waitMillis(500);

 go(-turn,1,abs(distance)-1);

 go_to_end();

 speed=0;correction=0;
 waitMillis(500);
 rotate_ninety(-rotation2);
 speed=0;correction=0;
 waitMillis(500);

 //go_to_end();
 //speed=0;correction=0;
 //waitMillis(500);

 basics.boxdrop();

 if (objects>=2){
  endd(8-end);
 }
 speed=0;correction=0;
 waitMillis(500);
 rotate_ninety(-rotation2);
 speed=0;correction=0;
 waitMillis(500);
 go(turn,1,abs(distance)-1);
 speed=0;correction=0;
 waitMillis(500);
 go_to_end();
 speed=0;correction=0;
 waitMillis(500);
 rotate_ninety(-rotation2);
 speed=0;correction=0;
 waitMillis(500);

 go(r,1,9-r);

 speed=0;correction=0;
 waitMillis(500);
 go_to_end();
 speed=0;correction=0;
 waitMillis(500);
 if (pos<=10) {rotate_ninety(rotation);speed=0;correction=0;
 waitMillis(500);
}
 else {rotate_oneeighty();speed =-120;correction=0;
  waitMillis(500);
  speed =0; correction=0;
  waitMillis(500);}
 
 }
 void pick_object(){
  basics.boxpickup();
  waitMillis(1000);
  //sensors.color();
  //waitMillis(40000);
}

int detect(){
  
  waitMillis(2000);
  sensors.color();

  return color_value;
  //waitMillis(40000);

}

int object(int r, int c,int pos){
     
  //uint16_t dist = sensor.readRangeSingleMillimeters();

  basics.line_follow();

  long duration = sensors.f_ultrasonic();
  
  if (duration != 0) distance =duration * 0.034 / 2;// 999;
  Serial.println(distance);
  //else distance = duration * 0.034 / 2;
  if (distance<=11.6){
    
    correction = 0;
    speed = 0;
    
    int rot;

    if (r%2==0) rot = 1;
    else rot =-1;
if (obstacal){
      avoid_obstacal(rot);
      return(2);
}else{
  basics.check();

    int colour =detect();
      if (colour==0){
      basics.take_back();
      avoid_obstacal(rot);
      return(2);


    }
    else{
      objects++;
      pick_object();
      obstacal=true;
      drop_object(colour+pos,r,c+1);
      obstacal=false;
      return(1);

    }

}
  

  }else return(0);
  }



void task_1(){
  go_back();
  speed=0;correction=0;
  waitMillis(500);
  rotate_ninety(1);
  speed=0;correction=0;
  waitMillis(500);
  go_to_end();
  speed=0;correction=0;
  waitMillis(500);
  rotate_ninety(-1);
  speed=0;correction=0;
  waitMillis(500);
  
  int row =1;

  while (row<=8){
   go(row,1,9);
 
 if (row%2 == 0) next_row(1,row);
 else next_row(-1,row);
row++;
Serial.println(row);
}
}








};
