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
#include <LiquidCrystal_I2C.h>


Servo left_arm; //left arm
Servo right_arm; //right arm
Servo upper; //platform
Servo elbow; //rotate
Servo base;

LiquidCrystal_I2C lcd(0x27, 16, 2);

enum MenuLevel { MAIN_MENU, TASK_MENU, INDIVIDUAL_MENU };
MenuLevel currentMenu = MAIN_MENU;

int mainIndex = 0;
int taskIndex = 0;
int indivIndex = 0;

unsigned long lastJoy = 0;
int joyDelay = 200;

String mainMenuItems[2] = {"Tasks", "Individual"};
String taskMenuItems[7] = {"Task 1","Task 2","Task 3","Task 4","Task 5","Task 6","Task 7"};
String indivMenuItems[5] = {"Ballpickup", "Boxpickup", "Boxdrop", "Line following", "Wall following"};

int lastMainIndex = -1;
int lastTaskIndex = -1;
int lastIndivIndex = -1;

MenuLevel lastMenu = (MenuLevel)-1;


// put function declarations here:
Motors motors;
Encoders encoders;
Sensors sensors;


void func() {
  encoders.update();
  sensors.update();
//motors.update(200,0,1);
motors.update(-speed,0,-correction);
  }

Ticker ticker1(func, 20, 0, MILLIS);



void waitMillis(unsigned long interval) {
    unsigned long start = millis();
    while (millis() - start < interval) {
        ticker1.update();  // keep Ticker alive
    }
}

int column = 1;

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

void go_back(){
  if(analogRead(rotate_IR_L)>=threshold && analogRead(rotate_IR_L)>=threshold){
    Serial.println(analogRead(rotate_IR_L));
  while(analogRead(rotate_IR_L)>=threshold && analogRead(rotate_IR_L)>=threshold) {
    speed = -150;correction=0;
    ticker1.update();
  }}
  speed =0;correction=0;
  waitMillis(500);
}

void go_to_end(){
  if (analogRead(rotate_IR_L)<=threshold && analogRead(rotate_IR_L)<=threshold){
  while(analogRead(rotate_IR_L)<=threshold && analogRead(rotate_IR_L)<=threshold){
    speed=150;correction=0;
  }}else {
    while(analogRead(rotate_IR_L)>=threshold && analogRead(rotate_IR_L)>=threshold){
      line_follow();
      ticker1.update();
    }}

    while(analogRead(rotate_IR_L)<=threshold && analogRead(rotate_IR_L)<=threshold ){
    line_follow();
      ticker1.update();
  }
  
  
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


void rotate_ninety(int d) {
  Serial.println(1111);
  while (rotate_ir>=threshold){
    line_follow();
    ticker1.update();
  }

  float ini_angle = encoders.robotAngle();

  while (abs(encoders.robotAngle()-ini_angle) <=90){
     correction = d;
    ticker1.update();
  }
  correction = 0;
  waitMillis(500);

}

void avoid_obstacal(int r){
   speed =0; correction=0;
  waitMillis(500);

  go_back();
  //waitMillis(500);

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

  //speed =150; correction=0;
  //waitMillis(1450);
  /*speed =0; correction=0;
  waitMillis(500);

  //waitMillis(500);
  rotate_ninety(r);*/
}

void rotate_oneeighty(){
float ini_angle = encoders.robotAngle();

  while (abs(encoders.robotAngle()-ini_angle) <=90){
     correction = 1;
    ticker1.update();
  }
  correction = 0;
  waitMillis(500);
}

void drop_object(int r, int c) {

  int c1 = c; int r1=r;

  int rotation = 0;

  if (r%2==0) rotation =1;
  else rotation =-1;
 // int row =1;

  //while (row<=8){
   //column = c;

    while (c < 9){
    line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
    ticker1.update();
  }
    c++;

  }
  //Serial.println(column);
  ticker1.update();
 
}

 
 go_to_end();
 speed=0;correction=0;
 waitMillis(500);
 rotate_ninety(rotation);
 speed=0;correction=0;
 waitMillis(500);
 while (r < 9){
    line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
    ticker1.update();
  }
    r++;
    
 }
    ticker1.update();}
 go_to_end();
 speed=0;correction=0;
 waitMillis(500);
 rotate_ninety(rotation);
 speed=0;correction=0;
 waitMillis(500);

 c=1;
 while (c < 9){
    line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
    ticker1.update();
  }
  c++;
  if (c=5){
    speed=0;correction=0;
    waitMillis(500);
      rotate_oneeighty();
      speed=0;correction=0;
      waitMillis(500);
      boxdrop();
      speed=0;correction=0;
      waitMillis(500);
      rotate_oneeighty();
      speed=0;correction=0;
      waitMillis(500);
    }
    
 }ticker1.update();}

 //*********************************************************************put object

go_to_end();

speed=0;correction=0;
 waitMillis(500);
 rotate_ninety(rotation);
 speed=0;correction=0;
 waitMillis(500);

 r=1;c=1;

 while (r <9-r1){
    line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
    ticker1.update();
  }
    r++;
 }ticker1.update();}
 go_to_end();
 speed=0;correction=0;
 waitMillis(500);
 rotate_ninety(rotation);
 speed=0;correction=0;
 waitMillis(500);

 while (c < c1){
    line_follow();
    if (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold){
    while (analogRead(IR_L2)>= threshold && analogRead(IR_L1)>= threshold && analogRead(IR_R2)>= threshold &&
  analogRead(IR_R1)>= threshold && analogRead(IR_M)>= threshold) {
    line_follow();
    ticker1.update();
  }
    c++;
 }ticker1.update();}
//00 go_to_end();
 speed=0;correction=0;
 waitMillis(50000);

 }


unsigned long readFrequency(bool fs2, bool fs3);
//char getDominantColor();

void task_2(){
  Serial.println("task2");

}
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
  upper.write(75);
  base.write(3);
    left_arm.write(0);  
  right_arm.write(90);
  
  pinMode(trigger_f, OUTPUT);
  pinMode(echo_f, INPUT);

  pinMode(trigger_l, OUTPUT);
  pinMode(echo_l, INPUT);

  pinMode(trigger_r, OUTPUT);
  pinMode(echo_r, INPUT);


  
  motors.enable_controllers();
  
  //task_1();

lcd.init();
lcd.backlight();

pinMode(14, INPUT_PULLUP); // joystick button



lcd.clear();
lcd.setCursor(0, 0);
lcd.print("> Task 1");
lcd.setCursor(0, 1);
lcd.print("  Task 2");


  
}


void wall_following(){
  Serial.print("d");
  //uint16_t dist = sensor.readRangeSingleMillimeters();

  digitalWrite(trigger_r, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger_r, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_r, LOW);

  long duration = pulseIn(echo_r, HIGH, 20000);
  
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
  //sensors.color();
  //waitMillis(40000);

  



}
int detect(){

 

  
  waitMillis(2000);
  sensors.color();

  return color_value;
  //waitMillis(40000);

}

 void object(int r, int c){
     
  //uint16_t dist = sensor.readRangeSingleMillimeters();

  line_follow();

  digitalWrite(trigger_f, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger_f, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger_f, LOW);

  long duration = pulseIn(echo_f, HIGH, 20000);
  Serial.println(duration);
  
  if (duration != 0) distance =duration * 0.034 / 2;// 999;
  Serial.println(distance);
  //else distance = duration * 0.034 / 2;
  if (distance<=7.5){
    correction = 0;
    speed = 0;
    left_arm.write(0);
    right_arm.write(90);
    base.write(3);



    moveSmooth(elbow, 180, 115,10);

    if (detect()==0){
      moveSmooth(elbow, 115, 180,10);
      avoid_obstacal(-r);
      column +=2;


    }
    else{
      pick_object();
      drop_object(r,c);

    }
    

    

  }
  }

void drawMenuOptimized(String items[], int size, int index) {
  lcd.clear();

  // Print current item
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.print(items[index]);

  // Print next item only if it exists
  lcd.setCursor(0, 1);
  if (index + 1 < size) {
    lcd.print("  ");
    lcd.print(items[index + 1]);
  } else {
    lcd.print("  "); // clear second line
  }
}

void handleJoystick(int &index, int size) {
  int y = analogRead(A8); // horizontal
  int x = analogRead(A9); // vertical

  if (millis() - lastJoy < joyDelay) return;

  // Navigate menu vertically (swapped axes)
  if (x < 300 && index > 0) { // joystick left → up
    index--;
    lastJoy = millis();
  }
  if (x > 700 && index < size - 1) { // joystick right → down
    index++;
    lastJoy = millis();
  }
}

bool isSelectPressed() {
  return digitalRead(14) == LOW;
}

void next_row(int d){
  float ini_distance1 = encoders.robotDistance();
 /* while (encoders.robotDistance()- ini_distance1 <= -1505) {
    Serial.println(encoders.robotDistance()- ini_distance1);
    line_follow();
    ticker1.update();
  }*/
/*Serial.println(encoders.robotDistance()- ini_distance1);
 speed=100;correction=0;*/
 /*speed=150;correction=0;
 waitMillis(500);
  speed=0;correction=0;
  waitMillis(600);*/
  go_to_end();
  speed=0;correction=0;
  waitMillis(500);
  rotate_ninety(d);
  waitMillis(500);

  /*float ini_distance2 = encoders.robotDistance();
  while (encoders.robotDistance()-ini_distance2 <= -1505) {
    Serial.println(111);
    line_follow();
    ticker1.update();
  }
  speed=150;correction=0;
 waitMillis(1500);
  speed=0;correction=0;
  waitMillis(500);*/

  go_to_end();
  speed=0;correction=0;
  waitMillis(500);
  rotate_ninety(d);
  waitMillis(500);
}  

void task_1(){
  
  int row =1;

  while (row<=8){
   column = 1;

    while (column < 9){
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

  object(row,column);
    
 }
 
 if (row%2 == 0) next_row(1);
 else next_row(-1);
row++;
Serial.println(row);
}
}


void updateMenus() {

  switch (currentMenu) {

    // ---------------- MAIN MENU ----------------
    case MAIN_MENU:
      handleJoystick(mainIndex, 2);

      if (mainIndex != lastMainIndex || currentMenu != lastMenu) {
        drawMenuOptimized(mainMenuItems, 2, mainIndex);
        lastMainIndex = mainIndex;
      }

      if (isSelectPressed()) {
        currentMenu = (mainIndex == 0) ? TASK_MENU : INDIVIDUAL_MENU;
        waitMillis(100);
      }
      break;

    // ---------------- TASK MENU ----------------
    case TASK_MENU:
      handleJoystick(taskIndex, 7);

      if (taskIndex != lastTaskIndex || currentMenu != lastMenu) {
        drawMenuOptimized(taskMenuItems, 7, taskIndex);
        lastTaskIndex = taskIndex;
      }

      if (isSelectPressed()) {
        if (taskIndex == 0) task_1();
        // add other tasks if needed
        waitMillis(100);
      }

      // GO BACK when joystick pushed left at first item
      if (analogRead(A9) < 100) { // left side
        currentMenu = MAIN_MENU;
        waitMillis(100);
      }
      break;

    // ---------------- INDIVIDUAL MENU ----------------
    case INDIVIDUAL_MENU:
      handleJoystick(indivIndex, 5);

      if (indivIndex != lastIndivIndex || currentMenu != lastMenu) {
        drawMenuOptimized(indivMenuItems, 5, indivIndex);
        lastIndivIndex = indivIndex;
      }

      if (isSelectPressed()) {
        if (indivIndex == 0) ballpickup();
        if (indivIndex == 1) boxpickup();
        if (indivIndex == 2) boxdrop();
        if (indivIndex == 3) line_follow();
        if (indivIndex == 4) wall_following();
        waitMillis(100);
      }

      // GO BACK when joystick pushed left at first item
      if (analogRead(A9) < 100) { // left side
        currentMenu = MAIN_MENU;
        waitMillis(100);
      }
      break;
  }

  lastMenu = currentMenu;
}




void loop() {

    // Always update ticker
 // ticker1.update();
  //updateMenus();



    // Always run line follow
    //line_follow();

    //boxpickup();
    //drop_object(7,5);
   task_1();
   //go_back();
   //Serial.println(analogRead(rotate_IR_L));
    //ll_following();
  //sensors.color();
  //delay(500);
  //ballpickup();
 // object();
  //feedforwardPWM(1);
  //feedforwardPWM(2);
  //Serial.println(encoders.leftRPS());
  //Serial.println(encoders.leftRPS());


    // -------- BLOCKING SERVO ARM CONTROL ------------
    //  left_arm.write(90);
    //  waitMillis(1500);
   // left_arm.write(0);
    //right_arm.write(90);
    //  waitMillis(1500);
    //Serial.println(analogRead(rotate_IR_L));
    //Serial.println(analogRead(rotate_IR_R));
   //moveSmooth(elbow,180 , 115, 10);
    //ft_arm.write(0);
    //ght_arm.write(90);
    //waitMillis(2000);
    //sensors.color();

   waitMillis(2000);
   //moveSmooth(elbow,115 , 180, 10);
 }
