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
#include "basics.h"
void task_2();  
#include "task_1.h"




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
Basics basics;
Task_1 task_1;


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

///int column = 1;

unsigned long readFrequency(bool fs2, bool fs3);
//char getDominantColor();

// void task_2(){
//   Serial.println("task2");
//   while(analogRead(IR_L2)<=threshold|| analogRead(IR_L1)<=threshold||analogRead(IR_M)<=threshold||analogRead(IR_R1)<=threshold||analogRead(IR_R2)<=threshold){
    
//     //if (analogRead(IR_L2<threshold)|| analogRead(IR_L1<threshold)||analogRead(IR_M<threshold)||analogRead(IR_R1<threshold)||analogRead(IR_R2<threshold)){
// basics.line_follow();
//     ticker1.update();
//     //}
    
//    if (analogRead(rotate_IR_L)<=threshold_2 || analogRead(rotate_IR_R)<=threshold_2){
//     speed =0; correction=0;
//     waitMillis(5000);
// Serial.println(22222222);
//    }


    
//   }
 
//   speed = 0;correction=0;

// }
// int black_space_count = 0;

void go_to_end(){
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


void dash_follow(int d){
 while (
    !((analogRead(IR_L2) >= threshold &&
    analogRead(IR_L1) >= threshold &&
    analogRead(IR_M)  >= threshold &&
    analogRead(IR_R1) >= threshold) ||
    (analogRead(IR_L1) >= threshold &&
    analogRead(IR_M)  >= threshold &&
    analogRead(IR_R1) >= threshold && analogRead(IR_R2) >= threshold))
  ) {
    
    //black_space_count++;
    // ROTATE IR FIRST — highest priority
    if (analogRead(rotate_IR_L) <= threshold_2 ||
        analogRead(rotate_IR_R) <= threshold_2) {
          int corre;
          if (analogRead(rotate_IR_L) <= threshold_2) corre =-1;
          else corre = 1;
speed = -100;correction=0;ticker1.update();waitMillis(700);
      // STOP immediately
      speed = 0;
      correction = 0;
      //waitMillis(500);
      ticker1.update();

      Serial.println("ROTATE IR TRIGGERED");
      waitMillis(500);   // pause 5 seconds

      speed=0;correction=corre;ticker1.update();waitMillis(d);

      speed = 0;
      correction = 0;
      //waitMillis(500);
      ticker1.update();

     // Serial.println("ROTATE IR TRIGGERED");
      waitMillis(500);   // pause 5 seconds
      while(analogRead(IR_L2) >= threshold ||
    analogRead(IR_L1) >= threshold ||
    analogRead(IR_M)  >= threshold ||
    analogRead(IR_R1) >= threshold ||
    analogRead(IR_R2) >= threshold){
      basics.line_follow();
      ticker1.update();
    }

      // after waiting, continue loop
      continue;
    }

    // otherwise follow line
  basics.line_follow();
   //speed=100;correction=0;
    ticker1.update();
  }

//  correction=0;speed=0;ticker1.update();waitMillis(500);
//   correction=1;speed=0;ticker1.update();waitMillis(200);
//   correction=0;speed=0;ticker1.update();waitMillis(500);

//   if(    analogRead(IR_L2) >= threshold ||
//     analogRead(IR_L1) >= threshold ||
//     analogRead(IR_M)  >= threshold ||
//     analogRead(IR_R1) >= threshold ||
//     analogRead(IR_R2) >= threshold){
//       waitMillis(500);
//     }
//     else{
//       correction=-1;speed=0;ticker1.update();waitMillis(600);
//   correction=0;speed=0;ticker1.update();waitMillis(500);
//     }
}


void dash_follow2(int d){
 while (!sensors.get_distance())  {
    
    //black_space_count++;
    // ROTATE IR FIRST — highest priority
    if (analogRead(rotate_IR_L) <= threshold_2 ||
        analogRead(rotate_IR_R) <= threshold_2) {
          int corre;
          if (analogRead(rotate_IR_L) <= threshold_2) corre =-1;
          else corre = 1;
speed = -100;correction=0;ticker1.update();waitMillis(700);
      // STOP immediately
      speed = 0;
      correction = 0;
      //waitMillis(500);
      ticker1.update();

      Serial.println("ROTATE IR TRIGGERED");
      waitMillis(500);   // pause 5 seconds

      speed=0;correction=corre;ticker1.update();waitMillis(d);

      speed = 0;
      correction = 0;
      //waitMillis(500);
      ticker1.update();

     // Serial.println("ROTATE IR TRIGGERED");
      waitMillis(500);   // pause 5 seconds
      while(analogRead(IR_L2) >= threshold ||
    analogRead(IR_L1) >= threshold ||
    analogRead(IR_M)  >= threshold ||
    analogRead(IR_R1) >= threshold ||
    analogRead(IR_R2) >= threshold){
      basics.line_follow();
      ticker1.update();
    }

      // after waiting, continue loop
      continue;
    }

    // otherwise follow line
  basics.line_follow();
   //speed=100;correction=0;
    ticker1.update();
  }


}











void task_2() {
//   Serial.println("task2");

basics.task_2_arm();
dash_follow2(800);

 

  speed = 0;
  correction = 0;
  waitMillis(500);

  go_to_end();
  speed = 0;
  correction = 0;
  waitMillis(500);

   

  task_1.rotate_oneeighty();
  speed = 0;
  correction = 0;
  waitMillis(500);

  speed = -100;
  correction = 0;
  waitMillis(4000);
speed = 0;
  correction = 0;
  waitMillis(500);

// speed=0;correction=-1;ticker1.update();waitMillis(100);
// speed = 0;
//   correction = 0;
//   waitMillis(500);


//   //task_1.go_back();
//   speed=100;correction=0;ticker1.update();waitMillis(2000);
//   speed = 0;
//   correction = 0;
//   waitMillis(500);

//   speed = 100;
//   correction = 0;
//   waitMillis(500);
//   speed = 0;
//   correction = 0;
//   waitMillis(500);

//   dash_follow(1100);
//   speed = 0;
//   correction = 0;
//   waitMillis(500);

//   go_to_end();
//   speed = 0;
//   correction = 0;
//   waitMillis(500);

//   speed=-100;correction=0;
//   waitMillis(3000);

//   speed = 0;
//   correction = 0;
//   waitMillis(500);

//   task_1.rotate_ninety(-1);
//   speed = 0;
//   correction = 0;
//   waitMillis(500);

//   speed=-100;correction=0,ticker1.update();waitMillis(2000);

//    speed = 0;
//   correction = 0;
//   waitMillis(500);
speed=100;correction=0,ticker1.update();waitMillis(1000);
   speed = 0;
  correction = 0;
   waitMillis(500);


task_1.rotate_ninety(1);
speed = 0;
  correction = 0;
   waitMillis(500);
   speed=100;correction=0,ticker1.update();waitMillis(4500);
   speed = 0;
  correction = 0;
   waitMillis(500);

   task_1.rotate_ninety(1);
   speed = 0;
  correction = 0;
   waitMillis(500);

   speed=100;correction=0,ticker1.update();waitMillis(3000);
   speed = 0;
  correction = 0;
   waitMillis(500);

   task_1.rotate_ninety(-1);
   speed = 0;
  correction = 0;

   basics.ramp_arm();
   waitMillis(500);

  while (analogRead(rotate_IR_L)>=threshold_2 && analogRead(rotate_IR_R)>=threshold_2)
  {
    speed=-300;correction=0;ticker1.update();
  }
speed = 0;
  correction = 0;
  ticker1.update();
  waitMillis(500);
  while(analogRead(IR_M)>=0){
    speed=0;correction=1;
  }

  speed=0;correction=0;ticker1.update();waitMillis(500);
  while (analogRead(IR_L2) <= threshold ||
    analogRead(IR_L1) <= threshold ||
    analogRead(IR_M)  <= threshold ||
    analogRead(IR_R1) <= threshold ||
    analogRead(IR_R2) <= threshold)
  {
    basics.line_follow();
  }
  
  

}

void task_21() {
  Serial.println("task2");

  unsigned long lineLostTime = 0;
  bool linePresent = true;

  while (true) {

    bool L2 = analogRead(IR_L2) >= threshold;
    bool L1 = analogRead(IR_L1) >= threshold;
    bool M  = analogRead(IR_M)  >= threshold;
    bool R1 = analogRead(IR_R1) >= threshold;
    bool R2 = analogRead(IR_R2) >= threshold;

    // if ANY sensor sees line, line is present
    if (L2 || L1 || M || R1 || R2) {
      linePresent = true;
      lineLostTime = millis();   // reset timer
    } 
    else {
      linePresent = false;
    }

    // if line missing for 80ms → exit loop
    if (!linePresent && (millis() - lineLostTime > 1000)) break;

    // follow line
    basics.line_follow();
    ticker1.update();


    // -------------------------------------------
    // ROTATE IR detection
    // -------------------------------------------
    if (analogRead(rotate_IR_L) <= threshold_2 ||
        analogRead(rotate_IR_R) <= threshold_2) {

      Serial.println(analogRead(rotate_IR_L));
      Serial.println(analogRead(rotate_IR_R));

      int corre = 0;

      if (analogRead(rotate_IR_L) <= threshold_2)
        corre = 1;     // rotate right
      else 
        corre = -1;    // rotate left

      speed = 0; correction = 0;
      waitMillis(300);

      speed = 0; correction = -corre;
      waitMillis(900);

      speed = 0; correction = 0;
      waitMillis(300);

      speed = -100; correction = 0;
      waitMillis(400);

      speed = 0; correction = 0;
      waitMillis(300);
    }

  } // end while

  speed = 0;
  correction = 0;
}




void setup() {
  motors.begin();
  encoders.begin();
  encoders.reset();
  sensors.begin();
  ticker1.start();
  basics.begin();
 // ticker1.start();
  Serial.begin(9600);

  Wire.begin(); 
  
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
        if (taskIndex == 0) task_1.task_1();
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
        if (indivIndex == 0) basics.ballpickup();
        if (indivIndex == 1) basics.boxpickup();
        if (indivIndex == 2) basics.boxdrop();
        if (indivIndex == 3) basics.line_follow();
        if (indivIndex == 4) basics.wall_following();
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

void task_3(){
while (/*(analogRead(IR_L2) >= threshold ||
    analogRead(IR_L1) >= threshold ||
    analogRead(IR_M)  >= threshold ||
    analogRead(IR_R1) >= threshold || analogRead(IR_R1) >= threshold)&&!*/!sensors.get_distance3()) {
      basics.line_follow();
      ticker1.update();
    }

    speed=0;correction=0;ticker1.update(),waitMillis(500);
    // if(analogRead(IR_L2) >= threshold ||
    // analogRead(IR_L1) >= threshold ||
    // analogRead(IR_M)  >= threshold ||
    // analogRead(IR_R1) >= threshold || analogRead(IR_R1) >= threshold) {

    // }else{
task_1.rotate_ninety(1);
speed=0;correction=0;waitMillis(500);
if(!sensors.get_l_distance()){
    while(!sensors.get_l_distance()){
      basics.wall_following();
      ticker1.update();
    }}
    speed=120;correction=0;ticker1.update();waitMillis(1000);
    speed=0;correction=0;ticker1.update(),waitMillis(500);
    task_1.rotate_ninety(-1);
    speed=0;correction=0;ticker1.update(),waitMillis(500);
    speed=120;correction=0;ticker1.update();waitMillis(1000);
    speed=0;correction=0;ticker1.update(),waitMillis(500);

    //pic ball
speed=-120;correction=0;ticker1.update();waitMillis(1000);
    speed=0;correction=0;ticker1.update(),waitMillis(500);
    task_1.rotate_ninety(1);
    speed=120;correction=0;ticker1.update();waitMillis(1000);
    speed=0;correction=0;ticker1.update(),waitMillis(500);

    while ((analogRead(IR_L2) <= threshold &&
    analogRead(IR_L1) <= threshold &&
    analogRead(IR_M)  <= threshold &&
    analogRead(IR_R1) <= threshold && analogRead(IR_R1) <= threshold))
    {
      basics.wall_following();
      ticker1.update();
    }
    task_1.rotate_ninety(1);
    speed=0;correction=0;ticker1.update(),waitMillis(500);
    
    while(analogRead(IR_L2) <= threshold ||
    analogRead(IR_L1) <= threshold ||
    analogRead(IR_M)  <= threshold ||
    analogRead(IR_R1) <= threshold || analogRead(IR_R1) <= threshold){
      speed=120;correction=0;ticker1.update();

    }
    task_1.rotate_ninety(1);
    speed=0;correction=0;ticker1.update(),waitMillis(500);

    
    }


    
   // }


void loop() {

    // Always update ticker
  ticker1.update();
  //updateMenus();
 //task_1.task_1();
task_3();
//task_2();
//task_1.go_to_end();
    // Always run line follow
    //line_follow();

    //boxpickup();
    //drop_object(7,5);
//task_1.task_1();
   //go_back();
  //Serial.println(analogRead(rotate_IR_L));
    //basics.wall_following();
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
    //Serial.println(analogRead(rotate_IR_R));
   // Serial.println(analogRead(IR_R1));
   //moveSmooth(elbow,180 , 115, 10);
    //ft_arm.write(0);
    //ght_arm.write(90);
    //waitMillis(2000);
    //sensors.color();
    //basics.wall_following();

  //waitMillis(20000);
   //moveSmooth(elbow,115 , 180, 10);
 }
