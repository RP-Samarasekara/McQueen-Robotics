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
void detectAndDisplay();
#include "task_1.h"




LiquidCrystal_I2C lcd(0x27, 16, 2);

enum MenuLevel { MAIN_MENU, TASK_MENU, INDIVIDUAL_MENU };
MenuLevel currentMenu = MAIN_MENU;

int mainIndex = 0;
int taskIndex = 0;
int indivIndex = 0;

unsigned long lastJoy = 0;
int joyDelay = 200;

const char* mainMenuItems[2] = {"Tasks", "Individual"};

const char* taskMenuItems[6] = {
  "Task 1",
  "Task 2",
  "Task 3",
  "Task 4",
  "Task 5",
  "Task 6"
};

const char* indivMenuItems[6] = {
  "Line follow",
  "Wall follow",
  "Ball pickup",
  "Box pickup",
  "Box drop",
  "Detect()"
};


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

void drawMenuOptimized(const char* items[], int size, int index) {
  lcd.clear();

  // current cursor
  lcd.setCursor(0, 0);
  lcd.print("> ");
  lcd.print(items[index]);

  // next cursor
  lcd.setCursor(0, 1);
  if (index + 1 < size) {
    lcd.print("  ");
    lcd.print(items[index + 1]);
  } else {
    lcd.print("  ");
  }
}

void handleJoystick(int &index, int size) {
  int x = analogRead(A9);  // vertical movement

  if (millis() - lastJoy < joyDelay) return;

  if (x < 300 && index > 0) {  // UP
    index--;
    lastJoy = millis();
  }
  if (x > 700 && index < size - 1) { // DOWN
    index++;
    lastJoy = millis();
  }
}


bool isSelectPressed() {
  return digitalRead(14) == LOW;
}




void task_3(){
while (!sensors.get_distance3()) {
      basics.line_follow();
      ticker1.update();
    }

    speed=0;correction=0;ticker1.update(),waitMillis(500);
    
task_1.rotate_ninety(1);
speed=0;correction=0;waitMillis(500);
speed=100;correction=0;waitMillis(300);
speed=0;correction=0;waitMillis(500);
//if(sensors.get_l_distance()){
//     while(speed!=0||correction!=0){
//       basics.wall_following2();
//       ticker1.update();
//     }//}
//     speed=120;correction=0;ticker1.update();waitMillis(1000);
//     speed=0;correction=0;ticker1.update(),waitMillis(500);
//     task_1.rotate_ninety(-1);
//     speed=0;correction=0;ticker1.update(),waitMillis(500);
//     speed=120;correction=0;ticker1.update();waitMillis(1000);
//     speed=0;correction=0;ticker1.update(),waitMillis(500);

//     //pic ball
// speed=-120;correction=0;ticker1.update();waitMillis(1000);
//     speed=0;correction=0;ticker1.update(),waitMillis(500);
//     task_1.rotate_ninety(1);
//     speed=120;correction=0;ticker1.update();waitMillis(1000);
//     speed=0;correction=0;ticker1.update(),waitMillis(500);

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
void task_4(){
  Serial.println(2);
}
int black_space_count = 0;
bool in_black_space = false;

void rotate_angle(int d, int angle){
  float ini_angle = encoders.robotAngle();

  while (abs(encoders.robotAngle()-ini_angle) <=angle){
     correction = d;
    ticker1.update();
  }
  correction = 0;
  waitMillis(500);
}
void task_5() {


  while (
    analogRead(IR_L2) <= threshold ||
    analogRead(IR_L1) <= threshold ||
    analogRead(IR_M)  <= threshold ||
    analogRead(IR_R1) <= threshold ||
    analogRead(IR_R2) <= threshold
  ) {
     bool on_black = (
    analogRead(IR_L2) <= threshold &&
    analogRead(IR_L1) <= threshold &&
    analogRead(IR_M)  <= threshold &&
    analogRead(IR_R1) <= threshold &&
    analogRead(IR_R2) <= threshold
);

// EDGE DETECTION


  if (!on_black) {
    in_black_space = false; // reset when leaving black
}
    if (on_black && !in_black_space) {
    black_space_count++;
    in_black_space = true;

    Serial.print("Black space count = ");
    Serial.println(black_space_count);

    if (black_space_count == 2) {
      waitMillis(800);
    
     rotate_angle(1, 100);   // or -1
    }

    else if (black_space_count == 4) {
      
      while (analogRead(rotate_IR_L) <= threshold_2) {
          speed = 100;
          correction = 0;
          ticker1.update();
        
    }
    
    waitMillis(800);
    rotate_angle(-1,120);}

    else if (black_space_count == 7) {

      while (analogRead(rotate_IR_L) <= threshold_2) {
          speed = 100;
          correction = 0;
          ticker1.update();
        
    }
    waitMillis(800);
    rotate_angle(1,60);
        // or 1
    }
    else if (black_space_count == 8) {

      while (analogRead(rotate_IR_L) <= threshold_2) {
          speed = 100;
          correction = 0;
          ticker1.update();
        
    }
    waitMillis(800);
    rotate_angle(1,80);}
    else if (black_space_count == 11){
      waitMillis(2000);
      speed=0;
      correction=0;

    }
}
    /*
    // ROTATE IR FIRST — highest priority

    if (black_space_count ==2){

      task_1.rotate_ninety(1);

    }
    else if (black_space_count ==4){

      task_1.rotate_ninety(-1);

    }*/
    if (analogRead(rotate_IR_L) <= threshold_2  ||
        analogRead(rotate_IR_R) <= threshold_2) {
          int corre;
          if (analogRead(rotate_IR_L) <= threshold_2) corre =-1;
          else corre = 1;
          speed = -100;correction=0;ticker1.update();waitMillis(800);
      // STOP immediately
      speed = 0;
      correction = 0;
      //waitMillis(500);
      ticker1.update();

      Serial.println("ROTATE IR TRIGGERED");
      waitMillis(500);   // pause 5 seconds

      speed=0;correction=corre;ticker1.update();waitMillis(600);
      // while(analogRead(IR_M)<=threshold){
      //   speed=0;correction=corre;
      //   ticker1.update();

      // }
    //    if (analogRead( IR_L1) >= threshold 
    //     analogRead(IR_R1) >= threshold) {
    //       int corre;
    //       if (analogRead(IR_L1) >= threshold) corre =-1;
    //       else corre = 1;

    //   // STOP immediately
    //   speed = 0;
    //   correction = 0;
    //   //waitMillis(500);
    //   ticker1.update();
    //   waitMillis(500);

    //   //Serial.println("ROTATE IR TRIGGERED");
    //   //waitMillis(500);   // pause 5 seconds
    //   while(analogRead(IR_M)<=threshold){
    //     speed=0;correction=corre;
    //     ticker1.update();

    //   }

    //   speed = 0;
    //   correction = 0;
    //   //waitMillis(500);
    //   ticker1.update();

    //  // Serial.println("ROTATE IR TRIGGERED");
    //   waitMillis(500); }

      ///speed = 0;
      //correction = 0;
      //waitMillis(500);
      ticker1.update();

     // Serial.println("ROTATE IR TRIGGERED");
      waitMillis(500);   // pause 5 seconds
      while((analogRead(IR_L2) >= threshold ||
    analogRead(IR_L1) >= threshold ||
    analogRead(IR_M)  >= threshold ||
    analogRead(IR_R1) >= threshold ||
    analogRead(IR_R2) >= threshold) && black_space_count!=4){
      basics.line_follow();
      ticker1.update();}
// after waiting, continue loop
      continue;
    }
    // if (black_space_count == 4){
    //   while (analogRead(rotate_IR_L) <= threshold_2 ||
    //     analogRead(rotate_IR_R) <= threshold_2){

    //       speed=

    //     }
    

    // otherwise follow line
    basics.line_follow();
   //speed=100;correction=0;
    ticker1.update();
  }

  //speed = 0;
  //correction = 0;
}
void task_6(){
  Serial.print("Hidden)");

}
void detectAndDisplay() {
  //int result = task_1.detect();  // call your original detect()

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Detected:");

  lcd.setCursor(0, 1);

  switch (color_value) {
    case 1: lcd.print("RED"); break;
    case 2: lcd.print("BLUE"); break;
    case 3: lcd.print("GREEN"); break;
    default: lcd.print("OBSTACLE"); break;
  }

  waitMillis(4000);   
  lcd.clear();


}

void updateMenus() {

  switch (currentMenu) {

    // -------- MAIN MENU --------
    case MAIN_MENU:
      handleJoystick(mainIndex, 2);

      if (mainIndex != lastMainIndex || currentMenu != lastMenu) {
        drawMenuOptimized(mainMenuItems, 2, mainIndex);
        lastMainIndex = mainIndex;
      }

      if (isSelectPressed()) {
        currentMenu = (mainIndex == 0) ? TASK_MENU : INDIVIDUAL_MENU;
        waitMillis(150);
      }
      break;



    // -------- TASK MENU --------
    case TASK_MENU:
      handleJoystick(taskIndex, 6);

      if (taskIndex != lastTaskIndex || currentMenu != lastMenu) {
        drawMenuOptimized(taskMenuItems, 6, taskIndex);
        lastTaskIndex = taskIndex;
      }

      if (isSelectPressed()) {
        if (taskIndex == 0) task_1.task_1();
        if (taskIndex == 1) task_2();
        if (taskIndex == 2) task_3();
        if (taskIndex == 3) task_4();
        if (taskIndex == 4) task_5();
        if (taskIndex == 5) task_6();

        waitMillis(150);
      }

      // Back to main menu
      if (analogRead(A9) < 100 && taskIndex == 0) {
        currentMenu = MAIN_MENU;
        waitMillis(150);
      }
      break;



    // -------- INDIVIDUAL MENU --------
    case INDIVIDUAL_MENU:
      handleJoystick(indivIndex, 6);

      if (indivIndex != lastIndivIndex || currentMenu != lastMenu) {
        drawMenuOptimized(indivMenuItems, 6, indivIndex);
        lastIndivIndex = indivIndex;
      }

      if (isSelectPressed()) {
        if (indivIndex == 0) basics.line_follow();
        if (indivIndex == 1) basics.wall_following();
        if (indivIndex == 2) basics.ballpickup();
        if (indivIndex == 3) basics.boxpickup();
        if (indivIndex == 4) basics.boxdrop();
        if (indivIndex == 5) task_1.detect();

        waitMillis(150);
      }

      // Back to main menu
      if (analogRead(A9) < 100 && indivIndex == 0) {
        currentMenu = MAIN_MENU;
        waitMillis(150);
      }
      break;
  }

  lastMenu = currentMenu;
}

void loop() {
  ticker1.update();
  updateMenus();
 //task_1.task_1();
//task_3();
//task_2();
 }
