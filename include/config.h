#pragma once

//*****************************encoder pin allocation***********************
const int LeftEncoderPin1 = 18 ;//15;
const int LeftEncoderPin2 = 19;//4;

const int RightEncoderPin1 = 2;//17;
const int RightEncoderPin2 = 3;//16; 

//****************************motor********************************************* */
const int LEFT_MOTOR_PWM = 9;
const int LEFT_MOTOR_IN1 = 39;
const int LEFT_MOTOR_IN2 = 38;

const int RIGHT_MOTOR_PWM = 8;
const int RIGHT_MOTOR_IN1 = 37;
const int RIGHT_MOTOR_IN2 = 36;

const int MAX_MOTOR_PERCENTAGE = 95;

const int MOTOR_RIGHT_POLARITY = 1;// change this to change direction of motor direction
const int MOTOR_LEFT_POLARITY = 1;

const int M_BALNCE_PWM = 0; //adjust this value to bring both the motors to same speed

const int LEFT_MIN_MOTOR_PERCENTAGE = 20;
const int RIGHT_MIN_MOTOR_PERCENTAGE = 13;
const int MIN_MOTOR_BIAS = 5;
const int maxMotorPercentage = 100;

//******************************robot dimensions*****************************/
const float MM_PER_ROTATION = 204.2;//tyre
const float PULSES_PER_ROTATION = 880;
const float ROBOT_RADIUS = 77.5;  //there is some error in this. Although it says radius put Wheel to wheel diameter
const float DEG_PER_MM_DIFFERENCE = 180.0/(2*ROBOT_RADIUS*PI);

const float ARRAY_TO_WHEEL_DISTANCE = 75.0;//65; //change this accordingly

const float RADIANS_PER_DEGREE = 0.0175;

const float ROT_KP = 3;//7;
const float ROT_KD = 2.5;//19;
const float ROT_KI = 0;
const float FWD_KP = 1.5;//2.5;//6.0;//7.0//1.0
const float FWD_KD = 2.2;//10;
const float FWD_KI = 0;
const float STR_KP = 2.5;//2.5, 100;
const float STR_KD = 0.1;

// IR pins
const int IR_L2 = A0;
const int IR_L1 = A1;
const int IR_M = A2;
const int IR_R1 = A3;
const int IR_R2 = A4;

const int rotate_IR_L = A11;
const int rotate_IR_R = A12;


const int threshold = 850;
 int last_nonzero_error = 0;
 int lastError = 0;
 float integral = 0;
 //int ir_error = 0; 
float ir_correction = 0;
const int Kp = 15;
const int Ki = 0;
const int Kd = 10;

float irCorrection = 0;


float speed = 100;
float correction = 0;

// wall following
const int trigger_f  = 52;
const int  echo_f =53; 

const int trigger_r  = 40; 
const int  echo_r =41; 

const int trigger_l  = 49;
const int  echo_l =50;



float distance =999;

float element = 0;
float wall_error = 0;

//robot movemets//
//const int TICKS_90 = static_cast<int>(( (ROBOT_RADIUS* 2 * 3.1416 / 2.0) / MM_PER_ROTATION) * PULSES_PER_ROTATION);

// rotation
float rotate_ir = A4;

const int s0 = 44;
const int s1 = 45;
const int s2 = 47;
const int s3 = 46;

const int color_out = 48;


//joystic

//vr_x =A9;
//vr_y = A8
//switch =14

