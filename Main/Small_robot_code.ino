/**
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         omniduino_move_all_directions.ino
* @brief        Robot moves in all directions with delays
* @details      Modified version to demonstrate all movement directions
*/

//Import library file
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_NeoPixel.h>
#include <I2Cdev.h>
// #include <MPU6050_6Axis_MotionApps_V6_12.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define BUZZER 10        //Define buzzer pins
#define KEY_PIN 8        //Define button pins
#define INTERRUPT_PIN 2  //Define MPU6050 pins
#define LED_PIN 5        //Define status indicator pins
#define RGB_PIN 9        //Define RGB pins
#define MAX_LED 4        //Car with 4 RGB lights

#define IR_SENSOR_L1 A3
#define IR_SENSOR_L2 A0
#define IR_SENSOR_R1 A2
#define IR_SENSOR_R2 A1
#define IR_SENSOR_MID A7

const char wheel[4][2] = {{10, 11}, {13, 12}, {15, 14}, {8, 9}};
int CarSpeedControl = 40;  // Default speed (0-160)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(KEY_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(IR_SENSOR_L1, INPUT);
  pinMode(IR_SENSOR_L2, INPUT);
  pinMode(IR_SENSOR_R1, INPUT);
  pinMode(IR_SENSOR_R2, INPUT);
  pinMode(IR_SENSOR_MID, INPUT);

  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); // Analog servos run at ~60 Hz updates
  brake();            // Initialize with motors stopped
  
  digitalWrite(LED_PIN, HIGH); // Turn off LED initially
}

/**
* Movement functions
*/
void run(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed); //Right rear wheel Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);  //Right front wheel Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed); //Left rear wheel Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed); //Left front wheel Forward
  pwm.setPWM(14, 0, 0);
}

void brake() {
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, 0);
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(10, 0, 0);

  pwm.setPWM(12, 0, 0);
  pwm.setPWM(13, 0, 0);
  pwm.setPWM(14, 0, 0);
  pwm.setPWM(15, 0, 0);
}

void back(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed); //Right rear wheel Reverse
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);  //Right front wheel Reverse

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed); //Left rear wheel Reverse
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed); //Left front wheel Reverse
}

void left(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed); //Right rear wheel(B type) Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);  //Right front wheel(A type) Reverse

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed); //Left rear wheel (A type) Reverse
  pwm.setPWM(15, 0, Speed); //Left front wheel(B type) Forward
  pwm.setPWM(14, 0, 0);
}

void right(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed); //Right front wheel(B type) Reverse
  pwm.setPWM(8, 0, Speed);  //Right rear wheel(A type) Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed); //Left front wheel(A type) Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed); //Left rear wheel(B type) Reverse
}

void spin_left(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed); //Right front wheel Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);  //Right rear wheel Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed); //Left front wheel Reserve
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed); //Left rear wheel Reserve
}

void spin_right(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed); //Right front wheel Reserve
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);  //Right rear wheel Reserve

  pwm.setPWM(13, 0, Speed); //Left front wheel Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed); //Left rear wheel Forward
  pwm.setPWM(14, 0, 0);
}

void front_left(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, Speed); //Right front wheel(B type) Forward
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0);      //Right rear wheel(A type) stop
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, 0);     //Left front wheel(A type) stop
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, Speed); //Left rear wheel(B type) Forward
  pwm.setPWM(14, 0, 0);
}

void front_right(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);     //Right front wheel(B type) stop
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, Speed);  //Right rear wheel(A type) Forward
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, Speed); //Left front wheel(A type) Forward
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, 0);     //Left rear wheel(B type) stop
  pwm.setPWM(14, 0, 0);
}

void left_rear(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);     //Right front wheel(B type) stop
  pwm.setPWM(11, 0, 0);
  pwm.setPWM(8, 0, 0);
  pwm.setPWM(9, 0, Speed);  //Right rear wheel(A type) Reserve

  pwm.setPWM(13, 0, 0);
  pwm.setPWM(12, 0, Speed); //Left front wheel(A type) Reserve
  pwm.setPWM(15, 0, 0);     //Left rear wheel(B type) stop
  pwm.setPWM(14, 0, 0);
}

void right_rear(int Speed) {
  Speed = map(Speed, 0, 160, 0, 2560);
  pwm.setPWM(10, 0, 0);
  pwm.setPWM(11, 0, Speed); //Right front wheel(B type) Reserve
  pwm.setPWM(8, 0, 0);      //Right rear wheel(A type) stop
  pwm.setPWM(9, 0, 0);

  pwm.setPWM(13, 0, 0);     //Left front wheel (A type) stop
  pwm.setPWM(12, 0, 0);
  pwm.setPWM(15, 0, 0);
  pwm.setPWM(14, 0, Speed); //Left rear wheel(B type) Reserve
}

void loop() {
  // Movement sequence with delays
  digitalWrite(LED_PIN, LOW); // Turn on LED during movement
  
  run(CarSpeedControl);       // Move forward
  delay(2000);                // 2 seconds
  brake();
  delay(500);                 // 0.5 second pause
  
  back(CarSpeedControl);      // Move backward
  delay(2000);
  brake();
  delay(500);
  
  left(CarSpeedControl);      // Move left
  delay(2000);
  brake();
  delay(500);
  
  right(CarSpeedControl);     // Move right
  delay(2000);
  brake();
  delay(500);
  
  spin_left(CarSpeedControl); // Spin left
  delay(2000);
  brake();
  delay(500);
  
  spin_right(CarSpeedControl);// Spin right
  delay(2000);
  brake();
  delay(500);
  
  front_left(CarSpeedControl);// Front-left diagonal
  delay(2000);
  brake();
  delay(500);
  
  front_right(CarSpeedControl);// Front-right diagonal
  delay(2000);
  brake();
  delay(500);
  
  left_rear(CarSpeedControl); // Left-rear diagonal
  delay(2000);
  brake();
  delay(500);
  
  right_rear(CarSpeedControl);// Right-rear diagonal
  delay(2000);
  brake();
  delay(500);
  
  digitalWrite(LED_PIN, HIGH); // Turn off LED after sequence
  while(1); // Stop after completing sequence (remove if you want it to repeat)
}
