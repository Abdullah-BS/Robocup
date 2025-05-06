/**
* Yahboom Omniduino Mecanum Wheel Movement Test
* Corrected motor coordination for proper omnidirectional movement
*/

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

// Motor definitions - CHANGE THESE IF MOVEMENT IS INCORRECT
// Try swapping motor numbers or reversing directions if movement is wrong
const uint8_t MOTOR_FRONT_RIGHT = 0;  // Wheel at front-right position
const uint8_t MOTOR_FRONT_LEFT = 1;   // Wheel at front-left position
const uint8_t MOTOR_REAR_LEFT = 2;    // Wheel at rear-left position
const uint8_t MOTOR_REAR_RIGHT = 3;   // Wheel at rear-right position

// Motor pins - CHANGE THESE TO MATCH YOUR WIRING
const uint8_t motorPins[4][2] = {
  {10, 11},  // Front-right motor (IN1, IN2)
  {13, 12},  // Front-left motor
  {15, 14},  // Rear-left motor
  {8, 9}     // Rear-right motor
};

int moveSpeed = 160;  // Speed (0-160)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup() {
  Serial.begin(115200);
  Serial.println("Omniduino Movement Test Starting...");
  
  pwm.begin();
  pwm.setPWMFreq(60);
  brake();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void setMotor(uint8_t motorNum, int speed) {
  // Constrain speed and map to PWM range
  speed = constrain(speed, -160, 160);
  int pwmSpeed = map(abs(speed), 0, 160, 0, 4095);
  
  Serial.print("Motor ");
  Serial.print(motorNum);
  Serial.print(" set to ");
  Serial.println(speed);
  
  if (speed > 0) {  // Forward
    pwm.setPWM(motorPins[motorNum][0], 0, pwmSpeed);
    pwm.setPWM(motorPins[motorNum][1], 0, 0);
  } else if (speed < 0) {  // Reverse
    pwm.setPWM(motorPins[motorNum][0], 0, 0);
    pwm.setPWM(motorPins[motorNum][1], 0, pwmSpeed);
  } else {  // Brake
    pwm.setPWM(motorPins[motorNum][0], 0, 0);
    pwm.setPWM(motorPins[motorNum][1], 0, 0);
  }
}


void brake() {
  for (int i = 0; i < 4; i++) {
    setMotor(i, 0);
  }
  Serial.println("Braking all motors");
}

// CORRECTED MOVEMENT FUNCTIONS
void run(int Speed) {
  // All wheels forward
  setMotor(0, -Speed);  // Front-right
  setMotor(1, -Speed);  // Front-left
  setMotor(2, Speed);  // Rear-left
  setMotor(3, Speed);  // Rear-right
}

void back(int Speed) {
  // All wheels backward
  setMotor(0, Speed);
  setMotor(1, Speed);
  setMotor(2, -Speed);
  setMotor(3, -Speed);
}

void right(int Speed) {
  // Strafe left
  setMotor(0, -Speed);
  setMotor(1, Speed);
  setMotor(2, -Speed);
  setMotor(3, Speed);
}

void left(int Speed) {
  // Strafe right
  setMotor(0, Speed);
  setMotor(1, -Speed);
  setMotor(2, Speed);
  setMotor(3, -Speed);
}

void forward_right(int Speed) {
  // Strafe right
  setMotor(0, -Speed);
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, Speed);
}

void forward_left(int Speed) {
  // Strafe right
  setMotor(0, 0);
  setMotor(1, -Speed);
  setMotor(2, Speed);
  setMotor(3, 0);
}

void back_right(int Speed) {
  // Strafe right
  setMotor(0, 0);
  setMotor(1, Speed);
  setMotor(2, -Speed);
  setMotor(3, 0);
}

void back_left(int Speed) {
  // Strafe right
  setMotor(0, Speed);
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, -Speed);
}

void rotate_right(int Speed) {
  // Strafe right
  setMotor(0, -Speed);
  setMotor(1, Speed);
  setMotor(2, Speed);
  setMotor(3, -Speed);
}

void rotate_left(int Speed) {
  // Strafe right
  setMotor(0, Speed);
  setMotor(1, -Speed);
  setMotor(2, -Speed);
  setMotor(3, Speed);
}

void motorAction(String command) {
  if (command == "forward") {
    run(moveSpeed);
  }
  else if (command == "backward") {
    back(moveSpeed);
  }
  else if (command == "left") {
    left(moveSpeed);
  }
  else if (command == "right") {
    right(moveSpeed);
  }
  else if (command == "stop") {
    brake();
  }
  else if (command == "rotate_left") {
    rotate_left(moveSpeed);
  }
  else if (command == "rotate_right") {
    rotate_right(moveSpeed);
  }
}

void loop() {

  if (Serial.available()) {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  Serial.println(cmd);
  motorAction(cmd);
  }


  // run(moveSpeed);       // Forward
  // delay(2000);
  // brake();
  // delay(500);
  
  // back(moveSpeed);      // Backward
  // delay(2000);
  // brake();
  // delay(500);
  
  // left(moveSpeed);      // Strafe left
  // delay(2000);
  // brake();
  // delay(500);
  
  // right(moveSpeed);     // Strafe right
  // delay(2000);
  // brake();
  // delay(500);
    
  // forward_left(moveSpeed);// Diagonal front-left
  // delay(2000);
  // brake();
  // delay(500);
  
  // forward_right(moveSpeed);// Diagonal front-right
  // delay(2000);
  // brake();
  // delay(500);

  // back_left(moveSpeed);// Diagonal front-right
  // delay(2000);
  // brake();
  // delay(500);

  // back_right(moveSpeed);// Diagonal front-right
  // delay(2000);
  // brake();
  // delay(500);

  // rotate_left(moveSpeed);// Diagonal front-right
  // delay(2000);
  // brake();
  // delay(500);

  // rotate_right(moveSpeed);// Diagonal front-right
  // delay(2000);
  // brake();
  // delay(500);

  
    }
