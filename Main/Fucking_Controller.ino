#include <Bluepad32.h>

// Motor control definitions
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

struct MOTOR_PINS {
  int pinIN1;
  int pinIN2;
  int pinEn;
  int pwmSpeedChannel;
};

// Your correct motor pins تعديل البينات هنا
MOTOR_PINS motorPins[] = {
  {25, 26, 32, 4},  // RIGHT_MOTOR (ENA, Motor A)
  {27, 14, 33, 5},  // LEFT_MOTOR (ENB, Motor B)
};

#define MAX_MOTOR_SPEED 255
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Motor control functions
void rotateMotor(int motorNumber, int motorSpeed) {
  if (motorSpeed < 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);    
  }
  else if (motorSpeed > 0) {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);       
  }
  else {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);      
  }
  ledcWrite(motorPins[motorNumber].pwmSpeedChannel, abs(motorSpeed));
}

void processCarMovement(int inputValue) {
  switch(inputValue) {
    case FORWARD:
      rotateMotor(RIGHT_MOTOR, -MAX_MOTOR_SPEED);  // Inverted to fix direction
      rotateMotor(LEFT_MOTOR, -MAX_MOTOR_SPEED);
      Serial.println("Forward");
      break;

    case BACKWARD:
      rotateMotor(RIGHT_MOTOR, MAX_MOTOR_SPEED);   // Inverted to fix direction
      rotateMotor(LEFT_MOTOR, MAX_MOTOR_SPEED);
      Serial.println("backward");

      break;

    case LEFT:
      rotateMotor(RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(LEFT_MOTOR, -MAX_MOTOR_SPEED);
      break;

    case RIGHT:
      rotateMotor(RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(LEFT_MOTOR, MAX_MOTOR_SPEED);
      break;

    case STOP:
    default:
      rotateMotor(RIGHT_MOTOR, STOP);
      rotateMotor(LEFT_MOTOR, STOP);
      break;
  }
}

void setUpPinModes() {
  for (int i = 0; i < 2; i++) {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);
    rotateMotor(i, STOP);
  }
}

// Bluepad32 callbacks
void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller connected at index %d\n", i);
      myControllers[i] = ctl;
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      processCarMovement(STOP);
      Serial.printf("Controller disconnected from index %d\n", i);
      break;
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  int leftX = ctl->axisX();
  int leftY = ctl->axisY();

  const int deadzone = 50;

  if (abs(leftY) > deadzone || abs(leftX) > deadzone) {
    if (leftY < -deadzone) {
      if (leftX < -deadzone) {
        processCarMovement(LEFT);
      } else if (leftX > deadzone) {
        processCarMovement(RIGHT);
      } else {
        processCarMovement(FORWARD);
      }
    }
    else if (leftY > deadzone) {
      processCarMovement(BACKWARD);
    }
    else if (leftX < -deadzone) {
      processCarMovement(LEFT);
    }
    else if (leftX > deadzone) {
      processCarMovement(RIGHT);
    }
  } else {
    processCarMovement(STOP);
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());

  setUpPinModes();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  Serial.println("Ready. Waiting for controllers...");
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  delay(10);
}
