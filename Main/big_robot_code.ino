#include <Bluepad32.h>
#include <vector>  // Add this line for std::vector

// Motor control definitions
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define FORWARD_LEFT 5
#define FORWARD_RIGHT 6
#define BACKWARD_LEFT 7
#define BACKWARD_RIGHT 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define STOP 0

#define BACK_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 1
#define FRONT_RIGHT_MOTOR 2
#define FRONT_LEFT_MOTOR 3

struct MOTOR_PINS {
  int pinIN1;
  int pinIN2;
  int pinEn; 
  int pwmSpeedChannel;
};

// Make motorPins global by moving it before any functions that use it
std::vector<MOTOR_PINS> motorPins = {
  {16, 17, 22, 4},  // BACK_RIGHT_MOTOR (L298N #1 - Channel A)
  {18, 19, 23, 5},  // BACK_LEFT_MOTOR (L298N #1 - Channel B)
  {26, 27, 14, 6},  // FRONT_RIGHT_MOTOR (L298N #2 - Channel A)
  {33, 25, 32, 7},  // FRONT_LEFT_MOTOR (L298N #2 - Channel B)
};

#define MAX_MOTOR_SPEED 200
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
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);                  
      break;
  
    case BACKWARD:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
  
    case RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
  
    case FORWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);  
      break;
  
    case FORWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);  
      break;
  
    case BACKWARD_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, STOP);   
      break;
  
    case BACKWARD_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);   
      break;
  
    case TURN_LEFT:
      rotateMotor(FRONT_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, -MAX_MOTOR_SPEED);  
      break;
  
    case TURN_RIGHT:
      rotateMotor(FRONT_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(BACK_RIGHT_MOTOR, -MAX_MOTOR_SPEED);
      rotateMotor(FRONT_LEFT_MOTOR, MAX_MOTOR_SPEED);
      rotateMotor(BACK_LEFT_MOTOR, MAX_MOTOR_SPEED);   
      break;
  
    case STOP:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);    
      break;
  
    default:
      rotateMotor(FRONT_RIGHT_MOTOR, STOP);
      rotateMotor(BACK_RIGHT_MOTOR, STOP);
      rotateMotor(FRONT_LEFT_MOTOR, STOP);
      rotateMotor(BACK_LEFT_MOTOR, STOP);z    
      break;
  }
}

void setUpPinModes() {
  for (int i = 0; i < motorPins.size(); i++) {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);  
    ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);  
    ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);     
    rotateMotor(i, STOP);  
  }
}

// Bluepad32 callbacks
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("Controller connected at index %d\n", i);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("No available slot for controller");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      processCarMovement(STOP); // Stop motors when controller disconnects
      Serial.printf("Controller disconnected from index %d\n", i);
      break;
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  // Read analog sticks (values range from -511 to 512)
  int leftX = ctl->axisX();
  int leftY = ctl->axisY();
  int rightX = ctl->axisRX();
  
  // Deadzone to prevent drifting
  const int deadzone = 50;
  
  if (abs(leftY) > deadzone || abs(leftX) > deadzone || abs(rightX) > deadzone) {
    // Forward/Backward (Left Stick Y-axis)
    if (leftY < -deadzone) {
      // Forward
      if (leftX < -deadzone) {
        processCarMovement(FORWARD_LEFT);
      } else if (leftX > deadzone) {
        processCarMovement(FORWARD_RIGHT);
      } else {
        processCarMovement(FORWARD);
      }
    } else if (leftY > deadzone) {
      // Backward
      if (leftX < -deadzone) {
        processCarMovement(BACKWARD_LEFT);
      } else if (leftX > deadzone) {
        processCarMovement(BACKWARD_RIGHT);
      } else {
        processCarMovement(BACKWARD);
      }
    } 
    // Left/Right (Left Stick X-axis)
    else if (leftX < -deadzone) {
      processCarMovement(LEFT);
    } else if (leftX > deadzone) {
      processCarMovement(RIGHT);
    }
    // Rotation (Right Stick X-axis)
    else if (rightX < -deadzone) {
      processCarMovement(TURN_LEFT);
    } else if (rightX > deadzone) {
      processCarMovement(TURN_RIGHT);
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
  
  // Initialize motor control
  setUpPinModes();
  
  // Setup Bluepad32 callbacks
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
