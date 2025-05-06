#include <Bluepad32.h>

// Motor control definitions
// #define RXD2 16
#define TXD2 1


#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

// Motor pins for Front and Back Motors
#define FRONT_LEFT_IN1 23
#define FRONT_LEFT_IN2 22
#define FRONT_RIGHT_IN1 6
#define FRONT_RIGHT_IN2 7

#define BACK_LEFT_IN1 19
#define BACK_LEFT_IN2 18
#define BACK_RIGHT_IN1 5
#define BACK_RIGHT_IN2 17


#define MAX_MOTOR_SPEED 255
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];


void processCarMovement(int inputValue) {
  switch(inputValue) {
    case FORWARD:
      sendCommand("forward");
      break;

    case BACKWARD:
      sendCommand("backward");  // Changed from "back" to match Arduino
      break;

    case LEFT:
      sendCommand("left");
      break;

    case RIGHT:
      sendCommand("right");
      break;

    case STOP:
    default:
      sendCommand("stop");  // Always send stop command when no movement
      break;
  }
}

void setUpPinModes() {
  for (int i = 0; i < 2; i++) {
    // pinMode(motorPins[i].pinIN1, OUTPUT);
    // pinMode(motorPins[i].pinIN2, OUTPUT);
    // ledcSetup(motorPins[i].pwmSpeedChannel, PWMFreq, PWMResolution);
    // ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmSpeedChannel);
    // rotateMotor(i, STOP);
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
  bool commandSent = false;
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
      commandSent = true;
    }
    else if (leftY > deadzone) {
      processCarMovement(BACKWARD);
      commandSent = true;
    }
    else if (leftX < -deadzone) {
      processCarMovement(LEFT);
      commandSent = true;
    }
    else if (leftX > deadzone) {
      processCarMovement(RIGHT);
      commandSent = true;
    }
  }
  
  // If no movement command was sent and joystick is centered, send stop
  if (!commandSent) {
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
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // UART to Arduino

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


void sendCommand(const char* command) {
  Serial2.println(command);  // UART to Arduino
  Serial.print("Sent: ");
  Serial.println(command);
}
