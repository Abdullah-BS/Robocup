// Motor pins for Front and Back Motors
#define FRONT_LEFT_IN1 23
#define FRONT_LEFT_IN2 22
#define FRONT_RIGHT_IN1 6
#define FRONT_RIGHT_IN2 7

#define BACK_LEFT_IN1 19
#define BACK_LEFT_IN2 18
#define BACK_RIGHT_IN1 5
#define BACK_RIGHT_IN2 17

// Motor control speeds
#define MAX_MOTOR_SPEED 255  // Max speed (PWM value)

// Setup function to initialize the motor control pins
void setup() {
  // Initialize motor pins as OUTPUT
  pinMode(FRONT_LEFT_IN1, OUTPUT);
  pinMode(FRONT_LEFT_IN2, OUTPUT);
  pinMode(FRONT_RIGHT_IN1, OUTPUT);
  pinMode(FRONT_RIGHT_IN2, OUTPUT);
  
  pinMode(BACK_LEFT_IN1, OUTPUT);
  pinMode(BACK_LEFT_IN2, OUTPUT);
  pinMode(BACK_RIGHT_IN1, OUTPUT);
  pinMode(BACK_RIGHT_IN2, OUTPUT);

  // Make all motors move forward
  moveAllForward();
}

// Loop function that keeps the motors running (no need to loop here, just for demonstration)
void loop() {
  // The car will move forward as long as the loop runs, no additional code needed
}

// Function to move all motors forward
void moveAllForward() {
  // Set the front left motor to move forward
  digitalWrite(FRONT_LEFT_IN1, HIGH);
  digitalWrite(FRONT_LEFT_IN2, LOW);

  // Set the front right motor to move forward
  digitalWrite(FRONT_RIGHT_IN1, HIGH);
  digitalWrite(FRONT_RIGHT_IN2, LOW);

  // Set the back left motor to move forward
  digitalWrite(BACK_LEFT_IN1, HIGH);
  digitalWrite(BACK_LEFT_IN2, LOW);

  // Set the back right motor to move forward
  digitalWrite(BACK_RIGHT_IN1, HIGH);
  digitalWrite(BACK_RIGHT_IN2, LOW);
}
