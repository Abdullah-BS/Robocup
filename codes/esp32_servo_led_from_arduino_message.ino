#include <ESP32Servo.h>

#define RXD2 16
#define TXD2 17
#define LED_PIN   26    // LED on GPIO26
#define SERVO_PIN 5     // Servo on GPIO5

Servo myServo;

void setup() {
  Serial.begin(9600);  // USB Serial Monitor
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART2 to Arduino
  Serial.println("ESP32 ready to receive commands from Arduino...");
  myServo.attach(SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    Serial.print("UART2 Received: ");
    Serial.println(msg);
      digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    myServo.write(0);
      delay(1000);
      myServo.write(90);
      delay(1000);
      myServo.write(180);



    
    // Example: act on command
    if (msg == "PING") {
      Serial2.println("PONG");
    }
  }
}
