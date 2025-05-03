#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(9600);  // USB Serial Monitor
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // UART2 to Arduino
  Serial.println("ESP32 ready to receive commands from Arduino...");
}

void loop() {
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    Serial.print("UART2 Received: ");
    Serial.println(msg);
    
    // Example: act on command
    if (msg == "PING") {
      Serial2.println("PONG");
    }
  }
}
