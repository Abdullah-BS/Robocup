void setup() {
  Serial.begin(9600); // UART to ESP32
  delay(2000);        // Give ESP32 time to boot
}

void loop() {
  Serial.println("Hello from Arduino");
  delay(1000); // Send every 1 second
}
