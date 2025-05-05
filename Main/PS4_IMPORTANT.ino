#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            return;
        }
    }
    Serial.println("Controller connected, but no empty slot");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Controller disconnected, index=%d\n", i);
            myControllers[i] = nullptr;
            return;
        }
    }
    Serial.println("Controller disconnected, not found");
}

void processGamepad(ControllerPtr ctl) {
    // Print button bitmask when any button is pressed
    uint32_t btn = ctl->buttons();
    if (btn) {
        Serial.printf("Buttons pressed: 0x%04x\n", btn);
    }

    // Print left stick movement
    int16_t lx = ctl->axisX(), ly = ctl->axisY();
    if (lx || ly) {
        Serial.printf("Left stick: X=%d, Y=%d\n", lx, ly);
    }

    // Print right stick movement
    int16_t rx = ctl->axisRX(), ry = ctl->axisRY();
    if (rx || ry) {
        Serial.printf("Right stick: X=%d, Y=%d\n", rx, ry);
    }
}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
            processGamepad(ctl);
        }
    }
}

void setup() {
    Serial.begin(115200);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void loop() {
    if (BP32.update()) processControllers();
    delay(150);
}
