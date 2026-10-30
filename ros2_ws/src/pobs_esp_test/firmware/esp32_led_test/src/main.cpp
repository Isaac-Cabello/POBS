
#include <Arduino.h>
#include <stdio.h>

const int LED_PINS[4] = {16, 17, 18, 19};
const unsigned long TIMEOUT_MS = 500;

unsigned long lastCommandTime[4] = {0, 0, 0, 0};
bool commandReceived[4] = {false, false, false, false};

char inputBuffer[32];
size_t inputLength = 0;

void processCommand(const char* command) {
    int index, state;
    char extra;

    if (sscanf(command, "LED,%d,%d%c",
               &index, &state, &extra) != 2) {
        return;
    }

    if (index < 0 || index >= 4 ||
        (state != 0 && state != 1)) {
        return;
    }

    digitalWrite(LED_PINS[index], state ? HIGH : LOW);
    lastCommandTime[index] = millis();
    commandReceived[index] = true;
}

void setup() {
    Serial.begin(115200);

    for (int i = 0; i < 4; i++) {
        pinMode(LED_PINS[i], OUTPUT);
        digitalWrite(LED_PINS[i], LOW);
    }

    Serial.println("POBS ESP32 READY");
}

void loop() {
    // Read serial commands without blocking the main loop.
    while (Serial.available() > 0) {
        char c = static_cast<char>(Serial.read());

        if (c == '\n') {
            inputBuffer[inputLength] = '\0';
            processCommand(inputBuffer);
            inputLength = 0;
        } else if (c != '\r') {
            if (inputLength < sizeof(inputBuffer) - 1) {
                inputBuffer[inputLength++] = c;
            } else {
                // Discard an oversized command.
                inputLength = 0;
            }
        }
    }

    // Each LED has its own communication timeout.
    unsigned long now = millis();

    for (int i = 0; i < 4; i++) {
        if (commandReceived[i] &&
            now - lastCommandTime[i] > TIMEOUT_MS) {
            digitalWrite(LED_PINS[i], LOW);
            commandReceived[i] = false;
        }
    }
}
