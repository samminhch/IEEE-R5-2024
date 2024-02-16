#include <Arduino.h>
#include "robot-movement.h"

void setup() {
    // Set up the DIP switch pins as inputs with pull-ups
    int dipSwitchPins[8] = {A0, A3, A4, A5, 10, 11, 12, 13};
    for (int i = 0; i < 8; i++) {
        pinMode(dipSwitchPins[i], INPUT_PULLUP);
    }

    // Initialize serial communication
    Serial.begin(9600);
}

void loop() {
    // Read the state of the DIP switch
    int dipSwitchState = 0;
    for (int i = 0; i < 8; i++) {
        dipSwitchState |= digitalRead(dipSwitchPins[i]) << i;
    }

    // Print the DIP switch state
    Serial.print("DIP Switch State: ");
    Serial.println(dipSwitchState);

    // Add any additional logic based on the DIP switch state here
    // For example:
    // if (dipSwitchState == 1) {
    //     // Do something
    // }
    // else if (dipSwitchState == 2) {
    //     // Do something else
    // }

    // Add a delay to prevent reading the DIP switch too frequently
    delay(1000);
}
