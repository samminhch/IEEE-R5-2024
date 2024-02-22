#include <Arduino.h>
#include "robot-movement.h"

// Define DIP switch pins
int dipSwitchPins[8] = {A0, A3, A4, A5, 10, 11, 12, 13};

// Define paths based on the switch state
const path* paths = nullptr;
int numPaths = 0;

void setup() {
    // Set up the DIP switch pins as inputs with pull-ups
    for (int i = 0; i < 8; i++) {
        pinMode(dipSwitchPins[i], INPUT_PULLUP);
    }

    // Initialize serial communication
    Serial.begin(9600);

    // Set paths based on DIP switch state
    int dipSwitchState = readDipSwitchState();
    setPaths(dipSwitchState);
}

void loop() {
    // Read the state of the DIP switch
    int dipSwitchState = readDipSwitchState();

    // Check if the DIP switch state has changed
    if (dipSwitchState != readDipSwitchState()) {
        setPaths(dipSwitchState);
    }

    // Execute the paths
    for (int i = 0; i < numPaths; i++) {
        turn(paths[i].angle_before, left_motor, right_motor);
        move(paths[i].distance, left_motor, right_motor);
        turn(paths[i].angle_after, left_motor, right_motor);
    }

    // Add a delay to prevent reading the DIP switch too frequently
    delay(1000);
}

// Read the state of the DIP switch
int readDipSwitchState() {
    int dipSwitchState = 0;
    for (int i = 0; i < 8; i++) {
        dipSwitchState |= digitalRead(dipSwitchPins[i]) << i;
    }
    return dipSwitchState;
}

// Set paths based on the DIP switch state
void setPaths(int dipSwitchState) {
    if (dipSwitchState == 1) {
        paths = paths_seeding;
        numPaths = sizeof(paths_seeding) / sizeof(paths_seeding[0]);
    } else {
        paths = paths_elimination;
        numPaths = sizeof(paths_elimination) / sizeof(paths_elimination[0]);
    }
}
