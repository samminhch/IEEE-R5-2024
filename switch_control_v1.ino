#include <Arduino.h>
#include "robot-movement.h"
#include <string>

// Define DIP switch pins
#define dipSwitchPin_1 A0;
#define dipSwitchPin_2 A1;
#define dipSwitchPin_3 A2;


// Define paths based on the switch state
const path* paths = nullptr;
int numPaths = 0;

void setup() {
    // Set up the DIP switch pins as inputs with pull-ups 
    pinMode(dipSwitchPin_1,INPUT_PULLUP);
    pinMode(dipSwitchPin_2,INPUT_PULLUP);
    pinMode(dipSwitchPin_3,INPUT_PULLUP);
    
    // Initialize serial communication
    Serial.begin(9600);

    // Set paths based on DIP switch state
    int dipSwitchState = readDipSwitchState();
    setPaths(dipSwitchState);
}

void loop() {
    // Read the state of the DIP switch
    string dipSwitchState = readDipSwitchState();
    i
    // Check if the DIP switch state has changed
    
    // Execute the paths
   

    // Add a delay to prevent reading the DIP switch too frequently
    delay(1000);
}

// Read the state of the DIP switch
string readDipSwitchState() {
    
    
    int tempdipSwitchState1 = analogRead(dipSwitchPin_1);
    int tempdipSwitchState2 = analogRead(dipSwitchPin_2);
    int tempdipSwitchState3 = analogRead(dipSwitchPin_3);
    string dipSwitchState1 = to_string(tempdipSwitchState1);
    string dipSwitchState2 = to_string(tempdipSwitchState2);
    string dipSwitchState3 = to_string(tempdipSwitchState3);;
    string dipSwitchState = dipSwitchState1 + dipSwitchState2 + dipSwitchState3;
    return dipSwitchState;
}

// Set paths based on the DIP switch state
void setPaths(string dipSwitchState) {
    switch(dipSwitchState){
        case "000" : path 
    }
}
