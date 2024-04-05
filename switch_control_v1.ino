
#include <Arduino.h>
#include "robot-movement.h"


// Define DIP switch pins
#define dipSwitchPin_1 A0
#define dipSwitchPin_2 A1
#define dipSwitchPin_3 A2

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
    int dipSwitchState = readDipSwitchState();
    
    // Check if the DIP switch state has changed
    
    // Execute the paths
   

    // Add a delay to prevent reading the DIP switch too frequently
    delay(1000);
}

// Read the state of the DIP switch
int readDipSwitchState() {
    int station = 4*(digitalRead(dipSwitchPin_1)) + 2*(digitalRead(dipSwitchPin_2)) + 1*(digitalRead(dipSwitchPin_3));
    return station;
}

// Set paths based on the DIP switch state
void setPaths(int dipSwitchState) {
    switch(dipSwitchState){
        case 0 : ;
        case 1 : ;
        case 2 : ;
        
    }
}
