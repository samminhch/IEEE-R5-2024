/*
 * robot-movement.h - Library for dual-motor robotics movement. This assumes
 * that you are using a DC motor connected to a L293D
 */
#ifndef ROBOT_MOVEMENT_H_
    #define ROBOT_MOVEMENT_H_

    #include <Arduino.h>
    #include <stdlib.h>

// TODO maybe add an encoder count / value to this struct?
struct motor
{
        const byte speed_pin;
        const byte backward_dir_pin;
        const byte forward_dir_pin;
};

// define constants
const PROGMEM int MOTOR_MAX = 1023;              // fastest speed of motors on ground
const PROGMEM int MOTOR_MIN = 0.85 * MOTOR_MAX;  // slowest speed of motors on ground

// Should only be called once in the setup function. Sets up the pinModes and
// input status for the left and right motors
void setup_motor(motor);

// speed should be a value from 0->100%
void spin_motor(motor, float speed);

// sets both direction pins of motor to LOW
void stop_motor(motor);
#endif
