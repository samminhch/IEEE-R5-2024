/*
  robot-movement.h - Library for dual-motor robotics movement. This assumes that
  you are using a DC motor connected to a L293D
*/
#ifndef ROBOT_MOVEMENT_H_
#define ROBOT_MOVEMENT_H_

#include <Arduino.h>

// maybe add an encoder count / value to this struct?
struct motor {
        int speed_pin;
        int direction_pin_1;
        int direction_pin_2;
};

// define constants
const double WHEEL_RADIUS = 0.33;  // inches
const int MOTOR_MIN       = 0;
const int MOTOR_MAX       = 255;

// Should only be called once in the setup function. Sets up the pinModes and
// input status for the left and right motors
void setup_motor(motor);

// TODO
void move_robot(motor left, motor right, double inches, double degrees);

// speed should be a value from 0-255
void spin_motor(motor, int speed, bool reversed);

// sets both direction pins of motor to LOW
void stop_motor(motor);

#endif
