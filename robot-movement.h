/*
  robot-movement.h - Library for dual-motor robotics movement. This assumes that
  you are using a DC motor connected to a L293D
*/
#ifndef ROBOT_MOVEMENT_H_
#define ROBOT_MOVEMENT_H_

#include <Arduino.h>

// TODO maybe add an encoder count / value to this struct?
struct motor
{
        const int speed_pin;
        const int backward_dir_pin;
        const int forward_dir_pin;
        const int encoder_pin;
        int encoder_count;
};

// define constants
const double WHEEL_RADIUS    = 0.33;  // inches
const int ENCODER_DISK_COUNT = 50;    // number of holes in encoder disk
const int MOTOR_MAX          = 1023;
const int MOTOR_MIN          = -MOTOR_MAX;

// Should only be called once in the setup function. Sets up the pinModes and
// input status for the left and right motors
void setup_motor(motor);

// this function is subject to change
void move_robot(motor left, motor right, double inches, double degrees);

// speed should be a value from MOTOR_MIN->MOTOR_MAX
void spin_motor(motor, int speed);

// sets both direction pins of motor to LOW
void stop_motor(motor);

#endif
