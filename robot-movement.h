/*
 * robot-movement.h - Library for dual-motor robotics movement. This assumes
 * that you are using a DC motor connected to a L293D, as well as an otpical
 * encoder with an encoder disk
 */
#ifndef ROBOT_MOVEMENT_H_
    #define ROBOT_MOVEMENT_H_

    #include <Arduino.h>
    #include <QuickPID.h>
    #include <stdlib.h>

// TODO maybe add an encoder count / value to this struct?
struct motor
{
        const int speed_pin;
        const int backward_dir_pin;
        const int forward_dir_pin;
        const int encoder_pin;
        volatile int encoder_count;
};

// define constants
const double WHEEL_RADIUS        = 1.5;               // wheel radius (inches)
const double DIST_BETWEEN_WHEELS = 3.937008;          // distance between wheels (inches)
const int ENCODER_DISK_COUNT     = 60;                // number of holes in encoder disk
const int MOTOR_MAX              = 1023;              // fastest speed of motors on ground
const int MOTOR_MIN              = 0.85 * MOTOR_MAX;  // slowest speed of motors on ground

// Should only be called once in the setup function. Sets up the pinModes and
// input status for the left and right motors
void setup_motor(motor);

// TODO calibrate the PID values within this function
void move(double inches, motor *left, motor *right);

void turn(double degrees, motor *left, motor *right);

// speed should be a value from MOTOR_MIN->MOTOR_MAX
void spin_motor(motor, int speed);

// sets both direction pins of motor to LOW
void stop_motor(motor);

#endif
