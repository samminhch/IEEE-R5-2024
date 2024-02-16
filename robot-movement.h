#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include "Arduino.h"

struct motor {
    int speed_pin;
    int backward_dir_pin;
    int forward_dir_pin;
    int encoder_pin;
    volatile int encoder_count;
};

void setup_motor(motor m);
void move(double inches, motor *left, motor *right);
void turn(double degrees, motor *left, motor *right);
void stop_motor(motor m);
void spin_motor(motor m, int speed_percentage);
void update_left_encoder();
void update_right_encoder();
int readDipSwitch();

#endif
