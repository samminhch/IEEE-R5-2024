#include "robot-movement.h"

int encoder_counter = 0;

void setup_motor(motor m) {
    pinMode(m.speed_pin, OUTPUT);
    pinMode(m.direction_pin_1, OUTPUT);
    pinMode(m.direction_pin_2, OUTPUT);

    digitalWrite(m.direction_pin_1, LOW);
    digitalWrite(m.direction_pin_2, LOW);
}

void move_robot(double inches, double degrees) {}

void stop_motor(motor m) {
    digitalWrite(m.direction_pin_1, LOW);
    digitalWrite(m.direction_pin_2, LOW);
}

void spin_motor(motor m, int speed) {
    bool reversed = speed < 0;
    if (speed < MOTOR_MIN || speed > MOTOR_MAX) {
        char buffer[64];
        sprintf(buffer, "%d is not in the range of %d->%d. Not executing function.", speed, MOTOR_MIN, MOTOR_MAX);
        Serial.println(buffer);

        return;
    }
    analogWrite(m.speed_pin, abs(speed));
    digitalWrite(m.direction_pin_1, reversed);
    digitalWrite(m.direction_pin_2, !reversed);
}
