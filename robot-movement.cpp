#include "Arduino.h"
#include "robot-movement.h"

void setup_motor(motor m)
{
    pinMode(m.speed_pin, OUTPUT);
    pinMode(m.backward_dir_pin, OUTPUT);
    pinMode(m.forward_dir_pin, OUTPUT);
    pinMode(m.encoder_pin, INPUT);

    m.encoder_count = 0;

    digitalWrite(m.backward_dir_pin, LOW);
    digitalWrite(m.forward_dir_pin, LOW);
}

void move_robot(double inches, motor left, motor right) {}

void stop_motor(motor m)
{
    digitalWrite(m.backward_dir_pin, LOW);
    digitalWrite(m.forward_dir_pin, LOW);
}

void spin_motor(motor m, int speed)
{
    bool reversed = speed < 0;
    if (speed < MOTOR_MIN || speed > MOTOR_MAX)
    {
        char buffer[64];
        sprintf(buffer,
                "%d is not in the range of %d->%d. Not executing function.",
                speed, MOTOR_MIN, MOTOR_MAX);
        Serial.println(buffer);

        return;
    }
    analogWrite(m.speed_pin, abs(speed));
    digitalWrite(m.backward_dir_pin, reversed);
    digitalWrite(m.forward_dir_pin, !reversed);
}
