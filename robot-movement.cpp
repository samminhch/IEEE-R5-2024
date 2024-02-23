#include "robot-movement.h"

#include "Arduino.h"

void setup_motor(motor m)
{
    pinMode(m.speed_pin, OUTPUT);
    pinMode(m.backward_dir_pin, OUTPUT);
    pinMode(m.forward_dir_pin, OUTPUT);
    pinMode(m.encoder_pin, INPUT);

    m.encoder_count = 0;

    stop_motor(m);
}

// i need to test if this code works somehow
void move(double inches, motor *left, motor *right)
{
    left->encoder_count  = 0;
    right->encoder_count = 0;
    float num_holes      = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);

    // by default, spin that @ 100%
    float base_speed  = 90;
    float right_speed = base_speed;

    int error       = right->encoder_count - left->encoder_count;
    int error_total = 0;
    int error_prev  = error;

    // PID values
    float kP = 10;
    float kI = 0.005;
    float kD = 0.000;

    // we won't be changing the left motor's speed
    spin_motor(*left, base_speed);
    while (left->encoder_count < num_holes && right->encoder_count < num_holes)
    {
        spin_motor(*right, right_speed);

        error           = right->encoder_count - left->encoder_count;
        error_total    += error;
        int error_diff  = error - error_prev;
        error_prev      = error;

        Serial.print("error:");
        Serial.print(error);

        // only adjusting right motor, it's always faster than left.
        float PID_output = kP * error + kI * error_total + kD * error_diff;
        right_speed      = base_speed - PID_output;

        Serial.print(",right_speed:");
        Serial.println(right_speed);
    }

    Serial.print("left_encoder_count:");
    Serial.print(left->encoder_count);
    Serial.print(",right_encoder_count:");
    Serial.println(right->encoder_count);
    stop_motor(*left);
    stop_motor(*right);
}

void turn(double degrees, motor *left, motor *right)
{
    float wheel_circumference  = 2 * PI * WHEEL_RADIUS;
    float arc_length           = radians(degrees) * DIST_BETWEEN_WHEELS / 2;
    float num_rotations        = arc_length * ENCODER_DISK_COUNT / wheel_circumference;
    num_rotations             *= 0.825;                        // it overturns, multiply it by a magic constant
    num_rotations              = (int) (num_rotations + 0.5);  // rounding the value
    int speed                  = 100;
    left->encoder_count        = 0;
    right->encoder_count       = 0;

    spin_motor(*right, degrees > 0 ? -speed : speed);
    spin_motor(*left, degrees > 0 ? speed : -speed);

    // hold the program hostage until the turns complete
    while (abs(left->encoder_count) < num_rotations && abs(right->encoder_count) < num_rotations)
    {
        // Serial.print("encoder_count_required:");
        // Serial.print(num_rotations);
        // Serial.print(",left_encoder_count:");
        // Serial.print(left->encoder_count);
        // Serial.print(",right_encoder_count:");
        // Serial.println(right->encoder_count);
    }
    stop_motor(*left);
    stop_motor(*right);
}

void stop_motor(motor m)
{
    digitalWrite(m.backward_dir_pin, LOW);
    digitalWrite(m.forward_dir_pin, LOW);
}

void spin_motor(motor m, float speed_percentage)
{
    bool reversed = speed_percentage < 0;
    // cap the values at [0%, 100%]
    speed_percentage = constrain(abs(speed_percentage), 0, 100);

    analogWrite(m.speed_pin, map(speed_percentage, 0, 100, MOTOR_MIN, MOTOR_MAX));
    digitalWrite(m.backward_dir_pin, reversed);
    digitalWrite(m.forward_dir_pin, !reversed);
}
