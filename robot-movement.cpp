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
    float num_holes  = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);
    int kP = 0.1, kI = 0, kD = 0;  // TODO set these values later
    float result_l, result_r;

    QuickPID pid_left((float *) &(left->encoder_count), &result_l, &num_holes);
    QuickPID pid_right((float*) &(right->encoder_count), &result_r, &num_holes);

    pid_left.SetTunings(kP, kI, kD);
    pid_right.SetTunings(kP, kI, kD);
    pid_left.SetOutputLimits(0, 100); // spin_motor takes speed %
    pid_right.SetOutputLimits(0, 100);

    pid_left.SetMode(QuickPID::Control::automatic);
    pid_right.SetMode(QuickPID::Control::automatic);
    do
    {
        if (pid_left.Compute())
        {
            spin_motor(*left, 100 - result_l);
        }
        if (pid_right.Compute())
        {
            spin_motor(*right, 100 -result_r);
        }

        Serial.print("pid_left:");
        Serial.print(result_l);
        Serial.print(",pid_right:");
        Serial.println(result_r);
    } while (left->encoder_count < num_holes && right->encoder_count < num_holes);
    stop_motor(*left);
    stop_motor(*right);
}

void turn(double degrees, motor *left, motor *right)
{
    int num_rotations    = (DIST_BETWEEN_WHEELS * degrees / 2) / (WHEEL_RADIUS * 360) * ENCODER_DISK_COUNT ;
    int speed            = 100;
    left->encoder_count  = 0;
    right->encoder_count = 0;

    spin_motor(*right, degrees > 0 ? -speed : speed);
    spin_motor(*left, degrees > 0 ? speed : -speed);

    // hold the program hostage until the turns complete
    while (abs(left->encoder_count) < abs(num_rotations) && abs(right->encoder_count) < abs(num_rotations)) {
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

void spin_motor(motor m, int speed_percentage)
{
    bool reversed = speed_percentage < 0;
    // cap the values at -100%, 100%
    speed_percentage = constrain(abs(speed_percentage), 0, 100);

    analogWrite(m.speed_pin, map(speed_percentage, 0, 100, 0, MOTOR_MAX));
    digitalWrite(m.backward_dir_pin, reversed);
    digitalWrite(m.forward_dir_pin, !reversed);
}
