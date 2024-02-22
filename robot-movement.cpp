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
    left->encoder_count        = 0;
    right->encoder_count       = 0;
    float num_holes            = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);
    int input_left             = left->encoder_count;
    int input_right            = right->encoder_count;
    int error_left             = num_holes - input_left;
    int error_right            = num_holes - input_right;
    int prev_error_left        = error_left;
    int prev_error_right       = error_right;
    float error_integral_left  = 0;
    float error_integral_right = 0;
    unsigned long prev_time    = micros();

    // PID constants -- they need adjusting
    float kP = 2;
    float kI = 0;
    float kD = 0;

    // by default, spin that @ 100%
    spin_motor(*left, 100);
    spin_motor(*right, 100);
    do
    {
        unsigned long time = micros();
        float time_diff    = ((float) (time - prev_time)) / 1e6;  // in seconds

        // update input and error values
        input_left  = left->encoder_count;
        input_right = right->encoder_count;

        // our error function
        error_left  = num_holes - input_left;
        error_right = num_holes - input_right;

        // calculate the change in our error over time
        int error_diff_left  = error_left - prev_error_left;
        int error_diff_right = error_right - prev_error_right;

        float error_derivative_left  = error_diff_left / time_diff;
        float error_derivative_right = error_diff_right / time_diff;

        // calculate the accumulated error in respect to time
        error_integral_left  += error_left * time_diff;
        error_integral_right += error_right * time_diff;

        // get the output of the PID function
        float output_left  = kP * error_left + kI * error_integral_left + kD * error_derivative_left;
        float output_right = kP * error_right + kI * error_integral_right + kD * error_derivative_right;

        // update previous values
        prev_error_left  = error_left;
        prev_error_right = error_right;
        prev_time        = time;

        // TODO: graph out these outputs to adjust PID values
        Serial.print("right_motor_encoder_count:");
        Serial.print(right->encoder_count);
        Serial.print("left_motor_encoder_count:");
        Serial.println(left->encoder_count);

        spin_motor(*left, output_left);
        spin_motor(*right, output_right);
    } while (error_left > 0 && error_right > 0);
    stop_motor(*left);
    stop_motor(*right);
}

// sync the motors by making them have the same angular velocity
void move_speed(double inches, motor *left, motor *right)
{
    {
        left->encoder_count  = 0;
        right->encoder_count = 0;
        float num_holes      = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);

        // angular velocity calculations
        int input_left, input_left_prev, input_right, input_right_prev;

        int error_left             = num_holes - input_left;
        int error_right            = num_holes - input_right;
        int error_left_prev        = error_left;
        int error_right_prev       = error_right;
        float error_integral_left  = 0;
        float error_integral_right = 0;
        unsigned long time_prev    = micros();

        // PID constants -- they need adjusting
        float target_angular_velocity = 4.0;  // 4 in/s
        float kP                      = 2;
        float kI                      = 0;
        float kD                      = 0;

        // by default, spin that @ 100%
        spin_motor(*left, 100);
        spin_motor(*right, 100);
        do
        {
            unsigned long time = micros();
            float time_diff    = ((float) (time - time_prev)) / 1e6;  // in seconds

            // update input and error values
            input_left  = left->encoder_count;
            input_right = right->encoder_count;

            // our error function
            // (a) get the angular velocity of both motors
            float angular_velocity_const = 2 * PI * WHEEL_RADIUS / time_diff;
            float angular_velocity_left =
                ((input_left - input_left_prev) / ENCODER_DISK_COUNT) * angular_velocity_const;
            float angular_velocity_right =
                ((input_right - input_right_prev) / ENCODER_DISK_COUNT) * angular_velocity_const;

            error_left  = target_angular_velocity - angular_velocity_left;
            error_right = target_angular_velocity - angular_velocity_right;

            // calculate the change in our error over time
            int error_diff_left  = error_left - error_left_prev;
            int error_diff_right = error_right - error_right_prev;

            float error_derivative_left  = error_diff_left / time_diff;
            float error_derivative_right = error_diff_right / time_diff;

            // calculate the accumulated error in respect to time
            error_integral_left  += error_left * time_diff;
            error_integral_right += error_right * time_diff;

            // get the output of the PID function
            float output_left  = kP * error_left + kI * error_integral_left + kD * error_derivative_left;
            float output_right = kP * error_right + kI * error_integral_right + kD * error_derivative_right;

            // update previous values
            input_left_prev  = input_left;
            input_right_prev = input_right;
            error_left_prev  = error_left;
            error_right_prev = error_right;
            time_prev        = time;

            // TODO: graph out these outputs to adjust PID values
            Serial.print("angular_velocity_left");
            Serial.print(angular_velocity_left);
            Serial.print("angular_velocity_right");
            Serial.println(angular_velocity_right);

            spin_motor(*left, output_left);
            spin_motor(*right, output_right);
        } while (error_left > 0 && error_right > 0);
        stop_motor(*left);
        stop_motor(*right);
    }
}

void turn(double degrees, motor *left, motor *right)
{
    int num_rotations    = (DIST_BETWEEN_WHEELS * abs(degrees) / 2) / (WHEEL_RADIUS * 360) * ENCODER_DISK_COUNT * 0.9;
    int speed            = 100;
    left->encoder_count  = 0;
    right->encoder_count = 0;

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

void spin_motor(motor m, int speed_percentage)
{
    bool reversed = speed_percentage < 0;
    // cap the values at [0%, 100%]
    speed_percentage = constrain(abs(speed_percentage), 0, 100);

    analogWrite(m.speed_pin, map(speed_percentage, 0, 100, MOTOR_MIN, MOTOR_MAX));
    digitalWrite(m.backward_dir_pin, reversed);
    digitalWrite(m.forward_dir_pin, !reversed);
}
