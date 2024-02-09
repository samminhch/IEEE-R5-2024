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
    left->encoder_count     = 0;
    right->encoder_count    = 0;
    float num_holes         = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);
    int input_left          = left->encoder_count;
    int input_right         = right->encoder_count;
    int prev_input_left     = input_left;
    int prev_input_right    = input_right;
    int error_left          = num_holes - input_left;
    int error_right         = num_holes - input_right;
    int prev_error_left     = error_left;
    int prev_error_right    = error_right;
    unsigned long prev_time = micros();

    // PID constants
    float kP = 2;
    float kI = 0;
    float kD = 0;

    spin_motor(*left, 100);
    spin_motor(*right, 100);
    do
    {
        unsigned long time      = micros();
        unsigned long time_diff = time - prev_time;

        // compute every 20 microseconds (arbitrary value)
        if (time_diff < 20)
        {
            continue;
        }

        // update input and error values
        input_left  = left->encoder_count;
        input_right = right->encoder_count;
        error_left  = num_holes - input_left;
        error_right = num_holes - input_right;

        int input_diff_left  = input_left - prev_input_left;
        int input_diff_right = input_right - prev_input_right;

        int error_diff_left  = error_left - prev_error_left;
        int error_diff_right = error_right - prev_error_right;

        float p_left  = kP * error_left - kP * input_diff_left;
        float p_right = kP * error_right - kP * input_diff_right;

        float i_left  = kI * error_left;
        float i_right = kI * error_right;

        float d_left  = kD * error_diff_left;
        float d_right = kD * error_diff_right;

        // update previous values
        prev_input_left  = input_left;
        prev_input_right = input_right;
        prev_error_left  = error_left;
        prev_error_right = error_right;

        float output_left  = p_left + i_left + d_left;
        float output_right = p_right + i_right + d_right;

        // print out the result (debugging)
        // Serial.print("result_left: ");
        // Serial.print(output_left);
        // Serial.print("\t\t\tresult_right: ");
        // Serial.println(output_right);

        // do something with these outputs?
        float motor_left = map(output_left, 0, num_holes, 90, 100);
        float motor_right = map(output_right, 0, num_holes, 90, 100);

        // print out the result (debugging)
        Serial.print("result_left: ");
        Serial.print(motor_left);
        Serial.print("\t\t\tresult_right: ");
        Serial.println(motor_right);

        spin_motor(*left, motor_left);
        spin_motor(*right, motor_right);
    } while (left->encoder_count < num_holes && right->encoder_count < num_holes);
    stop_motor(*left);
    stop_motor(*right);
}

void turn(double degrees, motor *left, motor *right)
{
    // check this equation later
    int num_rotations    = (DIST_BETWEEN_WHEELS * degrees / 2) / (WHEEL_RADIUS * 360) * ENCODER_DISK_COUNT * 0.9;
    int speed            = 100;
    left->encoder_count  = 0;
    right->encoder_count = 0;

    spin_motor(*right, degrees > 0 ? -speed : speed);
    spin_motor(*left, degrees > 0 ? speed : -speed);

    // hold the program hostage until the turns complete
    while (abs(left->encoder_count) < abs(num_rotations) && abs(right->encoder_count) < abs(num_rotations))
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
    // cap the values at -100%, 100%
    speed_percentage = constrain(abs(speed_percentage), 0, 100);

    analogWrite(m.speed_pin, map(speed_percentage, 0, 100, 0, MOTOR_MAX));
    digitalWrite(m.backward_dir_pin, reversed);
    digitalWrite(m.forward_dir_pin, !reversed);
}
