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

double get_distance(motor m) { return (float) m.encoder_count / ENCODER_DISK_COUNT * 2 * PI * WHEEL_RADIUS; }

// i need to test if this code works somehow
void move(double inches, motor left, motor right)
{
    int kP, kI, kD;
    int speed      = MOTOR_MAX * 0.8;  // 80% of max motor speed
    long prev_time = micros();

    double error_l, error_r, prev_error_l, prev_error_r, error_sum_l, error_sum_r;

    do
    {
        error_l = inches - get_distance(left);
        error_r = inches - get_distance(right);

        error_sum_l += error_l;
        error_sum_r += error_r;

        double time_elapsed = micros() - prev_time;
        prev_time           = time_elapsed;

        double proportional_l = kP * error_l * time_elapsed;
        double proportional_r = kP * error_r * time_elapsed;

        double integral_l = kI * error_sum_l * time_elapsed;
        double integral_r = kI * error_sum_r * time_elapsed;

        double derivative_l = kD * (error_l - prev_error_l) / time_elapsed;
        double derivative_r = kD * (error_r - prev_error_r) / time_elapsed;

        float result_l = proportional_l + integral_l + derivative_l;
        float result_r = proportional_r + integral_r + derivative_r;

        spin_motor(left, result_l);
        spin_motor(right, result_r);
        
        char buffer[64];
        sprintf(buffer, "pid_left:%.2f,pid_right:%.2f\n", result_l, result_r);
    } while (abs(error_sum_l) < 0.1 && abs(error_sum_r) < 0.1);
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
    if (speed_percentage < -100)
    {
        speed_percentage = -100;
    }
    else if (speed_percentage > 100)
    {
        speed_percentage = 100;
    }

    analogWrite(m.speed_pin, map(abs(speed_percentage), 0, 100, 0, MOTOR_MAX));
    digitalWrite(m.backward_dir_pin, reversed);
    digitalWrite(m.forward_dir_pin, !reversed);
}
