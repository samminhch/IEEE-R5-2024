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
    int max_speed        = MOTOR_MAX * 0.8;  // 80% of max motor speed
    float num_holes  = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);
    int kP = 0.1, kI = 0, kD = 0;  // TODO set these values later
    float result_l, result_r;

    QuickPID pid_left((float *) &left->encoder_count, &result_l, &num_holes);
    QuickPID pid_right((float *) &right->encoder_count, &result_r, &num_holes);

    pid_left.SetTunings(kP, kI, kD);
    pid_right.SetTunings(kP, kI, kD);
    pid_left.SetOutputLimits(-max_speed, max_speed);
    pid_right.SetOutputLimits(-max_speed, max_speed);

    pid_left.SetMode(QuickPID::Control::automatic);
    pid_right.SetMode(QuickPID::Control::automatic);

    // spin_motor(motor m, int speed_percentage) -> speed_pl = "speed percentage for left motor"
    int speed_pl = map(result_l, 0, max_speed, 0, 100); // result_l always < max_speed
    int speed_pr = map(result_r, 0, max_speed, 0, 100); // result_r always < max_speed 
    do
    {
        if (pid_left.Compute())
        {
            spin_motor(*left, speed_pl);
        }
        if (pid_right.Compute())
        {
            spin_motor(*right, speed_pr);
        }

        char buffer[64];
        sprintf(buffer, "pid_left:%.2f,pid_right:%.2f\n", speed_pl, speed_pr);
    } while (left->encoder_count < num_holes && right->encoder_count < num_holes);
}

void turn(double degrees, motor *left, motor *right)
{
    int num_rotations    = (DIST_BETWEEN_WHEELS * degrees) / (WHEEL_RADIUS * 360);
    int speed            = 80;
    left->encoder_count  = 0;
    right->encoder_count = 0;

    spin_motor(*right, degrees > 0 ? -speed : speed);
    spin_motor(*left, degrees > 0 ? speed : -speed);

    // hold the program hostage until the turns complete
    while (left->encoder_count < num_rotations && right->encoder_count < num_rotations)
        ;
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
