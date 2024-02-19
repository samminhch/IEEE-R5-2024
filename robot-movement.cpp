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
// Rewriting the PID control function from 2/8 meeting -MH

void move(double inches, motor *left, motor *right)
{
    left->encoder_count     = 0;
    right->encoder_count    = 0;
    float setpoint         = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);
    int actual_left          = left->encoder_count;
    int actual_right         = right->encoder_count;
    unsigned long prev_time = 0;
    float output_left = 0.0;
    float output_right = 0.0;
    float dt = 0.0;
    float integral_left = 0.0;
    float integral_right = 0.0;
    float previous_left = 0.0;
    float previous_right = 0.0;

    // PID constants
    float kP = 0.8;
    float kI = 0.20;
    float kD = 0.001;

    // Start spinning motors
    spin_motor(*left, 100);
    spin_motor(*right, 100);
    
    do
    {
        unsigned long now      = millis();
        dt = now - prev_time / 1000.0; // time in SEC
        prev_time = now;
        
        actual_left = left->encoder_count;
        actual_right = right->encoder_count;
        
        float error_left = setpoint - actual_left;
        float error_right = setpoint - actual_right;
        
        float proportional_left = error_left;
        float proportional_right = error_right;
        
        integral_left = error_left * dt;
        integral_right = error_right * dt;
        
        float derivative_left = (error_left - previous_left) / dt;
        float derivative_right = (error_right - previous_right) / dt;
        
        previous_left = error_left;
        previous_right = error_right;
        
        output_left = (kP * proportional_left) + (kI * integral_left) + (kD * derivative_left);
        output_right = (kP * proportional_right) + (kI * integral_right) + (kD * derivative_right);
        
        // Mapping PID results to motor speed percentages
        float motor_left = map(output_left, 0, setpoint, 90, 100);
        float motor_right = map(output_right, 0, setpoint, 90, 100);

        // print out the motor speed results (debugging)
        Serial.print("result_left: ");
        Serial.print(motor_left);
        Serial.print("\t\t\tresult_right: ");
        Serial.println(motor_right);

        spin_motor(*left, motor_left);
        spin_motor(*right, motor_right);
        

    }while (left->encoder_count < setpoint && right->encoder_count < setpoint);
    
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
