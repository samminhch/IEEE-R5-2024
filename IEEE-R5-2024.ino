#include "robot-movement.h"
#define STR_LEN 64

motor left_motor{6, 5, 4, 2};
motor right_motor{9, 8, 7, 3};

const int pot_left  = A2;
const int pot_right = A1;

void setup()
{
    // motor setup
    setup_motor(left_motor);
    setup_motor(right_motor);

    attachInterrupt(digitalPinToInterrupt(left_motor.encoder_pin), update_left_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(right_motor.encoder_pin), update_right_encoder, RISING);

    Serial.begin(9600);
}

// enable or disable the tests that you want
void loop()
{
    // testing turns
    turn(90, left_motor, right_motor);
    delay(1000);
    turn(-90, left_motor, right_motor);
    delay(1000);
    turn(180, left_motor, right_motor);
    delay(1000);

    // testing movement
    unsigned long start_time = micros();
    move(12, left_motor, right_motor)
    unsigned long time_elapsed = micros() - start_time;
    Serial.print("robot travelled at a speed (in/s): ");
    Serial.println(12 / time_elapsed * 1000000);
    turn(180, left_motor, right_motor);
    move(12, left_motor, right_motor)
}

void update_left_encoder()
{
    pinMode(left_motor.encoder_pin, INPUT);
    left_motor.encoder_count += digitalRead(left_motor.forward_dir_pin) ? 1 : -1;
    pinMode(left_motor.encoder_pin, OUTPUT);

    Serial.print("left_motor_encoder_count:" );
    Serial.println(left_motor.encoder_count);
}

void update_right_encoder()
{
    pinMode(right_motor.encoder_pin, INPUT);
    right_motor.encoder_count += digitalRead(right_motor.forward_dir_pin) ? 1 : -1;
    pinMode(right_motor.encoder_pin, OUTPUT);

    Serial.print("right_motor_encoder_count:");
    Serial.println(right_motor.encoder_count);
}
