#include "robot-movement.h"
#define STR_LEN 64

motor left_motor{5, 4, 3, 2};
motor right_motor{6, 7, 8, 9};

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

void loop()
{
    // set motor speed + direction
    spin_motor(left_motor, 80);
    spin_motor(right_motor, 80);

    // print to serial monitor
    char buffer[STR_LEN];
    sprintf_P(buffer, "left_count:%d,right_count:%d\n", left_motor.encoder_count, right_motor.encoder_count);

    delay(2500);
}

void update_left_encoder()
{
    pinMode(left_motor.encoder_pin, INPUT);
    left_motor.encoder_count += digitalRead(left_motor.forward_dir_pin) ? 1 : -1;
    pinMode(left_motor.encoder_pin, OUTPUT);
}

void update_right_encoder()
{
    pinMode(right_motor.encoder_pin, INPUT);
    right_motor.encoder_count += digitalRead(right_motor.forward_dir_pin) ? 1 : -1;
    pinMode(right_motor.encoder_pin, OUTPUT);
}
