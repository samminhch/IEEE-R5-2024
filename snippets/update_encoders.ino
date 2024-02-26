#include "robot-movement.h"
#define STR_MAX 80

// determine optical encoder sensor pins later
motor left_motor{11, 10, 9};
motor right_motor{6, 7, 8};

void setup()
{
    // motor setup
    setup_motor(left_motor);
    setup_motor(right_motor);

    attachInterrupt(left_motor.encoder_pin, update_left_encoder, RISING);
    attachInterrupt(left_motor.encoder_pin, update_right_encoder, RISING);

    Serial.begin(9600);
}

void loop()
{
    char buffer[STR_MAX];
    sprintf(buffer, "left_encoder_count:%d,right_encoder_count:%d\n",
            left_motor.encoder_count, right_motor.encoder_count);
}

void update_left_encoder()
{
    left_motor.encoder_count +=
        digitalRead(left_motor.forward_dir_pin) ? 1 : -1;
}

void update_right_encoder()
{
    right_motor.encoder_count +=
        digitalRead(right_motor.forward_dir_pin) ? 1 : -1;
}
