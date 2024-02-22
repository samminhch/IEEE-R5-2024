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

    // testing!!!
    delay(2500);
    unsigned long start_time = micros();
    int dist                 = 50;
    move(dist, &left_motor, &right_motor);
    float time_elapsed = (micros() - start_time) / 1000000.0;
    Serial.print("robot travelled at a speed (in/s): ");
    Serial.println(dist / time_elapsed);
    // turn(270, &left_motor, &right_motor);
    // spin_motor(left_motor, 100);
    // spin_motor(right_motor, 100);
    // delay(2500);
    // stop_motor(left_motor);
    // stop_motor(right_motor);
}

void loop()
{
    // we don't want anything to repeat atm
}

void update_left_encoder()
{
    left_motor.encoder_count += digitalRead(left_motor.forward_dir_pin) ? 1 : -1;

    // debugging
    // Serial.print("left_motor_encoder_count:" );
    // Serial.println(left_motor.encoder_count);
}

void update_right_encoder()
{
    right_motor.encoder_count += digitalRead(right_motor.forward_dir_pin) ? 1 : -1;

    // debugging
    // Serial.print("right_motor_encoder_count:");
    // Serial.print(right_motor.encoder_count);
    // Serial.print("\t\t");
}
