#include "robot-movement.h"
#define STR_LEN 64

const motor left_motor{11, 10, 9};
const motor right_motor{6, 7, 8};

const int pot_left  = A2;
const int pot_right = A1;

void setup() {
    // motor setup
    setup_motor(left_motor);
    setup_motor(right_motor);

    Serial.begin(9600);
}

void loop() {
    int pot_left_val  = analogRead(pot_left);
    int pot_right_val = analogRead(pot_right);

    int left_speed  = map(pot_left_val, 0, 1023, MOTOR_MIN, MOTOR_MAX);
    int right_speed = map(pot_right_val, 0, 1023, MOTOR_MIN, MOTOR_MAX);

    // set motor speed + direction
    spin_motor(left_motor, left_speed);
    spin_motor(right_motor, right_speed);

    // print to serial monitor
    char buffer[STR_LEN];
    sprintf(buffer, "Left_Motor_Speed:%d,Right_Motor_Speed:%d",
            left_speed, right_speed);
    Serial.println(buffer);
}
