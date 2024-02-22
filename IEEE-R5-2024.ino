#include "robot-movement.h"
#define ROUND     1
#define STR_LEN   64
#define NUM_PATHS 8

motor left_motor{6, 5, 4, 2};
motor right_motor{9, 8, 7, 3};
struct path
{
        const double distance;
        const double angle_before;
        const double angle_after;
};

const path paths_seeding[] = {
    {6,90,-90},
    {72,0,-90},
    {6,0,90},
    {6,0,-90},
    {12,0,0}
};

const path paths_elimination[] = {
    {101.76, 45,  -135}, // A ➡️ D
    {107.28, -27, -153}, // D ➡️ H
    {75.84,  -71, -161}, // H ➡️ F
    {107.28, -63, -153}, // F ➡️ B
    {101.76, -45, 135 }, // B ➡️ G
    {75.84,  -18, -108}, // G ➡️ E
    {75.84,  -18, -108}, // E ➡️ C
    {75.84,  -18, -108}, // C ➡️ A
};
int index = 0;

void setup()
{
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
    turn(paths_elimination[index].angle_before, &left_motor, &right_motor);
    move(paths_elimination[index].distance, &left_motor, &right_motor);
    turn(paths_elimination[index].angle_after, &left_motor, &right_motor);
    index = (index + 1) % NUM_PATHS;
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
