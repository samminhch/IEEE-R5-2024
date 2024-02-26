#include "robot-movement.h"
#define ROUND     1
#define STR_LEN   64
#define NUM_PATHS 8

// ultrasonic sensor pins
const int trigPin = 14;
const int echoPin = 15;

// motor pins
motor left_motor{9, 8, 7, 3};
motor right_motor{5, 4, 6, 2};
struct path
{
        const double distance;
        const double angle_before;
        const double angle_after;
};

const path paths_seeding[] = {
    {6,  90, -90},
    {72, 0,  -90},
    {6,  0,  90 },
    {6,  0,  -90},
    {12, 0,  0  }
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
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);

    Serial.begin(9600);

    // straight line test
    // unsigned long start_time = micros();
    // int dist                 = 100;
    // move(dist, &left_motor, &right_motor);
    // float time_elapsed = (micros() - start_time) / 1000000.0;
    // Serial.print("robot travelled at a speed (in/s): ");
    // Serial.println(dist / time_elapsed);
    
    // turn test
    // for (int i = 0; i < 4; i++) {
    //     turn(90, &left_motor, &right_motor);
    //     delay(2500);
    // }

    // straight line and back test
    // move(12, &left_motor, &right_motor);
    // move(12, &left_motor, &right_motor);


    // square test
    // for (int i = 0; i < 4; i++) {
        // move(12, &left_motor, &right_motor);
        // turn(90, &left_motor, &right_motor);
    // }
}

void loop()
{
    // going through the paths
    // turn(paths_elimination[index].angle_before, &left_motor, &right_motor);
    // move(paths_elimination[index].distance, &left_motor, &right_motor);
    // turn(paths_elimination[index].angle_after, &left_motor, &right_motor);
    // index = (index + 1) % NUM_PATHS;

    // getting dist
    // digitalWrite(trigPin, LOW);
    // delayMicroseconds(5);
    // digitalWrite(trigPin, HIGH);
    // delayMicroseconds(10);
    // digitalWrite(trigPin, LOW);
    //
    // unsigned long duration = pulseIn(echoPin, HIGH); // microseconds
    // float inches = duration / 2.0 / 74;
    // Serial.println("dist:");
    // Serial.println(inches);
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
