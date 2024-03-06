#include "robot-movement.h"
#define ROUND     1
#define STR_LEN   64
#define NUM_PATHS 8

/**************
 * ULTRASONIC *
 **************/
const byte trigPin = 14;
const byte echoPin = 15;

/**********
 * MOTORS *
 **********/
const PROGMEM motor left_motor{9, 8, 7};
const PROGMEM motor right_motor{5, 4, 6};

/**********
 * PATHS *
 **********/
struct path
{
        const float distance;
        const float angle_before;
        const float angle_after;
};

int seeding_index     = 0;
const PROGMEM path paths_seeding[] = {
    {6,  90, -90},
    {72, 0,  -90},
    {6,  0,  90 },
    {6,  0,  -90},
    {12, 0,  0  }
};

int elimination_index = 0;
const PROGMEM path paths_elimination[] = {
    {101.76, 45,  -135}, // A ➡️ D
    {107.28, -27, -153}, // D ➡️ H
    {75.84,  -71, -161}, // H ➡️ F
    {107.28, -63, -153}, // F ➡️ B
    {101.76, -45, 135 }, // B ➡️ G
    {75.84,  -18, -108}, // G ➡️ E
    {75.84,  -18, -108}, // E ➡️ C
    {75.84,  -18, -108}, // C ➡️ A
};

void setup()
{
    setup_motor(left_motor);
    setup_motor(right_motor);
    // setup ultrasonic sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);

    /******************
     * MOVEMENT TESTS *
     ******************/

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

unsigned long prev_time = millis();

bool blink_status = true;

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
    // blink every 500 milliseconds -- it's a lifeline!
    unsigned long current_time = millis();
    if (current_time - prev_time >= 500)
    {
        prev_time = current_time;
        digitalWrite(LED_BUILTIN, blink_status);
    }
}

{

}

{

}
