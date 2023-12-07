#include "robot-movement.h"
#define STR_LEN   64
#define NUM_PATHS 8

struct path
{
        const double distance;
        const double angles[2];
};

const path paths[] = {
    {.distance = 101.76, .angles = {45, -135} }, // A ➡️ D
    {.distance = 107.28, .angles = {-27, -153}}, // D ➡️ H
    {.distance = 75.84,  .angles = {-71, -161}}, // H ➡️ F
    {.distance = 107.28, .angles = {-63, -153}}, // F ➡️ B
    {.distance = 101.76, .angles = {-45, 135} }, // B ➡️ G
    {.distance = 75.84,  .angles = {-18, -108}}, // G ➡️ E
    {.distance = 75.84,  .angles = {-18, -108}}, // E ➡️ C
    {.distance = 75.84,  .angles = {-18, -108}}, // C ➡️ A
};
int index = 0;

motor left_motor{5, 4, 3, 2};
motor right_motor{6, 7, 8, 9};

void setup()
{
    // motor setup
    setup_motor(left_motor);
    setup_motor(right_motor);
}

void loop()
{
    turn(paths[index].angles[0]);
    move(paths[index].distance, left_motor, right_motor);
    turn(paths[index].angles[1]);
    index = (index + 1) % NUM_PATHS;
}


