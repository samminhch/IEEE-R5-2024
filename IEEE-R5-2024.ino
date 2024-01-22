#include "robot-movement.h"
#define ROUND     1
#define STR_LEN   64
#define NUM_PATHS 8

struct path
{
        const double distance;
        const double angle_before;
        const double angle_after;
};

const path paths_seeding[] = {
    // TODO get them numbers to put here
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

motor left_motor{5, 4, 3, 2};
motor right_motor{6, 7, 8, 9};

void setup()
{
    setup_motor(left_motor);
    setup_motor(right_motor);
}

void loop()
{
    turn(paths_elimination[index].angle_before, left_motor, right_motor);
    move(paths_elimination[index].distance, left_motor, right_motor);
    turn(paths_elimination[index].angle_after, left_motor, right_motor);
    index = (index + 1) % NUM_PATHS;
}
