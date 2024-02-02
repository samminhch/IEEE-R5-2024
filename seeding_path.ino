#include "robot-movement.h"
#define ROUND     2
#define STR_LEN   64

motor left_motor{5, 4, 3, 2};
motor right_motor{6, 7, 8, 9};
int index = 0;
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


void setup() {
  setup_motor(left_motor);
  setup_motor(right_motor);
}

void loop() {
  // put your main code here, to run repeatedly:
  turn(paths_seeding[index].angle_before, left_motor, right_motor);
  move(paths_seeding[index].distance, left_motor, right_motor);
  turn(paths_seeding[index].angle_after, left_motor, right_motor);
  index = (index + 1)
}
