Agent Jake21
agentjake21
Online
IEEE R5 Software and Sensors Team

beef — 01/17/2024 5:37 PM
as i mentioned in the google doc, i will be dividing the work according to who filled out the when2meet.
(also, i have been mostly posting in this chat (rather than in the software-sensors channel) bc i realized that some ppl lost access to that channel :( not sure why)
frog with a gun — 01/18/2024 9:28 AM
I actually like that a lot :D
frog with a gun — 01/18/2024 9:28 AM
is that something that the discord admins can fix on the server?
beef — 01/18/2024 9:36 AM
yea i can ask allison to see if she can fix it
frog with a gun — 01/18/2024 9:36 AM
awesome sauce :D
beef — 01/19/2024 8:31 AM
hi y'all! just to clarify: no meeting today (software/sensors meetings will start this monday). have a great weekend :D
beef — 01/21/2024 3:54 PM
happy sunday, y'all :) tomorrow 4-5pm is the first subteam meeting, but I will need to be in the meeting with Dr. Skinner during that time. so I have made your assignments for this week available in the "Spring Assignments" doc in our shared sub-team Google Drive (lemme know if you cannot find it).
frog with a gun — 01/22/2024 12:48 PM
Hey guys I'll be a little late to the meeting since I have a pretext to take at 3pm today ;-;
beef — 01/22/2024 3:17 PM
no worries!
frog with a gun — 01/22/2024 3:22 PM
Nevermind I'll make it in time yay
beef — 01/26/2024 11:00 AM
hi y'all! we have another subteam meeting today at 5:00 PM in the IEEE Room - the agenda/assignments for this meeting are in the spring assignments doc under Jan. 26. https://docs.google.com/document/d/1VZybefnta5nNxNBPl3evX8Eai-nwpTNivH_FUqN6QhA/edit?usp=sharing
Google Docs
Spring Assignments
Image
beef — 01/29/2024 2:03 PM
hi y'all! we have another subteam meeting today at 4:00 PM in the IEEE Room.
frog with a gun — 01/29/2024 4:16 PM
Sorry I'm coming a little late!!!
beef — 01/29/2024 4:17 PM
do you have the cable to connect the arduino micro to a laptop? if so, could you bring it over pls?
frog with a gun — 01/29/2024 4:20 PM
Unfortunately I do not ;-;
Is there not a micro USB cable lying around the lab?
I'll be over in 15ish minutes
Han Yildirim — 01/31/2024 3:57 PM
Hey guys, let’s push the meeting today to tmr so we can work with Kevin to make sure the robot works fully before testing.
beef — 01/31/2024 4:00 PM
o there was supposed to be a meeting today?
beef — 02/01/2024 11:09 AM
hey guys, i won’t be able to come to the robotics meeting tonight but i left y’all your assignments for today in the spring assignments doc.
frog with a gun — 02/01/2024 11:16 AM
Take care Mandy :D
Agent Jake21 — 02/01/2024 11:16 AM
Take care
Han Yildirim — 02/01/2024 7:16 PM
on my way
Han Yildirim — 02/05/2024 10:33 AM
Hey guys, just a reminder that theres a meeting today at 4:30 so we can test the robot’s moving distance accuracy
beef — 02/08/2024 5:34 PM
Reminder that we have a meeting tonight at 7:00 PM in the IEEE room!
beef — 02/09/2024 1:05 PM
hey guys, unfortunately i am not able to meet today but i will send assignments soon!
frog with a gun — 02/09/2024 1:08 PM
Oh I uploaded my PID code changes to the basic_encoder branch! Feel free to mess around with those values, or come up with your own unique solution to make the robot move straight!
Agent Jake21 — 02/09/2024 3:43 PM
Won’t be able to make it today so  @sinister_dude can we meet tomorrow
sinister_dude — 02/09/2024 3:57 PM
Can we do it online on discord?
Agent Jake21 — 02/09/2024 3:59 PM
Sure
Han Yildirim — 02/12/2024 3:56 PM
Just a reminder we have a meeting today, the goal is for Minh to work on the PID while we have a couple others to get a start on the ultrasound
I’ll make it there around 5
frog with a gun — 02/12/2024 4:10 PM
Hey, I won't be able to make it to any meetings this week, but I did manage to make the robot move straight (?) with my PID implementation. I encourage you all to mess with the code, as it's pushed on the basic_encoders branch of the repository.
beef — 02/15/2024 2:40 PM
Hi y'all, just a reminder for those who are able to come that we have a meeting tonight at 7:00 PM. we'll meet at the IEEE room first, then head off to con lab.
beef
 removed 
firebolt140
 from the group.
 — 02/15/2024 2:40 PM
beef — 02/15/2024 2:43 PM
i updated our gantt chart to reflect the timeline that we came up with last thursday. i also wrote another version of the pid ctrl (move() function) just in case we need another one once we test tonight. for those who are working on the wall-follower version of the move() function, continue! we need all the alternatives here haha.
beef
 removed 
Dominicc
 from the group.
 — 02/15/2024 2:47 PM
Agent Jake21 — 02/16/2024 4:17 PM
Hey @sinister_dude  r u coming to the robotics meeting
sinister_dude — 02/16/2024 4:25 PM
Sorry I don't think I'll be able to make it
frog with a gun — 02/19/2024 4:20 PM
There's a meeting today right?
sinister_dude — 02/19/2024 4:21 PM
I went to the room and Mandy and Kevin were in a meeting with someone and pretty much every chair was taken, so I'm just working on the code in my apartment
frog with a gun — 02/19/2024 4:21 PM
Oh
I just waiting outside because I noticed they were in a meeting lol
Han Yildirim — 02/19/2024 4:24 PM
i just got here, there’s a couple seats so i’ll be working on the code here
frog with a gun — 02/19/2024 4:24 PM
Wait lemme innn
Han Yildirim — 02/19/2024 4:24 PM
u can come if u want man
sinister_dude — 02/19/2024 4:24 PM
Aight I'll be there in 5 mins
sinister_dude — 02/19/2024 5:10 PM
#include "robot-movement.h"
#define STR_LEN 64

motor left_motor{7, 8, 9, 3};
motor right_motor{4, 5, 6 , 2};

Expand
IEEE-R5-2024.ino
3 KB
frog with a gun — 02/19/2024 5:48 PM
Quick question, does anyone know what the arduino micro is trying to tell us when it's power indicator light is blinking rapidly when running a program?
﻿
#include "robot-movement.h"
#define STR_LEN 64

motor left_motor{7, 8, 9, 3};
motor right_motor{4, 5, 6 , 2};

const int trigPin = 14;  
const int echoPin = 15; 
float duration, distance; 

const int pot_left  = A2;
const int pot_right = A1;

void setup()
{
    // motor setup
    setup_motor(left_motor);
    setup_motor(right_motor);

    attachInterrupt(digitalPinToInterrupt(left_motor.encoder_pin), update_left_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(right_motor.encoder_pin), update_right_encoder, RISING);

    pinMode(trigPin, OUTPUT); 
    pinMode(echoPin, INPUT);  
    digitalWrite(trigPin, LOW);  

    Serial.begin(9600); 
}

void loop() {
    // we don't want anything to repeat atm
    // pull distance values and do spin_motor
    do {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);  
    digitalWrite(trigPin, HIGH);  
    delayMicroseconds(10);  
    digitalWrite(trigPin, LOW);  
    duration = pulseIn(echoPin, HIGH);  
    distance = (duration*.0343)/2;
    Serial.println(distance);
    if (distance>=20 && distance <=30) {
      spin_motor(left_motor,100);
      spin_motor(right_motor,100);
      delay(200);
    }
    } while (distance>=20 && distance <=30);
    
    stop_motor(right_motor);
    stop_motor(left_motor);
   /* stop_motor(left_motor);
    spin_motor(right_motor,100);
    delay(2000);
    spin_motor(right_motor,100);
    spin_motor(left_motor,100);
    delay(2000);
    stop_motor(right_motor);
    stop_motor(left_motor); */
}
void update_left_encoder()
{
    pinMode(left_motor.encoder_pin, INPUT);
    left_motor.encoder_count += digitalRead(left_motor.forward_dir_pin) ? 1 : -1;
    pinMode(left_motor.encoder_pin, OUTPUT);

    Serial.print("left_motor_encoder_count:" );
    Serial.println(left_motor.encoder_count);
}

void update_right_encoder()
{
    pinMode(right_motor.encoder_pin, INPUT); right_motor.encoder_count += digitalRead(right_motor.forward_dir_pin) ? 1 : -1;
    pinMode(right_motor.encoder_pin, OUTPUT);

    Serial.print("right_motor_encoder_count:");
    Serial.print(right_motor.encoder_count);
    Serial.print("\t\t");
}
IEEE-R5-2024.ino
3 KB
