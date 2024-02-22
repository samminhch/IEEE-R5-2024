#include "robot-movement.h"
#define STR_LEN 64

motor left_motor{7, 8, 9, 3};
motor right_motor{4, 5, 6 , 2};

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
