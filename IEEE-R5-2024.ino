#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "robot-movement.h"

// comment this out when not connected to computer (i.e. actually competing)
// #define DEBUG
#ifdef DEBUG
    // comment out if you don't want to see debug prints on get_yaw()
    // #define GETYAW_DEBUG
    // comment out if you don't want to see debug prints on get_dist()
    // #define GETDIST_DEBUG
    // comment out if you don't want to see debug prints on move()
    // #define MOVE_DEBUG
    // comment out if you don't want to see debug prints on turn()
    #define TURN_DEBUG
    #define DPRINT(msg) Serial.print(msg);
    #define OK_PRINT(msg)           \
        Serial.print(F("[OKAY] ")); \
        Serial.print(F(msg));
    #define OK_PRINTLN(msg)         \
        Serial.print(F("[OKAY] ")); \
        Serial.println(F((msg)));
    #define ERR_PRINT(msg)           \
        Serial.print(F("[ERROR] ")); \
        Serial.print(F((msg)));
    #define ERR_PRINTLN(msg)         \
        Serial.print(F("[ERROR] ")); \
        Serial.println(F((msg)));
    #define DBG_PRINT(msg)           \
        Serial.print(F("[DEBUG] ")); \
        Serial.print(F((msg)));
    #define DBG_PRINTLN(msg)         \
        Serial.print(F("[DEBUG] ")); \
        Serial.println(F((msg)));
    #define WARN_PRINT(msg)         \
        Serial.print(F("[WARN] ")); \
        Serial.print(F((msg)));
    #define WARN_PRINTLN(msg)       \
        Serial.print(F("[WARN] ")); \
        Serial.println(F((msg)));
#endif

/**************
 * ULTRASONIC *
 **************/
struct ultrasonic
{
        const byte trig_pin;
        const byte echo_pin;
};

// can't store pins in PROGMEM, we're reading waaaay too quick for it to be useful
const ultrasonic front{10, 16};
// Sets the inches variable to the average of 5 calculated distances
// Returns true if distance was read successfully, false otherwise
bool get_dist(ultrasonic sensor, float &inches, uint8_t num_samples = 1, unsigned long timeout_millis = 100);

/***********
 * MPU6050 *
 ***********/
MPU6050 mpu;
bool mpu_connection_status;
bool mpu_calibrate = false;
// Sets the degrees variable to the calculated yaw.
// Returns true if angle was read successfully, false if timeout was reached
bool get_yaw(float &degrees, uint8_t num_samples = 1, unsigned long timeout_millis = 100);

/**********
 * MOTORS *
 **********/
// NOTE: left encoder is not hooked up to the robot
motor left_motor{5, 4, 6, 0};
motor right_motor{9, 8, 7, 2};

void update_left_encoder() { left_motor.encoder_count += digitalRead(left_motor.forward_dir_pin) ? 1 : -1; }

void update_right_encoder() { right_motor.encoder_count += digitalRead(right_motor.forward_dir_pin) ? 1 : -1; }

/**********
 * PATHS *
 **********/
struct path
{
        const float distance;
        const float angle_before;
        const float angle_after;
};

#define SEEDING
int seeding_index          = 0;
const path paths_seeding[] = {
    {4,  90, -90},
    {72, 0,  -90},
    {6,  0,  90 },
    {6,  0,  -90},
    {12, 0,  0  }
};

// #define ELIMS
#define ELIMS_PATH_LENGTH 8
int elimination_index          = 0;
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

void setup()
{

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400e3);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    Serial.begin(115200);
    delay(2000);

    // setup ultrasonic sensor
    pinMode(front.trig_pin, OUTPUT);
    pinMode(front.echo_pin, INPUT);
    digitalWrite(front.trig_pin, LOW);
#ifdef DEBUG
    OK_PRINTLN("Ultrasonic sensor pinmodes set");
#endif

    setup_motor(left_motor);
    setup_motor(right_motor);
    // attachInterrupt(digitalPinToInterrupt(left_motor.encoder_pin), update_left_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(right_motor.encoder_pin), update_right_encoder, RISING);

#ifdef DEBUG
    OK_PRINTLN("Motor pinmodes set");
#endif

    // setup mpu
    mpu.initialize();

    mpu_connection_status = mpu.testConnection();
#ifdef DEBUG
    DBG_PRINTLN("Testing MPU6050 connection...");

    if (mpu_connection_status)
    {
        OK_PRINTLN("MPU6050 connection successful");
    }
    else
    {
        ERR_PRINTLN("MPU6050 connection failed");
    }
#endif

    uint8_t dev_status = mpu.dmpInitialize();

    // offsets from calibration -- run the calibration program yourself to get offsets
    mpu.setXAccelOffset(-3642);
    mpu.setYAccelOffset(301);
    mpu.setZAccelOffset(5406);
    mpu.setXGyroOffset(42);
    mpu.setYGyroOffset(3);
    mpu.setZGyroOffset(-1);

    if (dev_status == 0)
    {
        if (mpu_calibrate)
        {
#ifdef DEBUG
            DBG_PRINTLN("Calibrating MPU's gyroscope and accelerometer...");
#endif
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
#ifdef DEBUG
            DPRINT("\n");
            DBG_PRINT("Offsets: ");
            mpu.PrintActiveOffsets();
#endif
        }
#ifdef DEBUG
        DBG_PRINTLN("Enabling MPU's DMP...");
#endif
        mpu.setDMPEnabled(true);
#ifdef DEBUG
        OK_PRINTLN("MPU's DMP enabled!");
#endif
    }
#ifdef DEBUG
    else
    {
        ERR_PRINT("DMP Initialization failed (code ");
        DPRINT(dev_status);
        DPRINT(")\n");
    }
#endif
    /******************
     * MOVEMENT TESTS *
     ******************/

    // straight line test
    // unsigned long start_time = micros();
    // int dist                 = 100;
    // move(dist);
    // float time_elapsed = (micros() - start_time) / 1000000.0;
    // Serial.print("robot travelled at a speed (in/s): ");
    // Serial.println(dist / time_elapsed);

    // turn test
    // for (int i = 0; i < 4; i++)
    // {
    //     turn(-90);
    //     delay(2500);
    // }

    // straight line and back test
    // move(12);
    // move(12);

    // square test
    // for (int i = 0; i < 4; i++) {
    // move(12);
    // turn(90);
    // }
}

void loop()
{
#ifdef DEBUG
    float distance, yaw;
    DBG_PRINT("");
    // if (get_dist(front, distance))
    // {
    //     DPRINT("Front Distance (inches): ")
    //     DPRINT(distance);
    //     DPRINT("\t");
    // }

    if (mpu_connection_status && get_yaw(yaw))
    {
        // it does drift a little, but it's no big deal I think, it drifts
        // less than an angle after a while
        DPRINT("Yaw (degrees): ");
        DPRINT(yaw);
    }
    DPRINT(F("\tRight Motor Encoder Count: "));
    DPRINT(right_motor.encoder_count);
    DPRINT(F("\n"));
#endif

#ifdef SEEDING
    turn(paths_seeding[seeding_index].angle_before);
    delay(500);
    move(paths_seeding[seeding_index].distance);
    delay(500);
    turn(paths_seeding[seeding_index].angle_after);
    seeding_index++;
    delay(500);
#elif ELIMS
    turn(paths_elimination[elimination_index].angle_before);
    delay(500);
    move(paths_elimination[elimination_index].distance);
    delay(500);
    turn(paths_elimination[elimination_index].angle_after);
    elimination_index = (elimination_index + 1) % ELIMS_PATH_LENGTH;
    delay(500);
#endif
}

bool get_yaw(float &degrees, uint8_t num_samples, unsigned long timeout_millis)
{
    float yaw_sum            = 0;
    uint8_t successful_reads = 0;
    unsigned long start_time = millis();

    while (successful_reads < num_samples)
    {
        uint8_t fifo_buffer[64];
        if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer) == 1)
        {
            successful_reads++;

            Quaternion q;
            mpu.dmpGetQuaternion(&q, fifo_buffer);
            yaw_sum += degrees(-atan2(2 * (q.w * q.z + -q.x * q.y), q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z));
        }
        else if ((millis() - start_time) >= timeout_millis)
        {
#ifdef GETYAW_DEBUG
            ERR_PRINTLN("Couldn't get yaw readings!");
#endif
            return false;
        }
    }

    degrees = yaw_sum / num_samples;
    return true;
}

// Sets the inches variable to the average of 5 calculated distances
// Returns true if distance was read successfully, false otherwise

bool get_dist(ultrasonic sensor, float &inches, uint8_t num_samples, unsigned long timeout_millis)
{
    float inches_sum         = 0;
    uint8_t successful_reads = 0;
    // unsigned long start_time = millis();

    while (successful_reads < num_samples)
    {
        // TODO find and handle errors when reading from ultrasonic sensor

        successful_reads++;
        // clear excess data
        digitalWrite(sensor.trig_pin, LOW);
        delayMicroseconds(5);

        digitalWrite(sensor.trig_pin, HIGH);
        delayMicroseconds(10);
        digitalWrite(sensor.trig_pin, LOW);

        unsigned long duration = pulseIn(sensor.echo_pin, HIGH);  // microseconds

        if (duration == 0)
        {
            return false;
        }

        // divide by 2, because we're measuring distance from when it bounces to
        // when it hits the sensor, and then by 74 in/microseconds (speed of sound)
        inches_sum += duration / 2.0 / 74;
    }
#ifdef GETDIST_DEBUG
    DBG_PRINT("Ultrasonic Distance Sum: ");
    DPRINT(inches_sum / num_samples);
    DPRINT("\n");
#endif
    inches = inches_sum / num_samples;
    return true;
}

// switches between multiple PID valeus based on certain conditions:
// 1) Makes sure that robot is a certain distance from the wall
// 2) Makes sure that the robot is actually going straight
// 3) Makes sure that the robot's wheels are spinning the same distance
void move(double inches)
{
    left_motor.encoder_count  = 0;
    right_motor.encoder_count = 0;
    float num_holes           = inches / (2 * PI * WHEEL_RADIUS / ENCODER_DISK_COUNT);

    float start_yaw, current_yaw;
    while (!get_yaw(start_yaw, 5))
        ;
    while (!get_yaw(current_yaw, 5))
        ;

    // error values
    float error       = start_yaw - current_yaw;
    float error_total = 0;
    float error_prev  = error;

    // by default, spin that @ 90%
    float base_speed  = 90;
    float left_speed  = base_speed;
    float right_speed = base_speed;

    // PID values
    float kP = 7.5;
    float kI = 0.05;
    float kD = 0.5;

    while (num_holes - abs(right_motor.encoder_count) > 0)
    {
        spin_motor(left_motor, left_speed);
        spin_motor(right_motor, right_speed);

        while (!get_yaw(current_yaw, 5))
            ;
#ifdef MOVE_DEBUG
        DBG_PRINT("Encoder Counts (target, left, right): ");
        DPRINT(num_holes);
        DPRINT(F(" "));
        DPRINT(left_motor.encoder_count);
        DPRINT(F(" "));
        DPRINT(right_motor.encoder_count);
        DPRINT(F("\t"));
        DPRINT(F("Yaw (target, current): "));
        DPRINT(start_yaw);
        DPRINT(F(" "));
        DPRINT(current_yaw);
        DPRINT(F("\n"));
#endif

        // update all of the error values
        error             = start_yaw - current_yaw;
        error_total      += error;
        float error_diff  = error - error_prev;
        error_prev        = error;

        float PID_output = kP * error + kI * error_total + kD * error_diff;

        left_speed  = base_speed - PID_output;
        right_speed = base_speed + PID_output;
    }
    stop_motor(left_motor);
    stop_motor(right_motor);
}

// + degrees = turn right, - degrees = turn left
void turn(float degrees)
{
    // determine turn direction
    bool turn_right = degrees > 0;

    // calculate the target degrees needed
    float yaw_threshold = 2;
    float yaw, target_yaw;
    while (!get_yaw(yaw, 10))
        ;

    float multiplier = (61. / 80) + (0.0071203704 * abs(degrees)) - (9. / 197027 * pow(abs(degrees), 2)) +
                       (1. / 9508695 * pow(abs(degrees), 3));
    #ifdef TURN_DEBUG
        DBG_PRINT("Turn multiplier: ");
        DPRINT(multiplier);
        DPRINT("\n");
    #endif
    degrees *= multiplier;

    target_yaw = yaw - degrees;

    // normalize degrees to be between -180, 180
    while (target_yaw > 180)
    {
        target_yaw -= 360;
    }
    while (target_yaw < -180)
    {
        target_yaw += 360;
    }

    float speed = 50;

    spin_motor(right_motor, turn_right ? -speed : speed);
    spin_motor(left_motor, turn_right ? speed : -speed);
    while (abs(target_yaw - yaw) > yaw_threshold)
    {
        while (!get_yaw(yaw))
            ;
#ifdef TURN_DEBUG
        DBG_PRINT("Yaw: ");
        DPRINT(yaw);
        DPRINT(F("\tTarget Yaw:"));
        DPRINT(target_yaw);
        DPRINT(F("\tabs. diff:"));
        DPRINT(abs(target_yaw - yaw));
        DPRINT(F("\n"));
#endif
    }
    stop_motor(left_motor);
    stop_motor(right_motor);
}
