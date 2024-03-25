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
    #define MOVE_DEBUG
// comment out if you don't want to see debug prints on turn()
// #define TURN_DEBUG  ;
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

#define ROUND     1
#define STR_LEN   64
#define NUM_PATHS 8

/**************
 * ULTRASONIC *
 **************/
struct ultrasonic
{
        const byte trig_pin;
        const byte echo_pin;
};

// can't store pins in PROGMEM, we're reading waaaay too quick for it to be useful
const ultrasonic side{14, 15};
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

int seeding_index                  = 0;
const PROGMEM path paths_seeding[] = {
    {6,  90, -90},
    {72, 0,  -90},
    {6,  0,  90 },
    {6,  0,  -90},
    {12, 0,  0  }
};

int elimination_index                  = 0;
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

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400e3);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

#ifdef DEBUG
    Serial.begin(115200);
    delay(2000);
#endif

    setup_motor(left_motor);
    setup_motor(right_motor);

#ifdef DEBUG
    OK_PRINTLN("Motor pinmodes set");
#endif

    // setup ultrasonic sensor
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);

#ifdef DEBUG
    OK_PRINTLN("Ultrasonic sensor pins set");
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
    mpu.setXAccelOffset(-3484);
    mpu.setYAccelOffset(279);
    mpu.setZAccelOffset(1458);
    mpu.setXGyroOffset(57);
    mpu.setYGyroOffset(-8);
    mpu.setZGyroOffset(5);

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
    }
#ifdef DEBUG
    else
    {
        ERR_PRINT("DMP Initialization failed (code ");
        DPRINT(dev_status);
        DPRINT(")\n");
    }
#endif

    pinMode(LED_BUILTIN, OUTPUT);

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
    // for (int i = 0; i < 4; i++) {
    //     turn(90);
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

unsigned long prev_time = millis();
bool blink_status       = true;

void loop()
{
    // end program if it can't connect to MPU
    if (!mpu_connection_status)
    {
#ifdef DEBUG
        ERR_PRINTLN("MPU connection failed... ending program");
#endif
        return;
    }

    // going through the paths
    // turn(paths_elimination[elimination_index].angle_before);
    // move(paths_elimination[elimination_index].distance);
    // turn(paths_elimination[elimination_index].angle_after);
    // elimination_index = (elimination_index + 1) % NUM_PATHS;

    // printing out values in debug mode!
#ifdef DEBUG
    float yaw, distance;
    if (get_dist(distance))
    {
        DBG_PRINT("Distance (inches):")
        DPRINT(distance);
        DPRINT("\t");
    }

    if (get_yaw(yaw))
    {
        // it does drift a little, but it's no big deal I think, it drifts
        // less than an angle after a while
        DBG_PRINT("Yaw (degrees):\t");
        DPRINT(yaw);
        DPRINT("\n");
    }
#endif

    // blink every 500 milliseconds -- it's a lifeline!
    unsigned long current_time = millis();
    if (current_time - prev_time >= 500)
    {
        prev_time = current_time;
        digitalWrite(LED_BUILTIN, blink_status);
        blink_status = !blink_status;
    }
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
bool get_dist(float &inches, uint8_t num_samples, unsigned long timeout_millis)
{
    float inches_sum         = 0;
    uint8_t successful_reads = 0;
    // unsigned long start_time = millis();

    while (successful_reads < num_samples)
    {
        // TODO find and handle errors when reading from ultrasonic sensor

        successful_reads++;
        // clear excess data
        digitalWrite(trigPin, LOW);
        delayMicroseconds(5);

        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        unsigned long duration = pulseIn(echoPin, HIGH);  // microseconds

        // divide by 2, because we're measuring distance from when it bounces to
        // when it hits the sensor, and then by 74 in/microseconds (speed of sound)
        inches_sum += duration / 2.0 / 74;
    }

    inches = inches_sum / num_samples;
    return true;
}

// This function assumes that the ultrasonic sensor is mounted to look forward
// and to a wall. It calculates distance by subtracting the ultrasonic sensor's
// distance from the distance found by the wall.
void move(float inches)
{
    // degrees will be used for PID to keep wheels spinning straight
    float distance, start_yaw, yaw, threshold = 2;

    while (!get_dist(distance))
        ;
    while (!get_yaw(start_yaw, 5))
        ;

    // validating given distance
    float distance_to_travel = distance - inches;
    if (distance_to_travel <= threshold)
    {
#ifdef MOVE_DEBUG
        ERR_PRINTLN("Given distance would result in robot bumping into wall... stopping command!");
#endif
        return;
    }

    // PID values
    const float kP = 10;
    const float kI = 0.005;
    const float kD = 0;

    while (!get_yaw(yaw, 5))
        ;
    float error       = start_yaw - yaw;
    float error_prev  = error;
    float error_total = 0;

    const float base_speed = 90;
    while (distance - inches > threshold)
    {
        // getting the average of 5 measurements
        while (!get_yaw(yaw, 5))
            ;
        error             = start_yaw - yaw;
        error_total      += error;
        float error_diff  = error - error_prev;

#ifdef MOVE_DEBUG
        DBG_PRINT("Error=")
        DPRINT(error);
#endif

        // this will need *a lot* of adjusting and testing!
        float pid_output = kP * error + kI * error_total + kD * error_diff;
        spin_motor(left_motor, base_speed + pid_output);
        spin_motor(right_motor, base_speed - pid_output);
    }
    stop_motor(left_motor);
    stop_motor(right_motor);
}

void turn(float degrees)
{
    // calculate the target degrees needed
    float yaw_threshold = 0.1;
    float yaw, target_yaw;
    while (!get_yaw(yaw, 10))
        ;
    target_yaw = yaw + degrees;

    float speed = 100;
    spin_motor(right_motor, degrees > 0 ? -speed : speed);
    spin_motor(left_motor, degrees > 0 ? -speed : speed);

    while (abs(target_yaw - yaw) > yaw_threshold)
    {

        while (!get_yaw(yaw, 10))
            ;
#ifdef TURN_DEBUG
        DBG_PRINT("Yaw: ");
        DPRINT(yaw);
        DPRINT(F("\tTarget Yaw:"));
        DPRINT(target_yaw);
        DPRINT(F("\n"));
#endif
    }
}
