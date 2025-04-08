#include <Wire.h>
#include <Servo.h>
#include <math.h>

const int MPU_ADDR = 0x68;
int16_t ax, ay, az, gx, gy, gz;
Servo servoX, servoY;

float accel_x_offset = 0, accel_y_offset = 0;
float gyro_x_offset = 0, gyro_y_offset = 0;
int levelpos = 90;

const int SWEEP_ANGLE = 30;
const int NUM_SAMPLES = 150;
const int TIME_PERIOD = 1000;

const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;

const int FEEDBACK_X = A0;
const int FEEDBACK_Y = A1;
const int SERVO_X_PIN = 9;
const int SERVO_Y_PIN = 10;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    servoX.attach(SERVO_X_PIN);
    servoY.attach(SERVO_Y_PIN);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    servoX.write(90);
    servoY.write(90);

    calibrateIMU();

    Serial.println("Stage,Accel,Gyro,Servo_Angle,Actual_Angle");
}

void loop() {
    collectAndSweepServo("Y", servoY, "Y", FEEDBACK_Y);
}

void calibrateIMU() {
    servoX.write(levelpos);
    servoY.write(levelpos);
    const int CALIBRATION_SAMPLES = 200;
    float ax_sum = 0, ay_sum = 0, gx_sum = 0, gy_sum = 0;

    Serial.println("Calibrating IMU... Please keep the device still.");

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        readMPU6050();
        ax_sum += ax / ACCEL_SCALE;
        ay_sum += ay / ACCEL_SCALE;
        gx_sum += gx / GYRO_SCALE;
        gy_sum += gy / GYRO_SCALE;
        delay(10);
    }

    accel_x_offset = ax_sum / CALIBRATION_SAMPLES;
    accel_y_offset = ay_sum / CALIBRATION_SAMPLES;
    gyro_x_offset = gx_sum / CALIBRATION_SAMPLES;
    gyro_y_offset = gy_sum / CALIBRATION_SAMPLES;

    Serial.println("Calibration Complete!");
}

void readMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

float getServoPosition(int feedbackPin) {
    int rawValue = analogRead(feedbackPin);
    rawValue = map(rawValue, 77, 655, 0, 180);
    return rawValue;
}

void collectAndSweepServo(const char* axis, Servo &servo, const char* logAxis, int feedbackPin) {
    float progress = 0;
    int newPos = levelpos;

    // ---- Forward sweep ----
    for (int i = 0; i < NUM_SAMPLES; i++) {
        logMotion(i, NUM_SAMPLES, true, servo, logAxis, feedbackPin);
    }

    // ---- Hold at max position ----
    const int HOLD_SAMPLES = 50;
    int maxPos = levelpos + SWEEP_ANGLE;
    servo.write(maxPos);

    for (int i = 0; i < HOLD_SAMPLES; i++) {
        readMPU6050();

        float accel_x = (ax / ACCEL_SCALE) - accel_x_offset;
        float accel_y = (ay / ACCEL_SCALE) - accel_y_offset;
        float gyro_x = (gx / GYRO_SCALE) - gyro_x_offset;
        float gyro_y = (gy / GYRO_SCALE) - gyro_y_offset;

        float angle_x = atan2(accel_y, sqrt(accel_x * accel_x + (az / ACCEL_SCALE) * (az / ACCEL_SCALE))) * 180 / M_PI;
        float angle_y = atan2(-accel_x, sqrt(accel_y * accel_y + (az / ACCEL_SCALE) * (az / ACCEL_SCALE))) * 180 / M_PI;

        float actualPos = getServoPosition(feedbackPin);

        if (strcmp(logAxis, "X") == 0) {
            Serial.print("Stage 1_Hold,");
            Serial.print(angle_y, 4); Serial.print(",");
            Serial.print(gyro_y, 4); Serial.print(",");
            Serial.print(maxPos - 90); Serial.print(",");
            Serial.println(actualPos + 29 - 90);
        } else {
            Serial.print("Stage 2_Hold,");
            Serial.print(angle_x, 4); Serial.print(",");
            Serial.print(gyro_x, 4); Serial.print(",");
            Serial.print(maxPos - 90); Serial.print(",");
            Serial.println(actualPos - 76);
        }

        delay(20);
    }

    // ---- Return sweep ----
    for (int i = 0; i < NUM_SAMPLES; i++) {
        logMotion(i, NUM_SAMPLES, false, servo, logAxis, feedbackPin);
    }

    // ---- Idle logging ----
    const int IDLE_SAMPLES = 100;
    servo.write(levelpos);

    for (int i = 0; i < IDLE_SAMPLES; i++) {
        readMPU6050();

        float accel_x = (ax / ACCEL_SCALE) - accel_x_offset;
        float accel_y = (ay / ACCEL_SCALE) - accel_y_offset;
        float gyro_x = (gx / GYRO_SCALE) - gyro_x_offset;
        float gyro_y = (gy / GYRO_SCALE) - gyro_y_offset;

        float angle_x = atan2(accel_y, sqrt(accel_x * accel_x + (az / ACCEL_SCALE) * (az / ACCEL_SCALE))) * 180 / M_PI;
        float angle_y = atan2(-accel_x, sqrt(accel_y * accel_y + (az / ACCEL_SCALE) * (az / ACCEL_SCALE))) * 180 / M_PI;

        float actualPos = getServoPosition(feedbackPin);

        if (strcmp(logAxis, "X") == 0) {
            Serial.print("Stage 1_Idle,");
            Serial.print(angle_y, 4); Serial.print(",");
            Serial.print(gyro_y, 4); Serial.print(",");
            Serial.print(levelpos - 90); Serial.print(",");
            Serial.println(actualPos + 29 - 90);
        } else {
            Serial.print("Stage 2_Idle,");
            Serial.print(angle_x, 4); Serial.print(",");
            Serial.print(gyro_x, 4); Serial.print(",");
            Serial.print(levelpos - 90); Serial.print(",");
            Serial.println(actualPos - 76);
        }

        delay(20);
    }
}

void logMotion(int i, int total, bool forward, Servo &servo, const char* logAxis, int feedbackPin) {
    readMPU6050();

    float accel_x = (ax / ACCEL_SCALE) - accel_x_offset;
    float accel_y = (ay / ACCEL_SCALE) - accel_y_offset;
    float gyro_x = (gx / GYRO_SCALE) - gyro_x_offset;
    float gyro_y = (gy / GYRO_SCALE) - gyro_y_offset;

    float angle_x = atan2(accel_y, sqrt(accel_x * accel_x + (az / ACCEL_SCALE) * (az / ACCEL_SCALE))) * 180 / M_PI;
    float angle_y = atan2(-accel_x, sqrt(accel_y * accel_y + (az / ACCEL_SCALE) * (az / ACCEL_SCALE))) * 180 / M_PI;

    float progress = pow((float)i / (total - 1), 3);
    int newPos = forward ?
        levelpos + (SWEEP_ANGLE * progress) :
        levelpos + (SWEEP_ANGLE * (1 - progress));

    servo.write(newPos);
    float actualPos = getServoPosition(feedbackPin);

    const char* stageLabel = forward ? (strcmp(logAxis, "X") == 0 ? "Stage 1" : "Stage 2") 
                                     : (strcmp(logAxis, "X") == 0 ? "Stage 1_Return" : "Stage 2_Return");

    Serial.print(stageLabel); Serial.print(",");
    if (strcmp(logAxis, "X") == 0) {
        Serial.print(angle_y, 4); Serial.print(",");
        Serial.print(gyro_y, 4); Serial.print(",");
        Serial.print(newPos - 90); Serial.print(",");
        Serial.println(actualPos + 29 - 90);
    } else {
        Serial.print(angle_x, 4); Serial.print(",");
        Serial.print(gyro_x, 4); Serial.print(",");
        Serial.print(newPos - 90); Serial.print(",");
        Serial.println(actualPos - 76);
    }

    int delayTime = TIME_PERIOD * (forward ? (1 - progress) : progress) / total;
    delay(max(delayTime, 10));
}
