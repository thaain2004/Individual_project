#include <Wire.h>
#include <Servo.h>
#include "KalmanFilter.h"

#define MPU6050_ADDR 0x68
#define ACCEL_SCALE 16384.0 // Accelerometer sensitivity scale factor (for ±2g)
#define GYRO_SCALE 131.0    // Gyroscope sensitivity scale factor (for ±250°/s)

// Create Kalman filter instances
KalmanFilter kalmanPitch(0.0005, 0.02, 0.8, 0);
KalmanFilter kalmanRoll(0.0005, 0.02, 0.8, 0);


// Servo objects for gimbal
Servo servoPitch;
Servo servoRoll;

float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
float pitch, roll, gyroPitchRate, gyroRollRate;

void setup() {
    Wire.begin();
    Wire.setClock(400000);  // Faster MPU6050 communication

    Serial.begin(9600);




    // Attach servos to pins
    servoPitch.attach(9); // Servo for pitch control
    servoRoll.attach(10); // Servo for roll control
    




    

    // Initialize MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B); // Power management register
    Wire.write(0x00); // Wake up MPU6050
    Wire.endTransmission();

    // Set accelerometer range to ±2g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C); // Accelerometer config register
    Wire.write(0x00);
    Wire.endTransmission();

    // Set gyroscope range to ±250°/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B); // Gyroscope config register
    Wire.write(0x00);
    Wire.endTransmission();
    servoPitch.write(90);
servoRoll.write(90);
delay(1000);


}

void loop() {
    // Read raw data from MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // Starting register for accelerometer data
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true); // Request 14 bytes (accel, temp, gyro)

    // Combine high and low bytes for each sensor axis
    accelX = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
    accelY = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
    accelZ = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
    Wire.read(); Wire.read(); // Skip temperature data
    gyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
    gyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
    gyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;

    // Calculate pitch and roll from accelerometer data
    pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
    roll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

    // Gyroscope rates
    gyroPitchRate = gyroX; // Pitch rate
    gyroRollRate = gyroY;  // Roll rate

/*
    // Calculate pitch and roll, considering coupling
    pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
    roll = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

     Gyroscope rates
    gyroPitchRate = gyroX + sin(roll * PI / 180.0) * gyroZ; // Adjust pitch rate with roll coupling
    gyroRollRate = gyroY + sin(pitch * PI / 180.0) * gyroZ; // Adjust roll rate with pitch coupling
*/


    // Delta time (dt)
    static unsigned long lastTime = millis();
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Update Kalman filters
    float filteredPitch = kalmanPitch.update(pitch, gyroPitchRate, dt);
    float filteredRoll = kalmanRoll.update(roll, gyroRollRate, dt);

    // Map the filtered values to servo angles (0° to 180°)
    int servoPitchAngle = map(filteredPitch, 90, -90, 10, 165);
    int servoRollAngle = map(filteredRoll, -90, 90, 0, 180);

    // Constrain servo angles to avoid exceeding limits
   servoPitchAngle = constrain(servoPitchAngle, 0, 180);
   servoRollAngle = constrain(servoRollAngle, 0, 145);

    // Write angles to servos
    servoPitch.write(servoRollAngle);
    servoRoll.write(servoPitchAngle);

    // Print debug information
    Serial.print("Filtered Roll: ");
    Serial.print(filteredPitch);
    Serial.print(" | Servo Roll Angle: ");
    Serial.print(servoPitchAngle);
    Serial.print(" | Filtered Pitch: ");
    Serial.print(filteredRoll);
    Serial.print(" | Servo Pitch Angle: ");
    Serial.println(servoRollAngle);
    delay(20);

    

   
}
