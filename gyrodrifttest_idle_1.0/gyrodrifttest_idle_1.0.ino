#include <Wire.h>
#include <Servo.h>

const int MPU_ADDR = 0x68; // MPU6050 I2C address
int16_t ax, ay, az, gx, gy, gz;

const int NUM_SAMPLES = 1500; // Number of samples
const int TIME_PERIOD = 30000; // Total time in milliseconds (3 seconds)
float dt = (float)TIME_PERIOD / NUM_SAMPLES / 1000.0; // Time step in seconds

float angle_x = 0, angle_y = 0; // Integrated gyro angles (degrees)
float bias_gx = 0, bias_gy = 0; // Gyro drift correction offsets

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // Power management register
    Wire.write(0);     // Wake up MPU6050
    Wire.endTransmission(true);

    // Calibrate gyro drift
    calibrateGyro();
}

void loop() {
    Serial.println("Time (s), Integrated_Angle_X (deg), Integrated_Angle_Y (deg)");

    for (int i = 0; i < NUM_SAMPLES; i++) {
        readMPU6050();

        // Convert gyro raw data to deg/sec (MPU6050 scale: 131 LSB per °/s)
        float gx_dps = (gx - bias_gx) / 131.0;
        float gy_dps = (gy - bias_gy) / 131.0;

        // Integrate angular velocity to estimate angle (Euler Integration)
        angle_x += gx_dps * dt;
        angle_y += gy_dps * dt;

        // Print values for MATLAB plotting
        Serial.print(i * dt, 3); Serial.print(","); // Time in seconds
        Serial.print(angle_x, 3); Serial.print(",");
        Serial.println(angle_y, 3);

        delay(TIME_PERIOD / NUM_SAMPLES); // Smooth sampling
    }

    Serial.println("\n--- Hold the sensor still and observe drift over time ---\n");
    delay(3000);
}

// Function to read raw MPU6050 data
void readMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Ignore temperature
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

// Function to calibrate gyro bias (average over 500 samples)
void calibrateGyro() {
    Serial.println("Calibrating gyroscope...");
    float sum_gx = 0, sum_gy = 0;
    
    for (int i = 0; i < 500; i++) {
        readMPU6050();
        sum_gx += gx;
        sum_gy += gy;
        delay(2);
    }

    bias_gx = sum_gx / 500.0;
    bias_gy = sum_gy / 500.0;

    Serial.print("Gyro bias: gx = "); Serial.print(bias_gx);
    Serial.print(", gy = "); Serial.println(bias_gy);
}
