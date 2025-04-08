#include <Wire.h>

const int MPU_ADDR = 0x68; // MPU6050 I2C address
int16_t ax_raw, ay_raw, az_raw;
float ax, ay;  // Acceleration in m/s²

const int NUM_SAMPLES = 1500;  // Number of samples
const int TIME_PERIOD = 30000; // Total recording time (30 sec)
float dt = (float)TIME_PERIOD / NUM_SAMPLES / 1000.0; // Time step in seconds

float velocity_x = 0, velocity_y = 0; // Velocity (m/s)
float displacement_x = 0, displacement_y = 0; // Displacement (m)
float bias_ax = 0, bias_ay = 0; // Accelerometer drift correction offsets

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // Initialize MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);  // Power management register
    Wire.write(0);     // Wake up MPU6050
    Wire.endTransmission(true);

    // Calibrate accelerometer drift
    calibrateAccelerometer();

    Serial.println("Time (s), Acceleration_X (m/s²), Velocity_X (m/s), Displacement_X (m), Acceleration_Y (m/s²), Velocity_Y (m/s), Displacement_Y (m)");
}

void loop() {
    for (int i = 0; i < NUM_SAMPLES; i++) {
        readMPU6050();

        // Convert raw accelerometer to m/s²
        ax = ((ax_raw - bias_ax) / 16384.0) * 9.81;
        ay = ((ay_raw - bias_ay) / 16384.0) * 9.81;

        // Apply threshold to ignore noise
        if (fabs(ax) < 0.05) ax = 0;
        if (fabs(ay) < 0.05) ay = 0;

        // Integrate acceleration to get velocity
        velocity_x += ax * dt;
        velocity_y += ay * dt;

        // If acceleration is zero, stop velocity accumulation
        if (ax == 0) velocity_x = 0;
        if (ay == 0) velocity_y = 0;

        // Integrate velocity to get displacement
        displacement_x += velocity_x * dt;
        displacement_y += velocity_y * dt;

        // Print values for MATLAB plotting
        Serial.print(i * dt, 3); Serial.print(",");
        Serial.print(ax, 3); Serial.print(",");
        Serial.print(velocity_x, 3); Serial.print(",");
        Serial.print(displacement_x, 3); Serial.print(",");
        Serial.print(ay, 3); Serial.print(",");
        Serial.print(velocity_y, 3); Serial.print(",");
        Serial.println(displacement_y, 3);

        delay(TIME_PERIOD / NUM_SAMPLES); // Smooth sampling
    }

    Serial.println("\n--- Hold the sensor still and observe drift over time ---\n");
    delay(3000);
}

// Function to read raw accelerometer data
void readMPU6050() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    
    Wire.requestFrom(MPU_ADDR, 6, true);
    if (Wire.available() < 6) {
        Serial.println("Error: MPU6050 not responding!");
        return;
    }

    ax_raw = Wire.read() << 8 | Wire.read();
    ay_raw = Wire.read() << 8 | Wire.read();
    az_raw = Wire.read() << 8 | Wire.read();
}

// Function to calibrate accelerometer bias
void calibrateAccelerometer() {
    Serial.println("Calibrating accelerometer...");
    float sum_ax = 0, sum_ay = 0;
    
    for (int i = 0; i < 500; i++) {
        readMPU6050();
        sum_ax += ax_raw;
        sum_ay += ay_raw;
        delay(2);
    }

    bias_ax = sum_ax / 500.0;
    bias_ay = sum_ay / 500.0;

    Serial.print("Accelerometer bias: ax = "); Serial.print(bias_ax, 2);
    Serial.print(", ay = "); Serial.println(bias_ay, 2);
}
