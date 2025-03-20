% Load CSV file (modify filename accordingly)
data = readmatrix('imu_data.csv');

% Extract accelerometer and gyroscope data
accel_x_data = data(:,1);  % First column → accel_x
gyro_x_data  = data(:,2);  % Second column → gyro_x
accel_y_data = data(:,3);  % Third column → accel_y
gyro_y_data  = data(:,4);  % Fourth column → gyro_y

% Number of data points
num_steps = size(data, 1);

% Kalman filter parameters
Q_angle = 0.001;        % Process noise variance for angle
Q_gyroBias = 0.003;     % Process noise variance for gyro bias
R_measure = 0.03;       % Measurement noise variance
dt = 0.01;              % Time step (based on data sample rate)

% Initial states for Roll and Pitch
angle_x = 0;    % Roll angle estimate
bias_x = 0;     % Roll gyro bias estimate
P_x = [1, 0; 0, 1];  % Error covariance matrix for roll

angle_y = 0;    % Pitch angle estimate
bias_y = 0;     % Pitch gyro bias estimate
P_y = [1, 0; 0, 1];  % Error covariance matrix for pitch

% Arrays for storing results
angle_estimates_x = zeros(num_steps, 1);
angle_estimates_y = zeros(num_steps, 1);

t = (0:num_steps-1) * dt;

for k = 1:num_steps
    % Read real IMU measurements from CSV
    newAngle_x = accel_x_data(k);  % Read accelerometer angle (X-axis)
    newRate_x  = gyro_x_data(k);   % Read gyroscope angular velocity (X-axis)
    
    newAngle_y = accel_y_data(k);  % Read accelerometer angle (Y-axis)
    newRate_y  = gyro_y_data(k);   % Read gyroscope angular velocity (Y-axis)

    % ---- Kalman Filter Update for Roll (X-axis) ----
    % Prediction step
    rate_x = newRate_x - bias_x;
    angle_x = angle_x + dt * rate_x;

    % Update error covariance matrix P_x
    P_x(1,1) = P_x(1,1) + dt * (dt * P_x(2,2) - P_x(1,2) - P_x(2,1) + Q_angle);
    P_x(1,2) = P_x(1,2) - dt * P_x(2,2);
    P_x(2,1) = P_x(2,1) - dt * P_x(2,2);
    P_x(2,2) = P_x(2,2) + Q_gyroBias * dt;

    % Measurement Update for Roll (X-axis)
    y_x = newAngle_x - angle_x;   % Measurement innovation
    S_x = P_x(1,1) + R_measure;   % Innovation covariance
    K_x = [P_x(1,1) / S_x; P_x(2,1) / S_x]; % Kalman gain for roll

    % Update roll angle and bias estimates
    angle_x = angle_x + K_x(1) * y_x;
    bias_x = bias_x + K_x(2) * y_x;

    % Update error covariance matrix P_x
    P_x = P_x - K_x * P_x(1,:);

    % Store the roll estimate
    angle_estimates_x(k) = angle_x;

    % ---- Kalman Filter Update for Pitch (Y-axis) ----
    % Prediction step
    rate_y = newRate_y - bias_y;
    angle_y = angle_y + dt * rate_y;

    % Update error covariance matrix P_y
    P_y(1,1) = P_y(1,1) + dt * (dt * P_y(2,2) - P_y(1,2) - P_y(2,1) + Q_angle);
    P_y(1,2) = P_y(1,2) - dt * P_y(2,2);
    P_y(2,1) = P_y(2,1) - dt * P_y(2,2);
    P_y(2,2) = P_y(2,2) + Q_gyroBias * dt;

    % Measurement Update for Pitch (Y-axis)
    y_y = newAngle_y - angle_y;   % Measurement innovation
    S_y = P_y(1,1) + R_measure;   % Innovation covariance
    K_y = [P_y(1,1) / S_y; P_y(2,1) / S_y]; % Kalman gain for pitch

    % Update pitch angle and bias estimates
    angle_y = angle_y + K_y(1) * y_y;
    bias_y = bias_y + K_y(2) * y_y;

    % Update error covariance matrix P_y
    P_y = P_y - K_y * P_y(1,:);

    % Store the pitch estimate
    angle_estimates_y(k) = angle_y;
end

% Plot the results
figure;

% Plot Roll (X-axis) angle estimate
subplot(2, 1, 1);
plot(t, angle_estimates_x, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Roll Angle Estimate (degrees)');
title('Kalman Filter Roll Angle Estimation');
grid on;

% Plot Pitch (Y-axis) angle estimate
subplot(2, 1, 2);
plot(t, angle_estimates_y, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pitch Angle Estimate (degrees)');
title('Kalman Filter Pitch Angle Estimation');
grid on;