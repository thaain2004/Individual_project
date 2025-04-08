% Load CSV file
data = readmatrix('imu_data3.csv');

% Extract data
accel_x_data = data(:,9);  % Column A
gyro_x_data  = data(:,10);  % Column B
accel_y_data = data(:,3);  % Column C
gyro_y_data  = data(:,4);  % Column D
servo_angle  = data(:,12);  % Column E

num_steps = size(data, 1);
dt = 0.01;       % Sample rate
t = (0:num_steps-1) * dt;

%% --- Complementary Filter ---
alpha = 0.85;

% Initialise
angle_x_cf = 0;
angle_y_cf = accel_y_data(1);
angle_estimates_cf_x = zeros(num_steps, 1);
angle_estimates_cf_y = zeros(num_steps, 1);

for k = 1:num_steps
    accel_angle_x = accel_x_data(k)-10;
    gyro_rate_x   = gyro_x_data(k);

    accel_angle_y = accel_y_data(k);
    gyro_rate_y   = gyro_y_data(k);

    % Complementary Filter
    angle_x_cf = alpha * (angle_x_cf + gyro_rate_x * dt) + (1 - alpha) * accel_angle_x;
    angle_y_cf = alpha * (angle_y_cf + gyro_rate_y * dt) + (1 - alpha) * accel_angle_y;

    angle_estimates_cf_x(k) = angle_x_cf;
    angle_estimates_cf_y(k) = angle_y_cf;
end

%% --- Kalman Filter ---
% Parameters
Q_angle = 0.001;
Q_gyroBias = 0.003;
R_measure = 0.008;

% Initial states
angle_x_kf = -5; bias_x = 0; P_x = [1, 0; 0, 1];
angle_y_kf = -5; bias_y = 0; P_y = [1, 0; 0, 1];

% Storage arrays
angle_estimates_kf_x = zeros(num_steps, 1);
angle_estimates_kf_y = zeros(num_steps, 1);

% Kalman Filter loop
for k = 1:num_steps
    newAngle_x = accel_x_data(k)-10;
    newRate_x  = gyro_x_data(k);
    
    newAngle_y = accel_y_data(k);
    newRate_y  = gyro_y_data(k);

    % ---- Roll (X) ----
    rate_x = newRate_x - bias_x;
    angle_x_kf = angle_x_kf + dt * rate_x;
    
    P_x(1,1) = P_x(1,1) + dt * (dt * P_x(2,2) - P_x(1,2) - P_x(2,1) + Q_angle);
    P_x(1,2) = P_x(1,2) - dt * P_x(2,2);
    P_x(2,1) = P_x(2,1) - dt * P_x(2,2);
    P_x(2,2) = P_x(2,2) + Q_gyroBias * dt;

    y_x = newAngle_x - angle_x_kf;
    S_x = P_x(1,1) + R_measure;
    K_x = [P_x(1,1) / S_x; P_x(2,1) / S_x];

    angle_x_kf = angle_x_kf + K_x(1) * y_x;
    bias_x = bias_x + K_x(2) * y_x;
    P_x = P_x - K_x * P_x(1,:);

    angle_estimates_kf_x(k) = angle_x_kf;

    % ---- Pitch (Y) ----
    rate_y = newRate_y - bias_y;
    angle_y_kf = angle_y_kf + dt * rate_y;
    
    P_y(1,1) = P_y(1,1) + dt * (dt * P_y(2,2) - P_y(1,2) - P_y(2,1) + Q_angle);
    P_y(1,2) = P_y(1,2) - dt * P_y(2,2);
    P_y(2,1) = P_y(2,1) - dt * P_y(2,2);
    P_y(2,2) = P_y(2,2) + Q_gyroBias * dt;

    y_y = newAngle_y - angle_y_kf;
    S_y = P_y(1,1) + R_measure;
    K_y = [P_y(1,1) / S_y; P_y(2,1) / S_y];

    angle_y_kf = angle_y_kf + K_y(1) * y_y;
    bias_y = bias_y + K_y(2) * y_y;
    P_y = P_y - K_y * P_y(1,:);

    angle_estimates_kf_y(k) = angle_y_kf;
end

%% --- Plot CF and KF in Subplots ---
figure;
set(gcf, 'Position', [100, 100, 1200, 600]);  % [left, bottom, width, height]

subplot(2,1,1);
plot(t, angle_estimates_cf_x, 'b', 'LineWidth', 2); hold on;
%plot(t, servo_angle, 'r--', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Complementary Filter Pitch Angle');
legend('Complementary Filter');
grid on;

subplot(2,1,2);
plot(t, angle_estimates_kf_x, 'g', 'LineWidth', 2); hold on;
%plot(t, servo_angle, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Kalman Filter Pitch Angle');
legend('Kalman Filter');
grid on;

