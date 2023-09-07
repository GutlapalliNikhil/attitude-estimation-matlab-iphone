% Load accelerometer and gyroscope data from a MAT file
pitch_roll = load('/MATLAB Drive/MobileSensorData/onlypitch.mat'); 

% Extract acceleration and angular velocity data as arrays
acceleration = table2array(pitch_roll.Acceleration);
angular_velocity = table2array(pitch_roll.AngularVelocity);

% Swap and negate acceleration components
acceleration = -[acceleration(:,2) acceleration(:,1) -acceleration(:,3)];
angular_velocity = [angular_velocity(:,2) angular_velocity(:,1) -angular_velocity(:,3)];

% Separate acceleration and angular velocity components
Ax = acceleration(:, 1);
Ay = acceleration(:, 2);
Az = acceleration(:, 3);

Gx = angular_velocity(:, 1);
Gy = angular_velocity(:, 2);
Gz = angular_velocity(:, 3);

% Define the time step and create a time vector
dt = 0.1;
len_g = length(Gx);
len_a = length(Ax);
t = (0:dt:(min(len_a, len_g)-1)*dt);

% Convert gyroscope measurements to radians
Gx_rad = Gx * pi / 180.0;
Gy_rad = Gy * pi / 180.0;
Gz_rad = Gz * pi / 180.0;

% 1) Calculate roll and pitch using accelerometer data
phi_hat_acc   = atan2(Ay, sqrt(Ax .^ 2 + Az .^ 2));
theta_hat_acc = atan2(-Ax, sqrt(Ay .^ 2 + Az .^ 2));

% 2) Estimate roll and pitch using gyroscope data
phi_hat_gyr   = zeros(1, length(t));
theta_hat_gyr = zeros(1, length(t));

for i = 2:length(t)
   p = Gx_rad(i);
   q = Gy_rad(i);
   r = Gz_rad(i);
   
   phi_hat   = phi_hat_gyr(i - 1);
   theta_hat = theta_hat_gyr(i - 1);
    
   phi_hat_gyr(i)   = phi_hat   + dt * (p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r);
   theta_hat_gyr(i) = theta_hat + dt * (cos(phi_hat) * q - sin(phi_hat) * r);
end

% 3) Implement the Complementary Filter
alpha = 0.1;

phi_hat_complimentary   = zeros(1, length(t));
theta_hat_complimentary = zeros(1, length(t));

for i = 2:length(t)
    p = Gx_rad(i);
    q = Gy_rad(i);
    r = Gz_rad(i);
   
    phi_hat   = phi_hat_complimentary(i - 1);
    theta_hat = theta_hat_complimentary(i - 1);
    
    phi_hat_gyr_comp   = phi_hat   + dt * (p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r);
    theta_hat_gyr_comp = theta_hat + dt * (cos(phi_hat) * q - sin(phi_hat) * r);
       
    phi_hat_complimentary(i)   = (1 - alpha) * phi_hat_gyr_comp   + alpha * phi_hat_acc(i);
    theta_hat_complimentary(i) = (1 - alpha) * theta_hat_gyr_comp + alpha * theta_hat_acc(i);    
end

% 4) Implement the Kalman Filter for attitude estimation
A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];
B = [dt 0 0 0; 0 0 dt 0]';
C = [1 0 0 0; 0 0 1 0];
P = eye(4);
Q = eye(4) * 0.01;
R = eye(2) * 10;
state_estimate = [0 0 0 0]';

phi_hat_kalman    = zeros(1, length(t));
bias_phi_kalman   = zeros(1, length(t));
theta_hat_kalman  = zeros(1, length(t));
bias_theta_kalman = zeros(1, length(t));

for i = 2:length(t)
    
    p = Gx_rad(i);
    q = Gy_rad(i);
    r = Gz_rad(i);
   
    phi_hat   = phi_hat_kalman(i - 1);
    theta_hat = theta_hat_kalman(i - 1);
    
    phi_dot   = p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r;
    theta_dot = cos(phi_hat) * q - sin(phi_hat) * r;
          
    % Predict
    state_estimate = A * state_estimate + B * [phi_dot, theta_dot]';
    P = A * P * A' + Q;
    
    % Update
    measurement = [phi_hat_acc(i) theta_hat_acc(i)]';
    y_tilde = measurement - C * state_estimate;
    S = R + C * P * C';
    K = P * C' * (S^-1);
    state_estimate = state_estimate + K * y_tilde;
    P = (eye(4) - K * C) * P;
    
    phi_hat_kalman(i)    = state_estimate(1);
    bias_phi_kalman(i)   = state_estimate(2);
    theta_hat_kalman(i)  = state_estimate(3);
    bias_theta_kalman(i) = state_estimate(4);
    
end

% Convert all angle estimates to degrees
phi_hat_acc = phi_hat_acc * 180.0 / pi;
theta_hat_acc = theta_hat_acc * 180.0 / pi;
phi_hat_gyr = phi_hat_gyr * 180.0 / pi;
theta_hat_gyr = theta_hat_gyr * 180.0 / pi;
phi_hat_complimentary = phi_hat_complimentary * 180.0 / pi;
theta_hat_complimentary = theta_hat_complimentary * 180.0 / pi;
phi_hat_kalman = phi_hat_kalman * 180.0 / pi;
theta_hat_kalman = theta_hat_kalman * 180.0 / pi;

% Plot the results
subplot(2, 1, 1);
plot(t, phi_hat_complimentary, t, phi_hat_kalman)
legend('Complimentary','Kalman')
xlabel('Time (s)')
ylabel('Angle (Degrees)')
title('Roll')

subplot(2, 1, 2);
plot(t, theta_hat_complimentary, t, theta_hat_kalman)
legend('Complementary', 'Kalman')
xlabel('Time (s)')
ylabel('Angle (Degrees)')
title('Pitch')