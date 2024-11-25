clc
close all
clear
% Parameters for thermal system
m = 0.5;                % Mass of the garment (kg)
c = 1000;               % Specific heat capacity (J/kg·K)
h_body_base = 5;        % Base heat transfer coefficient with body (W/K)
h_env_base = 2;         % Base heat transfer coefficient with environment (W/K)
T_set = 37;             % Target temperature (°C)

% Environmental conditions
T_env_base = 20;        % Base ambient temperature (°C)
T_body_base = 36.5;     % Base body temperature (°C)
h_loss = 0.8;           % Heat loss coefficient

% Actuator power limits
Power_max = 100;        % Maximum actuator power (W)
Power_min = -100;       % Minimum actuator power (W)

% Initial temperature
T = 20;                 % Starting temperature of garment (°C)
dt = 0.1;               % Time step for simulation (seconds)
time = 0:dt:1000;       % Total simulation time in seconds

% PID controller parameters
Kp = 10;                % Proportional gain
Ki = 5;                 % Integral gain
Kd = 1;                 % Derivative gain

% Initialize variables for PID control
error_sum = 0;
last_error = 0;

% Initialize arrays for plotting
T_history = zeros(size(time));     % Array to store temperature history
Power_history = zeros(size(time)); % Array to store actuator power

% Simulation loop
for i = 1:length(time)
    % Time-varying body and environment temperature
    T_body = T_body_base + 0.5 * sin(0.01 * time(i));  % Oscillating body temperature
    T_env = T_env_base + 5 * sin(0.005 * time(i));     % Oscillating ambient temperature
    
    % Dynamic heat transfer coefficients to model deformation effects
    deformation_factor = 1 + 0.2 * sin(0.002 * time(i));  % Simulated deformation factor
    h_body = h_body_base * deformation_factor;
    h_env = h_env_base * deformation_factor;

    % Error calculation
    error = T_set - T;
    
    % Nonlinear actuator response to error (simulating nonlinearity)
    Power = (Kp * error + Ki * error_sum + Kd * (error - last_error) / dt) * (1 + 0.1 * abs(error));
    Power = max(min(Power, Power_max), Power_min);  % Constrain Power within limits
    last_error = error;
    error_sum = error_sum + error * dt;
    
    % Actuator response with a nonlinear component
    Q_actuator = 0.8 * Power * (1 + 0.05 * abs(Power) / Power_max);
    Power_history(i) = Power;  % Track actuator power over time
    
    % Heat from body and environment with dynamic transfer coefficients
    Q_body = h_body * (T_body - T) + 0.1 * randn();
    Q_env = h_env * (T_env - T) + 0.1 * randn();
    Q_loss = h_loss * (T - T_env);
    
    % Differential equation for temperature change
    dTdt = (Q_actuator + Q_body + Q_env - Q_loss) / (m * c);
    T = T + dTdt * dt;
    
    % Store temperature for plotting
    T_history(i) = T;
end

% Plotting temperature with dynamic heat transfer and nonlinear actuator
figure;
subplot(2,1,1);
plot(time, T_history, '-b', 'LineWidth', 1.5, 'DisplayName', 'Temperature');
hold on;
yline(T_set, '--r', 'LineWidth', 1.5, 'DisplayName', 'Target Temperature');
xlabel('Time (seconds)');
ylabel('Temperature (°C)');
title('Temperature Control with Nonlinear Uniform Model');
legend('Location', 'Best');
grid on;

subplot(2,1,2);
plot(time, Power_history, '-m', 'DisplayName', 'Actuator Power');
xlabel('Time (seconds)');
ylabel('Power (W)');
title('Actuator Power over Time');
legend('Location', 'Best');
grid on;
