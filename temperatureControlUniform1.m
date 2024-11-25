clc
close all
clear
% Parameters for thermal system
m = 0.5;                % Mass of the garment (kg)
c = 1000;               % Specific heat capacity (J/kg·K)
h_body = 5;             % Heat transfer coefficient with body (W/K)
h_env = 2;              % Heat transfer coefficient with environment (W/K)
T_set = 22;             % Target temperature (°C)

% Environmental conditions
T_body = 36.5;          % Body temperature (°C)
T_env = 20;             % Ambient temperature (°C)
h_loss = 0.8;           % Heat loss coefficient

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
T_history = zeros(size(time));  % Array to store temperature history
Q_actuator_history = zeros(size(time));
% Simulation loop
for i = 1:length(time)
    % Calculate error between target temperature and current temperature
    error = T_set - T;
    
    % PID control: calculate control power for both heating and cooling
    error_sum = error_sum + error * dt;
    error_rate = (error - last_error) / dt;
    Power = Kp * error + Ki * error_sum + Kd * error_rate;
    last_error = error;
    
    % Bidirectional actuator: Power can be positive (heating) or negative (cooling)
    Q_actuator = Power;  % Actuator heat: adds or removes heat based on Power sign
    
    % Heat from body and environment
    Q_body = h_body * (T_body - T);
    Q_env = h_env * (T_env - T);
    
    % Heat loss to surroundings
    Q_loss = h_loss * (T - T_env);
    
    % Differential equation for temperature change
    dTdt = (Q_actuator + Q_body + Q_env - Q_loss) / (m * c);
    T = T + dTdt * dt;  % Update temperature using Euler integration
    
    % Store temperature for plotting
    T_history(i) = T;
    Q_actuator_history(i) = Q_actuator;
end

% Plotting temperature over time
figure;
subplot(2,1,1)
plot(time, T_history,'r', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Temperature (°C)');
title('Garment Temperature Control - Uniform Temperature Model');
grid on;
subplot(2,1,2)
plot(time, Q_actuator_history,'b', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Actuator Output');
grid on;

