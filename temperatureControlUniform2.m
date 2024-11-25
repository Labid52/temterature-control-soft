clc
close all
clear
% Parameters for thermal system
m = 0.5;                % Mass of the garment (kg)
c = 1000;               % Specific heat capacity (J/kg·K)
h_body = 5;             % Heat transfer coefficient with body (W/K)
h_env = 2;              % Heat transfer coefficient with environment (W/K)
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
Energy_used = zeros(size(time));   % Array to track total energy consumption
P_history = zeros(size(time));     % Proportional term history
I_history = zeros(size(time));     % Integral term history
D_history = zeros(size(time));     % Derivative term history

% Simulation loop
for i = 1:length(time)
    % Time-varying body and environment temperature
    T_body = T_body_base + 0.5 * sin(0.01 * time(i));  % Body temperature oscillates slightly
    T_env = T_env_base + 5 * sin(0.005 * time(i));     % Ambient temperature varies more significantly
    
    % Error calculation
    error = T_set - T;
    
    % PID control with heating/cooling power limits
    error_sum = error_sum + error * dt;
    error_rate = (error - last_error) / dt;
    P_term = Kp * error;
    I_term = Ki * error_sum;
    D_term = Kd * error_rate;
    Power = P_term + I_term + D_term;
    Power = max(min(Power, Power_max), Power_min);  % Constrain Power within limits
    last_error = error;
    
    % Actuator energy calculation and logging
    Energy_used(i) = abs(Power) * dt;  % Simple approximation of energy
    Power_history(i) = Power;           % Track actuator power over time
    P_history(i) = P_term;              % Store P term
    I_history(i) = I_term;              % Store I term
    D_history(i) = D_term;              % Store D term
    
    % Thermal response with actuator lag
    Q_actuator = 0.8 * Power;
    Q_body = h_body * (T_body - T) + randn() * 0.1;
    Q_env = h_env * (T_env - T) + randn() * 0.1;
    Q_loss = h_loss * (T - T_env);
    
    % Differential equation for temperature change
    dTdt = (Q_actuator + Q_body + Q_env - Q_loss) / (m * c);
    T = T + dTdt * dt;
    
    % Store temperature for plotting
    T_history(i) = T;
end

% Plotting temperature and PID components
figure;

subplot(3,1,1);
plot(time, T_history, '-b', 'LineWidth', 1.5, 'DisplayName', 'Temperature');
hold on;
yline(T_set, '--r', 'LineWidth', 1.5, 'DisplayName', 'Target Temperature');
xlabel('Time (seconds)');
ylabel('Temperature (°C)');
title('Temperature Control with Uniform Model');
legend('Location', 'Best');
grid on;

subplot(3,1,2);
plot(time, P_history, '-r', 'DisplayName', 'Proportional');
hold on;
plot(time, I_history, '-g', 'DisplayName', 'Integral');
plot(time, D_history, '-b', 'DisplayName', 'Derivative');
xlabel('Time (seconds)');
ylabel('PID Terms');
title('PID Component Contributions');
legend('Location', 'Best');
grid on;

subplot(3,1,3);
plot(time, Power_history, '-m', 'DisplayName', 'Actuator Power');
hold on;
plot(time, cumsum(Energy_used), '-k', 'DisplayName', 'Cumulative Energy');
xlabel('Time (seconds)');
ylabel('Power (W) / Energy (J)');
title('Actuator Power and Cumulative Energy Consumption');
legend('Location', 'Best');
grid on;
