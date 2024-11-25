clc;
close all;
clear;

% Parameters for thermal system
m = 0.5;                 % Mass of the garment (kg)
c = 1000;                % Specific heat capacity (J/kg·K)
h_body_base = 5;         % Base heat transfer coefficient with body (W/K)
h_env_base = 2;          % Base heat transfer coefficient with environment (W/K)
h_loss = 0.8;            % Heat loss coefficient
T_set = 37;              % Target temperature (°C)

% Environmental conditions
T_env_base = 20;         % Base ambient temperature (°C)
T_body_base = 36.5;      % Base body temperature (°C)

% Actuator power limits
Power_max = 100;         % Maximum actuator power (W)
Power_min = -100;        % Minimum actuator power (W)

% Simulation parameters
dt = 0.1;                % Time step for simulation (seconds)
time = 0:dt:1000;        % Total simulation time in seconds

% PID controller parameters
Kp = 10;                 % Proportional gain
Ki = 5;                  % Integral gain
Kd = 1;                  % Derivative gain

% Initialize variables for PID control
error_sum = 0;
last_error = 0;

% 2D Grid settings for the garment
N = 10;                  % Number of rows
M = 10;                  % Number of columns
T_grid = 20 * ones(N, M); % Initial temperature of each cell (°C)
Q_actuator = zeros(N, M); % Actuator power applied to each cell
efficiency = ones(N, M);  % Actuator efficiency

% Initialize arrays for plotting
T_history = zeros(length(time), 1);
Power_history = zeros(length(time), 1);
Efficiency_history = zeros(length(time), 1);

% Simulation loop
for t = 1:length(time)
    % Time-varying body and environment temperatures
    T_body = T_body_base + 0.5 * sin(0.01 * time(t));  
    T_env = T_env_base + 5 * sin(0.005 * time(t));    

    % Dynamic deformation factor and airflow
    deformation_factor = 1 + 0.2 * sin(0.002 * time(t));
    airflow_factor = 1 + 0.1 * sin(0.002 * time(t));
    h_body = h_body_base * deformation_factor;
    h_env = h_env_base * deformation_factor;
    h_air = h_env * airflow_factor;

    for i = 1:N
        for j = 1:M
            % Sensor noise
            sensor_noise = 0.2 * randn();
            T_measured = T_grid(i, j) + sensor_noise;

            % PID control for each cell
            error = T_set - T_measured;
            Power = (Kp * error + Ki * error_sum + Kd * (error - last_error) / dt) * (1 + 0.1 * abs(error));
            Power = max(min(Power, Power_max), Power_min);
            last_error = error;
            error_sum = error_sum + error * dt;

            % Time-varying actuator efficiency
            efficiency(i, j) = max(0.5, 1 - 0.0001 * t);
            Q_actuator(i, j) = 0.8 * Power * efficiency(i, j) * (1 + 0.05 * abs(Power) / Power_max);

            % Heat from body, environment, and loss
            Q_body = h_body * (T_body - T_grid(i, j)) + 0.1 * randn();
            Q_env = (h_env + h_air) * (T_env - T_grid(i, j)) + 0.1 * randn();
            Q_loss = h_loss * (T_grid(i, j) - T_env);

            % Heat conduction with adjacent cells
            Q_conduction = 0;
            if i > 1
                Q_conduction = Q_conduction + h_env * (T_grid(i-1, j) - T_grid(i, j));
            end
            if i < N
                Q_conduction = Q_conduction + h_env * (T_grid(i+1, j) - T_grid(i, j));
            end
            if j > 1
                Q_conduction = Q_conduction + h_env * (T_grid(i, j-1) - T_grid(i, j));
            end
            if j < M
                Q_conduction = Q_conduction + h_env * (T_grid(i, j+1) - T_grid(i, j));
            end

            % Differential equation for temperature change
            dTdt = (Q_actuator(i, j) + Q_body + Q_env - Q_loss + Q_conduction) / (m * c);
            T_grid(i, j) = T_grid(i, j) + dTdt * dt;
        end
    end

    % Store average temperature and power for plotting
    T_history(t) = mean(T_grid(:));
    Power_history(t) = mean(Q_actuator(:));
    Efficiency_history(t) = mean(efficiency(:));
end

% Visualization
figure;
subplot(3, 1, 1);
imagesc(T_grid);
colorbar;
title('Final Temperature Distribution in Garment');
xlabel('Columns');
ylabel('Rows');

subplot(3, 1, 2);
plot(time, T_history, '-b', 'LineWidth', 1.5);
hold on;
yline(T_set, '--r', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Average Temperature (°C)');
title('Average Temperature over Time');
legend('Temperature', 'Target Temperature');
grid on;

subplot(3, 1, 3);
plot(time, Power_history, '-m', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Average Actuator Power (W)');
title('Actuator Power over Time');
grid on;

figure;
plot(time, Efficiency_history, '-g', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Average Efficiency');
title('Actuator Efficiency over Time');
grid on;
