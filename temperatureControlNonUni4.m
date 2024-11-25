clc;
close all;
clear;

% Parameters for thermal system
m = 0.5;                % Mass of the garment (kg)
c = 1000;               % Specific heat capacity (J/kg·K)
h_body_base = 5;        % Base heat transfer coefficient with body (W/K)
h_env_base = 2;         % Base heat transfer coefficient with environment (W/K)
h_loss = 0.8;           % Heat loss coefficient
T_set = 37;             % Target temperature (°C)

% Environmental conditions
T_env_base = 20;        % Base ambient temperature (°C)
T_body_base = 36.5;     % Base body temperature (°C)

% Actuator power limits
Power_max = 100;        % Maximum actuator power (W)
Power_min = -100;       % Minimum actuator power (W)

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
T_grid_controlled = 20 * ones(N, M);  % Initial temperature with controller
T_grid_uncontrolled = 20 * ones(N, M); % Initial temperature without controller
Q_actuator = zeros(N, M); % Actuator power applied to each cell

% Initialize arrays for plotting
T_history_controlled = zeros(length(time), 1);
T_history_uncontrolled = zeros(length(time), 1);
Power_history = zeros(length(time), 1);
Efficiency_history = zeros(length(time), 1);

% Exponential decay constant for actuator efficiency
decay_rate = 0.000001;

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

    % Update actuator efficiency with exponential decay
    efficiency = max(0.99,exp(-decay_rate * t));
    
    for i = 1:N
        for j = 1:M
            % --- Controlled Case ---
            % Sensor noise
            sensor_noise = 0.2 * randn();
            T_measured = T_grid_controlled(i, j) + sensor_noise;

            % PID control for each cell
            error = T_set - T_measured;
            Power = (Kp * error + Ki * error_sum + Kd * (error - last_error) / dt) * (1 + 0.1 * abs(error));
            Power = max(min(Power, Power_max), Power_min);
            last_error = error;
            error_sum = error_sum + error * dt;

            % Nonlinear actuator response with exponential decay
            Q_actuator(i, j) = 0.8 * Power * efficiency * (1 + 0.05 * abs(Power) / Power_max);

            % Heat from body, environment, and loss (Controlled)
            Q_body_controlled = h_body * (T_body - T_grid_controlled(i, j)) + 0.1 * randn();
            Q_env_controlled = (h_env + h_air) * (T_env - T_grid_controlled(i, j)) + 0.1 * randn();
            Q_loss_controlled = h_loss * (T_grid_controlled(i, j) - T_env);

            % Heat conduction with adjacent cells (Controlled)
            Q_conduction_controlled = 0;
            if i > 1
                Q_conduction_controlled = Q_conduction_controlled + h_env * (T_grid_controlled(i-1, j) - T_grid_controlled(i, j));
            end
            if i < N
                Q_conduction_controlled = Q_conduction_controlled + h_env * (T_grid_controlled(i+1, j) - T_grid_controlled(i, j));
            end
            if j > 1
                Q_conduction_controlled = Q_conduction_controlled + h_env * (T_grid_controlled(i, j-1) - T_grid_controlled(i, j));
            end
            if j < M
                Q_conduction_controlled = Q_conduction_controlled + h_env * (T_grid_controlled(i, j+1) - T_grid_controlled(i, j));
            end

            % Update temperature for controlled case
            dTdt_controlled = (Q_actuator(i, j) + Q_body_controlled + Q_env_controlled - Q_loss_controlled + Q_conduction_controlled) / (m * c);
            T_grid_controlled(i, j) = T_grid_controlled(i, j) + dTdt_controlled * dt;

            % --- Uncontrolled Case ---
            % Natural heat transfer without actuator
            Q_body_uncontrolled = h_body * (T_body - T_grid_uncontrolled(i, j)) + 0.1 * randn();
            Q_env_uncontrolled = (h_env + h_air) * (T_env - T_grid_uncontrolled(i, j)) + 0.1 * randn();
            Q_loss_uncontrolled = h_loss * (T_grid_uncontrolled(i, j) - T_env);

            % Heat conduction with adjacent cells (Uncontrolled)
            Q_conduction_uncontrolled = 0;
            if i > 1
                Q_conduction_uncontrolled = Q_conduction_uncontrolled + h_env * (T_grid_uncontrolled(i-1, j) - T_grid_uncontrolled(i, j));
            end
            if i < N
                Q_conduction_uncontrolled = Q_conduction_uncontrolled + h_env * (T_grid_uncontrolled(i+1, j) - T_grid_uncontrolled(i, j));
            end
            if j > 1
                Q_conduction_uncontrolled = Q_conduction_uncontrolled + h_env * (T_grid_uncontrolled(i, j-1) - T_grid_uncontrolled(i, j));
            end
            if j < M
                Q_conduction_uncontrolled = Q_conduction_uncontrolled + h_env * (T_grid_uncontrolled(i, j+1) - T_grid_uncontrolled(i, j));
            end

            % Update temperature for uncontrolled case
            dTdt_uncontrolled = (Q_body_uncontrolled + Q_env_uncontrolled - Q_loss_uncontrolled + Q_conduction_uncontrolled) / (m * c);
            T_grid_uncontrolled(i, j) = T_grid_uncontrolled(i, j) + dTdt_uncontrolled * dt;
        end
    end

    % Store average temperatures for plotting
    T_history_controlled(t) = mean(T_grid_controlled(:));
    T_history_uncontrolled(t) = mean(T_grid_uncontrolled(:));
    Power_history(t) = mean(Q_actuator(:));
    Efficiency_history(t) = efficiency;
end
% Visualization
figure;
subplot(2, 1, 1);
imagesc(T_grid_controlled);
colorbar;
title('Final Temperature Distribution with Controller');
xlabel('Columns');
ylabel('Rows');

subplot(2, 1, 2);
imagesc(T_grid_uncontrolled);
colorbar;
title('Final Temperature Distribution without Controller');
xlabel('Columns');
ylabel('Rows');

% Visualization for Controlled vs Uncontrolled Temperature
figure;

% Subplot for Controlled Temperature
subplot(2, 1, 1);
plot(time, T_history_controlled, '-b', 'LineWidth', 1.5);
hold on;
yline(T_set, '--r', 'LineWidth', 1.5, 'DisplayName', 'Target Temperature');
xlabel('Time (seconds)');
ylabel('Average Temperature (°C)');
title('Controlled Temperature Over Time');
legend('Controlled', 'Target Temperature');
grid on;

% Subplot for Uncontrolled Temperature
subplot(2, 1, 2);
plot(time, T_history_uncontrolled, '-g', 'LineWidth', 1.5);
hold on;
yline(T_set, '--r', 'LineWidth', 1.5, 'DisplayName', 'Target Temperature');
xlabel('Time (seconds)');
ylabel('Average Temperature (°C)');
title('Uncontrolled Temperature Over Time');
legend('Uncontrolled', 'Target Temperature');
grid on;

% Figure for Actuator Power Over Time
figure;
plot(time, Power_history, '-m', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Average Actuator Power (W)');
title('Actuator Power Over Time');
grid on;

figure;
plot(time, Efficiency_history, '-g', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Actuator Efficiency');
title('Actuator Efficiency over Time');
grid on;
