clc
close all
clear
% Parameters for thermal system
m = 0.125;             % Mass of each garment segment (kg) (total 4 segments)
c = 1000;              % Specific heat capacity (J/kg·K)
h_body = 5;            % Heat transfer coefficient with body (W/K)
h_env = 2;             % Heat transfer coefficient with environment (W/K)
T_set = 37;            % Target temperature (°C)

% Environmental conditions
T_body = 36.5;         % Body temperature (°C)
T_env = 20;            % Ambient temperature (°C)
h_loss = 0.8;          % Heat loss coefficient

% Initial temperature for each segment
T_segments = [20, 20, 20, 20];   % Initial temperatures of 4 segments (°C)
dt = 0.1;                        % Time step for simulation (seconds)
time = 0:dt:1000;                % Total simulation time in seconds

% PID controller parameters (same for each segment)
Kp = 10;                         % Proportional gain
Ki = 5;                          % Integral gain
Kd = 1;                          % Derivative gain

% Initialize variables for PID control
error_sum = zeros(1, 4);
last_error = zeros(1, 4);

% Initialize arrays for plotting
T_history_segments = zeros(length(time), 4);  % Array to store temperature history of each segment

% Simulation loop
for i = 1:length(time)
    for j = 1:4  % Loop through each garment segment
        % Calculate error between target temperature and current temperature of each segment
        error = T_set - T_segments(j);
        
        % PID control: calculate control power for both heating and cooling
        error_sum(j) = error_sum(j) + error * dt;
        error_rate = (error - last_error(j)) / dt;
        Power = Kp * error + Ki * error_sum(j) + Kd * error_rate;
        last_error(j) = error;
        
        % Bidirectional actuator: Power can be positive (heating) or negative (cooling)
        Q_actuator = Power;  % Actuator heat: adds or removes heat based on Power sign
        
        % Heat from body and environment
        Q_body = h_body * (T_body - T_segments(j));
        Q_env = h_env * (T_env - T_segments(j));
        
        % Heat loss to surroundings and transfer between segments (for simple modeling)
        Q_loss = h_loss * (T_segments(j) - T_env);
        
        % Heat transfer between neighboring segments
        if j > 1  % Heat exchange with previous segment
            Q_transfer = 0.1 * (T_segments(j-1) - T_segments(j));
        else
            Q_transfer = 0;  % No transfer for the first segment
        end
        if j < 4  % Heat exchange with next segment
            Q_transfer = Q_transfer + 0.1 * (T_segments(j+1) - T_segments(j));
        end
        
        % Differential equation for temperature change of each segment
        dTdt = (Q_actuator + Q_body + Q_env - Q_loss + Q_transfer) / (m * c);
        T_segments(j) = T_segments(j) + dTdt * dt;  % Update temperature using Euler integration
        
        % Store temperature for plotting
        T_history_segments(i, j) = T_segments(j);
    end
end

% Plotting temperature over time for each segment
figure;
plot(time, T_history_segments(:, 1), 'r', 'LineWidth', 1.5); hold on;
plot(time, T_history_segments(:, 2), 'g', 'LineWidth', 1.5);
plot(time, T_history_segments(:, 3), 'b', 'LineWidth', 1.5);
plot(time, T_history_segments(:, 4), 'k', 'LineWidth', 1.5);
xlabel('Time (seconds)');
ylabel('Temperature (°C)');
title('Garment Temperature Control - Non-Uniform Temperature Model');
legend('Segment 1', 'Segment 2', 'Segment 3', 'Segment 4');
grid on;
