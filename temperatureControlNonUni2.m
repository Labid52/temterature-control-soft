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
T_segments = [20, 20, 20, 20];    % Controlled temperatures of 4 segments (°C)
T_uncontrolled = T_segments;      % Uncontrolled baseline temperatures

dt = 0.1;                         % Time step for simulation (seconds)
time = 0:dt:1000;                 % Total simulation time in seconds

% PID controller parameters (same for each segment)
Kp = 10;                          % Proportional gain
Ki = 5;                           % Integral gain
Kd = 1;                           % Derivative gain

% Initialize variables for PID control
error_sum = zeros(1, 4);
last_error = zeros(1, 4);

% Initialize arrays for plotting
T_history_segments = zeros(length(time), 4);       % Controlled temperature history
T_uncontrolled_history = zeros(length(time), 4);   % Uncontrolled temperature history

% Simulation loop
for i = 1:length(time)
    for j = 1:4  % Loop through each garment segment
        % Controlled temperature calculation with PID
        error = T_set - T_segments(j);
        error_sum(j) = error_sum(j) + error * dt;
        error_rate = (error - last_error(j)) / dt;
        Power = Kp * error + Ki * error_sum(j) + Kd * error_rate;
        last_error(j) = error;
        
        % Actuator-controlled heating or cooling power
        Q_actuator = Power;
        
        % Heat from body and environment for both controlled and uncontrolled
        Q_body = h_body * (T_body - T_segments(j));
        Q_env = h_env * (T_env - T_segments(j));
        
        % Uncontrolled temperatures: no actuator, only heat exchange
        Q_uncontrolled_body = h_body * (T_body - T_uncontrolled(j));
        Q_uncontrolled_env = h_env * (T_env - T_uncontrolled(j));
        
        % Heat loss to surroundings
        Q_loss = h_loss * (T_segments(j) - T_env);
        Q_uncontrolled_loss = h_loss * (T_uncontrolled(j) - T_env);
        
        % Heat transfer between segments
        if j > 1
            Q_transfer = 0.1 * (T_segments(j-1) - T_segments(j));
        else
            Q_transfer = 0;
        end
        if j < 4
            Q_transfer = Q_transfer + 0.1 * (T_segments(j+1) - T_segments(j));
        end
        
        % Update controlled temperature
        dTdt_controlled = (Q_actuator + Q_body + Q_env - Q_loss + Q_transfer) / (m * c);
        T_segments(j) = T_segments(j) + dTdt_controlled * dt;
        
        % Update uncontrolled temperature
        dTdt_uncontrolled = (Q_uncontrolled_body + Q_uncontrolled_env - Q_uncontrolled_loss + Q_transfer) / (m * c);
        T_uncontrolled(j) = T_uncontrolled(j) + dTdt_uncontrolled * dt;
        
        % Store temperatures for plotting
        T_history_segments(i, j) = T_segments(j);
        T_uncontrolled_history(i, j) = T_uncontrolled(j);
    end
end

% Plotting temperature over time for controlled and uncontrolled segments
figure;
for j = 1:4
    subplot(2,2,j);
    plot(time, T_history_segments(:, j), 'LineWidth', 1.5, 'DisplayName', 'Controlled');
    hold on;
    plot(time, T_uncontrolled_history(:, j), '--', 'LineWidth', 1.5, 'DisplayName', 'Uncontrolled');
    xlabel('Time (seconds)');
    ylabel(['Temperature of Segment ', num2str(j), ' (°C)']);
    title(['Segment ', num2str(j), ' Temperature Control']);
    legend;
    grid on;
end
