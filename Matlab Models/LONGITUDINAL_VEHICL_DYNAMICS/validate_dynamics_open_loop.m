%% validate_dynamics_open_loop.m - Validation of Vehicle Physics (Metric Only)
% This script simulates the vehicle's response to a constant torque

% 1. Load Parameters (Metric version)
init_params;

% 2. Simulation Setup
dt = 0.1;               % [s] Time step
t_end = 40;            % [s] Simulation duration
t = 0:dt:t_end;
N = length(t);

% State variables
v = zeros(1, N);        % Velocity [m/s]
x = zeros(1, N);        % Position [m]
a = zeros(1, N);        % Acceleration [m/s^2]

% Initial conditions
v(1) = 0; 
x(1) = 0;

% 3. Applied Input
T_engine = 100;         % [Nm] Constant engine torque
T_wheel = T_engine * ig * eff_drive;
F_prop = T_wheel / rw;

% 4. Calculate Effective Mass
m_eff = m + (Jw / rw^2) + (Jm * ig^2 / rw^2);

fprintf('>> Validation: Effective Mass = %.2f kg (Inertia overhead: %.1f%%)\n', ...
        m_eff, (m_eff-m)/m*100);

% 5. Simulation Loop (Euler Integration)
for i = 1:N-1
    F_roll = m * g * f_roll;
    F_aero = 0.5 * rho_air * Cd * A_front * v(i)^2;
    F_res = F_roll + F_aero;
    
    a(i) = (F_prop - F_res) / m_eff;
    
    v(i+1) = v(i) + a(i) * dt;
    x(i+1) = x(i) + v(i) * dt;
end
a(N) = a(N-1);

% 6. Plotting Results
figure('Name', 'Vehicle Dynamics Validation (Metric Only)');

subplot(3,1,1);
plot(t, v * ms_to_kmh, 'LineWidth', 2);
grid on; ylabel('Speed (km/h)');
title('Vehicle Response to Constant Torque (100 Nm)');

subplot(3,1,2);
plot(t, a / g_to_ms2, 'r', 'LineWidth', 2);
grid on; ylabel('Accel (g)');

subplot(3,1,3);
plot(t, x, 'g', 'LineWidth', 2);
grid on; ylabel('Distance (m)'); xlabel('Time (s)');

fprintf('>> Simulation Complete.\n');
fprintf('   - Final Speed: %.2f km/h\n', v(end) * ms_to_kmh);
fprintf('   - Distance Traveled: %.2f m\n', x(end));
