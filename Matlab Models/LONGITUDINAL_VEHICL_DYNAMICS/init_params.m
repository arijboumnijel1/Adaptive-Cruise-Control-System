%% init_params.m - Centralized Initialization Script for ACC Project
% This script initializes all constants and parameters for the 
% Intelligent Autonomous Driving System (ACC) simulation.

clear; clc;

%% 1. Conversion Factors
kmh_to_ms = 1/3.6;
ms_to_kmh = 3.6;
g_to_ms2  = 9.81;

%% 2. Vehicle Dynamics Parameters
m = 1500;                       % [kg] Total vehicle mass
g = g_to_ms2;                   % [m/s^2] Acceleration due to gravity
a_slope = 0;                    % [rad] Road slope (0 = flat)

% Powertrain & Wheels
Jm = 0.25;                      % [kg.m2] Engine rotational inertia
Jw = 4.0;                       % [kg.m2] Wheel rotational inertia (total for 4 wheels)
rw = 0.28;                      % [m] Effective rolling radius
ig = 5.0;                       % [-] Total gear ratio (differential + gearbox avg)
eff_drive = 0.95;               % [-] Driveline efficiency

% Road-load / Environment
f_roll = 0.015;                 % [-] Rolling resistance coefficient
Cd = 0.35;                      % [-] Aerodynamic drag coefficient
A_front = 0.5;                  % [m^2] Frontal area (simplified for small vehicle)
rho_air = 1.226;                % [kg/m^3] Air density at sea level

% Braking System
max_brake_torque = 1500 * 4;    % [N*m] Estimated total max braking torque

%% 3. Sensor Parameters (WP1 - Perception)
% Radar Characteristics (Long Range Radar - LRR)
radar_range_max = 200;          % [m] Maximum detection distance
radar_range_min = 1.0;          % [m]
radar_fov_horiz = 20;           % [deg] Horizontal field of view
radar_dist_res  = 0.5;          % [m] Distance resolution
radar_dist_std  = 0.1;          % [m] Measurement noise standard deviation

% Vision Characteristics (Camera)
cam_fov_horiz = 60;             % [deg] Wider field of view for lane detection
cam_range_max = 100;            % [m] Reliable detection distance for vision
cam_dist_std  = 0.5;            % [m] Vision is less precise than radar for distance

%% 4. ACC Functional Parameters (Metric)

% Activation Thresholds
v_min_acc_kmh = 40;                       % [km/h] Min activation speed (~25 mph)
v_min_acc_ms = v_min_acc_kmh * kmh_to_ms; % [m/s] 
v_max_acc_kmh = 180;                      % [km/h] Max operating speed
v_max_acc_ms = v_max_acc_kmh * kmh_to_ms; % [m/s]

% Performance & Comfort Limits
accel_max_g = 0.2;                        % [g] Max longitudinal acceleration
decel_max_g = 0.2;                        % [g] Max longitudinal deceleration
a_max = accel_max_g * g;                  % [m/s^2]
a_min = -decel_max_g * g;                 % [m/s^2]

% Time Gap parameters
default_time_gap = 1.5;                   % [s] Standard safe headway
min_time_gap = 1.0;                       % [s]
max_time_gap = 2.2;                       % [s]
d_safe_min = 5.0;                         % [m] Minimum safety distance (standstill)

%% 5. Controller Pre-sets
Kp_speed = 0.6; 
Ki_speed = 0.05;
Kd_speed = 0.01;

Kp_gap = 1.2;
Ki_gap = 0.02;
Kd_gap = 0.1;

%% Initial Conditions for Simulation
v_ego_init = 50 * kmh_to_ms;              % [m/s] Start at 50 km/h
x_ego_init = 0;                           % [m]
v_target_init = 40 * kmh_to_ms;           % [m/s] Lead vehicle speed
d_rel_init = 60;                          % [m] Initial relative distance

fprintf('>> ACC Environment Initialized (Metric Units Only).\n');
fprintf('   - Min Activation Speed: %.0f km/h\n', v_min_acc_kmh);
fprintf('   - Comfort Accel Limit: %.2f g\n', accel_max_g);
