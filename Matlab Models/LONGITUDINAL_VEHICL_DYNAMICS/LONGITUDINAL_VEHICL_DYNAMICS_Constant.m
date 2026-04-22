%% LONGITUDINAL_VEHICLE_DYNAMICS_Model - Constants (base workspace)
% Date: 2026-04-22
%
% Notes:
% - Throttle/brake inputs are normalized commands in [0, 1] (unitless).
% - This model converts wheel torque to tractive/brake force with:
%       F = T / Wheel_Radius_cst
% - Aerodynamic drag in this model is assumed:
%       F_aero = k_drag * v^2
%   where v is in m/s and F_aero in N.

%% Vehicle parameters
mass_of_the_vehicle = 1500;               % [kg]

%% Wheel / powertrain parameters
Wheel_Radius_cst = 0.30;                  % [m] effective wheel radius
max_drive_wheel_torque_cst = 2200;        % [N*m] max drive torque at wheels (total)
maxi_brake_torque_per_wheel_cst = 1500;   % [N*m] max brake torque per wheel

%% Environment / road-load parameters
acceleration_due_to_gravity_cst = 9.81;   % [m/s^2]
C_rr = 0.010;                             % [-] rolling resistance coefficient
k_drag = 0.40;                            % [N/(m/s)^2] (= kg/m)

%% Optional quick sanity prints (comment out if undesired)
% fprintf('Loaded vehicle constants: m=%.1f kg, R=%.3f m\n', mass_of_the_vehicle, Wheel_Radius_cst);
% fprintf('Torque limits: drive=%.0f N*m, brake(per wheel)=%.0f N*m\n', max_drive_wheel_torque_cst, maxi_brake_torque_per_wheel_cst);
% fprintf('Road loads: Crr=%.4f, k_drag=%.3f kg/m\n', C_rr, k_drag);