%% import_fusion_data.m - Import Fused Perception Data for ACC Simulation
% This script reads the fused sensor data (Radar + Vision) from a CSV file
% and converts it into MATLAB timeseries objects for Simulink.

vars_to_clear = {'dist_target_ts', 'v_rel_target_ts', 'fusion_data'};
clear(vars_to_clear{:});

%% 1. Configuration
csv_file_path = '../../Data/acc_fusion_final.csv';
if ~exist(csv_file_path, 'file')
    error('File not found: %s. Please check the path.', csv_file_path);
end

%% 2. Import Data
fprintf('>> Reading fused data from: %s\n', csv_file_path);

% Detect options to ensure correct data types
opts = detectImportOptions(csv_file_path);
opts.VariableNamingRule = 'preserve';
fusion_table = readtable(csv_file_path, opts);

%% 3. Extract Time and Signals
% time_sec is the relative time starting from 0
t = fusion_table.time_sec;

% Fused signals (Distance and Relative Velocity)
d_fused = fusion_table.dist_fused;
v_fused = fusion_table.v_rel_fused;

%% 4. Create Timeseries Objects
% These objects are directly readable by Simulink "From Workspace" blocks
dist_target_ts = timeseries(d_fused, t, 'Name', 'Fused_Distance');
v_rel_target_ts = timeseries(v_fused, t, 'Name', 'Fused_Rel_Velocity');

% Set metadata for clarity
dist_target_ts.DataInfo.Units = 'm';
v_rel_target_ts.DataInfo.Units = 'm/s';

%% 5. Verification Plot (Optional)
figure('Name', 'Fused Perception Data Verification');

subplot(2,1,1);
plot(dist_target_ts, 'LineWidth', 1.5);
grid on;
title('Fused Target Distance');
ylabel('Distance [m]');
xlabel('Time [s]');

subplot(2,1,2);
plot(v_rel_target_ts, 'LineWidth', 1.5, 'Color', [0.85 0.32 0.1]);
grid on;
title('Fused Relative Velocity');
ylabel('Rel. Velocity [m/s]');
xlabel('Time [s]');

fprintf('>> Data imported successfully. Variables "dist_target_ts" and "v_rel_target_ts" are ready for Simulink.\n');
