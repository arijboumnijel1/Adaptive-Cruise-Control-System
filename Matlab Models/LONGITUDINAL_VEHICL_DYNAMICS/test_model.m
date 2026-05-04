%% test_model.m - Simulation of a driving scenario (Ultra-Robust)
% This script runs the high-fidelity vehicle model and handles results extraction.

% 1. Initialization
init_params;
model_name = 'Vehicle_Dynamics';

% 2. Scenario Definition
t_end = 50;             
Ts = 0.01;              
t = (0:Ts:t_end)';
N = length(t);

u_th = zeros(N, 1);     
u_br = zeros(N, 1);     
slope = zeros(N, 1);    

u_th(t >= 0 & t < 15) = 0.3;     
u_th(t >= 15 & t < 30) = 0.05;   
u_br(t >= 30 & t < 40) = 0.2;    
u_br(t >= 40) = 0.5;             

% 3. Prepare Simulink Input
ds = [t, u_th, u_br, slope];

% 4. Run Simulation
fprintf('>> Running Simulation of %s...\n', model_name);
set_param(model_name, 'LoadExternalInput', 'on', 'ExternalInput', 'ds');
out = sim(model_name, 'StopTime', num2str(t_end));

% 5. Extract Results (Multi-strategy extraction)
fprintf('>> Extracting results...\n');

try
    % Strategy 1: Find by signal name in 'yout' dataset
    v_el = out.yout.find('Velocity_ms');
    v_sim = v_el{1}.Values.Data;
    t_sim = v_el{1}.Values.Time;
    
    x_el = out.yout.find('Position_m');
    x_sim = x_el{1}.Values.Data;
catch
    try
        % Strategy 2: Direct access if Dataset structure is different
        v_sim = out.yout{1}.Values.Data;
        t_sim = out.yout{1}.Values.Time;
        x_sim = out.yout{2}.Values.Data;
        warning('Used fallback indexing for result extraction.');
    catch
        % Strategy 3: Traditional tout/yout matrix format
        if isfield(out, 'yout') && isnumeric(out.yout)
            t_sim = out.tout;
            v_sim = out.yout(:,1);
            x_sim = out.yout(:,2);
        else
            error('Could not extract simulation results. Please check Outport settings.');
        end
    end
end

% 6. Visualization
figure('Name', 'Hi-Fi Vehicle Model Test Results');

subplot(2,1,1);
plot(t_sim, v_sim * ms_to_kmh, 'b', 'LineWidth', 2);
grid on; hold on;
yline(v_min_acc_kmh, '--r', 'ACC Min Threshold');
ylabel('Speed (km/h)');
title('Scenario: Acceleration -> Coasting -> Braking');

subplot(2,1,2);
plot(t_sim, x_sim, 'g', 'LineWidth', 2);
grid on;
ylabel('Position (m)');
xlabel('Time (s)');

fprintf('>> Simulation Complete.\n');
fprintf('   - Max Speed Reached: %.2f km/h\n', max(v_sim) * ms_to_kmh);
fprintf('   - Total Distance: %.2f m\n', x_sim(end));
