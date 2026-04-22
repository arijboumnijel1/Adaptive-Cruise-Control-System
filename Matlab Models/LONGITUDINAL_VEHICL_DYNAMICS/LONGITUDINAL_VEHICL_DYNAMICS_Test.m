% Load constants
run("LONGITUDINAL_VEHICL_DYNAMICS_Constant.m");

% Model name
mdl = "LONGITUDINAL_VEHICL_DYNAMICS_Model";   % <-- adjust to your exact model name

% Time
tEnd = 20;
Ts   = 0.01;
t    = (0:Ts:tEnd)';

% Inputs (examples)
u_th = zeros(size(t));      % throttle command [0..1]
u_br = zeros(size(t));      % brake command    [0..1]
theta = zeros(size(t));     % road angle [rad]

% Scenario: accelerate then brake
u_th(t >= 1 & t < 10) = 0.25;
u_br(t >= 12) = 0.30;
theta(:) = 0;

% Initial conditions inputs (if these are Inports in your model)
v0 = 0;     % m/s
x0 = 0;     % m

% Build SimulationInput and set external inputs
in = Simulink.SimulationInput(mdl);

% IMPORTANT: these names must match your Inport block names OR you must use the port order.
% If your Inports are named exactly:
% 1) throttle Pedal command
% 2) brake Pedal command
% 3) Initial Speed
% 4) Initial Position
% 5) road incline angle
%
% then create a dataset in that same order:

U = [u_th, u_br, v0*ones(size(t)), x0*ones(size(t)), theta];

in = in.setModelParameter("LoadExternalInput","on", ...
                          "ExternalInput","U");

% Put U into base workspace for the sim call
assignin("base","U",[t U]); % time in col1 then signals

% Run
out = sim(in);

% Plot if signals are logged (or use your Outports)