%% assemble_closed_loop.m - Assemblage Automatique du Modèle ACC en Boucle Fermée
% Ce script crée le modèle Simulink global "ACC_System_Closed_Loop.slx",
% implémente le bloc VCI (Vehicle Control Interface) en blocs natifs,
% et connecte tous les composants ADAS en boucle fermée.

clear; clc; close all;

%% 1. Configuration des chemins et paramètres
script_path = fileparts(mfilename('fullpath'));
models_path = fullfile(script_path, '..', '02_Models');
addpath(models_path);

% Charger les paramètres d'initialisation dans le workspace
init_params;

model_name = 'ACC_System_Closed_Loop';
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% Créer et ouvrir le nouveau système
new_system(model_name);
open_system(model_name);

fprintf('>> Création du modèle global "%s.slx"...\n', model_name);

%% 2. Ajout des Blocs de Perception (From Workspace)
% Distance relative (Timeseries)
add_block('simulink/Sources/From Workspace', [model_name '/From_Workspace_D_rel'], ...
    'VariableName', 'dist_target_ts', 'Position', [40, 240, 150, 270]);

% Vitesse relative (Timeseries)
add_block('simulink/Sources/From Workspace', [model_name '/From_Workspace_V_rel'], ...
    'VariableName', 'v_rel_target_ts', 'Position', [40, 310, 150, 340]);

%% 3. Ajout des Constantes de Commande Utilisateur et Profils
% Bouton Master ON/OFF (CruiseSwitch)
add_block('simulink/Sources/Constant', [model_name '/Const_CruiseSwitch'], ...
    'Value', '1', 'OutDataTypeStr', 'boolean', 'Position', [40, 40, 90, 60]);

% Bouton SET d'activation de la consigne (SetSwitch)
add_block('simulink/Sources/Constant', [model_name '/Const_SetSwitch'], ...
    'Value', '1', 'OutDataTypeStr', 'boolean', 'Position', [40, 80, 90, 100]);

% Pédale de frein conducteur (BrakePedal) - Priorité Sécurité
add_block('simulink/Sources/Constant', [model_name '/Const_BrakePedal'], ...
    'Value', '0', 'OutDataTypeStr', 'boolean', 'Position', [40, 120, 90, 140]);

% Présence véhicule cible (LeadVehicle_Detected)
add_block('simulink/Sources/Constant', [model_name '/Const_LeadVehicle_Detected'], ...
    'Value', '1', 'OutDataTypeStr', 'boolean', 'Position', [40, 180, 90, 200]);

% Vitesse de consigne (V_set - convertie en m/s, ex: 90 km/h)
add_block('simulink/Sources/Constant', [model_name '/Const_V_set'], ...
    'Value', '90 * 1/3.6', 'Position', [400, 30, 480, 60]);

% Consigne d'espacement temporel (Set_Gap - en s)
add_block('simulink/Sources/Constant', [model_name '/Const_Set_Gap'], ...
    'Value', 'default_time_gap', 'Position', [400, 180, 480, 200]);

% Profil de pente de la route (Road_Slope_rad - à plat = 0)
add_block('simulink/Sources/Constant', [model_name '/Const_Road_Slope'], ...
    'Value', '0', 'Position', [800, 380, 850, 400]);

%% 4. Importation du Superviseur (Stateflow Chart)
% Chargement du modèle de supervision
load_system('ACC_Mode_Manager');

% Copie du Chart Stateflow directement dans le modèle global (pour éviter les refs complexes de Chart)
add_block('ACC_Mode_Manager/ACC_Mode_Manager', [model_name '/ACC_Mode_Manager'], ...
    'Position', [220, 40, 350, 200]);

close_system('ACC_Mode_Manager', 0);

%% 5. Ajout du Contrôleur Cascade PID (Model Reference)
add_block('simulink/Ports & Subsystems/Model', [model_name '/ACC_Controller'], ...
    'ModelName', 'ACC_Controller', 'Position', [530, 80, 680, 220]);

%% 6. Ajout de la Vehicle Control Interface (VCI - Nouveau Sous-Système)
vci_path = [model_name '/Vehicle_Control_Interface'];
add_block('simulink/Ports & Subsystems/Subsystem', vci_path, 'Position', [760, 220, 900, 320]);

% Construction interne de la VCI (Modèle Physique Inverse)
% A. Entrées de la VCI
add_block('simulink/Sources/In1', [vci_path '/Acceleration_Command'], 'Position', [20, 40, 50, 55]);
add_block('simulink/Sources/In1', [vci_path '/V_ego'], 'Position', [20, 110, 50, 125]);
add_block('simulink/Sources/In1', [vci_path '/Road_Slope_rad'], 'Position', [20, 210, 50, 225]);

% B. Calcul de la Force d'Inertie : F_inertia = m_eff * a_cmd
m_eff_val = 'm + (Jw/rw^2) + (Jm*ig^2/rw^2)';
add_block('simulink/Math Operations/Gain', [vci_path '/Gain_Inertia'], ...
    'Gain', m_eff_val, 'Position', [150, 30, 230, 65]);
add_line(vci_path, 'Acceleration_Command/1', 'Gain_Inertia/1');

% C. Calcul de la Force Aérodynamique : F_aero = 0.5 * rho * Cd * A * v^2
add_block('simulink/Math Operations/Math Function', [vci_path '/Square_V'], 'Function', 'pow', 'Position', [120, 105, 150, 130]);
add_block('simulink/Sources/Constant', [vci_path '/Const_Power'], 'Value', '2', 'Position', [80, 120, 100, 135]);
add_block('simulink/Math Operations/Gain', [vci_path '/Gain_Aero'], ...
    'Gain', '0.5 * rho_air * Cd * A_front', 'Position', [200, 105, 300, 135]);

add_line(vci_path, 'V_ego/1', 'Square_V/1');
add_line(vci_path, 'Const_Power/1', 'Square_V/2');
add_line(vci_path, 'Square_V/1', 'Gain_Aero/1');

% D. Calcul de la Force de Roulement : F_roll = m * g * f_roll
add_block('simulink/Sources/Constant', [vci_path '/Const_Roll'], ...
    'Value', 'm * g * f_roll', 'Position', [200, 160, 280, 180]);

% E. Calcul de la Force de Pente : F_slope = m * g * sin(slope)
add_block('simulink/Math Operations/Trigonometric Function', [vci_path '/Sin_Slope'], ...
    'Operator', 'sin', 'Position', [120, 205, 150, 230]);
add_block('simulink/Math Operations/Gain', [vci_path '/Gain_Slope'], ...
    'Gain', 'm * g', 'Position', [200, 205, 280, 235]);

add_line(vci_path, 'Road_Slope_rad/1', 'Sin_Slope/1');
add_line(vci_path, 'Sin_Slope/1', 'Gain_Slope/1');

% F. Somme des forces : F_req = F_inertia + F_aero + F_roll + F_slope
add_block('simulink/Math Operations/Sum', [vci_path '/Sum_Forces'], ...
    'Inputs', '++++', 'Position', [360, 30, 395, 240]);

add_line(vci_path, 'Gain_Inertia/1', 'Sum_Forces/1');
add_line(vci_path, 'Gain_Aero/1', 'Sum_Forces/2');
add_line(vci_path, 'Const_Roll/1', 'Sum_Forces/3');
add_line(vci_path, 'Gain_Slope/1', 'Sum_Forces/4');

% G. Calcul des commandes brutes d'actionneurs
add_block('simulink/Math Operations/Gain', [vci_path '/Gain_Throttle_Raw'], ...
    'Gain', 'rw / (2200 * eff_drive)', 'Position', [450, 55, 560, 85]);
add_block('simulink/Math Operations/Gain', [vci_path '/Gain_Brake_Raw'], ...
    'Gain', '-rw / max_brake_torque', 'Position', [450, 185, 560, 215]);

add_line(vci_path, 'Sum_Forces/1', 'Gain_Throttle_Raw/1');
add_line(vci_path, 'Sum_Forces/1', 'Gain_Brake_Raw/1');

% H. Logique de répartition exclusive par Switchs (avec zone morte de 15 N)
% Switch Propulsion
add_block('simulink/Signal Routing/Switch', [vci_path '/Switch_Throttle'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '15', 'Position', [620, 45, 650, 105]);
add_block('simulink/Sources/Constant', [vci_path '/Const_Zero_Th'], 'Value', '0', 'Position', [580, 90, 600, 105]);

add_line(vci_path, 'Gain_Throttle_Raw/1', 'Switch_Throttle/1');
add_line(vci_path, 'Sum_Forces/1', 'Switch_Throttle/2');
add_line(vci_path, 'Const_Zero_Th/1', 'Switch_Throttle/3');

% Gain de négation pour le contrôle du switch freinage (Simulink ne supportant que u2 > seuil)
add_block('simulink/Math Operations/Gain', [vci_path '/Gain_Neg_Forces'], ...
    'Gain', '-1', 'Position', [450, 125, 500, 145]);
add_line(vci_path, 'Sum_Forces/1', 'Gain_Neg_Forces/1');

% Switch Freinage
add_block('simulink/Signal Routing/Switch', [vci_path '/Switch_Brake'], ...
    'Criteria', 'u2 > Threshold', 'Threshold', '15', 'Position', [620, 175, 650, 235]);
add_block('simulink/Sources/Constant', [vci_path '/Const_Zero_Br'], 'Value', '0', 'Position', [580, 220, 600, 235]);

add_line(vci_path, 'Gain_Brake_Raw/1', 'Switch_Brake/1');
add_line(vci_path, 'Gain_Neg_Forces/1', 'Switch_Brake/2');
add_line(vci_path, 'Const_Zero_Br/1', 'Switch_Brake/3');

% I. Saturations de sécurité [0, 1]
add_block('simulink/Discontinuities/Saturation', [vci_path '/Sat_Throttle'], ...
    'UpperLimit', '1', 'LowerLimit', '0', 'Position', [690, 60, 720, 90]);
add_block('simulink/Discontinuities/Saturation', [vci_path '/Sat_Brake'], ...
    'UpperLimit', '1', 'LowerLimit', '0', 'Position', [690, 190, 720, 220]);

add_line(vci_path, 'Switch_Throttle/1', 'Sat_Throttle/1');
add_line(vci_path, 'Switch_Brake/1', 'Sat_Brake/1');

% J. Sorties de la VCI
add_block('simulink/Sinks/Out1', [vci_path '/Throttle_Normalized'], 'Position', [760, 65, 790, 80]);
add_block('simulink/Sinks/Out1', [vci_path '/Brake_Normalized'], 'Position', [760, 195, 790, 210]);

add_line(vci_path, 'Sat_Throttle/1', 'Throttle_Normalized/1');
add_line(vci_path, 'Sat_Brake/1', 'Brake_Normalized/1');

%% 7. Ajout du Modèle Physique Ego (Model Reference)
add_block('simulink/Ports & Subsystems/Model', [model_name '/Vehicle_Dynamics'], ...
    'ModelName', 'Vehicle_Dynamics', 'Position', [960, 210, 1100, 310]);

%% 8. Blocs Intermédiaires pour Rebouclage & Calculs Physiques
% A. Calcul de la distance de sécurité : D_safe = V_ego * Set_Gap + d_safe_min
add_block('simulink/Math Operations/Product', [model_name '/Prod_D_safe'], 'Position', [450, 270, 480, 300]);
add_block('simulink/Math Operations/Sum', [model_name '/Sum_D_safe'], 'Inputs', '++', 'Position', [510, 275, 540, 305]);
add_block('simulink/Sources/Constant', [model_name '/Const_d_safe_min'], 'Value', 'd_safe_min', 'Position', [450, 320, 480, 340]);

add_line(model_name, 'Const_Set_Gap/1', 'Prod_D_safe/1');
add_line(model_name, 'Prod_D_safe/1', 'Sum_D_safe/1');
add_line(model_name, 'Const_d_safe_min/1', 'Sum_D_safe/2');

% B. Calcul du Time_Gap_Actual : Gap = Clearance / V_ego_saturated
% Bloc de saturation de vitesse minimale pour éviter la division par zéro à l'arrêt (ISO 26262 / Sécurité active)
add_block('simulink/Discontinuities/Saturation', [model_name '/Sat_V_ego_Min'], ...
    'UpperLimit', '100', 'LowerLimit', '0.5', 'Position', [180, 340, 210, 370]);

add_block('simulink/Math Operations/Divide', [model_name '/Div_Gap_Actual'], 'Inputs', '*/', 'Position', [240, 280, 270, 310]);

add_line(model_name, 'From_Workspace_D_rel/1', 'Div_Gap_Actual/1');

%% 9. CÂBLAGE COMPLET EN BOUCLE FERMÉE (Lignes Globales)

% 1. Câblage des entrées constantes du Mode Manager (Stateflow)
add_line(model_name, 'Const_CruiseSwitch/1', 'ACC_Mode_Manager/1');
add_line(model_name, 'Const_SetSwitch/1', 'ACC_Mode_Manager/2');
add_line(model_name, 'Const_BrakePedal/1', 'ACC_Mode_Manager/3');
add_line(model_name, 'Const_LeadVehicle_Detected/1', 'ACC_Mode_Manager/5');
add_line(model_name, 'Div_Gap_Actual/1', 'ACC_Mode_Manager/6');
add_line(model_name, 'Const_Set_Gap/1', 'ACC_Mode_Manager/7');

% 2. Câblage des entrées du Contrôleur PID (ACC_Controller)
add_line(model_name, 'Const_V_set/1', 'ACC_Controller/1');          % Port 1 : V_set
add_line(model_name, 'From_Workspace_D_rel/1', 'ACC_Controller/3');  % Port 3 : D_rel
add_line(model_name, 'Sum_D_safe/1', 'ACC_Controller/4');            % Port 4 : D_safe
add_line(model_name, 'Const_Set_Gap/1', 'ACC_Controller/5');         % Port 5 : Set_Gap
add_line(model_name, 'ACC_Mode_Manager/1', 'ACC_Controller/6');     % Port 6 : ACC_Mode

% 3. Câblage du Contrôleur vers la VCI
add_line(model_name, 'ACC_Controller/1', 'Vehicle_Control_Interface/1');

% 4. Câblage de la Pente (Constant) vers la VCI et vers la Physique
add_line(model_name, 'Const_Road_Slope/1', 'Vehicle_Control_Interface/3');
add_line(model_name, 'Const_Road_Slope/1', 'Vehicle_Dynamics/3');

% 5. Câblage du VCI vers le modèle physique de Dynamique
add_line(model_name, 'Vehicle_Control_Interface/1', 'Vehicle_Dynamics/1'); % Throttle
add_line(model_name, 'Vehicle_Control_Interface/2', 'Vehicle_Dynamics/2'); % Brake

% 6. RETOUR DE VITESSE (V_ego) - LE BOUCLAGE MAJEUR AVEC CASSE DE BOUCLE ALGÉBRIQUE (Unit Delay)
% Ajout d'un bloc Unit Delay de 10 ms (pas d'échantillonnage de 0.01s) pour casser la boucle algébrique
% induite par le rebouclage instantané de la vitesse réelle vers les contrôleurs et le manager Stateflow.
add_block('simulink/Discrete/Unit Delay', [model_name '/Delay_V_ego'], ...
    'SampleTime', '0.01', 'Position', [1130, 160, 1160, 190]);

% Relier la sortie vitesse brute de Vehicle_Dynamics vers l'entrée du Unit Delay
add_line(model_name, 'Vehicle_Dynamics/1', 'Delay_V_ego/1');

% Relier la vitesse retardée (V_ego) en retour vers les blocs récepteurs directs
add_line(model_name, 'Delay_V_ego/1', 'Vehicle_Control_Interface/2');
add_line(model_name, 'Delay_V_ego/1', 'ACC_Controller/2');
add_line(model_name, 'Delay_V_ego/1', 'ACC_Mode_Manager/4');

% Rebouclage avec saturation de vitesse minimale (0.5 m/s) pour le calcul du Time Gap
% et de la distance de sécurité D_safe pour éviter la divergence à l'arrêt complet
add_line(model_name, 'Delay_V_ego/1', 'Sat_V_ego_Min/1');
add_line(model_name, 'Sat_V_ego_Min/1', 'Div_Gap_Actual/2');
add_line(model_name, 'Sat_V_ego_Min/1', 'Prod_D_safe/2');

%% 10. Ajout des Ports de Sortie Globaux (pour l'extraction de résultats)
add_block('simulink/Sinks/Out1', [model_name '/Out_V_ego'], 'Position', [1200, 150, 1230, 165]);
add_block('simulink/Sinks/Out1', [model_name '/Out_ACC_Mode'], 'Position', [1200, 200, 1230, 215]);
add_block('simulink/Sinks/Out1', [model_name '/Out_Throttle'], 'Position', [1200, 250, 1230, 265]);
add_block('simulink/Sinks/Out1', [model_name '/Out_Brake'], 'Position', [1200, 300, 1230, 315]);
add_block('simulink/Sinks/Out1', [model_name '/Out_D_rel'], 'Position', [1200, 350, 1230, 365]);

% Câblage des ports de sortie globaux
add_line(model_name, 'Vehicle_Dynamics/1', 'Out_V_ego/1');
add_line(model_name, 'ACC_Mode_Manager/1', 'Out_ACC_Mode/1');
add_line(model_name, 'Vehicle_Control_Interface/1', 'Out_Throttle/1');
add_line(model_name, 'Vehicle_Control_Interface/2', 'Out_Brake/1');
add_line(model_name, 'From_Workspace_D_rel/1', 'Out_D_rel/1');

% Configuration du callback InitFcn pour ré-importer les données de perception après le clear destructif d'init_params
set_param(model_name, 'InitFcn', 'init_params; import_fusion_data;');

% Configuration du solveur en Pas Fixe Synchrone de 10 ms (Standard MIL/SIL ADAS)
% pour éliminer le bruit numérique de dérivation, les itérations d'oscillations du solveur continu
% et garantir la parfaite synchronisation des signaux discrets [100% conformes].
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode3');
set_param(model_name, 'FixedStep', '0.01');

%% 11. Enregistrement, Layout et Clôture
save_system(model_name);
fprintf('>> Succès ! Modèle en boucle fermée "%s.slx" entièrement assemblé et enregistré.\n', model_name);
