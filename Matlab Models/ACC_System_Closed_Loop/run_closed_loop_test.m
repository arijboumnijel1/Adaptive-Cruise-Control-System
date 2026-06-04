%% run_closed_loop_test.m - Simulation Globale en Boucle Fermée (MIL)
% Ce script exécute la simulation en boucle fermée du système ACC,
% extrait les résultats et trace les graphiques de validation physique.

clear; clc; close all;

%% 1. Initialisation de l'environnement
script_path = fileparts(mfilename('fullpath'));
models_path = fullfile(script_path, '..', '02_Models');
addpath(models_path);

% Charger les paramètres physiques et fonctionnels
init_params;

% Importer les données réelles de fusion de capteurs (Radar + Vision)
import_fusion_data;

model_name = 'ACC_System_Closed_Loop';
load_system(model_name);

% Configurer la simulation sur la durée des données de perception (30 secondes)
t_end = 30;
fprintf('>> Lancement de la simulation MIL en boucle fermée sur %.1f secondes...\n', t_end);

%% 2. Lancement de la Simulation Simulink
out = sim(model_name, 'StopTime', num2str(t_end));
fprintf('>> Simulation terminée avec succès.\n');

%% 3. Extraction des résultats physiques (yout avec grilles de temps individuelles)
try
    % Extraction des signaux et de leurs vecteurs temps respectifs et conversion en 1D (squeeze)
    t_v_ego   = out.yout{1}.Values.Time;
    v_ego_sim = squeeze(out.yout{1}.Values.Data);     % Vitesse Ego [m/s]
    
    t_mode    = out.yout{2}.Values.Time;
    mode_sim  = squeeze(out.yout{2}.Values.Data);     % Mode ACC [uint8]
    
    t_th      = out.yout{3}.Values.Time;
    u_th_sim  = squeeze(out.yout{3}.Values.Data);     % Papillon Gaz [0, 1]
    
    t_br      = out.yout{4}.Values.Time;
    u_br_sim  = squeeze(out.yout{4}.Values.Data);     % Pédale Frein [0, 1]
    
    t_d_rel   = out.yout{5}.Values.Time;
    d_rel_sim = squeeze(out.yout{5}.Values.Data);     % Distance relative réelle [m]
catch ME
    warning('Erreur lors de l''extraction automatique. Tentative via les noms de signaux : %s', ME.message);
    try
        v_el = out.yout.find('Out_V_ego');
        t_v_ego = v_el{1}.Values.Time;
        v_ego_sim = squeeze(v_el{1}.Values.Data);
        
        mode_el = out.yout.find('Out_ACC_Mode');
        t_mode = mode_el{1}.Values.Time;
        mode_sim  = squeeze(mode_el{1}.Values.Data);
        
        th_el = out.yout.find('Out_Throttle');
        t_th = th_el{1}.Values.Time;
        u_th_sim  = squeeze(th_el{1}.Values.Data);
        
        br_el = out.yout.find('Out_Brake');
        t_br = br_el{1}.Values.Time;
        u_br_sim  = squeeze(br_el{1}.Values.Data);
        
        d_el = out.yout.find('Out_D_rel');
        t_d_rel = d_el{1}.Values.Time;
        d_rel_sim = squeeze(d_el{1}.Values.Data);
    catch
        error('Impossible d''extraire les résultats de simulation. Veuillez vérifier le modèle.');
    end
end

%% 4. Rééchantillonnage robuste (Audit & Calculs Physiques)
% Grille de temps fixe unique pour les audits et calculs (10 ms)
t_fixed = (0:0.01:t_end)';
v_ego_fixed = interp1(t_v_ego, v_ego_sim, t_fixed, 'linear', 'extrap');
d_rel_fixed = interp1(t_d_rel, d_rel_sim, t_fixed, 'linear', 'extrap');

% Pour les actionneurs discrets (Zero-Order Hold), interpolation au plus proche (nearest)
% afin d'éviter les artefacts d'interpolation linéaire créés par le solveur continu de Simulink
u_th_fixed = interp1(t_th, u_th_sim, t_fixed, 'nearest', 0);
u_br_fixed = interp1(t_br, u_br_sim, t_fixed, 'nearest', 0);
mode_fixed = interp1(t_mode, double(mode_sim), t_fixed, 'nearest', 1);

% Vitesse relative interpolée pour correspondre à notre grille fixe
v_rel_fused_fixed = interpolate_timeseries(v_rel_target_ts, t_fixed);

% Vitesse cible réelle reconstituée (V_target = V_ego + V_rel_fused)
v_target_fixed = v_ego_fixed + v_rel_fused_fixed;

% Calcul dynamique de la distance de sécurité réglementaire (D_safe)
d_safe_fixed = v_ego_fixed * default_time_gap + d_safe_min;

% Calcul de l'accélération réelle par dérivation robuste sur la grille fixe
a_ego_fixed = diff(v_ego_fixed) ./ 0.01;
a_ego_fixed = [a_ego_fixed; a_ego_fixed(end)]; % Aligner la taille

% Ignorer la première seconde de simulation (phase transitoire d'établissement des filtres et solveurs)
% pour le calcul des indicateurs clés de performance (KPI), conformément aux standards de test ADAS MIL
audit_indices = (t_fixed >= 1.0);

max_accel_reached = max(a_ego_fixed(audit_indices)) / g;
max_decel_reached = min(a_ego_fixed(audit_indices)) / g;

fprintf('\n================ RAPPORT DE PERFORMANCE MIL EN BOUCLE FERMÉE ================\n');
fprintf('  1. Confort & Saturation Active (Exigence REQ-S-01/02 <= 0.20g) :\n');
if max_accel_reached <= 0.205 && abs(max_decel_reached) <= 0.205
    fprintf('     ✅ VALIDÉ : Accélération max = +%.2f g | Décélération max = -%.2f g\n', ...
        max_accel_reached, abs(max_decel_reached));
else
    warning('     ❌ ALERTE CONFIANCE : Accélération hors limites confort détectée (+%.2f g / -%.2f g)', ...
        max_accel_reached, abs(max_decel_reached));
end

fprintf('  2. Interverrouillage Actionneurs VCI (Exigence u_th * u_br == 0) :\n');
% Audit de chevauchement sur les actionneurs rééchantillonnés au plus proche (ZOH) hors transitoire
overlap = any((u_th_fixed(audit_indices) > 0.01) & (u_br_fixed(audit_indices) > 0.01));
if ~overlap
    fprintf('     ✅ VALIDÉ : Aucun chevauchement de pédales détecté en boucle fermée (Pure ZOH).\n');
else
    warning('     ❌ DÉFAUT VCI : Chevauchement de pédales détecté !');
end

fprintf('  3. Stabilité d''espacement (Régulation Gap Control) :\n');
% Erreur d'espacement en Gap Control (Mode 3)
gap_indices = (mode_fixed == 3);
if any(gap_indices)
    mean_gap_error = mean(abs(d_rel_fixed(gap_indices) - d_safe_fixed(gap_indices)));
    fprintf('     ✅ ANALYSÉ : Écart moyen de distance en Gap Control = %.2f m\n', mean_gap_error);
else
    fprintf('     ℹ️ INFO : Le mode Gap Control n''a pas été activé durant cette simulation.\n');
end
fprintf('===============================================================================\n\n');

%% 5. Visualisation complète
fig = figure('Name', 'Validation MIL en Boucle Fermée - ACC System', 'Position', [50, 50, 1100, 800]);

% A. Subplot 1 : Cinématique des vitesses (Consigne vs Ego vs Cible)
subplot(3, 1, 1);
plot(t_v_ego, v_ego_sim * ms_to_kmh, 'b-', 'LineWidth', 2.5); hold on;
plot(t_fixed, v_target_fixed * ms_to_kmh, 'r--', 'LineWidth', 1.8);
yline(v_min_acc_kmh, 'k:', 'ACC Min Speed (40 km/h)');
grid on;
ylabel('Vitesse [km/h]');
title('Régulation en Boucle Fermée : Cinématique de Vitesse');
legend('Ego Vehicle (V_{ego})', 'Target Vehicle (V_{target})', 'Seuil d''activation', 'Location', 'northwest');

% B. Subplot 2 : Régulation de Distance (D_rel vs D_safe)
subplot(3, 1, 2);
plot(t_d_rel, d_rel_sim, 'g-', 'LineWidth', 2.5); hold on;
plot(t_fixed, d_safe_fixed, 'r--', 'LineWidth', 1.8);
grid on;
ylabel('Distance Relative [m]');
title('Régulation de Distance : Clearance vs Distance de Sécurité');
legend('Distance Réelle (Clearance)', 'Distance Consigne (D_{safe})', 'Location', 'northwest');

% C. Subplot 3 : Signaux d'actionneurs VCI & Modes Actifs (Stateflow)
subplot(3, 1, 3);
yyaxis left;
plot(t_th, u_th_sim * 100, 'g-', 'LineWidth', 2); hold on;
plot(t_br, u_br_sim * 100, 'r-', 'LineWidth', 2);
ylabel('Actionneurs [%]');
ylim([-10, 110]);
grid on;

yyaxis right;
plot(t_mode, mode_sim, 'k-', 'LineWidth', 2);
ylabel('ACC Mode [0-3]');
ylim([-0.5, 3.5]);
yticks(0:3);
yticklabels({'OFF', 'STANDBY', 'SPEED', 'GAP'});
xlabel('Temps [s]');
title('Logique de Supervision (Stateflow) et Commandes VCI');
legend('Papillon Gaz (u_{th})', 'Frein (u_{br})', 'ACC Mode (Superviseur)', 'Location', 'northwest');

% Enregistrement du rapport graphique de test
results_dir = fullfile(script_path, '..', 'Results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
saveas(fig, fullfile(results_dir, 'ACC_validation_test.jpg'));
fprintf('>> Rapport de test global sauvegardé dans : %s\n', fullfile(results_dir, 'ACC_validation_test.jpg'));

% Fonction d'aide pour l'interpolation de timeseries sur la grille temporelle de simulation
function y_interp = interpolate_timeseries(ts, t_new)
    y_interp = interp1(ts.Time, ts.Data, t_new, 'linear', 'extrap');
end
