%% validate_vci_logic.m - Script de Validation Intermédiaire pour le VCI
% Ce script valide mathématiquement et physiquement la logique d'inversion
% dynamique et de répartition de commande (VCI) sur différents profils de conduite.

clear; clc; close all;

%% 1. Chargement des paramètres du projet
% Ajustement du chemin pour exécuter le script depuis n'importe quel dossier
script_path = fileparts(mfilename('fullpath'));
addpath(script_path);

% Exécuter le script d'initialisation des paramètres existants
init_params;

% Redéfinir script_path car init_params contient un "clear" qui vide le workspace
script_path = fileparts(mfilename('fullpath'));

fprintf('>> Script de validation VCI initialisé.\n');

%% 2. Calcul de la masse effective (m_eff)
% Masse effective prenant en compte l'inertie linéaire et de rotation
m_eff = m + (Jw / rw^2) + (Jm * ig^2 / rw^2);
fprintf('   - Masse à vide (m): %.1f kg\n', m);
fprintf('   - Masse effective (m_eff): %.2f kg (Inerties roues & moteur incluses)\n', m_eff);

%% 3. Définition du Scénario de Test Temporel
% Durée et pas de temps
t_end = 40; % [s]
Ts = 0.01;  % [s]
t = (0:Ts:t_end)';
N = length(t);

% Profil de vitesse ego (v) - commence à 50 km/h et évolue
v = zeros(N, 1);
v(1) = 50 * kmh_to_ms;

% Profil d'accélération commandée (a_cmd)
a_cmd = zeros(N, 1);
% Phase 1 : Accélération modérée (0.8 m/s2)
a_cmd(t >= 2 & t < 10) = 0.8;
% Phase 2 : Maintien de vitesse (0 m/s2)
a_cmd(t >= 10 & t < 15) = 0;
% Phase 3 : Forte accélération proche de la limite (1.8 m/s2)
a_cmd(t >= 15 & t < 22) = 1.8;
% Phase 4 : Décélération douce (-0.5 m/s2)
a_cmd(t >= 22 & t < 28) = -0.5;
% Phase 5 : Fort freinage à la limite de confort (-1.9 m/s2)
a_cmd(t >= 28 & t < 35) = -1.9;
% Phase 6 : Stabilisation (0 m/s2)
a_cmd(t >= 35) = 0;

% Profil de pente (Road Slope in percent, converted to radians)
slope_percent = zeros(N, 1);
slope_percent(t >= 12 & t < 25) = 6.0;   % Pente montante de 6%
slope_percent(t >= 25 & t < 33) = -4.0;  % Pente descendante de 4%
slope_rad = atan(slope_percent / 100);

% Simulation simplifiée de l'évolution de la vitesse pour le calcul des résistances
for k = 1:N-1
    v(k+1) = v(k) + a_cmd(k) * Ts;
    if v(k+1) < 0
        v(k+1) = 0; % Vitesse ne peut pas être négative
    end
end

%% 4. Algorithme VCI - Calcul des Forces et Répartition
% Allocations des vecteurs de résultats
F_inertia = zeros(N, 1);
F_aero = zeros(N, 1);
F_roll = zeros(N, 1);
F_slope = zeros(N, 1);
F_req = zeros(N, 1);
u_th = zeros(N, 1);
u_br = zeros(N, 1);

% Paramètres de gain de traction et de freinage de la VCI (modèle inverse)
K_prop = (2200 * eff_drive) / rw;
K_brake = max_brake_torque / rw;

% Zone morte (deadband) en Newtons pour éviter le chattering
deadband = 15.0; 

for k = 1:N
    % A. Calcul de chaque composante de force physique
    F_inertia(k) = m_eff * a_cmd(k);
    F_aero(k) = 0.5 * rho_air * Cd * A_front * (v(k)^2);
    F_roll(k) = m * g * f_roll;
    F_slope(k) = m * g * sin(slope_rad(k));
    
    % B. Sommation pour obtenir la force totale requise aux roues
    F_req(k) = F_inertia(k) + F_aero(k) + F_roll(k) + F_slope(k);
    
    % C. Logique de répartition exclusive avec Hystérésis / Zone Morte
    if F_req(k) > deadband
        % Traction requise
        u_th(k) = F_req(k) / K_prop;
        u_br(k) = 0;
    elseif F_req(k) < -deadband
        % Freinage requis
        u_th(k) = 0;
        u_br(k) = -F_req(k) / K_brake;
    else
        % Zone morte : roue libre (neutre)
        u_th(k) = 0;
        u_br(k) = 0;
    end
    
    % D. Saturation stricte [0, 1] (ISO 26262 / ASIL B)
    u_th(k) = min(1.0, max(0.0, u_th(k)));
    u_br(k) = min(1.0, max(0.0, u_br(k)));
end

%% 5. Vérification Formelle de Sécurité
overlap_detected = any((u_th > 0.001) & (u_br > 0.001));
out_of_bounds = any(u_th < 0 | u_th > 1 | u_br < 0 | u_br > 1);

fprintf('\n================ RAPPORT D''AUDIT SÉCURITÉ VCI ================\n');
if ~overlap_detected
    fprintf('✅ INTERVERROUILLAGE VALIDÉ : Aucun chevauchement (u_th * u_br == 0) détecté.\n');
else
    warning('❌ DÉFAUT D''INTERVERROUILLAGE : Activation simultanée détectée !');
end

if ~out_of_bounds
    fprintf('✅ SATURATION DES ACTIONNEURS VALIDÉE : u_th et u_br restent strictement dans [0, 1].\n');
else
    warning('❌ DÉFAUT DE SATURATION : Commande hors limites détectée !');
end
fprintf('===============================================================\n\n');

%% 6. Visualisation des Performances
fig = figure('Name', 'Validation Physique de la VCI', 'Position', [100, 100, 1000, 750]);

% Subplot 1 : Consigne d'accélération et vitesse résultante
subplot(3, 1, 1);
yyaxis left;
plot(t, a_cmd, 'b-', 'LineWidth', 2);
ylabel('Accélération Demandée [m/s^2]');
grid on; hold on;
yline(a_max, '--r', 'Acc Max Limit');
yline(a_min, '--r', 'Dec Max Limit');
title('Scénario de Test : Cinématique et Profil Routier');

yyaxis right;
plot(t, v * ms_to_kmh, 'g-', 'LineWidth', 1.8);
ylabel('Vitesse Ego Simulée [km/h]');
legend('a_{cmd}', 'Limite Haute', 'Limite Basse', 'Vitesse Ego', 'Location', 'northwest');

% Subplot 2 : Décomposition des Forces Physiques
subplot(3, 1, 2);
plot(t, F_req, 'k-', 'LineWidth', 2); hold on;
plot(t, F_inertia, 'b--', 'LineWidth', 1.2);
plot(t, F_aero, 'r--', 'LineWidth', 1.2);
plot(t, F_roll, 'm--', 'LineWidth', 1.2);
plot(t, F_slope, 'c--', 'LineWidth', 1.2);
grid on;
ylabel('Forces aux Roues [N]');
title('Bilan des Forces et Modèle Physique Inverse');
legend('F_{req} (Totale)', 'F_{inertie}', 'F_{aéro}', 'F_{roulement}', 'F_{pente}', 'Location', 'northwest');

% Subplot 3 : Signaux d'actionneurs (Pédales)
subplot(3, 1, 3);
plot(t, u_th * 100, 'g-', 'LineWidth', 2); hold on;
plot(t, u_br * 100, 'r-', 'LineWidth', 2);
grid on;
ylabel('Ouverture / Pression [%]');
xlabel('Temps [s]');
title('Signaux d''Actionneurs Normalisés (Sorties VCI)');
legend('Papillon Gaz (u_{th})', 'Pression Frein (u_{br})', 'Location', 'northwest');

% Sauvegarde de la figure de validation
results_dir = fullfile(script_path, '..', 'Results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
saveas(fig, fullfile(results_dir, 'VCI_validation_test.jpg'));
fprintf('>> Graphique de performance sauvegardé dans : %s\n', fullfile(results_dir, 'VCI_validation_test.jpg'));
