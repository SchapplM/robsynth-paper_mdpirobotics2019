% Startskript für die Histogramm-Auswertung:
% Starte alle Matlab-Skripte für verschiedene Einstellungen der Histogramme

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

start_dof_values = {'3T3R', '3T2R'};
start_q0range_values = [0.2, 1.0];
usr_extstart = true;
for start_dof = 1:2
  for start_q0range = 1:2
    % Benutzer-Einstellungen setzen
    usr_DoF = start_dof_values{start_dof};
    usr_range_q0 = start_q0range_values(start_q0range);
    % Skript starten
    serrob_ik_histogram
  end
end

serrob_ik_histogram_combinedfigure