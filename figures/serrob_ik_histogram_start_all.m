% Startskript für die Histogramm-Auswertung:
% Starte alle Matlab-Skripte für verschiedene Einstellungen der Histogramme

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear
% Debug: Bei Änderung des IK-Algorithmus alle Vorlagen neu erzeugen.
% serroblib_create_template_functions({}, false, false);
usr_load_data = false; % Set to true, if existing results data shall only be reloaded for plotting
start_dof_values = {'3T3R', '3T2R'};
start_q0range_values = [0.2, 1.0];
usr_extstart = true;
% Funktionen für ausgewählte Roboter neu generieren
usr_create_template_functions = false;
% Auswahl der Roboter für Histogramm. Leer: Alle.
usr_whitelist_robots = {};
% usr_whitelist_robots = {'S6RRRRRP9'};
% usr_whitelist_robots = {'S6RRPRRR14','S6RRRRRR10', 'S6RRRRRR3', 'S6RRRRRR5'};
% usr_whitelist_robots = {'S6RRRRRP9', 'S6RRRRRP10', 'S6RRRRRP11', 'S6RRRRRR10'};
% usr_whitelist_robots = {'S6PRPRPR2'};

if usr_create_template_functions
  N = 6;
  serroblibpath=fileparts(which('serroblib_path_init.m'));
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
  Names_Ndof = l.Names_Ndof;
  I_novar = l.AdditionalInfo(:,2) == 0;
  if isempty(usr_whitelist_robots)
    template_gen_list = Names_Ndof(I_novar);
  else
    template_gen_list = usr_whitelist_robots;
  end
  serroblib_create_template_functions(template_gen_list, false, false); % Kompiliert wird in Histogramm-Skript
end
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