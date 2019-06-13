% Code-Generierung für die Funktionen, die für das Histogramm benötigt
% werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

respath = fileparts(which('serrob_ik_histogram.m'));

use_mex = true;
%% Init: Datenbank laden
N = 6;
EE_FG = logical([1 1 1 1 1 1]);
serroblibpath=fileparts(which('serroblib_path_init.m'));
mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
[~,I] = serroblib_filter_robots(N, EE_FG, true(1,6));
I_novar = l.AdditionalInfo(:,2) == 0;
I_ges = I&I_novar;
II = find(I_ges);
nRob = length(II);
% II = II([1, 50, 80, 200, 329])
% II = II([5, 60, 90, 250, 325, 327]);
% II = 85;
fprintf('Berechne die IK-Statistik für %d Roboter\n', length(II));
%% Alle Roboter durchgehen
% Kugelgelenke enden und kinematisch unterschiedlich sind
ii = 0;
t0 = tic();
% Zeilen: Laufende Nummer des Roboters
% Spalten: Kein Erfolg; Aufsteigende Anzahl Versuche bis Erfolg
% Einträge: Anzahl versuche
IK_hist_Anz_ges = NaN(length(II), 4);
IK_hist_Anz_mG_ges = NaN(length(II), 3); % Zähle, bei wie vielen die Grenzen verletzt wurden
ii1 = find(II == 508); II = II(ii1:end);
% Untersuche IK für alle Roboter
for iFK = II'
  t1 = tic();
  ii = ii + 1;
  % Statistik für inverse Kinematik
  IKtry_ii = NaN(ntest_Par, ntest_Kon); % Anzahl der Versuche für IK
  IKerg_ii = zeros(ntest_Par, ntest_Kon); % Status für IK: 0=kein Erfolg, 1=Erfolg, 2=Grenzen verletzt
  
  Name = l.Names_Ndof{iFK};
  RS = serroblib_create_robot_class(Name);
  RS.fill_fcn_handles(use_mex, false);
  % Kompiliere nur die Funktionen, die benötigt werden (falls sie
  % aktualisiert werden müssen)
  if use_mex 
    % Teste Mex-Funktionen
    for mextry = 1:3
      % 1: IK testen, bei Bedarf neu kompilieren
      % 2: IK testen, neu generieren (nicht kompilieren)
      % 3: IK testen und kompilieren
      err = false;
      try 
        RS.invkin2(rand(6,1), rand(RS.NQJ,1));
        RS.invkin2_traj(rand(1,6), rand(1,6), rand(1,6), 0, rand(RS.NQJ,1));
        RS.fkine(rand(RS.NQJ,1));
      catch
        err = true;
        if mextry ~= 2
          for fcnname = {'convert_par2_MPV_fixb', 'invkin_eulangresidual', ...
              'fkine_fixb_rotmat_mdh_sym_varpar', 'invkin_traj', 'jacobig_mdh_num'}
            matlabfcn2mex({sprintf('%s_%s', Name, fcnname{1})});
            % matlabfcn2mex({sprintf('%s_%s', Name, fcnname{1})}, false, true, true);
          end
        end
      end
      if err == false
        break;
      elseif mextry == 2
        % Debug: Code neu generieren
        serroblib_generate_code({Name}, true, false, 2)
      end
    end
  end
  fprintf('%d/%d: %s Initialisierung i.O.\n', ii, length(II), Name);
  continue
end
