% Untersuche die Statistik des Erfolgs der inversen Kinematik für alle
% seriellen Roboter
% 
% Siehe auch: serroblib_gen_bitarrays.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-12
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Einstellungen
ntest_Par = 10;
ntest_Kon = 20;
ntryIK = 10;
s_3T3R = struct('retry_limit', 1, 'scale_lim', 0.0, 'normalize', true);
use_mex = true;

resdir = fileparts(which('serrob_ik_histogram.m'));
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

fprintf('Berechne die IK-Statistik für %d Roboter\n', length(II));
%% Alle Roboter durchgehen
% Kugelgelenke enden und kinematisch unterschiedlich sind
ii = 0;
t0 = tic();
for iFK = II'
  t1 = tic();
  ii = ii + 1;
  IKtry_ii = NaN(ntest_Par, ntest_Kon);
  IKerg_ii = zeros(ntest_Par, ntest_Kon);
  
  Name = l.Names_Ndof{iFK};
  % Zufällige Parameter
%   try
    RS = serroblib_create_robot_class(Name);
%   catch
%     warning('Geht nicht: %s', Name);
%     break
%   end
  continue
  RS.fill_fcn_handles(use_mex, false);
  if use_mex % Kompiliere nur die eine Funktion, die benötigt wird
    for fcnname = {'convert_par2_MPV_fixb', 'invkin_eulangresidual'}
      matlabfcn2mex({sprintf('%s_%s', Name, fcnname{1})});
    end
  end
  % Grenzen für Gelenk-Koordinaten festlegen
  RS.qlim(RS.MDH.sigma==0,:) = repmat([-pi, pi], sum(RS.MDH.sigma==0), 1);
  RS.qlim(RS.MDH.sigma==1,:) = repmat([-0.5, 0.5], sum(RS.MDH.sigma==1), 1);
  % IK-Zufallswerte generieren
  t2 = tic();
  for kkpar = 1:ntest_Par
    TSS = RS.gen_testsettings(true, true);
    for i = 1:ntest_Kon
      q = TSS.Q(i,:)'; 
      T_E = RS.fkineEE(q); % Ziel-Pose (erreichbar, da aus direkter Kin.)
      xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
      for j = 1:ntryIK
        q0 = RS.qlim(:,1) + rand(RS.NQJ,1).*(RS.qlim(:,2)-RS.qlim(:,1));
        [q_ik,Phi] = RS.invkin2(xE, q0, s_3T3R);
        if max(abs(Phi)) < 1e-7
          % IK erfolgreich
          IKtry_ii(kkpar, i) = j;
          if all(q_ik>RS.qlim(:,1)) && all(q_ik<RS.qlim(:,2))
            IKerg_ii(kkpar, i) = 1;
          else
            IKerg_ii(kkpar, i) = 2;
          end
          break;
        else
          continue
        end
      end % Über erlaubte Versuche für IK
    end % Über verschiedene Konfigurationen
    t_act = toc(t2);
    T_per_par = t_act/kkpar;
    fprintf('%d Parameter in %1.1fs gerechnet. Voraussichtlich noch %1.1fs für die restlichen %d für %s\n', ...
      kkpar, t_act, T_per_par*(ntest_Par-kkpar), ntest_Par-kkpar, Name);
  end % Über verschiedene Kinematikparameter
  fprintf('%d Roboter bis jetzt gerechnet. Dauer bis jetzt: %1.1fs %s in %1.1fs abgeschlossen. Restdauer ca. %1.1fs für %d Roboter.\n', ...
    ii, toc(t0), Name, toc(t1), toc(t0)/ii*(length(II)-ii), length(II)-ii);
  if iFK > 5
    return
  end
end