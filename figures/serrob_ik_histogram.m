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
sum_try = ntest_Par*ntest_Kon;
ntryIK = 10;
% Einstellungen für IK
% * retry_limit auf Null, damit die Anzahl der Versuche hier gezählt werden
%   können
% * scale_lim auf Null für 3T3R, da die Dämpfung der Grenzüberschreitung
%   sowieso keine Richtungsänderung bringt
% * normalize auf 1, da das Endergebnis zur Grenzbeurteilung im
%   Normal-Bereich liegen muss

s_3T3R = struct('retry_limit', 0, 'scale_lim', 0.0, 'normalize', true, ...
  'K', 0.6*ones(6,1), 'n_max', 1e3);
use_mex = true;
usr_debug_limits = false; % Untersuchung, warum Grenzen verletzt werden konnten.
usr_debug_nosuccess = false; % Untersuchung, warum die IK fehlschlägt
% Wiederhole versuche bei Grenzverletzung. Ziel: Nur Lösungen innerhalb der
% Grenzen werden gewertet 
usr_repeat_limviol = true; 

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

% Untersuche IK für alle Roboter
for iFK = II'
  t1 = tic();
  ii = ii + 1;
  % Statistik für inverse Kinematik
  IKtry_ii = NaN(ntest_Par, ntest_Kon); % Anzahl der Versuche für IK
  IKerg_ii = zeros(ntest_Par, ntest_Kon); % Status für IK: 0=kein Erfolg, 1=Erfolg, 2=Grenzen verletzt
  
  Name = l.Names_Ndof{iFK};
  

  % Zufällige Parameter
%   try
    RS = serroblib_create_robot_class(Name);
%   catch
%     warning('Geht nicht: %s', Name);
%     break
%   end
%   continue
  RS.fill_fcn_handles(use_mex, false);
  % Kompiliere nur die eine Funktion, die benötigt wird (falls sie
  % aktualisiert werden muss)
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
              'fkine_fixb_rotmat_mdh_sym_varpar', 'invkin_traj'}
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
        if max(abs(Phi)) < 1e-7 && all(~isnan(Phi))
          % IK erfolgreich
          IKtry_ii(kkpar, i) = j;
          if all(q_ik>RS.qlim(:,1)) && all(q_ik<RS.qlim(:,2))
            % Alle Grenzen wurden eingehalten
            IKerg_ii(kkpar, i) = 1;
          else
            % Grenzen wurden verletzt
            IKerg_ii(kkpar, i) = 2;
            if usr_debug_limits
              % debug: Ursache für Grenzverletzung
              RS.t2x(RS.fkineEE(q_ik)) - xE
              return
              RS.fill_fcn_handles(false)
            end
          end
          if usr_repeat_limviol && IKerg_ii(kkpar, i) == 2
            % Führe die IK nochmal aus, damit die Grenzen dann nicht
            % verletzt werden.
            continue
          else
            break;
          end
        elseif j == ntryIK
          % Beim letzten Versuch kein Erfolg
          continue
        else
          % Beim Nicht-letzten Versuch kein Erfolg
          if usr_debug_nosuccess
            warning on;
            warning('IK nicht erfolgreich\n');
            % s_3T3R.K = [ones(3,1)*0.1; ones(3,1)*0.3]
            return;
          else
            continue
          end
        end
      end % Über erlaubte Versuche für IK
    end % Über verschiedene Konfigurationen
    t_act = toc(t2);
    T_per_par = t_act/kkpar;
    fprintf('%d Parametersatz in %1.1fs gerechnet. Voraussichtlich noch %1.1fs für die restlichen %d für %s\n', ...
      kkpar, t_act, T_per_par*(ntest_Par-kkpar), ntest_Par-kkpar, Name);
  end % Über verschiedene Kinematikparameter
  % Zähle die Ergebnisse der inversen Kinematik und bilde Histogrammdaten
  % Erfolg beim 1.-x. Versuch
  for j = 1:size(IK_hist_Anz_ges,2)-2 % zwei Klassen Abzug: "niO" und "mehr als x"
    % Zähle, wie viele IK %d versuche brauchten
    IK_hist_Anz_ges(ii,j) = sum(IKtry_ii(:) == j);
  end
  % Erfolg bei mehr als x Versuche notwendig:
  IK_hist_Anz_ges(ii,end-1) = sum(IKtry_ii(:) > (size(IK_hist_Anz_ges,2)-2));
  % Anzahl kein Erfolg:
  IK_hist_Anz_ges(ii,end) = sum(IKerg_ii(:) == 0);
  
  % Anzahl Erfolg aufteilen in mit und ohne Grenzen
  IK_hist_Anz_mG_ges(ii,1) = sum(IKerg_ii(:) == 1); % Grenzen richtig, IK richtig
  IK_hist_Anz_mG_ges(ii,2) = sum(IKerg_ii(:) == 2); % Grenzen falsch, IK richtig
  IK_hist_Anz_mG_ges(ii,3) = sum(IKerg_ii(:) == 0); % IK falsch
  
  if sum(IK_hist_Anz_ges(ii,:)) ~= sum_try
    error('Summe für Histogramm stimmt nicht');
  end
  if sum(IK_hist_Anz_mG_ges(ii,:)) ~= sum_try
    error('Summe für Histogramm stimmt nicht');
  end
  
  fprintf('%d Roboter bis jetzt gerechnet. Dauer bis jetzt: %1.1fs %s in %1.1fs abgeschlossen. Restdauer ca. %1.1fs für %d Roboter.\n', ...
    ii, toc(t0), Name, toc(t1), toc(t0)/ii*(length(II)-ii), length(II)-ii);
end
% Prozentuale Angabe für die IK-Statistik
IK_hist_Ant_ges = IK_hist_Anz_ges / sum_try;
IK_hist_Ant_mG_ges = IK_hist_Anz_mG_ges / sum_try;
%% Ergebnisse plotten
figure(1);clf;hold on;
title('IK without regarding joint limits');
% IK_Hist_j = zeros(size(IK_hist_Ant_ges,1), 1);
legbarhdl = NaN(size(IK_hist_Ant_ges,2),1);
xticklabels = {};
for i = 1:size(IK_hist_Ant_ges,1)
  xticklabels{i} = l.Names_Ndof{II(i)};
end

for i = 1:size(IK_hist_Ant_ges,1)
  % Setze Startwert für Balkenhöhe auf Maximal-Zahl. Dadurch wird der
  % zuerst gezeichnete Balken immer voll
  IK_Hist_j = sum(IK_hist_Ant_ges(i,:));
  for j = size(IK_hist_Ant_ges,2):-1:1
    % Stapele alle bisherigen Balken: Ziehe die Höhe des aktuellen Balkens
    % von der laufenden Höhe ab, damit der neue, weiter vorne gezeichnete
    % Balken kleiner ist
    % Plotten
    fprintf('Rob %d; Balken %d: Höhe %1.1f\n', i, j, IK_Hist_j);
    legbarhdl(j) = bar(i, 100*IK_Hist_j);
    IK_Hist_j = IK_Hist_j - IK_hist_Ant_ges(i,j);
  end
  set(gca, 'ColorOrderIndex', 1)
end
ylabel('occurence in percent');
set(gca, 'xtick', 1:size(IK_hist_Ant_ges,1));
set(gca, 'xticklabel', xticklabels)
legend(legbarhdl, {'1.V.', '2.V.', '>2.V.', 'wrong'}, 'location', 'northoutside', 'orientation', 'horizontal');

figure(2);clf;hold on;
title('IK regarding joint limits');
% IK_Hist_j = zeros(size(IK_hist_Anz_mG_ges,1), 1);
legbarhdl = NaN(size(IK_hist_Ant_mG_ges,2),1);
for i = 1:size(IK_hist_Ant_mG_ges,1)
  % Setze Startwert für Balkenhöhe auf Maximal-Zahl. Dadurch wird der
  % zuerst gezeichnete Balken immer voll
  IK_Hist_j = sum(IK_hist_Ant_mG_ges(i,:));
  for j = size(IK_hist_Ant_mG_ges,2):-1:1
    % Stapele alle bisherigen Balken: Ziehe die Höhe des aktuellen Balkens
    % von der laufenden Höhe ab, damit der neue, weiter vorne gezeichnete
    % Balken kleiner ist
    % Plotten
    fprintf('Rob %d; Balken %d: Höhe %1.1f\n', i, j, IK_Hist_j);
    legbarhdl(j) = bar(i, 100*IK_Hist_j);
    IK_Hist_j = IK_Hist_j - IK_hist_Ant_mG_ges(i,j);
  end
  set(gca, 'ColorOrderIndex', 1)
end
ylabel('occurence in percent');
legend(legbarhdl, {'correct', 'limits violated', 'wrong'}, 'location', 'northoutside', 'orientation', 'horizontal');
set(gca, 'xtick', 1:size(IK_hist_Ant_ges,1));
set(gca, 'xticklabel', xticklabels)
