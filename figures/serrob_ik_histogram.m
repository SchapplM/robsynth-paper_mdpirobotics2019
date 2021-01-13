% Untersuche die Statistik des Erfolgs der inversen Kinematik für alle
% seriellen Roboter
% Das Skript kann von außen aufgerufen werden
% Einstellungen für die Untersuchungen:
% 

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover


if ~exist('usr_extstart', 'var')
  clear
  clc
end


%% Einstellungen
ntest_Par = 50;
ntest_Kon = 50;
sum_try = ntest_Par*ntest_Kon;
ntryIK = 20;
% Einstellungen für IK
% * retry_limit auf Null, damit die Anzahl der Versuche hier gezählt werden
%   können
% * scale_lim auf Null für 3T3R, da die Dämpfung der Grenzüberschreitung
%   sowieso keine Richtungsänderung bringt
% * normalize auf 1, da das Endergebnis zur Grenzbeurteilung im
%   Normal-Bereich liegen muss
% IK-Einstellungen: Nehme ansonsten Standard-Einstellungen
s_IK = struct('retry_limit', 0, 'scale_lim', 0.0, 'normalize', true, ...
  'n_max', 2e3, 'maxrelstep', 0.05, 'I_EE', logical([1 1 1 1 1 1]));
if strcmp(usr_DoF, '3T3R')
  s_IK.scale_lim = 0.0;
  s_IK.I_EE = logical([1 1 1 1 1 1]);
elseif strcmp(usr_DoF, '3T2R')
  s_IK.scale_lim = 0.0;
  s_IK.I_EE = logical([1 1 1 1 1 0]);
  s_IK.wn = [0.99;0.01;0.0]; % Keine Konditionszahl-NB
else
  error('EE-FG %s noch nicht implementiert', usr_DoF);
end
if ~exist('usr_DoF', 'var')
  usr_DoF = '3T3R'; % EE-FG für die IK-Auswertung
end
use_mex = true;
usr_debug_limits = false; % Untersuchung, warum Grenzen verletzt werden konnten.
usr_debug_nosuccess = false; % Untersuchung, warum die IK fehlschlägt
if ~exist('usr_range_q0', 'var')
  usr_range_q0 = 1; % Streuung der Startwerte in Prozent der Grenzen um Ist-Lage herum
end
if ~exist('usr_load_data', 'var')
  usr_load_data = false; % Generiere die IK-Statistik neu, ohne nur die Ergebnisse zu laden
end
if ~exist('usr_whitelist_robots', 'var')
  usr_whitelist_robots = {}; % Alle Roboter, falls nichts anderes vorgegeben
end
% Wiederhole versuche bei Grenzverletzung. Ziel: Nur Lösungen innerhalb der
% Grenzen werden gewertet
usr_repeat_limvioluntil = ntryIK-5; % Versuche bis 5 Versuche vor Ende ohne Grenzverletzung. Dann nehme GV in Kauf
usr_debug_robot = false;
usr_num_classes = 7; % Anzahl der Histogramm-Klassen für einzelne Versuche (3 -> 1., 2., 3., >3., wrong)
usr_num_leg_classes = 3;
usr_sort_numrotjoints = true;
usr_parcomp = true; % Parallele Berechnung (ca. Faktor 2 schneller)
usr_forcerecompilecheck = true;
respath = fileparts(which('serrob_ik_histogram.m'));

%% Init: Datenbank laden
N = 6;
EE_FG = logical([1 1 1 1 1 1]);
serroblibpath=fileparts(which('serroblib_path_init.m'));
mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
Names_Ndof = l.Names_Ndof;
[~,I] = serroblib_filter_robots(N, EE_FG, true(1,6));
I_novar = l.AdditionalInfo(:,2) == 0;

if isempty(usr_whitelist_robots)
  % Standard: Alle Roboter auswählen (dauert sehr lange)
  I_select = true(length(l.Names_Ndof),1);
else
  % Debug: Roboter auswählen für schnelle Generierung des Bildes.
  I_select = false(length(l.Names_Ndof),1);
  for i = 1:length(usr_whitelist_robots)
    I_select = I_select | strcmp(l.Names_Ndof(:), usr_whitelist_robots{i});
  end
  if sum(I_select) ~= length(usr_whitelist_robots)
    error('Eine der gesuchten Kinematiken ist nicht verfügbar');
  end
end
I_ges = I & I_novar & I_select;
II = find(I_ges);
II_orig = II;
nRob = length(II);
fprintf('Berechne die IK-Statistik für %d Roboter\n', length(II));
%% Alle Roboter durchgehen
% Zeilen: Laufende Nummer des Roboters
% Spalten: Kein Erfolg; Aufsteigende Anzahl Versuche bis Erfolg
% Einträge: Anzahl Versuche
IK_hist_Anz_ges = NaN(length(II), 3+usr_num_classes);
IK_hist_Anz_mG_ges = NaN(length(II), 3); % Zähle, bei wie vielen die Grenzen verletzt wurden
% ii1 = find(II == 455); II = II(1:244);
I_RobAusw = false(length(II),1);
nbars_hist = size(IK_hist_Anz_ges,2);
% Untersuche IK für alle Roboter
t0 = tic();
if usr_load_data
  II = []; % Damit die Schleife mit der Berechnung gar nicht erst gestartet wird
end
parfor ii = 1:length(II)
  iFK = II(ii);
  t1 = tic();
  % Statistik für inverse Kinematik
  IKtry_ii = NaN(ntest_Par, ntest_Kon); % Anzahl der Versuche für IK
  IKerg_ii = zeros(ntest_Par, ntest_Kon); % Status für IK: 0=kein Erfolg, 1=Erfolg, 2=Grenzen verletzt
  
  Name = Names_Ndof{iFK};
  err = false;

  % Zufällige Parameter
%   try
    RS = serroblib_create_robot_class(Name);
%   catch
%     warning('Geht nicht: %s', Name);
%     break
%   end
%   continue
  RS.gen_testsettings(true, true);
  RS.fill_fcn_handles(use_mex, false);
  % Kompiliere nur die eine Funktion, die benötigt wird (falls sie
  % aktualisiert werden muss)
  if use_mex 
    % Teste Mex-Funktionen
    for mextry = 1:3
      if mextry == 3 && usr_parcomp
        break
      end
      fprintf('[%d/%d] %s; Starte Versuch 1 zur Initialisierung\n', ...
        ii, nRob, Name);
      % 1: IK testen, bei Bedarf neu kompilieren
      % 2: IK testen, neu generieren (nicht kompilieren)
      % 3: IK testen und kompilieren
      err = false;
      try
        if usr_forcerecompilecheck && mextry == 1
          error('Dummy Error to enforce rechecking');
        end
        RS.invkin2(rand(6,1), rand(RS.NQJ,1), s_3T3R);
        RS.invkin2(rand(6,1), rand(RS.NQJ,1), s_3T2R);
        RS.fkine(rand(RS.NQJ,1));
        RS.jacobig(rand(RS.NQJ,1));        
      catch
        if ~usr_forcerecompilecheck
          fprintf('[%d/%d] %s; Versuch %d: Fehler beim Aufruf der kompilierten Funktionen\n', ...
            ii, nRob, Name, mextry);
        end
        err = true;
        if mextry ~= 2
          fcnnames = {'convert_par2_MPV_fixb', 'invkin_eulangresidual', ...
              'fkine_fixb_rotmat_mdh_sym_varpar', 'jacobig_mdh_num'};
          for kk = 1:length(fcnnames)
            matlabfcn2mex({sprintf('%s_%s', Name, fcnnames{kk})});
            % matlabfcn2mex({sprintf('%s_%s', Name, fcnname{1})}, false, true, true);
          end
        end
      end
      if err == false
        break;
      elseif mextry == 2 && ~usr_parcomp
        % Debug: Code neu generieren (wenn nicht Parallel gerechnet wird)
        serroblib_generate_code({Name}, true, false, 2);
      end
    end
  end
  if err
    fprintf('[%d/%d] %s Initialisierung fehlgeschlagen\n', ii, nRob, Name);
  else
    fprintf('[%d/%d] %s Initialisierung i.O.\n', ii, nRob, Name);
  end
  % Rang-Prüfung. Durch Fehler sind eventuell Roboter mit Rang < 6 enthalten
  maxrank = 0;
  J_test = NaN(6, RS.NJ); % Initialize to avoid parfor warning.
  for i_rt = 1:10
    RS.gen_testsettings(true, true);
    J_test = RS.jacobig(rand(RS.NQJ,1));
    rankJ = rank(J_test);
    if rankJ > maxrank
      maxrank = rankJ;
    end
  end
  if maxrank < 6
    fprintf('[%d/%d] Die Jacobi-Matrix hat nur Rang %d! Ignoriere %s\n', ...
      ii, nRob, maxrank, Name);
    continue
  end
  I_RobAusw(ii) = true; % Der Roboter wird tatsächlich untersucht.
  % Debuggen des Roboters
  if usr_debug_robot
    RS.fill_fcn_handles(false);
    RS.update_EE([0.1;0.2;0.3]);
    q_test = rand(RS.NQJ,1);
    
    rank(J_test)
    s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 0);
    figure(100);clf;
    hold on; grid on;
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    view(3);
    RS.plot( q_test, s_plot );
    title(sprintf('%s', Name));
  end
  
  % Grenzen für Gelenk-Koordinaten festlegen
  RS.qlim(RS.MDH.sigma==0,:) = repmat([-pi, pi], sum(RS.MDH.sigma==0), 1);
  RS.qlim(RS.MDH.sigma==1,:) = repmat([-0.5, 0.5], sum(RS.MDH.sigma==1), 1);
  % IK-Zufallswerte generieren
  t2 = tic();
  for kkpar = 1:ntest_Par
    TSS = RS.gen_testsettings(true, true);
    delta_qlim = (RS.qlim(:,2)-RS.qlim(:,1));
    for i = 1:ntest_Kon
      q = TSS.Q(i,:)'; 
      T_E = RS.fkineEE(q); % Ziel-Pose (erreichbar, da aus direkter Kin.)
      xE = [T_E(1:3,4); r2eul(T_E(1:3,1:3), RS.phiconv_W_E)];
      for j = 1:ntryIK
        % Grenzen für Anfangs-Stellung für IK festlegen
        q0_min = max(q-usr_range_q0*delta_qlim, RS.qlim(:,1));
        q0_max = min(q+usr_range_q0*delta_qlim, RS.qlim(:,2));
        % Gleichverteilte Zufallszahlen zwischen q0_min und q0_max
        q0 = q0_min + rand(RS.NQJ,1).*(q0_max-q0_min);
        [q_ik,Phi,~,Stats] = RS.invkin2(xE, q0, s_IK);
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
              if ~usr_parcomp % geht nicht in parfor
                error('Angehalten wegen Grenzverletzung. Beginne mit dem Debuggen');
              end
              % beak;
              % RS.fill_fcn_handles(false)
            end
          end
          if j <= usr_repeat_limvioluntil && IKerg_ii(kkpar, i) == 2
            % Führe die IK nochmal aus, damit die Grenzen dann nicht
            % verletzt werden. Mache das nicht bis zum Schluss, damit noch
            % zwischen Grenzverletzung und "geht gar nicht" unterschieden
            % werden kann
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
            warning('IK nicht erfolgreich');
            % Debuggen der inversen Kinematik. Hier Algorithmus verändern.
            % Dafür auf nicht kompilierte Funktion wechseln:
            RS.fill_fcn_handles(false);
            [q_ik,Phi,~,Stats] = RS.invkin2(xE, q0, s_IK);
            disp(Phi');
            disp(cond(RS.jacobig(q0)))
            figure(1000);clf;
            subplot(3,3,1); hold on;
            plot(Stats.Q);
            set(gca, 'ColorOrderIndex', 1);
            plot([1;size(Stats.Q,1)], repmat(q',2,1), '--');
            ylabel('q'); xlabel('iter');grid on;
            subplot(3,3,2);
            plot(diff(Stats.Q));
            ylabel('delta q'); xlabel('iter');grid on;
            subplot(3,3,3);
            plot(Stats.PHI);
            ylabel('Phi'); xlabel('iter');grid on;
            subplot(3,3,4);
            plot(diff(Stats.PHI));
            ylabel('Delta Phi'); xlabel('iter');grid on;
            subplot(3,3,5);
            plot(diff(sum(Stats.PHI.^2,2).^0.5))
            ylabel('Delta norm(Phi)'); xlabel('iter');grid on;
            subplot(3,3,6); hold on;
            cond_groundtruth = cond(q);
            plot(Stats.condJ);
            plot([1;size(Stats.condJ,1)], [1;1]*cond_groundtruth, '--');
            ylabel('cond(J_IK)'); xlabel('iter');grid on;
            subplot(3,3,7); hold on;
            plot(Stats.lambda);
            legend({'lambda', 'lambda_mult'}, 'interpreter', 'none');
            ylabel('lambda'); xlabel('iter');grid on;
            subplot(3,3,8); hold on;
            manip_all = NaN(size(Stats.Q,1),1);
            for iii = 1:size(Stats.Q,1)
              J_iii = RS.jacobig(Stats.Q(iii,:)');
              manip_all(iii) = det(J_iii*J_iii');
            end
            plot(manip_all);
            ylabel('manip'); xlabel('iter');grid on;
            subplot(3,3,9);
            stairs(Stats.rejcount);
            ylabel('rejcount');
            linkxaxes
            error('Halte hier');
            % s_3T3R.K = [ones(3,1)*0.1; ones(3,1)*0.3]
            % return; % geht nicht in parfor
          else
            continue
          end
        end
      end % Über erlaubte Versuche für IK
    end % Über verschiedene Konfigurationen
    t_act = toc(t2);
    T_per_par = t_act/kkpar;
    if kkpar==1, ps_str = 'Parametersatz'; else, ps_str = 'Parametersätze'; end
    fprintf(['[%d/%d] %d %s in %1.1fs gerechnet. Hier %d/%d erfolgreich. ', ...
      'Voraussichtlich noch %1.1fs für die restlichen %d für %s\n'], ...
      ii, nRob, kkpar, ps_str, t_act, sum(IKerg_ii(kkpar,:)==1), ntest_Kon, ...
      T_per_par*(ntest_Par-kkpar), ntest_Par-kkpar, Name);
  end % Über verschiedene Kinematikparameter
  % Zähle die Ergebnisse der inversen Kinematik und bilde Histogrammdaten
  % Erfolg beim 1.-x. Versuch
  IK_hist_Anz_ges_ii = NaN(1, nbars_hist);
  IK_hist_Anz_mG_ges_ii = NaN(1,3);
  for j = 1:nbars_hist-3 % drei Klassen Abzug: "mehr als x", "niO Grenze verletzt" und "niO falsch"
    % Zähle, wie viele IK %d Versuche brauchten bis Erfolg (erg=1)
    IK_hist_Anz_ges_ii(j) = sum(IKerg_ii(:) == 1 & IKtry_ii(:) == j);
  end

  % Erfolg bei mehr als x Versuche bis zur iO-IK notwendig:
  IK_hist_Anz_ges_ii(end-2) = sum(IKerg_ii(:) == 1 & IKtry_ii(:) > (nbars_hist-3));
  % Anzahl Kombinationen mit Grenzverletzung
  IK_hist_Anz_ges_ii(end-1) = sum(IKerg_ii(:) == 2);
  % Anzahl kein Erfolg:
  IK_hist_Anz_ges_ii(end) = sum(IKerg_ii(:) == 0);
  
  % Anzahl Erfolg aufteilen in mit und ohne Grenzen
  IK_hist_Anz_mG_ges_ii(1) = sum(IKerg_ii(:) == 1); % Grenzen richtig, IK richtig
  IK_hist_Anz_mG_ges_ii(2) = sum(IKerg_ii(:) == 2); % Grenzen falsch, IK richtig
  IK_hist_Anz_mG_ges_ii(3) = sum(IKerg_ii(:) == 0); % IK falsch
  
  % Prüfe, ob alle Werte zugeordnet wurden.
  if sum(IK_hist_Anz_ges_ii) ~= sum_try
    error('Summe für Histogramm stimmt nicht');
  end
  if sum(IK_hist_Anz_mG_ges_ii) ~= sum_try
    error('Summe für Histogramm stimmt nicht');
  end
  IK_hist_Anz_mG_ges(ii,:) = IK_hist_Anz_mG_ges_ii(1,1:3);
  IK_hist_Anz_ges(ii,:) = IK_hist_Anz_ges_ii(1,1:nbars_hist); % Zuweisung in dieser Art für parfor
  
  if ~usr_parcomp
    fprintf('%d Roboter bis jetzt gerechnet. Dauer bis jetzt: %1.1fs %s in %1.1fs abgeschlossen. Restdauer ca. %1.1fs für %d Roboter.\n', ...
      ii, toc(t0), Name, toc(t1), toc(t0)/ii*(length(II)-ii), length(II)-ii);
  end
end
% Prozentuale Angabe für die IK-Statistik
IK_hist_Ant_ges = IK_hist_Anz_ges / sum_try;
IK_hist_Ant_mG_ges = IK_hist_Anz_mG_ges / sum_try;
fprintf('Insgesamt %d Roboter in %1.0fs berechnet.\n', nRob, toc(t0));
%% Ergebnisse speichern
if ~usr_load_data
  fprintf('IK-Ergebnisse gespeichert\n');
  save(fullfile(respath, sprintf('serrob_ik_histogramm_%s_startoff%1.0f.mat', usr_DoF, 100*usr_range_q0)));
else
  fprintf('IK-Ergebnisse geladen\n');
  load(fullfile(respath, sprintf('serrob_ik_histogramm_%s_startoff%1.0f.mat', usr_DoF, 100*usr_range_q0)));
end
% Debug: Reduziere Ergebnisse bis zum letzten, der gelaufen ist:
% nRob = find(II == iFK);

% Debug: Nur einen Teil der Ergebnisse nehmen
% IK_hist_Ant_ges = IK_hist_Ant_ges(1:nRob,:);
% IK_hist_Ant_mG_ges = IK_hist_Ant_mG_ges(1:nRob,:);
% IK_hist_Anz_ges = IK_hist_Anz_ges(1:nRob,:);
% IK_hist_Anz_mG_ges = IK_hist_Anz_mG_ges(1:nRob,:);

%% Vor-Auswertung
% Roboter, bei denen die IK gar nicht funktioniert.
I_niO = IK_hist_Ant_mG_ges(:,3) > 0.99;
if any(I_niO)
  disp('Es gibt Roboter, bei denen es gar nicht funktioniert:');
  Names_niO = l.Names_Ndof(II_orig(~I_RobAusw))';
  disp(Names_niO);
end
% Reduziere die Ergebnis-Variablen um die nicht ausgewählten Roboter
IK_hist_Ant_ges = IK_hist_Ant_ges(I_RobAusw,:);
IK_hist_Ant_mG_ges = IK_hist_Ant_mG_ges(I_RobAusw,:);
IK_hist_Anz_ges = IK_hist_Anz_ges(I_RobAusw,:);
IK_hist_Anz_mG_ges = IK_hist_Anz_mG_ges(I_RobAusw,:);
II = II(I_RobAusw);

%% Sortiere die Roboter danach, wie viele Drehgelenke sie haben
if usr_sort_numrotjoints
  NrotJ_all = l.AdditionalInfo(II,5);
  [NrotJ_sort, I_sortrotJ] = sort(NrotJ_all);

  IK_hist_Ant_ges = IK_hist_Ant_ges(I_sortrotJ,:);
  IK_hist_Ant_mG_ges = IK_hist_Ant_mG_ges(I_sortrotJ,:);
  IK_hist_Anz_ges = IK_hist_Anz_ges(I_sortrotJ,:);
  IK_hist_Anz_mG_ges = IK_hist_Anz_mG_ges(I_sortrotJ,:);
  II = II(I_sortrotJ);
end

%% Erzeuge Ergebnis-Tabelle
ResCell = [l.Names_Ndof(II)', cell(length(II),3)];
ResTab = cell2table(ResCell);
ResTab.Properties.VariableNames = {'Name', 'Anteil_Erfolgreich', ...
  'Anteil_Grenzverletzung', 'Anteil_Fehler'};
ResTab.Anteil_Erfolgreich = sum(IK_hist_Ant_ges(:,1:end-2),2);
ResTab.Anteil_Grenzverletzung = IK_hist_Ant_ges(:,end-1);
ResTab.Anteil_Fehler = IK_hist_Ant_ges(:,end);

if ~usr_load_data
  save(fullfile(respath, sprintf('serrob_ik_histogramm_%s_startoff%1.0f_filt.mat', usr_DoF, 100*usr_range_q0)));
end
%% Ergebnisse plotten
figure(1);clf;hold on;
title(sprintf('%s IK without regarding joint limits', usr_DoF));
% IK_Hist_j = zeros(size(IK_hist_Ant_ges,1), 1);
legbarhdl = NaN(size(IK_hist_Ant_ges,2),1);
xticklabels = {};
for i = 1:size(IK_hist_Ant_ges,1)
  xticklabels{i} = l.Names_Ndof{II(i)};
end
legentries = {};
Farben = {};
color_red = [1, 0, 0];
color_green = [0, 1, 0];
color_orange = [1, 0.65, 0];
for i = 1:size(IK_hist_Ant_ges,2)-3
  legentries{i} = sprintf('try %d', i); %#ok<SAGROW>
  ant_gut_schlecht = (i-1) / (size(IK_hist_Ant_ges,2)-2);
  Farben{i} = color_green*(1-ant_gut_schlecht) + color_orange*ant_gut_schlecht; %#ok<SAGROW>
end
legentries{size(IK_hist_Ant_ges,2)-2} = sprintf('try < %d', usr_repeat_limvioluntil);
legentries{size(IK_hist_Ant_ges,2)-1} = sprintf('limits viol.');
legentries{size(IK_hist_Ant_ges,2)} = 'wrong';
Farben{size(IK_hist_Ant_ges,2)-2} = color_orange; % x Versuche ist orange
Farben{size(IK_hist_Ant_ges,2)-1} = 0.9*color_red; % Grenzen verletzt hellrot
Farben{size(IK_hist_Ant_ges,2)} = 0.5*color_red; % Falsch ist dunkelrot
for i = 1:size(IK_hist_Ant_ges,1)
  % Setze Startwert für Balkenhöhe auf Maximal-Zahl. Dadurch wird der
  % zuerst gezeichnete Balken immer voll
  IK_Hist_j = sum(IK_hist_Ant_ges(i,:));
  for j = size(IK_hist_Ant_ges,2):-1:1
    % Stapele alle bisherigen Balken: Ziehe die Höhe des aktuellen Balkens
    % von der laufenden Höhe ab, damit der neue, weiter vorne gezeichnete
    % Balken kleiner ist
    % Plotten
    % fprintf('Rob %d; Balken %d: Höhe %1.1f\n', i, j, IK_Hist_j);
    legbarhdl(j) = bar(i, 100*IK_Hist_j);
    set(legbarhdl(j), 'EdgeColor', 'none', 'FaceColor', Farben{j});
    IK_Hist_j = IK_Hist_j - IK_hist_Ant_ges(i,j);
  end
  set(gca, 'ColorOrderIndex', 1)
end
xlabel('Number of Serial Link Kinematics');
xlim([0.5, length(II)+0.5]); % Die Menge der Roboter kann sich reduzieren. Daher Limits nicht mit `nRob`
ylabel('IK success state in %');
ylim([0, 100]);
set(gca, 'xtick', 1:size(IK_hist_Ant_ges,1));
set(gca, 'xticklabel', xticklabels)
% Legendeneinträge wieder händisch reduzieren
legbarhdl = legbarhdl([1:usr_num_leg_classes, end-2:end]);
legentries = legentries([1:usr_num_leg_classes, end-2:end]);
legentries{usr_num_leg_classes} = '...';
leg1hdl = legend(legbarhdl, legentries, 'location', 'northoutside', 'orientation', 'horizontal');
saveas(1, fullfile(respath, sprintf('serrob_ik_histogramm1_%s_startoff%1.0f.fig', usr_DoF, 100*usr_range_q0)));

figure(2);clf;hold on;
title(sprintf('%s IK regarding joint limits', usr_DoF));
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
    % fprintf('Rob %d; Balken %d: Höhe %1.1f\n', i, j, IK_Hist_j);
    legbarhdl(j) = bar(i, 100*IK_Hist_j, 'DisplayName', sprintf('Rob%dCl%d', i, j));
    set(legbarhdl(j), 'EdgeColor', 'none');
    IK_Hist_j = IK_Hist_j - IK_hist_Ant_mG_ges(i,j);
  end
  set(gca, 'ColorOrderIndex', 1)
end
ylabel('occurence in percent');
ylim([0, 100]);
legend(legbarhdl, {'correct', 'limits violated', 'wrong'}, 'location', 'northoutside', 'orientation', 'horizontal');
set(gca, 'xtick', 1:size(IK_hist_Ant_ges,1));
set(gca, 'xticklabel', xticklabels)
saveas(2, fullfile(respath, sprintf('serrob_ik_histogramm2_%s_startoff%1.0f.fig', usr_DoF, 100*usr_range_q0)));

% Histogramm über Gesamtverteilung der Erfolgsrate über alle Roboter
IK_Hist_success = sum(IK_hist_Ant_ges(:,1:end-2),2); % Lasse die Klassen "wrong" und "lim. viol." weg
figure(3);clf;
histogram(IK_Hist_success);
saveas(3, fullfile(respath, sprintf('serrob_ik_histogramm3_robots_%s_startoff%1.0f.fig', usr_DoF, 100*usr_range_q0)));

%% Histogramme exportieren

figure(1);
title('');

set_size_plot_subplot(1,...
  15.5,4.5,gca,...
  0.07,0.01,0.13,0.18,... % bl,br,hu,hd,
  0,0) % bdx,bdy)
set(gca, 'XTickMode', 'auto', 'XTickLabelMode', 'auto')
% set(gca, 'xtick', 1:size(IK_hist_Ant_ges,1));
% set(gca, 'xticklabel', 1:size(IK_hist_Ant_ges,1));
set(leg1hdl, 'position', [0.15    0.92    0.70    0.05], ...
  'orientation', 'horizontal');
figure_format_publication()
saveas(1, fullfile(respath, sprintf('serrob_ik_hist_%s_startoff%1.0f.fig', usr_DoF, 100*usr_range_q0)));
export_fig(1, fullfile(respath, sprintf('serrob_ik_hist_%s_startoff%1.0f.pdf', usr_DoF, 100*usr_range_q0)));