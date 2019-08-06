% Industrieroboter mit Trajektorie und unterschiedlichen IK-Einstellungen
% Zeichne Bild für PKM-3T2R-Paper
% 
% Beispiel-Daten aus 
% [HuoBar2008] Huo, Baron: The joint-limits and singularity avoidance in
% robotic welding (2008)
% Trajektorie wird aber nicht von dort übernommen; lässt sich nur mit sehr
% viel Aufwand nachmachen.
% 
% Siehe auch: SerRob_class_example_Industrieroboter.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

% Typ des seriellen Roboters auswählen (Industrieroboter Fanuc M-710 iC/50)
SName='S6RRRRRR10V2';
% Modellparameter auswählen (hinterlegt aus Datenblatt)
RName='S6RRRRRR10V2_FANUC6';

RS = serroblib_create_robot_class(SName, RName);
RS.fill_fcn_handles(false);
RS.fill_fcn_handles(true, false);
% RS.mex_dep(true);
% matlabfcn2mex({'S6RRRRRR10V2_invkin_traj', 'S6RRRRRR10V2_invkin_eulangresidual'});

% Manuelle Einstellung der Startpose, da Gelenkoffset gegenüber [HuoBar2008]
q0 = zeros(6,1);
q0(2) = pi/2;

% Transformationsmatrix zum Werkstück ähnlich wie bei [HuoBar2008]
% [HuoBar2008] equ. 33
T_part = transl([1.15; 0.2; -0.2;]);

% EE geschätzt aus [HuoBar2008]
% RS.update_EE([0.200; 0;-0.100], [pi;0;0]);
% Nehme eigenen EE, weil parallele Werkzeugachse zu letzter Robo-Achse bei
% der Optimierung blöd ist (betrifft dann nur letzte Achse)
RS.update_EE([-0.200; 0;-0.100], [pi/2;-pi/2;0]);
%% Kartesische Trajektorie
% Würfel-Trajektorie erstellen
% Startpunkt selbst festlegen (so dass es gut passt)
T_E = RS.fkineEE(q0);
x0Ref = NaN(6,1);
x0Ref(1:3) = T_part(1:3,4) + [-0.200;0;0.400];
x0Ref(4:6) = [pi;0;0];
x0 = x0Ref;
% Start in Grundstellung
k=1; XE = x0';
% Beginn Rechteck (direkt in definiertem Start)
for trajwdh = 1:1
  d1=0.5;
  d2 = 0.8;
  k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,-d2,0  0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,d2,0  0,0,0];
end
[X,XD,XDD,T] = traj_trapez2_multipoint(XE, 1, 1e-1, 1e-2, 1e-3, 0.25);

%% Roboter in Nullstellung plotten (mit im Gelenkraum entworfener Trajektorie)
% Feste Startstellung für IK definieren (so dass Jacobi lösbar ist)
q0_ik_fix = q0 + [0;25;-35;0;15;0]*pi/180;
s_plot = struct( 'ks', [1:RS.NJ, RS.NJ+2], 'straight', 0);
figure(1);clf;
hold on;
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
view(3);
RS.plot( q0_ik_fix, s_plot );
title(sprintf('Nullstellung: %s', RS.descr));
plot3(X(:,1), X(:,2), X(:,3));
cond(RS.jacobig(q0_ik_fix));
%% Zielfunktion testen
figure(100);clf;
for i = 1:RS.NQJ
  q_range = ((2*RS.qlim(i,1)):1e-1*pi/180:(2*RS.qlim(i,2)))';
  v_range = NaN(length(q_range),2);
  g_range = NaN(length(q_range),2);
  for jj = 1:length(q_range)
    [v_range(jj,1),g_range(jj,1)] = invkin_optimcrit_limits1(q_range(jj), RS.qlim(i,:));
    [v_range(jj,2),g_range(jj,2)] = invkin_optimcrit_limits2(q_range(jj), RS.qlim(i,:));
  end
  subplot(2,6,sprc2no(2,6,1,i)); hold on;
  plot(q_range, v_range);
  v_range_int = repmat(v_range(1,:), length(q_range), 1);
  for kk = 1:2
    v_range_int(:,kk) = v_range_int(:,kk) + cumtrapz(q_range, g_range(:,kk));
  end
  plot(q_range, v_range_int, '--');
  legend({'crit1', 'crit2', 'crit1 int', 'crit2 int'});
  xlabel(sprintf('q %d', i));
  ylim([0,5]);
  grid on;
  subplot(2,6,sprc2no(2,6,2,i)); hold on;
  plot(q_range, g_range);
  ylim([-5,5]);
  grid on;
end

%% Inverse Kinematik mit verschiedenen Einstellungen
value_axori = (-180:15:180)*pi/180;
if any(abs(value_axori) > 2*pi)
  error('Eingabe muss in rad sein');
end
N_axori = length(value_axori);
Q_ges = NaN(length(T), RS.NQJ, N_axori);
Q_opt_ges = NaN(length(T), RS.NQJ, N_axori);
Q_nopt_ges = NaN(length(T), RS.NQJ, N_axori);
traj_iO = false(N_axori,3);
I_EE_3T2R = logical([1 1 1 1 1 0]);
I_EE_3T3R = logical([1 1 1 1 1 1]);
h_ges = NaN(length(T), N_axori);
h_opt_ges = NaN(length(T), N_axori);
h_nopt_ges = NaN(length(T), N_axori);
phiE_opt_ges = NaN(length(T), N_axori);
phiE_nopt_ges = NaN(length(T), N_axori);
x_ges = NaN(length(T), 6, N_axori);
x_opt_ges = NaN(length(T), 6, N_axori);
x_nopt_ges = NaN(length(T), 6, N_axori);
fprintf('Starte Berechnung der IK\n');
for ii = 1:N_axori
  q0_ik = q0_ik_fix;
  % Startwerte für IK einstellen
  % q0_ik = 0.5*(RS.qlim(:,1) + RS.qlim(:,2));
  % Nehme den Startwert einer vorherigen funktionierenden Trajektorie
  % Das funktioniert nicht so gut: Man kommt von schlechten Anfangswerten
  % nicht mehr weg ... Das wird daher nur für 3T2R ohne Optimierung gezeigt
%   if ii > 1
%     for jj = ii-1:-1:1
%       if traj_iO(jj,1)
%         q0_ik = Q_ges(1,:,jj)';
%         break;
%       end
%     end
%   end
  
  X_ii = X;
  X_ii(:,6) = value_axori(ii);
  x_ges(:,:,ii) = X_ii;
  fprintf('Starte IK-Berechnung %d/%d mit phiz=%1.1f deg\n', ii, N_axori, 180/pi*value_axori(ii));
  %% 3T3R IK
  tic();
  [Q_ii, QD_ii, QDD_ii, Phi_ii] = RS.invkin2_traj(X_ii,XD,XDD,T,q0_ik_fix,...
    struct('n_max', 1000, 'Phit_tol', 1e-7, 'Phir_tol', 1e-7, ...
    'I_EE', I_EE_3T3R, 'reci', true, 'K', 0.7*ones(RS.NQJ,1), ...
    'scale_lim', 0));
  
  if max(abs(Phi_ii(:))) > 1e-3
    warning('Trajektorie %d konnte nicht für 3T3R berechnet werden. Max Error %1.1f', ii, max(abs(Phi_ii(:))));
  else
    traj_iO(ii,1) = true;
  end
  fprintf('%d: IK für 3T3R berechnet. Dauer: %1.1fs\n', ii, toc());
  Q_ges(:,:,ii) = Q_ii;
  %% 3T2R IK mit Optimierung
  if ii > 1
    % Die Durchläufe mit Redundanz sind immer gleich, daher nur ein
    % Durchlauf notwendig
    % Ergebnisse aus vorheriger Iteration speichern
    Q_opt_ges(:,:,ii) = Q_opt_ii;
  else
    
    tic();
    s_start = struct('n_min', 1000, 'n_max', 2500, 'Phit_tol', 1e-7, 'Phir_tol', 1e-7, ...
      'I_EE', I_EE_3T2R, 'reci', true, 'wn', [0;1], 'K', 0.7*ones(RS.NQJ,1), ...
      'Kn', 0.7*ones(RS.NQJ,1));
    q0_ik_fix2 = RS.invkin2(X_ii(1,:)',q0_ik_fix,s_start);
    invkin_optimcrit_limits2(q0_ik_fix2, RS.qlim);
    fprintf('%d: IK für 3T2R-Startpose (mit Opt.) berechnet. Dauer: %1.1fs\n', ii, toc());
    [Q_opt_ii, QD_opt_ii, QDD_opt_ii, Phi_opt_ii] = RS.invkin2_traj(X_ii,XD,XDD,T,q0_ik_fix2,...
      struct('n_min', 50, 'n_max', 1500, 'Phit_tol', 1e-7, 'Phir_tol', 1e-7, ...
      'I_EE', I_EE_3T2R, 'reci', true, 'wn', [0;1], 'K', 0.7*ones(RS.NQJ,1), ...
      'Kn', 0.7*ones(RS.NQJ,1)));
    if max(abs(Phi_opt_ii(:))) > 1e-3
      warning('Trajektorie %d konnte nicht für 3T2R berechnet werden. Max Error %1.1f', ii, max(abs(Phi_opt_ii(:))));
    else
      traj_iO(ii,2) = true;
    end
    fprintf('%d: IK für 3T2R (mit Opt.) berechnet. Dauer: %1.1fs\n', ii, toc());
    Q_opt_ges(:,:,ii) = Q_opt_ii;
  end
  
  %% 3T2R IK ohne Optimierung
  tic();
  [Q_nopt_ii, QD_nopt_ii, QDD_nopt_ii, Phi_nopt_ii] = RS.invkin2_traj(X_ii,XD,XDD,T,Q_ii(1,:)',...
    struct('n_min', 20, 'n_max', 100, 'Phit_tol', 1e-7, 'Phir_tol', 1e-7, ...
    'I_EE', I_EE_3T2R, 'reci', true, 'wn', zeros(2,1), 'K', 0.7*ones(RS.NQJ,1), ...
    'Kn', zeros(RS.NQJ,1), 'scale_lim', 0));
  if max(abs(Phi_nopt_ii(:))) > 1e-3
    warning('Trajektorie %d konnte nicht für 3T2R (ohne Optimierung) berechnet werden. Max Error %1.1f', ii, max(abs(Phi_nopt_ii(:))));
  else
    traj_iO(ii,3) = true;
  end
  fprintf('%d: IK für 3T2R (ohne Opt.) berechnet. Dauer: %1.1fs\n', ii, toc());
  Q_nopt_ges(:,:,ii) = Q_nopt_ii;
  
  %% Weitere Berechnungen
  for kk = 1:length(T)
    % Berechne Zielfunktion
    h_ges(kk,ii) = invkin_optimcrit_limits2(Q_ii(kk,:)', RS.qlim);
    h_opt_ges(kk,ii) = invkin_optimcrit_limits2(Q_opt_ii(kk,:)', RS.qlim);
    h_nopt_ges(kk,ii) = invkin_optimcrit_limits2(Q_nopt_ii(kk,:)', RS.qlim);

    % Direkte Kinematik berechnen für EE-Winkel
    x_E_opt_kk = RS.t2x(RS.fkineEE(Q_opt_ii(kk,:)'));
    x_opt_ges(kk,:,ii) = x_E_opt_kk;
    phiE_opt_ges(kk,ii) = x_E_opt_kk(6);
    x_E_nopt_kk = RS.t2x(RS.fkineEE(Q_nopt_ii(kk,:)'));
    x_nopt_ges(kk,:,ii) = x_E_nopt_kk;
    phiE_nopt_ges(kk,ii) = x_E_nopt_kk(6);
    % Testen
    % x_E_kk = RS.t2x(RS.fkineEE(Q_ii(kk,:)'));
  end
  if any(h_opt_ges(:,ii)) > any(h_ges(:,ii)) || any(h_opt_ges(:,ii)) > any(h_nopt_ges(:,ii))
    error('Zielfunktion ohne Optimierung ist besser');
  end
  continue
  % Debug
  RS.fill_fcn_handles(true, true);
  RS.fill_fcn_handles(false);
end
%% Suche beste und schlechteste Zielfunktion
hminmax = NaN(length(T),2);
phi_worst = NaN(length(T),1);
phi_best = NaN(length(T),1);
for ii = 1:length(T)
  hminmax(ii,:) = minmax2([h_ges(ii,:), h_opt_ges(ii,:), h_nopt_ges(ii,:)]);
  % Finde den besten und schlechtesten EE-Winkel heraus
  [~,I_worst] = max(h_ges(ii,:));
  phi_worst(ii) = value_axori(I_worst);
  
  [h_opt_best,I_best_opt] = min(h_opt_ges(ii,:));
  phi_best(ii) = phiE_opt_ges(ii, I_best_opt);
  [h_nopt_best,I_best_nopt] = min(h_nopt_ges(ii,:));
  if h_nopt_best < h_opt_best
    phi_best(ii) = phiE_nopt_ges(ii,I_best_nopt);
  end
end

%% Suche Anzahl der Überschreitungen
n_viol_opt = NaN(N_axori,RS.NQJ);
n_viol_nopt = NaN(N_axori,RS.NQJ);
n_viol = NaN(N_axori,RS.NQJ);
for ii = 1:N_axori
  for jj = 1:RS.NQJ
    n_viol_opt(ii,jj) = sum((Q_opt_ges(:,jj,ii) > RS.qlim(jj,2)) | ...
                        (Q_opt_ges(:,jj,ii) < RS.qlim(jj,1)));
    n_viol_nopt(ii,jj) = sum((Q_nopt_ges(:,jj,ii) > RS.qlim(jj,2)) | ...
                        (Q_nopt_ges(:,jj,ii) < RS.qlim(jj,1)));
    n_viol(ii,jj) = sum((Q_ges(:,jj,ii) > RS.qlim(jj,2)) | ...
                        (Q_ges(:,jj,ii) < RS.qlim(jj,1)));
  end
end
sum(n_viol_opt(:))
sum(n_viol_nopt(:))
sum(n_viol(:))
%% Auswertung zum Testen
% Plotte diverse Daten und suche für das Bild im Paper die "schönsten" aus
% value_axori*180/pi
values_Ausw = [-150; -120; -90; 45; 0; 15; 60]*pi/180;
ii_Ausw = NaN(length(values_Ausw),1);
legnames = {};
for i = 1:length(values_Ausw)
  I = find(value_axori==values_Ausw(i));
  if isempty(I)
    warning('Auswahl %1.1f nicht gefunden', values_Ausw(i));
    continue
  end
  ii_Ausw(i) = I;
  legnames{i} = sprintf('%d: phi0=%1.0f deg', I, values_Ausw(i)*180/pi); %#ok<SAGROW>
end

figure(7);clf;
for k = 1:RS.NQJ
  subplot(2,4,k);hold on;
  for ii = ii_Ausw
    plot(T, squeeze(Q_ges(:,k,ii))/RS.qunitmult_eng_sci(k));
  end
  set(gca, 'ColorOrderIndex', 1);
  for ii = ii_Ausw
    plot(T, squeeze(Q_opt_ges(:,k,ii))/RS.qunitmult_eng_sci(k), '--');
  end
  set(gca, 'ColorOrderIndex', 1);
  for ii = ii_Ausw
    plot(T, squeeze(Q_nopt_ges(:,k,ii))/RS.qunitmult_eng_sci(k), ':');
  end
  set(gca, 'ColorOrderIndex', 1);
  plot([0;T(end)], RS.qlim(k,1)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  plot([0;T(end)], RS.qlim(k,2)*[1;1]/RS.qunitmult_eng_sci(k), 'r--');
  xlabel('t [s]');
  ylabel(sprintf('q_%d / %s', k, RS.qunit_eng{k}));
  grid on;
  title(sprintf('Zeitverlauf Gelenkgrößen Achse %d',k));
  grid on;
end
legend(legnames);
subplot(2,4,7);hold on;
for ii = ii_Ausw
  plot(T, h_ges(:,ii));
end
set(gca, 'ColorOrderIndex', 1);
for ii = ii_Ausw
  plot(T, h_opt_ges(:,ii), '--');
end
set(gca, 'ColorOrderIndex', 1);
for ii = ii_Ausw
  plot(T, h_nopt_ges(:,ii), ':');
end
plot(T, hminmax, 'r--');
ylabel('Zielfunktion');
ylim([0,100])

subplot(2,4,8);hold on;
plot(T, 180/pi*phiE_opt_ges);
plot(T, 180/pi*phi_best, 'g--');
plot(T, 180/pi*phi_worst, 'r--');
ylabel('phi_z in deg');
linkxaxes

figure(8);clf;hold on;
stairs(sum(n_viol,2));
stairs(sum(n_viol_opt,2));
stairs(sum(n_viol_nopt,2));
legend({'3T3R', 'opt', 'no opt'});

figure(9);clf;
for i = 1:6
  subplot(2,3,i); hold on;
  ylabel(sprintf('x %d', i));
  for ii = 1:N_axori
    plot(T, x_opt_ges(:,i, ii));
    plot(T, x_nopt_ges(:,i, ii), ':');
  end  
end

figure(10);clf;
for ii = 1:N_axori
  subplot(5,5,ii); hold on;
  title(sprintf('Nr. %d, phi3=%1.0f deg', ii, 180/pi*value_axori(ii)));
  plot(T, 180/pi*x_opt_ges(:,6, ii));
  plot(T, 180/pi*x_nopt_ges(:,6, ii), ':');
  ylabel('phiz_E in deg');
end  


% % Animation
% Q = Q_opt_ges(:,:,4);
% s_anim = struct( 'gif_name', '');
% figure(9);clf;
% hold on;
% plot3(X(:,1), X(:,2), X(:,3));
% grid on;
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% view(3);
% title('Animation der kartesischen Trajektorie');
% RS.anim( Q(1:100:end,:), s_anim, s_plot);

%% Ergebnisse speichern
respath = fileparts(which('serrob_traj.m'));
save(fullfile(respath, 'serrob_traj_data_all.mat'));
save(fullfile(respath, 'serrob_traj_data.mat'), 'T', 'Q_ges', 'Q_opt_ges', 'Q_nopt_ges', ...
  'h_ges', 'h_opt_ges', 'h_nopt_ges', 'hminmax');

%% Bilder für Paper erzeugen
% Bilder: 
% * Gelenkwinkel Achse 1,5 als einzelne Bilder; Linien für
% konstante EE-Winkel, mit/ohne Optimierung; Linien für Zielfunktion
% * Zielfunktion bester, schlechtester, zu den einzelnen Linien aus den
% Achs-Bildern (mit gleicher Formatierung)
% Zusätzlich: Roboter-Bild mit Trajektorie (auch wenn es nicht schön ist;
% und die MDH-Tabelle)
figure(20);clf;
axhdl = NaN(1,4);
RobAx = [1;5];
ii_Ausw = [3, 12,16];
value_axori(ii_Ausw)*180/pi;
format = {'k', 'd', '-', 2; ...
          'b', 's', '-', 4; ...
          'm', 'x', '-', 3; ...
          'k', '',  ':', 1; ...
          'b', '',  ':', 1; ...
          'm', '',  ':', 1; ...
          'g', 'v', '-', 6};
%         ...
%           'r', '',  '--', 0; ...
%           'r', '',  '--', 0};
for i = 1:2
  axhdl(1,i) = subplot(1,4,i);hold on;
  k = RobAx(i);
  linhdl = NaN(7,1);
  axscale = (RS.qlim(k,2) - RS.qlim(k,1))/2;
  axoff = (RS.qlim(k,2) + RS.qlim(k,1))/2;
  for ii = 1:length(ii_Ausw)
    jj = ii_Ausw(ii);
    linhdl(ii) = plot(T, (-axoff+squeeze(Q_ges(:,k,jj)))/axscale);
  end
  for ii = 1:length(ii_Ausw)
    jj = ii_Ausw(ii);
    linhdl(3+ii) = plot(T, (-axoff+squeeze(Q_nopt_ges(:,k,jj)))/axscale, ':');
  end
%   for ii = ii_Ausw
    linhdl(7) = plot(T, (-axoff+squeeze(Q_opt_ges(:,k,1)))/axscale, '--');
%   end
%   linhdl(10) = plot([0;T(end)], (-axoff+RS.qlim(k,1)*[1;1])/axscale, 'r--');
%   linhdl(11) = plot([0;T(end)], (-axoff+RS.qlim(k,2)*[1;1])/axscale, 'r--');
  leglinhdl = line_format_publication(linhdl, format);
  grid on;
  title(sprintf('$q_%d\\ (\\mathrm{scaled})$', k), 'interpreter', 'latex');
  xlabel('Time in s');
  % xticklabels([]);
end
leghdl = legend(leglinhdl([1:3,4,7]), ...
  {sprintf('$\\beta_3=%1.0f^\\circ$', 180/pi*value_axori(ii_Ausw(1))), ...
   sprintf('$\\beta_3=%1.0f^\\circ$', 180/pi*value_axori(ii_Ausw(2))), ...
   sprintf('$\\beta_3=%1.0f^\\circ$', 180/pi*value_axori(ii_Ausw(3))), ...
  '$\mathrm{3T2R, no\\ opt}$', '$\mathrm{3T2R, opt}$'}, 'interpreter', 'latex');
% Verlauf der EE-Winkel
axhdl(1,3) = subplot(1,4,3);hold on;
linhdl = NaN(7,1);
for ii = 1:length(ii_Ausw)
  jj = ii_Ausw(ii);
  linhdl(ii) = plot(T, 180/pi*x_ges(:,6,jj));
end
for ii = 1:length(ii_Ausw)
  jj = ii_Ausw(ii);
  linhdl(3+ii) = plot(T, 180/pi*x_nopt_ges(:,6,jj));
end
linhdl(7) = plot(T, 180/pi*x_opt_ges(:,6,1));
line_format_publication(linhdl, format);
grid on
title('$\beta_3$ in deg', 'interpreter', 'latex');
xlabel('Time in s');
% xticklabels([]);

axhdl(1,4) = subplot(1,4,4);hold on;
linhdl = NaN(7,1);
for ii = 1:length(ii_Ausw)
  jj = ii_Ausw(ii);
  linhdl(ii) = plot(T, h_ges(:,jj));
end
for ii = 1:length(ii_Ausw)
  jj = ii_Ausw(ii);
  linhdl(3+ii) = plot(T, h_nopt_ges(:,jj));
end
% for ii = ii_Ausw
  linhdl(7) = plot(T, h_opt_ges(:,1));
% end
% linhdl(10) = plot(T, hminmax(:,1), 'r--');
% linhdl(11) = plot(T, hminmax(:,2), 'r--');
ylim([0.5,7])
grid on;
line_format_publication(linhdl, format);
title('$\mathrm{optimization\ criterion}$', 'interpreter', 'latex');
xlabel('Time in s');
% xticklabels([]);

figure_format_publication(axhdl)

set_size_plot_subplot(20,...
  15.5,6.5,axhdl,...
  0.05,0.01,0.16,0.13,... % bl,br,hu,hd,
  0.05,0) % bdx,bdy)
set(leghdl, 'position', [0.1    0.93    0.8    0.05], 'orientation', 'horizontal', 'interpreter', 'latex');

export_fig(20, fullfile(respath, 'serrob_traj_nullspace_optim.pdf'));



%% 3D-Bild des Roboters
figure(21);clf;
s_plot = struct( 'ks', [1, RS.NJ+2], 'straight', 0);
hold on;
grid on;
% xlabel('$x$ in m', 'interpreter', 'latex');
% ylabel('$y$ in m', 'interpreter', 'latex');
% zlabel('$z$ in m', 'interpreter', 'latex');
set(gca, 'XTICKLABEL', {});
set(gca, 'YTICKLABEL', {});
set(gca, 'ZTICKLABEL', {});
view(3);
RS.plot( q0_ik_fix, s_plot );
plot3(X(:,1), X(:,2), X(:,3), 'k-', 'LineWidth', 2);
figure_format_publication()
set(gca, 'Box', 'off');
ax = get(gca, 'XAXIS'); set(ax, 'Visible', 'off');
ax = get(gca, 'YAXIS'); set(ax, 'Visible', 'off');
ax = get(gca, 'ZAXIS'); set(ax, 'Visible', 'off');

% set(gca, 'XColor', [1 1 1]);
% set(gca, 'YColor', [1 1 1]);
% set(gca, 'ZColor', [1 1 1]);
set_size_plot_subplot(21,...
  8,8,gca,...
  0.01,0.01,0.0,0.01,... % bl,br,hu,hd,
  0,0) % bdx,bdy)
% export_fig(21, fullfile(respath, 'serrob_traj_zero_pose.pdf'));
export_fig serrob_traj_zero_pose.png -r864

fprintf('Mittelpunkt der Trajektorie: %1.0f/%1.0f/%1.0f\n', ...
  1e3*mean(minmax2(X(:,1)')), 1e3*mean(minmax2(X(:,2)')), 1e3*mean(minmax2(X(:,3)')));
%% Verlauf des EE-Winkels für Präsentation
% X_opt_ges = NaN(length(T), 6);
% Q_opt_ii = Q_opt_ges(:,:,1); % Gleiche Auswahl wie für Bild im Paper
% for kk = 1:length(T)
%   % Direkte Kinematik berechnen für EE-Winkel
%   x_E_opt_kk = RS.t2x(RS.fkineEE(Q_opt_ii(kk,:)'));
% %   phiE_opt_ges(kk,ii) = x_E_opt_kk(6);
% %   x_E_nopt_kk = RS.t2x(RS.fkineEE(Q_nopt_ii(kk,:)'));
% %   phiE_nopt_ges(kk,ii) = x_E_nopt_kk(6);
%   
%   X_opt_ges(kk,:) = x_E_opt_kk;
% end
% 

figure(30);clf;
plot(T, 180/pi*x_opt_ges(:,6, 1));

%% Bild der Achsverläufe für Powerpoint neu formatieren
axhdl2 = reshape(axhdl, 2, 2);
% for i = 1:length(axhdl2(:))
%   axes(axhdl2(i)); %#ok<LAXES>
%   thdl = get(axhdl2(i), 'title');
%   ylabel(get(thdl, 'String'), 'interpreter', 'latex');
% end
for i = 1:length(axhdl2(:))
  axes(axhdl2(i)); %#ok<LAXES>
  ylabel('');
end
set_size_plot_subplot(20,...
  13,13,axhdl2,...
  0.05,0.01,0.12,0.05,... % bl,br,hu,hd,
  0.07,0.09) % bdx,bdy)
for i = 1:length(axhdl2(:))
  axes(axhdl2(i)); %#ok<LAXES>
  % Ticklabels formatieren
  set(gca, 'FontSize', 8);
  thdl = get(axhdl2(i), 'title');
  set(thdl, 'FontSize', 14);
  % Achsenbeschriftung formatieren
  for label = {'XLABEL', 'YLABEL', 'ZLABEL'}
    h = get(gca, cell2str(label));
    set(h, 'FontSize', 14);
  end
end
set(leghdl, 'FontSize', 8);
set(leghdl, 'position', [0.1    0.94    0.8    0.05]);
export_fig(20, fullfile(respath, 'serrob_traj_nullspace_optim_powerpoint.pdf'));
export_fig serrob_traj_nullspace_optim_powerpoint.png -r864
%% Animation für Präsentation
% Benutze AVI-Video, da bei GIF das gesamte Video vorab gespeichert werden
% muss
Q_anim = Q_opt_ges(:,:,1);
figure(200);clf;
s_plot = struct( 'ks', RS.NJ+2, 'straight', 0);
% s_anim = struct( 'gif_name', fullfile(respath, 'serrob_traj_anim_opt3T2R.gif'), ...
%                  'avi_name', fullfile(respath, 'serrob_traj_anim_opt3T2R.avi'));
s_anim = struct( 'avi_name', fullfile(respath, 'serrob_traj_anim_opt3T2R.avi'));
hold on;
grid on;
xlabel('$x$ in m', 'interpreter', 'latex');
ylabel('$y$ in m', 'interpreter', 'latex');
zlabel('$z$ in m', 'interpreter', 'latex');
view(3);
plot3(X(:,1), X(:,2), X(:,3), 'k-', 'LineWidth', 2);
figure_format_publication()
set(200, 'units','normalized','outerposition',[0 0 1 1])
RS.anim( Q_anim(1:5:end,:), s_anim, s_plot);