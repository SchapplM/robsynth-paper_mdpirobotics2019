% Bilder mit 3RRR-PKM generieren
% Vier Bilder, jew. verschiedene kin. ZB
% 
% Skalierung der Bilder:
% Breite: 155mm insgesamt; für vier Bilder 35mm für ein Bild; für drei
%         Bilder mehr
% Höhe: 30mm (oberer Teil wird in Inkscape sowieso abgeschnitten
% Größenangaben im Matlab-Plot auch in mm. Dann übertragen der Daten in
% Inkscape. Nutzen der Matlab-Bilder in Inkscape als Vorlage zum drüberzeichnen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc
respath = fullfile(fileparts(which('pkm_3RRR_figure_gen.m')));
if isempty(respath)
  error('Ordner des Skripts muss im Pfad sein');
end
%% Benutzereingaben
l1 = 15;
l2 = 15;
dB = 25;
dP = 8;

%% Init
if isempty(which('serroblib_path_init.m'))
  warning('Repo mit seriellen Robotermodellen ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end

% Typ des seriellen Roboters auswählen (Drei Drehgelenke)
SName='S3RRR1';

% Instanz der Roboterklasse erstellen
RS = serroblib_create_robot_class(SName);
RS.fill_fcn_handles(false);
% RS.mex_dep(true)

% Parameter setzen
[beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh] = ...
  S3RRR1_pkin2mdhparam(zeros(length(RS.pkin),1));
d_mdh(:) = 0;
a_mdh(2) = l1;
a_mdh(3) = l2;
pkin = S3RRR1_mdhparam2pkin(beta_mdh, b_mdh, alpha_mdh, a_mdh, theta_mdh, d_mdh, qoffset_mdh);
RS.update_mdh(pkin);
RS.I_EE = logical([1 1 0 0 0 1]); % Für IK der Beinketten mit invkin_ser

%% Klasse für PKM erstellen
RP = ParRob('P3RRR1');
RP = RP.create_symmetric_robot(3, RS, dB, dP);
RP = RP.initialize();
RP.update_EE_FG(logical([1 1 0 0 0 1])); % Für IK der PKM
RP.update_base([0;0;0], [0;0;-30]*pi/180);
%% Startpose bestimmen
% Mittelstellung im Arbeitsraum
X0L = [ [7;-2;0]; [0;0;15]*pi/180 ];

% Inverse Kinematik berechnen und testen
[q0, Phi] = RP.invkin1(X0L, rand(RP.NJ,1));
if any(abs(Phi) > 1e-8) || any(isnan(Phi))
  warning('Inverse Kinematik (für Gesamt-PKM) konnte in Startpose nicht berechnet werden');
end

%% Roboter in Startpose plotten (zum Testen)
figure(1);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [], 'straight', 0);
RP.plot( q0, X0L, s_plot );
view(0,90)
xlim([-28,28])
ylim([-25,28])

%% Roboter mit verschiedenen Ist-Posen aus der ersten Gelenkkette erstellen
% Für Kinematik-Schema der Zwangsbedingungen für Führungskette
for i = 1:4
  if i == 1
    % Plattform von 1. Beinkette aus verdreht und an falscher Position
    Xi = X0L + [[-15; 5; 0]; [0;0;35]*pi/180];
  elseif i == 2
    % Plattform von 1. Beinkette aus verdreht und an richtiger Position
    Xi = X0L + [[0; 0; 0]; [0;0;45]*pi/180];
  elseif i == 3
    % Plattform von 1. Beinkette aus richtig orientiert aber an falscher Position
    Xi = X0L + [[-10; 10; 0]; [0;0;0]*pi/180];
  elseif i == 4
    % Plattform von 1. Beinkette aus richtig orientiert und positioniert
    Xi = X0L;
  end
  [qi, Phi] = RP.invkin1(Xi, q0);
  if any(abs(Phi) > 1e-8) || any(isnan(Phi))
    warning('Inverse Kinematik (für Gesamt-PKM) konnte nicht berechnet werden');
  end
  
  figure(10+i);clf;
  hold on;grid on;
  xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
  view(3);
  s_plot = struct( 'ks_legs', [], 'straight', 0);
  RP.plot( qi, X0L, s_plot );
  view(0,90)
  xlim([-28,28])
  ylim([-25,28])

  set_size_plot_subplot(10+i,...
    4,4,gca,...
    0,0,0,0,0,0)
  export_fig(10+i, fullfile(respath, sprintf('pkm_3RRR_figure_gen_Fig3_%d.pdf',i)));
end

%% Roboter mit verschiedenen Ist-Posen aus der zweiten Gelenkkette erstellen
X0F = [ [-7;-5;0]; [0;0;25]*pi/180 ];
[q0F, Phi] = RP.invkin1(X0F, rand(RP.NJ,1));
if any(abs(Phi) > 1e-8) || any(isnan(Phi))
  warning('Inverse Kinematik (für Gesamt-PKM) konnte in Startpose nicht berechnet werden');
end
figure(20);clf;
hold on;grid on;
xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
view(3);
s_plot = struct( 'ks_legs', [], 'straight', 0);
RP.plot( q0F, X0F, s_plot );
view(0,90)
xlim([-28,28])
ylim([-25,28])


% Für Kinematik-Schema der Zwangsbedingungen für Folgekette
for i = 3%1:3
  if i == 1
    % Plattform von 2. Beinkette aus verdreht und an falscher Position
    Xi = X0F + [[9; 10; 0]; [0;0;-100]*pi/180];
  elseif i == 2
    % Plattform von 2. Beinkette aus verdreht und an richtiger Position
    Xi = X0F + [[0; 0; 0]; [0;0;45]*pi/180];
  elseif i == 3
    % Plattform von 1. Beinkette aus richtig orientiert und positioniert
    Xi = X0F;
  end
  [qi, Phi] = RP.invkin1(Xi, q0);
  if any(abs(Phi) > 1e-8) || any(isnan(Phi))
    warning('Inverse Kinematik (für Gesamt-PKM) konnte nicht berechnet werden');
  end
  
  figure(20+i);clf;
  hold on;grid on;
  xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');
  view(3);
  s_plot = struct( 'ks_legs', [], 'straight', 0);
  RP.plot( qi, X0F, s_plot );
  view(0,90)
  xlim([-28,28])
  ylim([-25,28])

  set_size_plot_subplot(20+i,...
    4.8,6,gca,...
    0,0,0,0,0,0)
  export_fig(20+i, fullfile(respath, sprintf('pkm_3RRR_figure_gen_Fig4_%d.pdf',i)));
end