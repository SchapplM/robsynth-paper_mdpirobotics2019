% Verbinde die verschiedenen Histogramm-Bilder zu einem Bild

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2018-10
% (C) Institut für Mechatronische Systeme, Universität Hannover


close all

% Liste mit Bilddateien laden
respath = fileparts(which('serrob_ik_histogram.m'));

CellFigfiles = cell(4,1);
CellFigfiles{1} = fullfile(respath, 'serrob_ik_hist_3T2R_startoff100.fig');
CellFigfiles{2} = fullfile(respath, 'serrob_ik_hist_3T2R_startoff20.fig');
CellFigfiles{3} = fullfile(respath, 'serrob_ik_hist_3T3R_startoff100.fig');
CellFigfiles{4} = fullfile(respath, 'serrob_ik_hist_3T3R_startoff20.fig');
% CellFigfiles{1} = fullfile(respath, 'serrob_ik_hist_3T2R_startoff20.fig');
% CellFigfiles{2} = fullfile(respath, 'serrob_ik_hist_3T2R_startoff20.fig');
% CellFigfiles{3} = fullfile(respath, 'serrob_ik_hist_3T2R_startoff20.fig');
% CellFigfiles{4} = fullfile(respath, 'serrob_ik_hist_3T2R_startoff20.fig');

% Neue Legende erzeugen
figHandle = figure(200);clf;
axhdl = set_fig2subfig(figHandle,CellFigfiles);
chax1 = get(axhdl(1), 'children');
legentries = chax1([1 3 5 7 8 9 10]);
legh = legend(legentries, {'try 1', 'try 3', 'try 5', '...', 'try < 15', 'limits viol.', 'wrong'});

% Einzelne Subplots formatieren
th = NaN(4,1);
axes(axhdl(1))
% Die Titel sehen im PDF anders als als im Matlab-Figure (Schriftart)
th(1) = title('(a) Inverse Kinematics Statistics for 3T2R Tasks with Arbitrary Initial Values');
ylges = ylabel('Frequency of occurence of IK results in %'); % Für alle vier Subplots
% ylabel('Frequency of IK results in %');
axes(axhdl(2))
th(2) = title('(b) Inverse Kinematics Statistics for 3T2R Tasks with Initial Value Near Desired Pose');
% ylabel('Frequency of IK results in %');
axes(axhdl(3))
th(3) = title('(c) Inverse Kinematics Statistics for 3T3R Tasks with Arbitrary Initial Values');
axes(axhdl(4))
th(4) = title('(d) Inverse Kinematics Statistics for 3T3R Tasks with Initial Value Near Desired Pose');
% ylabel('Frequency of IK results in %');
xlabel('Number of Serial Link Kinematics');
[X_off, X_slope] = get_relative_position_in_axes(axhdl(1), 'x');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl(1), 'y');
set(ylges, 'Position', [X_off+X_slope*(-1.09), Y_off+Y_slope*(0.5), -1], ...
  'HorizontalAlignment', 'Right');
remove_inner_labels(axhdl, 1)
linkxaxes

for i = 1:4
  [X_off, X_slope] = get_relative_position_in_axes(axhdl(i), 'x');
  [Y_off, Y_slope] = get_relative_position_in_axes(axhdl(i), 'y');
  set(th(i), 'HorizontalAlignment', 'Left', 'Position', [X_off+X_slope*(-0.95), Y_off+Y_slope*(1.02), 0]);
end

% Abstand zwischen den Säulen entfernen. Dieser führt zu merkwürdigen
% Interpolationseffekten im pdf.
for i = 1:4
  fprintf('Subplot %d\n', i);
  t1 = tic();
  ch = get(axhdl(i), 'Children');
  for j = 1:length(ch)
    fprintf('Subplot %d: Bearbeite %d/%d. Voraussichtlich noch %1.0fs für Subplot.\n', ...
      i, j, length(ch), toc(t1)/j*(length(ch)-j));
    set(ch(j), 'BarWidth', 1.0);
  end
end

%% Formatieren
figure_format_publication(axhdl)
set_size_plot_subplot(figHandle,...
  15.5,13,axhdl,...
  0.07,0.005,0.1,0.06,... % bl,br,hu,hd,
  0,0.05) % bdx,bdy)
set(legh, 'position', [0.15    0.955    0.70    0.04], ...
  'orientation', 'horizontal');

saveas(figHandle, fullfile(respath, sprintf('serrob_ik_hist_all.fig')));
export_fig(figHandle, fullfile(respath, sprintf('serrob_ik_hist_all.pdf')));
export_fig(figHandle, fullfile(respath, sprintf('serrob_ik_hist_all.eps')));
export_fig(figHandle, fullfile(respath, sprintf('serrob_ik_hist_all.png')));

%% Zusätzliche Informationen
NrotJ_all = l.AdditionalInfo(II,5);
n_i = 0;
for i = 1:length(II)
  if n_i < NrotJ_all(i)
    fprintf('Ab Nummer %d (%s): %d Drehgelenke\n', ...
      i, l.Names_Ndof{II(i)}, NrotJ_all(i));
    n_i = NrotJ_all(i);
  end
end
Namelist = {'S6RRRRRR10', 'S6RRPRRR14'};
for i = 1:length(Namelist)
  ii_i = find(strcmp(l.Names_Ndof(II), Namelist{i}));
  fprintf('Modell %s: Nr. %d\n', Namelist{i}, ii_i);
end

%% Histogramm für alle Ergebnisse zeichnen
matfiles = cell(1,4); % TODO: Bei Erstellung auch gefilterte Daten laden
matfiles{1} = fullfile(respath, 'serrob_ik_histogramm_3T2R_startoff100.mat');
matfiles{2} = fullfile(respath, 'serrob_ik_histogramm_3T2R_startoff20.mat');
matfiles{3} = fullfile(respath, 'serrob_ik_histogramm_3T3R_startoff100.mat');
matfiles{4} = fullfile(respath, 'serrob_ik_histogramm_3T3R_startoff20.mat');
figure(10);clf;
axhdl = NaN(1,4);
titles = {'3T2R general', '3T2R with $\Delta \boldmath{q}^0 < 20\%$', '3T3R general', '3T3R with $\Delta \boldmath{q}^0 < 20\%$'};
for i = 1:4
  axhdl(i) = subplot(1,4,i);
  erg_i = load(matfiles{i}, 'IK_hist_Ant_ges');
  HistData = erg_i.IK_hist_Ant_ges(I_RobAusw,:);
  IK_Hist_success = sum(HistData(:,1:end-2),2); % Lasse die Klassen "wrong" und "lim. viol." weg
  if i == 4
    edges = [98,100];
  else
    edges = [80,100];
  end % ,'BinLimits',edges
  h = histogram(100*IK_Hist_success,'Normalization','cdf');
  set(h,'FaceAlpha',1); % https://stackoverflow.com/questions/49078762/how-to-save-a-matlab-histogram-as-vector-graphic
  title(titles{i}, 'interpreter', 'latex');
  if i == 2 || i == 4
    xlim([98,100.3]);
  end
  % Bild-Beschriftung einfügen
  [X_off, X_slope] = get_relative_position_in_axes(axhdl(i), 'x');
  [Y_off, Y_slope] = get_relative_position_in_axes(axhdl(i), 'y');
  text(X_off+X_slope*(-0.94), Y_off+Y_slope*(0.78), ['(',char(96+i),')']);
  grid on;
end
figure_format_publication(axhdl)
set_size_plot_subplot(10,...
  15.5,3,axhdl,...
  0.07,0.005,0.14,0.27,... % left,right,up,down,
  0.01,0.00) % bdx,bdy)
remove_inner_labels(axhdl, 2)
axes(axhdl(1));
ylabel('cumulated frequ.', 'horizontalAlignment', 'center');
axes(axhdl(2));
xlabel('IK success in percent');
linkaxes(axhdl, 'y')

saveas(10, fullfile(respath, sprintf('serrob_ik_hist_cdf_all.fig')));
export_fig(10, fullfile(respath, sprintf('serrob_ik_hist_cdf_all.pdf')));

%% Einzelauswertungen
for i = 1:4
  erg_i = load(matfiles{i}, 'IK_hist_Ant_ges', 'I_RobAusw', 'l', 'II');
  HistData = erg_i.IK_hist_Ant_ges(erg_i.I_RobAusw,:);
  NumWrong = HistData(:,end-1)+HistData(:,end);
  NumRight = 1-NumWrong;
  fprintf('i=%d: Untere Grenze für Erfolgsquote: %1.0f%%\n', i,  100*min(NumRight));
  [n,x] = hist(NumRight);
  n_rel = n/sum(n);
  fprintf('i=%d: Anteil der Container mit Mitte über 90%%: %1.1f%%\n', i, 100*sum(n_rel(x > 0.9)));
  
  % TODO: Die Nummer kann erst ausgewertet werden, wenn die Sortierung der
  % Daten auch geladen wird
  for jj = 5:6
    Ijj = (erg_i.l.AdditionalInfo(erg_i.II(erg_i.I_RobAusw),5) == jj);
    [p_worst, ii_worst] = max(sum(HistData(Ijj,end-1:end),2));
    IIjj = find(Ijj);
    iii_worst = IIjj(ii_worst);
    II_Ausw = erg_i.II(erg_i.I_RobAusw);
    Name_worst = erg_i.l.Names_Ndof{II_Ausw(iii_worst)};
    fprintf('i=%d: Schlechteste Erfolgsquote %dR: %1.2f%% bei %d (%s)\n', ...
      i, jj, 100*(1-p_worst), NaN*iii_worst, Name_worst);
  end
end