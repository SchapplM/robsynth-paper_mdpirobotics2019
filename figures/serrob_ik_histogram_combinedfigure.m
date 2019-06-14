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
export_fig(figHandle, fullfile(respath, sprintf('serrob_ik_hist_all.png')));