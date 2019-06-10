% Hexapod mit Trajektorie und unterschiedlichen IK-Einstellungen
% Zeichne Bild für PKM-3T2R-Paper
% 
% Führe vorher Beispiel aus Robotik-Repo aus: ParRob_class_example_6UPS_3T2R.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-06
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

robrepo = fileparts(which('robotics_toolbox_test_repo.m'));
resfile = fullfile(robrepo, 'examples_tests', 'results', 'ParRob_class_example_6UPS_3T2R_results.mat');
data = load(resfile);
export_path = fileparts(which('parrob_traj.m'));
%% Ergebnis-Bild zusammenstellen
format1 = {'k', '', '-', 5; ...
           'b', 's', '-', 4; ...
           'r', 'd', '-', 3};
format2 = {'g', 'o', '-', 2; ...
           'b', 'v', '-', 4; ...
           'r', 's', '-', 5; ...
           'k', '',  '--', 1};
format3 = {'r', 's', '-', 5; ... % gleiche Farben wie Ende von format2
           'k', '',  '--', 1};

figure(30);clf;
axhdl = NaN(2,2);
axhdl(1,1) = subplot(2,2,sprc2no(2,2,1,1));
linhdl11=plot(data.t, data.X1_ist(:,1:3));
leglinhdl1 = line_format_publication(linhdl11, format1);
leg1 = legend(leglinhdl1, {'$x$', '$y$', '$z$'}, 'location', 'northoutside', ...
  'orientation', 'horizontal', 'interpreter', 'latex');
ylabel('Position in m'); grid on;
axhdl(1,2) = subplot(2,2,sprc2no(2,2,1,2)); hold on;
linhdl21=plot(data.t, 180/pi*data.X1_ist(:,4:6));
linhdl22=plot(data.t, 180/pi*data.X2_ist(:,6));
leglinhdl2 = line_format_publication([linhdl21;linhdl22], format2);
leg2 = legend(leglinhdl2, {'$\beta_1$', '$\beta_2$', '$\beta_3 (IK1)$', '$\beta_3  (IK2)$'}, ...
  'location', 'northoutside', 'orientation', 'horizontal', 'interpreter', 'latex');
ylabel('Angles in deg'); grid on;
axhdl(2,1) = subplot(2,2,sprc2no(2,2,2,1)); hold on;
linhdl31=plot(data.t, data.H11_t(:,1));
linhdl32=plot(data.t, data.H12_t(:,1));
leglinhdl3 = line_format_publication([linhdl31;linhdl32], format3);
leg3 = legend(leglinhdl3, {'ser. IK', 'par. IK'}, ...
  'location', 'northoutside', 'orientation', 'horizontal', 'interpreter', 'latex');
xlabel('Time in s');
ylabel('opt. crit. 1'); grid on;
axhdl(2,2) = subplot(2,2,sprc2no(2,2,2,2)); hold on;
linhdl41=plot(data.t, log10(data.H21_t(:,1)));
linhdl42=plot(data.t, log10(data.H22_t(:,1)));
leglinhdl4 = line_format_publication([linhdl41;linhdl42], format3);
leg4 = legend(leglinhdl4, {'ser. IK', 'par. IK'}, ...
  'location', 'northoutside', 'orientation', 'horizontal', 'interpreter', 'latex');
xlabel('Time in s');
ylabel('log(opt. crit. 2)'); grid on;
linkxaxes
xlim(minmax2(data.t'))
figure_format_publication(axhdl)
remove_inner_labels(axhdl, 1);

set_size_plot_subplot(30,...
  15.5,6,axhdl,...
  0.07,0.02,0.10,0.15,... % bl,br,hu,hd,
  0.1,0.1) % bdx,bdy)
set_y_autoscale(axhdl(:),0.1)
[X_off, X_slope] = get_relative_position_in_axes(axhdl(1,1), 'x');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl(1,1), 'y');
axes(axhdl(1,1));
text(X_off+X_slope*(-1.25),Y_off+Y_slope*(-1.4),'(a)');
[X_off, X_slope] = get_relative_position_in_axes(axhdl(1,2), 'x');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl(1,2), 'y');
axes(axhdl(1,2));
text(X_off+X_slope*(-1.3),Y_off+Y_slope*(-1.4),'(b)');
[X_off, X_slope] = get_relative_position_in_axes(axhdl(2,1), 'x');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl(2,1), 'y');
axes(axhdl(2,1));
text(X_off+X_slope*(-1.25),Y_off+Y_slope*(-1.4),'(c)');
[X_off, X_slope] = get_relative_position_in_axes(axhdl(2,2), 'x');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl(2,2), 'y');
axes(axhdl(2,2));
text(X_off+X_slope*(-1.3),Y_off+Y_slope*(-1.4),'(d)');

set(leg1, 'position', [0.1    0.92    0.3    0.05]);
set(leg2, 'position', [0.6    0.92    0.3    0.05]);
set(leg3, 'position', [0.1    0.50    0.3    0.05]);
set(leg4, 'position', [0.6    0.50    0.3    0.05]);
export_fig(20, fullfile(export_path, 'parrob_traj_results1.pdf'));
