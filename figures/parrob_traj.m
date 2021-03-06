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
if isempty(export_path)
  error('Dieses Skript muss in den Matlab-Pfad hinzugefügt werden oder im aktuellen Ordner liegen');
end
%% Ergebnis-Bild zusammenstellen
format1 = {'k', '', '-', 5; ...
           'b', 's', '-', 4; ...
           'r', 'd', '-', 3};
format2 = {'b', 'o', '--', 5; ...
           'b', 'v', '-', 7; ...
           'r', 's', '--', 5; ...
           'k', '',  '-', 1};
format3 = {'r', 's', '--', 7; ... % gleiche Farben wie Ende von format2
           'k', '',  '-', 1};

figure(30);clf;
axhdl = NaN(2,2);
ylhdl = NaN(2,2);
axhdl(1,1) = subplot(2,2,sprc2no(2,2,1,1));
linhdl11=plot(data.t, data.X1_ist(:,1:3));
leglinhdl1 = line_format_publication(linhdl11, format1);
leg1 = legend(leglinhdl1, {'$x$', '$y$', '$z$'}, 'location', 'northoutside', ...
  'orientation', 'horizontal', 'interpreter', 'latex');
ylhdl(1,1) = ylabel('Position in m'); grid on;
axhdl(1,2) = subplot(2,2,sprc2no(2,2,1,2)); hold on;
linhdl21=plot(data.t, 180/pi*data.X1_ist(:,4:6));
linhdl22=plot(data.t, 180/pi*data.X2_ist(:,6));
leglinhdl2 = line_format_publication([linhdl21;linhdl22], format2);
leg2 = legend(leglinhdl2, {'$\beta_1$', '$\beta_2$', '$\beta_3$ (ser. IK)', '$\beta_3$  (par.  IK)'}, ...
  'location', 'northoutside', 'orientation', 'horizontal', 'interpreter', 'latex');
ylhdl(1,2) = ylabel('Angles in deg'); grid on;
axhdl(2,1) = subplot(2,2,sprc2no(2,2,2,1)); hold on;
linhdl31=plot(data.t, data.H11_t(:,1));
linhdl32=plot(data.t, data.H12_t(:,1));
leglinhdl3 = line_format_publication([linhdl31;linhdl32], format3);
leg3 = legend(leglinhdl3, {'ser. IK', 'par. IK'}, ...
  'location', 'northoutside', 'orientation', 'horizontal', 'interpreter', 'latex');
xlabel('Time in s');
ylhdl(2,1) = ylabel('opt. crit. 1'); grid on;
axhdl(2,2) = subplot(2,2,sprc2no(2,2,2,2)); hold on;
linhdl41=plot(data.t, log10(data.H21_t(:,1)));
linhdl42=plot(data.t, log10(data.H22_t(:,1)));
leglinhdl4 = line_format_publication([linhdl41;linhdl42], format3);
leg4 = legend(leglinhdl4, {'ser. IK', 'par. IK'}, ...
  'location', 'northoutside', 'orientation', 'horizontal', 'interpreter', 'latex');
xlabel('Time in s');
ylhdl(2,2) = ylabel('log(opt. crit. 2)'); grid on;
linkxaxes
xlim(minmax2(data.t'))

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
ylp=get(ylhdl(1,1), 'Position'); set(ylhdl(1,1), 'position', [X_off+X_slope*(-1.2), ylp(2),0]);

axes(axhdl(1,2));
text(X_off+X_slope*(-1.3),Y_off+Y_slope*(-1.4),'(b)');
[X_off, X_slope] = get_relative_position_in_axes(axhdl(2,1), 'x');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl(2,1), 'y');
ylp=get(ylhdl(1,2), 'Position'); set(ylhdl(1,2), 'position', [X_off+X_slope*(-1.2), ylp(2),0]);

axes(axhdl(2,1));
text(X_off+X_slope*(-1.25),Y_off+Y_slope*(-1.4),'(c)');
[X_off, X_slope] = get_relative_position_in_axes(axhdl(2,2), 'x');
[Y_off, Y_slope] = get_relative_position_in_axes(axhdl(2,2), 'y');
ylp=get(ylhdl(2,1), 'Position'); set(ylhdl(2,1), 'position', [X_off+X_slope*(-1.2), ylp(2),0]);

axes(axhdl(2,2));
text(X_off+X_slope*(-1.3),Y_off+Y_slope*(-1.4),'(d)');
ylp=get(ylhdl(2,2), 'Position'); set(ylhdl(2,2), 'position', [X_off+X_slope*(-1.2), ylp(2),0]);

set(leg1, 'position', [0.1    0.92    0.3    0.06]);
set(leg2, 'position', [0.51    0.92    0.4    0.06]);
set(leg3, 'position', [0.1    0.50    0.3    0.06]);
set(leg4, 'position', [0.6    0.50    0.3    0.06]);

figure_format_publication(axhdl)

export_fig(30, fullfile(export_path, 'parrob_traj_results1.pdf'));

%% Roboter mit Trajektorie
serroblib_addtopath({'S6RRPRRR14V3'});
RP = data.RP;
figure(40);clf;
hold on;grid on;
xlabel('$x$ in m', 'interpreter', 'latex');
ylabel('$y$ in m', 'interpreter', 'latex');
zlabel('$z$ in m', 'interpreter', 'latex');
view(3);
s_plot = struct( 'ks_legs', [], 'straight', 0);
RP.plot( data.q1,  data.X0,  s_plot );
plot3(data.XL(:,1), data.XL(:,2), data.XL(:,3), 'k-', 'LineWidth', 2);
figure_format_publication()
set_size_plot_subplot(40,...
  8,8,gca,...
  0.01,0.01,0.0,0.01,... % bl,br,hu,hd,
  0,0) % bdx,bdy)
cd(export_path);
export_fig parrob_traj_zero_pose.png -r864
%% Ausgabe der Trajektorien-Eckwerte (für Text im Paper)
data.XL
