% Test des Orientierungs-Residuums nach Goldenberg
% 
% Ergebnis:
% Residuum nach Gl. (5) und (6) sind Euler-Winkel (wie bei mir verwendet);
% nach Gl. (7) sind keine Euler-Winkel, sondern Projektion
% 
% Quelle:
% [GoldenbergBenFen1985] "A complete generalized solution to the inverse
% kinematics of robots, IEEE Journal on Robotics and Automation, 1985

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

% Definition einer beliebigen Differenz-Rotationsmatrix
Ra = eye(3); % actual
Rt = rotx(rand())*roty(rand())*rotz(rand()); % target
R_at=Ra'*Rt;

Match = false(3,12);

% Komponenten der Rotationsmatrix
n_a = Ra(:,1);
o_a = Ra(:,2);
a_a = Ra(:,3);
n_t = Rt(:,1);
o_t = Rt(:,2);
a_t = Rt(:,3);

% Residuum mit Euler-Winkeln: [GoldenbergBenFen1985] (5)
r_phi = atan2(o_a'*a_t, ... % r23 (2=o; 3=a)
              n_a'*a_t);    % r13 (1=n; 3=a); Fehler in Formel in Quelle
r_theta = atan2(n_a'*a_t*cos(r_phi)+o_a'*a_t*sin(r_phi), ...
                a_a'*a_t);
r_psi = atan2(-n_a'*n_t*sin(r_phi)+o_a'*n_t*cos(r_phi), ...
              -n_a'*o_t*sin(r_phi)+o_a'*o_t*cos(r_phi));
r_Gold5 = [r_phi;r_theta;r_psi];

% Residuum mit Yaw-Pitch-Roll: [GoldenbergBenFen1985] (6)
r_phi = atan2(o_a'*n_t, ...
              n_a'*n_t);
r_theta = atan2(-a_a'*n_t, ...
                n_a'*n_t*cos(r_phi)+o_a'*n_t*sin(r_phi));
r_psi = atan2(n_a'*a_t*sin(r_phi)-o_a'*a_t*cos(r_phi), ...
              -n_a'*o_t*sin(r_phi)+o_a'*o_t*cos(r_phi));
r_Gold6 = [r_phi;r_theta;r_psi];

% Residuum mit xyz rotation axes: [GoldenbergBenFen1985] (7)
r_phi =   1/2 * (a_a'*o_t - a_t'*o_a);
r_theta = 1/2 * (n_a'*a_t - n_t'*a_a);
r_psi =   1/2 * (o_a'*n_t - o_t'*n_a);
r_Gold7 = [r_phi;r_theta;r_psi];

r_Gold_ges = 180/pi*[r_Gold5, r_Gold6, r_Gold7];

% Prüfe, welche Euler-Winkel das sind
r_Eul_ges = NaN(3,12);
for i = 1:12
  r_Eul_ges(:,i) = 180/pi*r2eul(R_at, uint8(i));
end

for i = 1:12
  for j = 1:3
    if max( abs((r_Gold_ges(:,j) - r_Eul_ges(:,i))) ) < 1e-10
      Match(j,i) = true;
      fprintf('Goldenberg Residuum %d entspricht Euler-Winkeln %d (%s)\n', ...
        j, i, euler_angle_properties(uint8(i)));
    end
  end
end