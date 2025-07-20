%% Agent-centered rotation (2D) - sandbox
% By: Joris van Gool
% Last edited: 18/07/2025
clear; close all; clc; rng(1);

% Parameters
l = 2;         % distance potential exponent
I2 = eye(2);   % 2D identity matrix 
c = 1;          % controller gain

% Time parameters
T = 1;         % end time
dt = 1e-3;     % time step
time = 0:dt:T; % time vector

% Incidence matrix (triangle)
B = [ 1  0 -1;
     -1  1  0;
      0 -1  1];
B_bar = kron(B,I2);

% Desired edge lengths
d = [3,3,3]';
[nA, nE] = size(B);

% Initial positions
p0 = init_triangle(d);

%%%%%%%%%%%%%% virtual triangle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Incidence matrix (virtual triangle)
Bv = [ 1  0 -1  1  0  0;   % agent 1
     -1  1  0  0  1  0;   % agent 2
      0  0  0 -1 -1 -1;   % agent 3 (center)
      0 -1  1  0  0  1];  % agent 4
Bv_bar = kron(Bv,I2);

% Desired edge length (virtual triangle)
p4 = 3*p0(5:6) - p0(1:2) - p0(3:4);
db = [norm(p0(1:2) - p0(3:4)), norm(p0(3:4) - p4), norm(p4 - p0(1:2))]';

% Motion parameters
omega_star = (2*pi);
mus = omega_star*mu_rotation(db);
mu = [mus(1:3); zeros(nE,1)]; mu_tilde = [mus(4:6); zeros(nE,1)];

% Compute A
A = zeros(size(Bv));
for i = 1:4
    for k = 1:6
        if Bv(i,k) == 1
            A(i, k) = mu(k);
        elseif Bv(i,k) == -1
            A(i, k) = mu_tilde(k);
        end
    end
end
A_bar = kron(A,I2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ODE45 call
odefun = @(t, p) formation_dynamics(t, p, B_bar, A_bar, d,c, l);
options = odeset('RelTol', 1e-16, 'AbsTol', 1e-16); 
[t_out, P_out] = ode45(odefun, [0 T], p0, options);

% Store results for plotting
P = P_out';
Z_norms = zeros(nE, length(t_out));
E = zeros(nE, length(t_out));
for t = 1:length(t_out)
    p = P(:, t);
    z = B_bar' * p;
    z_norm = vecnorm(reshape(z, 2, [])', 2, 2);
    e = z_norm.^l - d.^l;
    Z_norms(:, t) = z_norm;
    E(:, t) = e;
end

%% Plot results
% Plot errros over time
figure;
plot(t_out, E, 'LineWidth', 2);
xlabel("Time, t, [s]");
ylabel("Error, e_k, [m^2]");
title("Edge error over time");
legend(arrayfun(@(k) ['e_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false), 'Location','northeast');
grid on;

% Plot distances over time
figure;
plot(t_out(2:end), Z_norms(:,2:end), 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Distance, ||z_k||, [m]");
title("Edge length over time");
legend(arrayfun(@(k) ['z_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false), 'Location', 'northeast');
grid on;

%% 2D Animation
Ns = 300; % speedup factor (integer value only)
figure;
hold on;
grid on;
axis equal;
colors = [1 0 0; 0 1 0; 0 0 1];

% Agent trajectories
traj = cell(nA, 1);

tic
for t = 1:Ns:length(t_out)
    clf;
    hold on; grid on; axis equal;

    % Extract agent positions
    P_t = reshape(P(:, t), 2, [])';

    % Loop over agents
    for i = 1:nA
        % Update the trajectories
        traj{i} = [traj{i}; P_t(i, :)];

        % Colors
        agent_color = colors(mod(i-1, 3) + 1, :);

        % Plot the trajectories + positions
        plot(traj{i}(:, 1), traj{i}(:, 2), 'LineWidth', 1.5, 'Color', agent_color);
        scatter(P_t(i, 1), P_t(i, 2), 100, 'o', 'MarkerFaceColor', agent_color, 'MarkerEdgeColor', agent_color);
    end

    % Plot edges
    for k = 1:nE
        agents = find(B(k, :));
        plot(P_t(agents, 1), P_t(agents, 2), 'k-', 'LineWidth', 1.5);
    end

    % Fill the triangle
    fill(P_t(:, 1), P_t(:, 2), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

    % Title + small pause
    title(['Time: ', num2str(t_out(t)), ' seconds']);
    pause(1e-3);
end
toc

%% %%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%
function dpdt = formation_dynamics(~, p, B_bar, A_bar, d,c, l)
% Compute z(t)
z = B_bar' * p;
S = [eye(6),zeros(6,2)]; % selection matrix

%%%%%%%%% Option 1: positions are known %%%%%%%%%
% p4 = 3*p(5:6) - p(3:4) - p(1:2);
% pv = [p;p4];
% zv = Bv_bar'*pv;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%% Option 2: positions are unknown %%%%%%%
% Known edge vectors from base triangle
z1 = z(1:2);       % Edge 1: p1 - p2
z4 = -z(5:6);      % Edge 4: p1 - p3
z5 =  z(3:4);      % Edge 5: p2 - p3

% Build z_known and indexing
z_known_values = [z1; z4; z5];              % Known edge values (z1, z4, z5)
known_idx      = [1:2, 7:8, 9:10];          % Their positions in zv
unknown_idx    = setdiff(1:12, known_idx);  % Unknown vector components

% Constraint matrix A
A_base = [
     1  1  1   0  0   0 ;   % z1 + z2 + z3 = 0
     1  0  0  -1  1   0 ;   % z1 - z4 + z5 = 0
     0  1  0   0 -1   1 ;   % z2 - z5 + z6 = 0
     0  0  1   1  0  -1 ;   % z3 + z4 - z6 = 0
     0  0  0   1  1   1     % z4 + z5 + z6 = 0 (centroid constraint)
];
A_base_bar = kron(A_base, eye(2));

% Partition and solve
A_known   = A_base_bar(:, known_idx);
A_unknown = A_base_bar(:, unknown_idx);
b         = -A_known * z_known_values;
x         = A_unknown \ b; % Solve for unknown components

% Assemble full edge vector zv
zv = zeros(12, 1);
zv(known_idx)   = z_known_values;
zv(unknown_idx) = x;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Calculate error
z_norm = vecnorm(reshape(z, 2, [])', 2, 2);
e = z_norm.^l - d.^l;

% Block diagonal matrix Dz
Dz = zeros(6, 3);
for i = 1:3
    Dz(2*i-1:2*i, i) = z(2*i-1:2*i);
end

% Compute control input
dpdt = -c * B_bar * Dz *e + S * A_bar * zv;
end

%

% Calculate initial triangle
function p0 = init_triangle(d)
    % Invalid triangle
    if any([d(1)+d(2)<=d(3), d(2)+d(3)<=d(1), d(3)+d(1)<=d(2)])
        error('Invalid triangle: violates triangle inequality.');
    end
    
    % Compute inital positon of triangle to match d
    a = d(1); r1 = d(3); r2 = d(2);
    x = (r1^2 - r2^2 + a^2) / (2*a);
    y = sqrt(r1^2 - x^2);
    p0 = [0;0; d(1);0; x;y];
end