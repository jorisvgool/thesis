%% Gradient-based formation control with disturbance (3D)
% By: Joris van Gool
% Last edited: 18/07/2025
clear; close all; clc; rng(88);

% Model parameters
l = 2;         % distance potential exponent
I3 = eye(3);   % 3D identity matrix 
c = 1;         % controller gain

% Time parameters
T = 10;        % end time
dt = 1e-2;     % time step
time = 0:dt:T; % time vector
t_step = 5;    % f_ext ~= 0 after
N = length(time);

% Incidence matrix (triangle)
B = [ 1  0 -1;
     -1  1  0;
      0 -1  1];
B_bar = kron(B,I3);
[nA, nE] = size(B);

% Desired edge lengths
d = [3,3,3]';

% Initial positions
p0 = [0; 0; 0; sqrt(2.75); 1.5; 2; sqrt(2.75); -1.5 ; 2];

% Define external vel. input
f_ext = [-1 0 0 0 0 0 0 0 0]'; % constant external input
% f_ext = zeros(9,1);          % no external input

% Solve system using ODE
odefun = @(t, p) formation_dynamics(t, p, B_bar, f_ext, d, c, l, t_step);
options = odeset('RelTol', 1e-16, 'AbsTol', 1e-16);
[t_out, P_out] = ode45(odefun, time, p0, options);

% Store results for plotting (edge length + errors)
P = P_out';
Z = zeros(nE * 3, length(t_out));
E = zeros(nE, length(t_out));
for t = 1:length(t_out)
    p = P(:, t);
    z = B_bar' * p;
    z_matrix = reshape(z, 3, [])';
    e = vecnorm(z_matrix, 2, 2).^l - d.^l;
    Z(:, t) = z;
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
Z_matrix = reshape(Z, 3, nE, []);
Z_norms = squeeze(vecnorm(Z_matrix, 2, 1));
plot(time(2:end), Z_norms(:,2:end), 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Distance, ||z_k||, [m]");
title("Edge length over time");
legend(arrayfun(@(k) ['z_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false), 'Location', 'northeast');
grid on;


%% 3D Animation
Ns = round(N/25); % speedup factor (integer value only)
figure;
hold on;
grid on;
axis equal;
colors = [1 0 0; 0 1 0; 0 0 1];
traj = cell(nA, 1);

tic
for t = 1:Ns:N
    clf;
    hold on;
    grid on;
    axis equal;
    view(3);
    
    % Extract agent positions
    P_t = reshape(P(:, t), 3, [])';
    
    % Loop over agents
    for i = 1:nA
        traj{i} = [traj{i}; P_t(i, :)];
        agent_color = colors(mod(i-1, 3) + 1, :);
        plot3(traj{i}(:, 1), traj{i}(:, 2), traj{i}(:, 3), 'LineWidth', 1.5, 'Color', agent_color);
        scatter3(P_t(i, 1), P_t(i, 2), P_t(i, 3), 100, 'o', 'MarkerFaceColor', agent_color, 'MarkerEdgeColor', agent_color);
    end
    
    % Plot edges
    for k = 1:nE
        agents = find(B(k, :));
        plot3(P_t(agents, 1), P_t(agents, 2), P_t(agents, 3), 'k-', 'LineWidth', 1.5);
    end
    
    % Fill the triangle
    fill3(P_t(:, 1), P_t(:, 2), P_t(:, 3), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

    % Title + small pause
    title(['Time: ', num2str((t-1)*dt), ' seconds']);
    pause(1e-2);
end
toc

%% %%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%

% ODE function
function dpdt = formation_dynamics(t, p, B_bar, f_ext, d, c, l, t_step)
    % Compute relative positions
    z = B_bar' * p;
    
    % Calculate error
    z_norm = vecnorm(reshape(z, 3, [])', 2, 2);
    e = z_norm.^l - d.^l;
    
    % Block diagonal matrix Dz
     Dz = zeros(9, 3); 
    for i = 1:3
        Dz(3*i-2:3*i, i) = z(3*i-2:3*i);
    end
    Dz_tilde = diag(z_norm.^(l-2));

    % Step external input
    if t < t_step
        f_ext = zeros(9,1);
    end
    
    % Compute control input
    dpdt = -c*B_bar*Dz*Dz_tilde*e + f_ext;
    dpdt([3 6 9]) =0;
end
