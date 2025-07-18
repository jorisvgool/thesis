%% Gradient-based formation control with disturbance (2D) - square
% By: Joris van Gool
% Last edited: 18/07/2025
clear; close all; clc; rng(80);

% Model parameters
l = 2;         % distance potential exponent
I2 = eye(2);   % 2D identity matrix 
c = 0.5;         % controller gain

% Time parameters
T = 1;         % end time
dt = 1e-2;     % time step
time = 0:dt:T; % time vector

% Incidence matrix (triangle)
B = [  1   1   0   0   0;
      -1   0   1   1   0;
       0   0  -1   0   1;
       0  -1   0  -1  -1];
B_bar = kron(B,I2);
[nA, nE] = size(B);

% Desired edge lengths
d = [3,3,3,sqrt(18),3]';

% Initial positions
p0 = [0 0 1 6 5 3 1 2];

% Define external vel. input
% f_ext = [5 0 0 0 0 0]'; % constant external input
f_ext = zeros(nA*2,1);         % no external input

% Solve system using ODE
odefun = @(t, p) formation_dynamics(t, p, B_bar, f_ext, d, c, l);
options = odeset('RelTol', 1e-16, 'AbsTol', 1e-16);
[t_out, P_out] = ode45(odefun, time, p0, options);

% Store results for plotting (edge length + errors)
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
plot(time(2:end), Z_norms(:,2:end), 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Distance, ||z_k||, [m]");
title("Edge length over time");
legend(arrayfun(@(k) ['z_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false), 'Location', 'northeast');
grid on;

%% 2D Animation
Ns = round(length(t_out)/70); % speedup factor (integer value only)
figure;
hold on;
grid on;
axis equal;
colors = [1 0 0; 0 1 0; 0 0 1 ; 1 0 1];

% Agent trajectories
traj = cell(nA, 1);

tic
for t = [1:Ns:length(t_out),length(t_out)]
    clf;
    hold on; grid on; axis equal;

    % Extract agent positions
    P_t = reshape(P(:, t), 2, [])';

    % Loop over agents
    for i = 1:nA
        % Update the trajectories
        traj{i} = [traj{i}; P_t(i, :)];

        % Colors
        agent_color = colors(mod(i-1, 4) + 1, :);

        % Plot the trajectories + position
        plot(traj{i}(:, 1), traj{i}(:, 2), 'LineWidth', 1.5, 'Color', agent_color);
        scatter(P_t(i, 1), P_t(i, 2), 100, 'o', 'MarkerFaceColor', agent_color, 'MarkerEdgeColor', agent_color);
    end

    % Plot edges
    for k = 1:nE
        agents = find(B(:, k));
        plot(P_t(agents, 1), P_t(agents, 2), 'k-', 'LineWidth', 1.5);
    end

    % Fill the triangle
    fill(P_t(:, 1), P_t(:, 2), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
   
    % Title + small pause
    title(['Time: ', sprintf('%.3f', t_out(t)), ' seconds']);
    pause(1e-3);
end
toc

%% %%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%

% ODE function
function dpdt = formation_dynamics(~, p, B_bar, f_ext, d, c, l)
    % Compute relative positions
    z = B_bar' * p;
    
    % Calculate error
    z_norm = vecnorm(reshape(z, 2, [])', 2, 2);
    e = z_norm.^l - d.^l;
    
    % Block diagonal matrix Dz
    Dz = zeros(10, 5);
    for i = 1:5
        Dz(2*i-1:2*i, i) = z(2*i-1:2*i);
    end
    Dz_tilde = diag(z_norm.^(l-2));
    
    % Compute control input
    dpdt = -c*B_bar*Dz*Dz_tilde*e + f_ext;
end