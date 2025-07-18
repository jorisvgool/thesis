%% Distributed formation motion control (2D) - example (fig 3.3)
% By: Joris van Gool
% Last edited: 18/07/2025
clear; close all; clc;

% Model parameters
l = 1;
I2 = eye(2);
c = 1;

% Time parameters
T = 60;
dt = 1e-5;
time = 0:dt:T;

% Incidence matrix
B = [ 1  0 -1;
     -1  1  0;
      0 -1  1];
[nA, nE] = size(B);
B_bar = kron(B,I2);

% Desired edge lengths
d = [3,3,3]';

% Initial positions
p0 = init_triangle(d);

omega_star = -pi/15;
pdot_star = [0;4/3];

% Compute motion parameters
mus_w = omega_star*mu_rotation(d);
mus_v = mu_translation(d,pdot_star);
mus = zeros(6,length(time));
for k = 1:numel(time)
    if mod(floor(8*time(k)/T), 2) == 0
        mus(:,k) = mus_v;
    else
        mus(:,k) = mus_w;
    end
end

% Solve system using ODE (custom settings)
odefun = @(t, p) formation_dynamics(t, p, B, B_bar, mus, dt, d, c, l);
options = odeset('RelTol', 1e-16, 'AbsTol', 1e-16);
[t_out, P_out] = ode45(odefun, time, p0, options);

% Transpose and store for plotting
P = P_out';
Z = zeros(nE * 2, length(t_out));
E = zeros(nE, length(t_out));
for t = 1:length(t_out)
    p = P(:, t);
    z = B_bar' * p;
    z_matrix = reshape(z, 2, [])';
    e = vecnorm(z_matrix, 2, 2).^l - d.^l;
    
    Z(:, t) = z;
    E(:, t) = e;
end

%% Plot errros over time
figure;
plot(t_out, E, 'LineWidth', 2);
xlabel("Time, t, [s]");
ylabel("Error, e_k, [m^2]");
title("Error over time");
legend(arrayfun(@(k) ['e_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false));
grid on;


%% 2D Animation
Ns = round(length(t_out)/100); % speedup factor (integer value only)
figure;
hold on;
grid on;
axis equal;
colors = [1 0 0; 0 1 0; 0 0 1];

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
        agent_color = colors(mod(i-1, 3) + 1, :);

        % Plot the trajectories + position
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
    title(['Time: ', sprintf('%.3f', t_out(t)), ' seconds']);
    pause(1e-3);
end
toc

%% %%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%
function dpdt = formation_dynamics(t, p, B, B_bar, mus, dt, d, c, l)
% Compute relative positions
z = B_bar' * p;

% Motion parameters
idx = max(1,min(size(mus,2), floor(t/dt)));
mus_t = mus(:,idx);
mu = mus_t(1:3); mu_tilde = mus_t(4:6);

% Define matrix A
A = zeros(3,3);
for i = 1:3
    for k = 1:3
        if B(i,k) == 1
            A(i, k) = mu(k);
        elseif B(i,k) == -1
            A(i, k) = mu_tilde(k);
        end
    end
end
A_bar = kron(A,eye(2));

% Calculate error
z_norm = vecnorm(reshape(z, 2, [])', 2, 2);
e = z_norm.^l - d.^l;

% Block diagonal matrix Dz
Dz = zeros(6, 3);
for i = 1:3
    Dz(2*i-1:2*i, i) = z(2*i-1:2*i);
end
Dz_tilde = diag(z_norm.^(l-2));

% Compute control input
dpdt = -c * B_bar * Dz * Dz_tilde * e + A_bar*z;
end

%%%

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