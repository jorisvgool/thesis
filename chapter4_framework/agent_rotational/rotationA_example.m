%% Agent-centered rotation (2D) - visual example
% By: Joris van Gool
% Last edited: 18/07/2025
clear; close all; clc; rng(1);

% Model parameters
l = 2;         % distance potential exponent
I2 = eye(2);   % 2D identity matrix 
c = 1;         % controller gain

% Time parameters
T = 1;         % end time
dt = 1e-3;     % time step
time = 0:dt:T; % time vector

% Incidence matrix (triangle)
B = [ 1  0 -1  1  0  0;   % agent 1
     -1  1  0  0  1  0;   % agent 2
      0 -1  1  0  0  1;   % agent 3
      0  0  0 -1 -1 -1];  % agent 4 (center)
B_bar = kron(B,I2);

% Desired edge length
d = [3; 3; 3; sqrt(3); sqrt(3); sqrt(3)];
[nA, nE] = size(B);

% Equilateral triangle + centroid
x = [0, 3, 1.5]; 
y = [0, 0, sqrt(3)/2 * 3];
x_c = mean(x);
y_c = mean(y);

% Initial position vector
p0 = reshape([x; y], [], 1);
p0 = [p0; x_c; y_c];  % append center as agent 4

% Compute rotational motion parameters
mus = (2*pi/3) * mu_rotation(d(1:3));
mu = [mus(1:3) ; 0 ; 0 ; 0];
mu_tilde = [mus(4:6) ; 0 ; 0 ; 0];

% Define A(mu)
A = zeros(size(B));
for i = 1:nA
    for k = 1:nE
        if B(i,k) == 1
            A(i, k) = mu(k);
        end
        if B(i,k) == -1
            A(i, k) = mu_tilde(k);
        end
    end
end
A_bar = kron(A,I2);

% Solve system using ODE
odefun = @(t, p) formation_dynamics(t, p, B_bar, A_bar, d, c, l);
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
plot(t_out(2:end), Z_norms(:,2:end), 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Distance, ||z_k||, [m]");
title("Edge length over time");
legend(arrayfun(@(k) ['z_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false), 'Location', 'northeast');
grid on;

%% 2D Animation
Ns = 100; % speedup factor (integer value only)
figure;
hold on;
grid on;
axis equal;
colors = [1 0 0;     % red
          0 1 0;     % green
          0 0 1;     % blue
          1 0.5 0];  % orange (for the center)

% Agent trajectories
traj = cell(nA, 1);
C = p0(7:8);

tic
for t = 1:Ns:length(time)
    clf; 
    hold on;
    grid on;
    axis equal;
    xlim([C(1)-2.5 C(1)+2.5])
    ylim([C(2)-2.5 C(2)+2.5])
    
    % Extract agent positions
    P_t = reshape(P(:, t), 2, [])';
    
    % Loop over agents
    for i = 1:nA
        % Update the trajectories
        traj{i} = [traj{i}; P_t(i, :)];
        
        % Colors
        agent_color = colors(mod(i-1, 4) + 1, :);  % This cycles through the colors
        
        % Plot the trajectories + positions
        plot(traj{i}(:, 1), traj{i}(:, 2), 'LineWidth', 1.5, 'Color', agent_color);
        scatter(P_t(i, 1), P_t(i, 2), 100, 'o', 'MarkerFaceColor', agent_color, 'MarkerEdgeColor', agent_color);
    end
    
    % Plot edges
    for k = 1:nE
        edge = find(B(:, k) ~= 0);  % indices of connected agents
        if numel(edge) == 2
            plot(P_t(edge, 1), P_t(edge, 2), 'k-', 'LineWidth', 1.5);
        end
    end

    
    % Fill the triangle
    fill(P_t([1 2 4], 1), P_t([1 2  4], 2), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    
    % Title + small pause
    title(['Time: ', num2str((t-1)*dt), ' seconds']);
    pause(1e-3);
end
toc

%% %%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%

% ODE function
function dpdt = formation_dynamics(~, p, B_bar, A_bar, d, c, l)
    % Compute relative positions
    z = B_bar' * p;
    
    % Calculate error
    z_norm = vecnorm(reshape(z, 2, [])', 2, 2);
    e = z_norm.^l - d.^l;
    
    % Block diagonal matrix Dz
    Dz = zeros(12, 6);
    for i = 1:6
        Dz(2*i-1:2*i, i) = z(2*i-1:2*i);
    end
    Dz_tilde = diag(z_norm.^(l-2));
    
    % Compute control input
    dpdt = -c*B_bar*Dz*Dz_tilde*e + A_bar*z;
end