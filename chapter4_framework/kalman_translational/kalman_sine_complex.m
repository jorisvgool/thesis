%% Predictive translational motion control (3D) - simple sinusoidal motion
% By: Joris van Gool
% Last edited: 19/07/2025
clear; close all; clc; rng(1);

% Note: A custom fixed-step RK4 solver is used instead of ODE45, 
% which uses a variable-step integrator. Fixed-step integration 
% is required here to maintain consistent timing for discrete-time 
% control and estimation (i.e., Kalman filtering).

% KALMAN active?
KALMAN = true;

% Parameters
l = 2;          % distance potential exponent
I2 = eye(2);    % identity matrix (2D)
c = 0.1;        % controller gain

% Time parameters
T = 20;        % simulation time [s]
dt = 1e-4;     % time step [s]
time = 0:dt:T;
N = length(time);

% Incidence matrix (triangle)
B = [  1  0  -1;
      -1  1   0;
       0 -1   1];
B_bar = kron(B,I2);

% Desired edge length
d = [3; 3; 3];
[nA, nE] = size(B);
Bk = [0 0 1 0 1 0 ; 0 0 0 1 0 1]';

% External force
fx = 2 * sin(1.1 * time);
fy = 0*time;
f_ext = [fx .* (time >= 0); zeros(5, N)];

% Simulation parameters
P = zeros(2*nA, N); % Positions storage
p = [0; 0; (sqrt(3)/2)*3; 3/2; (sqrt(3)/2)*3; -3/2];
h = [0,2,2];
P(:, 1) = p;

% More storage
Z = zeros(nE, length(time));
E = zeros(nE,length(time));
e = zeros(nE,1);
Evel = zeros(2,length(time));
VHAT = zeros(2,length(time));

% Storage for Kalman results
p1_mes = zeros(2, N);

% Kalman Filter Initialization
x_est = [p(1:2); zeros(12,1); 1.25 ; 1.25];
P_est = eye(16);

% Measurement noise covariance
R = 0 * eye(2); 

% Main simulation loop
for t = 2:N
    
   % Calculate z(t)
    z = B_bar'*p;
    
    % Calculate error
    z_norm = vecnorm(reshape(z, 2, [])', 2, 2);
    Z(:, t) = z_norm;
    e = z_norm.^l - d.^l;
    E(:,t) = e;
    
    % Block diagonal matrix Dz
    Dz = zeros(6, 3);
    for i = 1:3
        Dz(2*i-1:2*i, i) = z(2*i-1:2*i);
    end

    % Kalman's filter
    switch KALMAN
        case true
        % Add measurement noise (if applicable)
        p1_mes(:, t) = p(1:2) + mvnrnd([0; 0], R)';
    
        % Apply Kalman Filter
        [x_est, P_est] = kalman_filter(x_est, P_est, p1_mes(:, t),dt,time(t));
        v_hat = x_est(3:4);
        VHAT(:,t) = v_hat;
        Evel(:,t) = f_ext(1:2,t)-v_hat;
        
        case false
        v_hat = [0;0];
    end

    % Compute position update
    p_dot = -c*B_bar*Dz*e + Bk*v_hat + f_ext(:,t);
    p = RK4_solver(p, p_dot, dt);

    % Store updated positions
    P(:, t) = p;
end

%% Plot results
% Plot error over time
figure;
plot(time(2:end), E(:,2:end), 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Error E_k [m^2]");
title("Error over time");
legend(arrayfun(@(k) ['e_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false), 'Location', 'northwest');
grid on;

% Plot distances over time
figure;
plot(time(2:end), Z(:,2:end), 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Distance [m]");
title("Distances over time");
legend(arrayfun(@(k) ['z_', num2str(k), '(t)'], 1:nE, 'UniformOutput', false), 'Location', 'northwest');
grid on;

%% Compute RMSE for x-component
rmse_x = sqrt(mean((f_ext(1,:) - VHAT(1,:)).^2));

% Define time span to show (first third)
idx_end = floor(length(time) / 3);

% Plot velocity signals
figure;
plot(time(2:idx_end), f_ext(1,2:idx_end), 'LineWidth', 2); hold on;
plot(time(2:idx_end), VHAT(1,2:idx_end), '--', 'LineWidth', 2);

xlabel("Time [s]");
ylabel("Velocity x [m/s]");
title(sprintf("(RMSE = %.4f)", rmse_x));
legend({'v_x (true)', 'v_x (estimated)'}, 'Location', 'best');
grid on;


%% 3D Animation
Ns = round(N/25); % Update interval
trail_length = 100; % Number of steps before trajectories disappear
figure;
hold on;
grid on;
axis equal;
colors = [1 0 0; 0 1 0; 0 0 1]; % Colors for agents

% Store agent trajectories
traj = cell(nA, 1); 
for i = 1:nA
    traj{i} = [];
end

tic
for t = 1:Ns:length(time)
    clf; 
    hold on;
    grid on;
    axis equal;
    view(3);
    
    % Extract current agent positions
    P_t = [reshape(P(:, t), 2, [])', h'];

    % Compute centroid (center of the formation)
    centroid = mean(P_t, 1);
    
    % Adjust axis limits dynamically to keep formation centered
    plot_range = 2;
    xlim([centroid(1) - plot_range, centroid(1) + plot_range]);
    ylim([centroid(2) - plot_range, centroid(2) + plot_range]);
    zlim([centroid(3) - plot_range, centroid(3) + plot_range]);

    % Loop over agents
    for i = 1:nA
        agent_color = colors(mod(i-1, 3) + 1, :);
        
        % Store trajectory
        traj{i} = [traj{i}; P_t(i, :)];
        if size(traj{i}, 1) > trail_length
            traj{i}(1, :) = [];
        end
        
        % Plot trajectory
        plot3(traj{i}(:, 1), traj{i}(:, 2), traj{i}(:, 3), '-', ...
            'LineWidth', 1.5, 'Color', [agent_color, 0.5]); % Faint lines

        % Scatter agents at their current positions
        scatter3(P_t(i, 1), P_t(i, 2), P_t(i, 3), 100, 'o', ...
            'MarkerFaceColor', agent_color, 'MarkerEdgeColor', agent_color);
    end

    % Plot edges of the triangle
    for k = 1:nE
        agents = find(B(k, :));
        plot3(P_t(agents, 1), P_t(agents, 2), P_t(agents, 3), 'r-', 'LineWidth', 2);
    end

    % Fill the triangle with transparency
    fill3(P_t(:, 1), P_t(:, 2), P_t(:, 3), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');

    % Title + small pause
    title(['Time: ', num2str((t-1)*dt), ' seconds']);
    pause(1e-2);
end
toc

%% %%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_est, P_est] = kalman_filter(x_est, P_est, z, dt, t)
    % Extract estimated frequencies
    omega1 = x_est(15);
    omega2 = x_est(16);

    % Trigonometric terms
    sin1 = sin(omega1 * t); cos1 = cos(omega1 * t);
    sin2 = sin(omega2 * t); cos2 = cos(omega2 * t);

    % Initialize transition matrix
    A = eye(16);

    % --- Index Map (matching desired state structure) ---
    %  1-2  -> p (position)
    %  3-4  -> v (velocity)
    %  5-6  -> a0 (constant term)
    %  7-8  -> a1 (linear term)
    %  9-10 -> a2 (quadratic term)
    % 11-12 -> alpha (sinusoidal amplitude)
    % 13-14 -> beta (cosine amplitude)
    % 15-16 -> omega (frequencies)

    % Position update from velocity model v(t)
    A(1,3) = dt;
    A(2,4) = dt;

    % Velocity update from polynomial + harmonic model
    A(3,3) = 0;
    A(3,5)  = 1;
    A(3,7)  = t;
    A(3,9)  = t^2;
    A(3,11) = sin1;
    A(3,13) = cos1;
    A(3,15) = t * (x_est(11)*cos1 - x_est(13)*sin1);
    
    A(4,4) = 0;
    A(4,6)  = 1;
    A(4,8)  = t;
    A(4,10) = t^2;
    A(4,12) = sin2;
    A(4,14) = cos2;
    A(4,16) = t * (x_est(12)*cos2 - x_est(14)*sin2);

    % Observation matrix (measuring position)
    H = zeros(2, 16);
    H(1,1) = 1;
    H(2,2) = 1;

    % Control input matrix (not used, but usefull for extensions)
    % B = zeros(16, 2);
    % B(3,1) = 1;
    % B(4,2) = 1;
    % 
    % B(1,1) = dt;
    % B(2,2) = dt;


    % Noise matrices
Q = diag([ ...
    1e-8, 1e-8, ...     % position
    1e-5, 1e-5, ...     % velocity
    1e-4, 1e-4, ...     % a0
    1e-5, 1e-5, ...     % a1
    1e-5, 1e-5, ...     % a2
    1e-3, 1e-3, ...     % alpha
    1e-3, 1e-3, ...     % beta
    1e-5, 1e-5  ...     % omega
]);
R = 1e-5 * eye(2); % Observation noise

    % EKF prediction
    x_pred = A * x_est;
    P_pred = A * P_est * A' + Q;

    % Kalman gain + update
    K = P_pred * H' / (H * P_pred * H' + R);
    x_est = x_pred + K * (z - H * x_pred);
    P_est = (eye(16) - K * H) * P_pred;
end

function [x_next] = RK4_solver(x_prev, x_dot, dt)
% RK4_SOLVER - One-step integration using the fourth-order Runge-Kutta method.
%
% Syntax:
%    x_next = RK4_solver(x_prev, x_dot, dt)
%
% Inputs:
%    x_prev - nx1 vector representing the current state of the system
%    x_dot  - nx1 vector representing the time derivative of the state at the current time
%    dt     - Scalar time step for integration
%
% Outputs:
%    x_next - nx1 vector representing the estimated state at the next time step

% Calculate the RK4 intermediate steps
k1 = x_dot;
k2 = x_dot + 0.5 * dt * k1;
k3 = x_dot + 0.5 * dt * k2;
k4 = x_dot + dt * k3;

% Compute the next state
x_next = x_prev + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
end