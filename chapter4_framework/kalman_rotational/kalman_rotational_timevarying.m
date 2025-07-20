%% Predictive rotational motion with virtual control framework (2D) - time-varying gamma
% By: Joris van Gool
% Last edited: 20/07/2025
clear; close all; clc; rng(99)

% Note: A custom fixed-step RK4 solver is used instead of ODE45, 
% which uses a variable-step integrator. Fixed-step integration 
% is required here to maintain consistent timing for discrete-time 
% control and estimation (i.e., Kalman filtering).

% Kalman enabled?
KALMAN = true;

% Parameters
l = 2;          % distance potential exponent
I2 = eye(2);    % 2D identity
c = 0.5;        % controller gain

% Time parameters
T = 60;         % simulation time [s]
dt = 0.001;     % time step [s]
time = 0:dt:T;
N = length(time);

% Incidence matrix (triangle)
B = [  1  0  -1;
      -1  1   0;
       0 -1   1];
[nA, nE] = size(B);
B_bar = kron(B,I2);

% Desired edge length
d = [3; 3; 3];

% Simulations parameters
P = zeros(2*nA, length(time)); % positions storage
p = [0; 0;3; 0; 1.5; sqrt(3)/2 * 3];  % inital positons
p0 =p; P(:, 1) = p0;

% Gamma motion parameters
ang_start = 0;    % start angle
ang_end = pi;     % finish angle
aB = [1; 0];      % marker face vector

% Initialize vector storage
a = zeros(2, N);
aO = zeros(2, N);

% Angular velocity coefficients
b0 = 0;
b1 = 0.1;
b2 = 0.002;

% Define dynamic gamma
for t = 1:N
    if t <= N/2
        theta = ang_start;
    else
        % Smooth interpolation
        t_alt = time(t)-time(round(N/2));
        theta = b0+b1*t_alt+b2*t_alt^2;
    end

   % Rotation matrices
    R_theta = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    R_theta_90 = [cos(theta + pi/2), -sin(theta + pi/2); sin(theta + pi/2), cos(theta + pi/2)];

    % Apply to base vector
    a(:, t) = R_theta * aB;
    aO(:, t) = R_theta_90 * aB;
end

% More storage
Z = zeros(3*nE, length(time));
E = zeros(3*nE,length(time));
TH = zeros(2,N);

% kalan init
x_est = [ang_start; 0; 0; 0];  % Initial state
P_est = eye(4);                % Initial covariance matrix
Rk = 0 * eye(1); 
W = zeros(1,N);
G = zeros(1,N);

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
mus = mu_rotation(db);
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

% Main loop
for t = 2:N
   
   
    % Calculate control parameters
    z = B_bar'*p;
    L = vecnorm(reshape(z, 2, [])', 2, 2);
    Z(1:3, t) = L;
    e = L.^l - d.^l;
    E(1:3,t) = e;
    p_dot = zeros(6,1);

    %%%%%%%%%%%% agent 1 perspective %%%%%%%%%%%%%%%%%%%
    % Local measurements (distance/angle)
    th = acos(dot(z(5:6), aO(:,t)) / (norm(z(5:6)) * norm(aO(:,t))));
    TH(1,t) = th;
    zn = L(3);

    % Rotation to world frame
    gamma = atan2(aO(2,t), aO(1,t))-pi/2;
    hoek = gamma -th;
    % hoek = gamma; % alternative option
    R = [cos(hoek), -sin(hoek);
     sin(hoek),  cos(hoek)];
    G(t) = gamma;
    G = unwrap(G);

    % Define virtual triangle
    z1 = zn*[-2*sin(th) ; 0];
    z2 = zn*[sin(th) ; -cos(th)]; 
    z3 = zn*[sin(th) ; cos(th)];
    zA1 = [z1;z2;z3];
    zA1 = blkdiag(R, R, R)*zA1;

    % Calculate error
    z_norm = vecnorm(reshape(zA1, 2, [])', 2, 2);
    Z(4:6, t) = z_norm;
    e = z_norm.^l - d.^l;
    E(4:6,t) = e;
    
    % Block diagonal matrix Dz
    Dz = zeros(6, 3);
    for i = 1:3
        Dz(2*i-1:2*i, i) = zA1(2*i-1:2*i);
    end
    z_tilde = z_norm.^(l-2);
    Dz_tilde = diag(z_tilde);
    
    % Control law
    u = -c*B_bar*Dz*Dz_tilde*e;
    p_dot(1:2) = u(1:2);
    %%%%%%%%%%%% agent 1 perspective %%%%%%%%%%%%%%%%%%%

    %%%%%%%%%%%% agent 2 perspective %%%%%%%%%%%%%%%%%%%
    % Local measurements (distance/angle)
    th = acos(dot(z(3:4), aO(:,t)) / (norm(z(3:4)) * norm(aO(:,t))))-pi;
    TH(2,t) = th;
    zn = L(2);
    
    % Rotation to world frame
    gamma = atan2(aO(2,t), aO(1,t))-pi/2;
    hoek = gamma - th;
    % hoek = gamma; % alternative option
    R = [cos(hoek), -sin(hoek);
     sin(hoek),  cos(hoek)];

    % Define virtual triangle
    z1 = zn*[2*sin(th) ; 0];
    z2 = zn*[-sin(th) ; -cos(th)]; 
    z3 = zn*[-sin(th) ; cos(th)];
    zA2 = [z1;z2;z3];
    zA2 = blkdiag(R, R, R)*zA2;

    % Calculate error
    z_norm = vecnorm(reshape(zA2, 2, [])', 2, 2);
    Z(7:9, t) = z_norm;
    e = z_norm.^l - d.^l;
    E(7:9,t) = e;
    
    % Block diagonal matrix Dz
    Dz = zeros(6, 3);
    for i = 1:3
        Dz(2*i-1:2*i, i) = zA2(2*i-1:2*i);
    end
    z_tilde = z_norm.^(l-2);
    Dz_tilde = diag(z_tilde);

    % Control law
    u = -c*B_bar*Dz*Dz_tilde*e;
    p_dot(3:4) = u(3:4);
    %%%%%%%%%%%% agent 2 perspective %%%%%%%%%%%%%%%%%%%

    %%% kalman filtering
    switch KALMAN
        case true
            % Add measurement noise (if applicable)
            th_mes = G(t) + mvnrnd([0; 0], Rk)';
        
            % Apply Kalman Filter
            [x_est, P_est] = kalman_filter(x_est, P_est, th_mes,dt);
            omega_hat = x_est(2);
            W(t) = omega_hat;
    
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
    
            % Calculate control input
            uk = -omega_hat*A_bar * zv;
            uk = uk(1:6);
        
        case false
            uk = zeros(6,1);
    end

    % Solve ODE via RK4
    p = RK4_solver(p, p_dot+uk, dt);

    % Store updated positions
    P(:, t) = p;
end

%% Plot results (errors over time)
figure

% Plot 1: True errors (global frame)
subplot(3,1,1);
plot(time(2:end), E(1:3,2:end)', 'LineWidth', 2);
xlabel("Time, t, [s]");
ylabel("Error, e_k, [m]");
title("True error (global frame)");
grid on;

% Shared legend
legend('e_1(t)', 'e_2(t)', 'e_3(t)', 'Location', 'eastoutside');


% Plot 2: Virtual errors for Agent 1
subplot(3,1,2);
plot(time(2:end), E(4:6,2:end)', 'LineWidth', 2);
xlabel("Time, t, [s]");
ylabel("Error, e_k, [m]");
title("Virtual error (agent 1)");
grid on;

% Plot 3: Virtual errors for Agent 2
subplot(3,1,3);
plot(time(2:end), E(7:9,2:end)', 'LineWidth', 2);
xlabel("Time, t, [s]");
ylabel("Error, e_k, [m]");
title("Virtual error (agent 2)");
grid on;

%% Plot results (angles over time)
figure;

% Subplot 1: Theta_1 over time
subplot(2,1,1);
plot(time(2:end), TH(1,2:end), 'r','LineWidth', 2);
xlabel("Time [s]");
ylabel("Angle, \theta_1, [rad]");
title("Agent 1");
grid on;
legend('\theta_1')
ylim([0.3 1])

% Subplot 2: Theta_2 over time
subplot(2,1,2);
plot(time(2:end), TH(2,2:end),'b', 'LineWidth', 2);
xlabel("Time [s]");
ylabel("Angle, \theta_2, [rad]");
title("Agent 2");
grid on;
legend('\theta_2')
ylim([-0.7 0])

%% 2D Animation
Ns = 50; % speedup factor (integer value only)
figure;
hold on;
grid on;
axis equal;
colors = [1 0 0; 0 1 0; 0 0 1];

% Agent trajectories
traj = cell(nA, 1);

tic
for t = 1:Ns:length(time)
    clf; 
    hold on;
    grid on;
    axis equal;
    
    % Extract agent positions
    P_t = reshape(P(:, t), 2, [])';
    P3 = P_t(3,:);

    slope = a(2,t) / a(1,t);

    plot(P3(1) + 0.5*[-cos(atan(slope)) cos(atan(slope))], P3(2) + 0.5*[-sin(atan(slope)) sin(atan(slope))], 'k', 'LineWidth', 2);
    plot(P3(1) + 0.5*[-sin(atan(slope)) sin(atan(slope))], P3(2) + 0.5*[ cos(atan(slope)) -cos(atan(slope))], 'k--', 'LineWidth', 1);
    
    % Loop over agents
    for i = 1:nA
        % Update the trajectories
        traj{i} = [traj{i}; P_t(i, :)];
        
        % Colors
        agent_color = colors(mod(i-1, 3) + 1, :);  % This cycles through the colors
        
        % Plot the trajectories + positions
        plot(traj{i}(:, 1), traj{i}(:, 2), 'LineWidth', 1.5, 'Color', agent_color);
        scatter(P_t(i, 1), P_t(i, 2), 100, 'o', 'MarkerFaceColor', agent_color, 'MarkerEdgeColor', agent_color);
    end
    
    % Plot edges
    for k = 1:nE
        agents = find(B(k, :));
        plot(P_t(agents, 1), P_t(agents, 2), 'r-', 'LineWidth', 1.5);
    end
    
    % Fill the triangle
    fill(P_t(:, 1), P_t(:, 2), 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    

    % Title + small pause
    title(['Time: ', num2str((t-1)*dt), ' seconds']);
    pause(1e-3);
end
toc 

% Increase final limits a little
xL = xlim; yL = ylim;
xlim(xlim*1.1)
ylim(ylim*1.1)


%% Local functions
function [x_next] = RK4_solver(x_prev, x_dot, dt)
% RK4_SOLVER - Solve a system of ordinary differential equations using the 
% fourth-order Runge-Kutta method.
% 
% Syntax:  [x_next] = RK4_solver(x_prev, x_dot, dt)
%
% Inputs:
%    x_prev - 5x1 array with the previous state of the system
%    x_dot - 5x1 array from aircraft_dynamics with derivatives of the system
%    dt - time step size.
% 
% Outputs:
%    x_next - 5x1 array with the next state of the system

% Calculate the RK4 coefficients
k1 = x_dot;
k2 = x_dot + 0.5*dt*k1;
k3 = x_dot + 0.5*dt*k2;
k4 = x_dot + dt*k3;

% Use the RK4 coefficients to calculate the next position
x_next = x_prev + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

end

function [x_est, P_est] = kalman_filter(x_est, P_est, z, dt)
% Implements a Kalman filter for position and velocity estimation

% Define matrices
Akf = [1, dt,     0.5*dt^2, (1/6)*dt^3;
       0,  1,          dt,   0.5*dt^2;
       0,  0,           1,          dt;
       0,  0,           0,           1];

H = [1, 0, 0, 0];  % Observation matrix

Q = diag([0.1, 1, 10, 10]);
R = 1e-3;  % Larger measurement noise

% Prediction step
x_pred = Akf * x_est;
P_pred = Akf * P_est * Akf' + Q;

% Kalman Gain
K = P_pred * H' / (H * P_pred * H' + R);

% Update step
x_est = x_pred + K * (z - H * x_pred);
P_est = (eye(4) - K * H) * P_pred;

end