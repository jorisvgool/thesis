function mus_prime = mu_rotation(d)
% mu_rotational  Calculates the distributed motion parameters for rigid rotational motion
%                of a triangular formation around its centroid in 2D.
%
%    Syntax:
%        mus_prime = mu_rotational(d)
%
%    Description:
%        Returns the distributed motion parameter vector that produces unit-rate 
%        (1 rad/s) clockwise rotational motion of a 2D triangular formation 
%        around its centroid under gradient-based control.
%
%    Input:
%        d - A 3×1 vector of desired edge lengths d = [d1; d2; d3]
%
%    Output:
%        mus_prime - A 6×1 vector of distributed motion parameters:
%                    [mu1; mu2; mu3; mu_tilde1; mu_tilde2; mu_tilde3]
%
%    Notes:
%        The output can be scaled by the desired angular rate omega_star to produce
%        the final motion parameters:
%
%        mus = mus_prime * omega_star;
%
%    Example:
%        d = [1; 1; 1];                   % Edge lengths of equilateral triangle
%        mus_prime = mu_rotational(d);    % Get base motion parameters
%        mus = mu_prime * (0.5 * 2*pi);   % Rotate at 0.5 revolutions per second (CW)
%
%    Reference:
%        van Gool, J. P. (2025). Distributed rigid formation motion control of 
%        multi-agent systems (Master’s thesis, University of Groningen).

% Disable warnings
warning('off')

% Model parameters
alpha0 = 1;
T = 1;
B = [ 1  0 -1;
     -1  1  0;
      0 -1  1];
[nA, nE] = size(B);
B_bar = kron(B,eye(2));
w = compute_w;
p0 = init_triangle;
c = 1; l = 2;

% Define objective function for optimization
objective = @(alpha) cost_function(alpha);

% Use fminsearch to find alpha
options = optimset('TolX', 1e-10, 'TolFun', 1e-12, 'Display', 'off');
alpha_opt = fminsearch(objective, alpha0, options); 

% Define output
mus_prime = alpha_opt * w;

%%%%%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%
function dpdt = formation_dynamics(~, p, A_bar)
% Compute relative positions
z = B_bar' * p;

% Calculate error
z_norm = vecnorm(reshape(z, 2, [])', 2, 2);
e = z_norm.^l - d.^l;

% Block diagonal matrix Dz
Dz = zeros(6, 3);
for i = 1:3
    Dz(2*i-1:2*i, i) = z(2*i-1:2*i);
end

% Compute control input
dpdt = -c * B_bar * Dz *e + A_bar*z;
end

%%%

function w = compute_w
% Preallocate f and f_tilde
f = zeros(3,1);
f_tilde = zeros(3,1);

% Compute f_k and f_tilde_k for k = 1,2,3
for k = 1:3
    kp1 = mod(k,3) + 1;
    kp2 = mod(k+1,3) + 1;
    
    f(k)       = -1.5 * d(k)^2 + 0.5 * (d(kp1)^2 - d(kp2)^2);
    f_tilde(k) =  1.5 * d(kp2)^2 + 0.5 * (d(k)^2 - d(kp1)^2);
end

% Build coefficient matrix A
A = [...
    f(1), 0,    0,    0,    0,    f_tilde(1);
    0,    f(2), 0,    f_tilde(2), 0,    0;
    0,    0,    f(3), 0,    f_tilde(3), 0;
    1,   -1,    0,    1,   -1,    0;
    0,    1,   -1,    0,    1,   -1;
   -1,    0,    1,   -1,    0,    1];

% Find null space
w = null(A); 
end


%%%

function J = cost_function(alpha)
    % Define motion parameters
    omega = 2*pi;
    mu = omega*alpha*w(1:3);
    mu_tilde =  omega*alpha*w(4:6);

    % Compute matrix A
    A = zeros(size(B));
    for i = 1:nA
        for k = 1:nE
            if B(i,k) == 1
                A(i, k) = mu(k);
            elseif B(i,k) == -1
                A(i, k) = mu_tilde(k);
            end
        end
    end
    A_bar = kron(A,eye(2));

    % Solve dynamical system through ODE
    optionsODE = odeset('RelTol',1e-8, 'AbsTol',1e-11, 'MaxStep',1e-2);
    odefun = @(t, p) formation_dynamics(t, p, A_bar);
    [t_out, P_out] = ode45(odefun, [0 T], p0,optionsODE);
    P = P_out';
    
    % Caluclate centroid and relative position to centroid
    P_agents = reshape(P, 2, 3, []);
    centroids = squeeze(mean(P_agents, 2));
    P_rel = P_agents - reshape(centroids, 2, 1, size(P_agents, 3));

    % Find angles as function of time
    x_rel = squeeze(P_rel(1, 1, :));
    y_rel = squeeze(P_rel(2, 1, :));
    angles = -unwrap(atan2(y_rel, x_rel));

    % Use polyfit to find omega from theta(t)
    coeffs = polyfit(t_out, angles, 1);
    omega_hat = coeffs(1);

    % Minimize cost function
    J = (omega_hat - omega)^2;
end

%%%

function p0 = init_triangle
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

end