function mus = mu_translation(d,pdot_star,z_star)
% mu_translational  Calculates the distributed motion parameters for rigid translational
%                   motion of a triangular formation in 2D.
%
%    Syntax:
%        mus = mu_translational(d, pdot_star)
%        mus = mu_translational(d, pdot_star, z_star)
%
%    Description:
%        Returns the distributed motion parameter vector that produces 
%        uniform translational motion with desired centroid velocity 
%        pdot_star for a 2D triangular formation. The optional z_star 
%        defines a reference configuration; if omitted, one is generated 
%        from the edge lengths d.
%
%    Input:
%        d          - A 3×1 vector of desired edge lengths d = [d1; d2; d3]
%        pdot_star  - A 2×1 vector of desired centroid velocity [vx; vy]
%        z_star     - (Optional) 6×1 stacked edge-vector reference
%                     [z1; z2; z3] (default: computed from d)
%
%    Output:
%        mus - A 6×1 vector of distributed motion parameters:
%              [mu1; mu2; mu3; mu_tilde1; mu_tilde2; mu_tilde3]
%
%    Example:
%        d   = [1; 1; 1];               % Edge lengths of equilateral triangle
%        v   = [0.1; 0];                % 0.1 m/s in +x direction
%        mus = mu_translational(d, v);  % Compute motion parameters
%
%    Reference:
%        van Gool, J. P. (2025). Distributed rigid formation motion control of
%        multi-agent systems (Master’s thesis, University of Groningen).

% Model parameters
B = [ 1  0 -1;
     -1  1  0;
      0 -1  1];
B_bar = kron(B,eye(2));

% Triangle orientation (argument=optional)
if nargin < 3 || isempty(z_star)
    z_star = B_bar' * init_triangle;
end
z1 = z_star(1:2); z2 = z_star(3:4); z3 = z_star(5:6);

% Construct matrix M(z*)
O = zeros(2,1);
M = [z1, O, O, O, O, z3;
     O, z2, O, z1, O, O;
     O, O, z3, O, z2, O];

% Solve for motion parameters
mus= pinv(M)*kron(ones(3,1),pdot_star);

%%%%%%%%%%%%%%%%%%%%%%%% Local functions %%%%%%%%%%%%%%%%%%%%%%%%%%%
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
